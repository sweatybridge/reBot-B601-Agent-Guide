"""Isolated execute + extract + detect activity for the durable oracle workflow.

This is the capture/planner merge at the process boundary: the planner half
(play the trajectory on the edge, persist the gateway's telemetry) and the
capture half (decode the recorded trajectory, find the stillest hold frame,
detect the tip) run in ONE isolated activity process. pg_durable orchestrates
them with `df.http()` and checkpoints each step; nothing native runs in a
postmaster child, and a crash fails the workflow step instead of resetting the
cluster.

    POST /execute  {exec_id}
      At-most-once: claims `pending -> playing` BEFORE moving. A replay sees a
      non-pending status and returns `{claimed: false}` without commanding the
      arm. Starts the MediaMTX recording around the play, ingests the completed
      file as `evidence.media_fragment`, and records the clock offset.
    POST /extract  {exec_id}
      SQL decides the per-hold windows/fragments and the stillest frame
      (`extraction_windows`, `select_hold_frame`); this service only decodes
      the requested windows and JPEG-encodes the chosen frame.
    POST /detect   {exec_id}
      Reads the extracted stills + geometry from the substrate and runs the
      green-tip detector (oracle_vision.detect_tip).

`REBOT_WORKFLOW_DRY=1` is the arm-parked validation mode: /execute never
contacts the gateway or the edge, it records a synthetic telemetry block from
the plan; /detect uses the geometry-free green centroid. It is for offline
tests only and is refused on a host with REBOT_EXECUTE=1.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
APP = ROOT / "app"
for path in (str(HERE), str(APP)):
    if path not in sys.path:
        sys.path.insert(0, path)

import extract_service  # noqa: E402
import mediamtx  # noqa: E402
import oracle_vision  # noqa: E402
from store import SubstrateStore  # noqa: E402

DSN = os.environ.get("REBOT_PG_DSN", "")
WORKSPACE = os.environ.get("REBOT_PG_WORKSPACE", "default")
PLANNER = os.environ.get("REBOT_PLANNER", "http://planner:8000").rstrip("/")
PLANNER_TOKEN = os.environ.get("REBOT_PLANNER_TOKEN", "")
MEDIAMTX_API = os.environ.get("REBOT_MEDIAMTX_API", "http://mediamtx:9997").rstrip("/")
RECORDINGS = Path(os.environ.get("REBOT_RECORDINGS", str(ROOT / "work" / "recordings")))
CAMERA = os.environ.get("REBOT_CAMERA", "rtsp://mediamtx:8554/phone")
DRY = os.environ.get("REBOT_WORKFLOW_DRY", "0") == "1"
EXECUTE = os.environ.get("REBOT_EXECUTE", "0") == "1"
KP = os.environ.get("REBOT_KP", "50,2:150,3:150")
KD = os.environ.get("REBOT_KD", "3,2:10,3:10,4:5,5:4,6:4,7:4")
PLAY_TIMEOUT = int(os.environ.get("REBOT_PLAY_TIMEOUT", "900"))

STORE = SubstrateStore(DSN) if DSN else None


def _json(sql, args=()):
    with STORE._conn() as c:
        row = c.execute(sql, args).fetchone()
    return next(iter(row.values())) if row else None


def _http_json(url, body, timeout=360, method="POST", token=""):
    data = json.dumps(body).encode("utf-8") if body is not None else None
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Rebot-Token"] = token
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as answer:
            return answer.status, json.loads(answer.read() or b"{}")
    except urllib.error.HTTPError as error:
        try:
            payload = json.loads(error.read() or b"{}")
        except (ValueError, OSError):
            payload = {"ok": False, "error": "gateway returned %s" % error.code}
        return error.code, payload
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError("cannot reach the gateway at %s: %s" % (PLANNER, error)) from error


def _video_duration(path):
    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            return 0.0
        fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
        frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
        if fps > 0 and frames > 0:
            return float(frames) / float(fps)
    finally:
        cap.release()
    return 0.0


def _ingest_recording(exec_id, segments, play_wall):
    """Ingest every segment of the recording, in playlist order.

    MediaMTX rolls a new file at `recordSegmentDuration` (10 s), so a normal
    trajectory is several files. Each becomes one fragment with a cumulative
    episode-relative span; the clock offset is relative to the FIRST segment's
    start (its name is the recording's wall-clock zero), not to the moment
    `enable()` returned.
    """
    ex = _json("SELECT execution_get(%s)", (exec_id,))
    episode = (ex or {}).get("episode_id")
    if not episode:
        raise RuntimeError("execution %s has no episode" % exec_id)
    first_start = None
    last_end = 0.0
    for seq, segment in enumerate(segments):
        data = Path(segment.path).read_bytes()
        sha = STORE.put_blob(data, "fragment")
        duration = _video_duration(segment.path)
        if duration <= 0:
            duration = 10.0  # MediaMTX recordSegmentDuration fallback
        # The cumulative episode span is computed by the catalog, not here.
        added = _json("SELECT execution_add_segment(%s,%s,%s,%s,'mp4')",
                      (exec_id, seq, sha, duration))
        last_end = float((added or {}).get("t_end", last_end))
        if first_start is None:
            first_start = float(segment.start)
    t_offset = float(play_wall - (first_start if first_start is not None else play_wall))
    _json("SELECT execution_set_recording(%s,%s)", (exec_id, t_offset))
    return {"segments": len(segments), "duration_s": round(last_end, 3),
            "t_offset": round(t_offset, 3)}


def execute(exec_id):
    ex = _json("SELECT execution_get(%s)", (exec_id,))
    if ex is None:
        raise KeyError("unknown execution %s" % exec_id)
    if not _json("SELECT execution_claim_play(%s)", (exec_id,)):
        return {"ok": True, "claimed": False, "executed": False,
                "status": ex.get("status"), "exec_id": exec_id}

    plan_id = ex["plan_id"]
    if DRY:
        if EXECUTE:
            # A dry endpoint on a host that can move the arm is a safety bug;
            # the claim is already taken, so dead-letter it rather than guess.
            _json("SELECT execution_mark_dead(%s)", (exec_id,))
            return {"ok": False, "claimed": True, "executed": False,
                    "error": "dry mode is refused when REBOT_EXECUTE=1"}
        tel = _json("SELECT execution_dry_telemetry(%s,%s)", (plan_id, KP))
        return {"ok": True, "claimed": True, "executed": True, "dry": True,
                "exec_id": exec_id, "telemetry": tel}

    recorder = None
    record_wall = None
    try:
        path_name = mediamtx.path_from_camera(CAMERA) or "phone"
        try:
            recorder = mediamtx.Recorder(MEDIAMTX_API, path=path_name, recordings=RECORDINGS)
            recorder.enable()
            record_wall = time.time()
            # Let MediaMTX open the segment before the first frame is commanded.
            time.sleep(0.2)
        except Exception as exc:  # noqa: BLE001 - recording is evidence, not a gate
            recorder = None
            record_wall = None
            recording_error = str(exc)
        else:
            recording_error = None
        play_wall = time.time()
        status, payload = _http_json(PLANNER + "/execute",
                                     {"id": plan_id, "kp": KP, "kd": KD},
                                     timeout=PLAY_TIMEOUT, token=PLANNER_TOKEN)
        if status != 200 or not payload.get("ok") or payload.get("executed") is not True:
            # The edge may have aborted with a telemetry log. Return it; the
            # graph's execution_after_play stores it and marks the ledger played
            # with an error, so the oracle can still attribute the fault. No
            # auto-replay: the claim is already taken.
            return {"ok": False, "claimed": True, "executed": False,
                    "error": payload.get("error", "the gateway refused the play"),
                    "telemetry": payload.get("telemetry")}
        recording = None
        if recorder is None:
            recording = {"error": recording_error or "recording unavailable"}
        else:
            try:
                recorder.disable()
                time.sleep(0.5)
                segments = recorder.segments(since=record_wall - 2.0) if record_wall else []
                if segments:
                    recording = _ingest_recording(exec_id, segments, play_wall)
            except Exception as exc:  # noqa: BLE001 - evidence loss is reported, not fatal
                recording = {"error": str(exc)}
        return {"ok": True, "claimed": True, "executed": True,
                "exec_id": exec_id, "telemetry": payload.get("telemetry"),
                "recording": recording, "recording_error": recording_error}
    except Exception:
        _json("SELECT execution_mark_dead(%s)", (exec_id,))
        raise
    finally:
        if recorder is not None:
            try:
                recorder.disable()
            except Exception:  # noqa: BLE001
                pass


def _fragments(episode_id):
    with STORE._conn() as c:
        rows = c.execute(
            "SELECT f.seq, f.t_start, f.t_end, f.codec, b.bytes "
            "FROM evidence.media_fragment f JOIN evidence.blob b ON b.sha256=f.sha256 "
            "WHERE f.episode_id=%s ORDER BY f.seq", (episode_id,)).fetchall()
    return [{"seq": r["seq"], "t_start": float(r["t_start"]), "t_end": float(r["t_end"]),
             "codec": r["codec"], "data": bytes(r["bytes"])} for r in rows]


def extract(exec_id):
    ex = _json("SELECT execution_get(%s)", (exec_id,))
    if ex is None:
        raise KeyError("unknown execution %s" % exec_id)
    if not ex.get("episode_id"):
        return {"holds": [], "missed": [], "error": "no episode"}
    fragments = _fragments(ex["episode_id"])
    if not fragments:
        return {"holds": [], "missed": ex.get("hold_windows") or [],
                "error": "no recording fragments"}
    by_seq = {f["seq"]: f for f in fragments}
    # Planning and selection are SQL (0022); this service only decodes and
    # JPEG-encodes the frames the catalog asks for.
    plan = _json("SELECT extraction_windows(%s)", (exec_id,)) or {}
    holds, missed = [], []
    for window in plan.get("windows") or []:
        idx = window.get("index")
        frags = [by_seq[s] for s in (window.get("fragments") or []) if s in by_seq]
        if not frags:
            missed.append(idx)
            continue
        frames = extract_service.decode_window_frames(frags, window["w0"], window["w1"])
        if not frames:
            missed.append(idx)
            continue
        choice = _json("SELECT select_hold_frame(%s,%s,%s::float8[])",
                       (exec_id, idx, [t for t, _frame in frames]))
        if not choice:
            missed.append(idx)
            continue
        best_t = float(choice["t"])
        _, frame = min(frames, key=lambda tf: abs(tf[0] - best_t))
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            missed.append(idx)
            continue
        h, w = frame.shape[:2]
        holds.append({"index": idx, "t": best_t, "score": choice["score"],
                      "frame_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
                      "width": w, "height": h})
    return {"holds": holds, "missed": missed}


def detect(exec_id):
    req = _json("SELECT execution_detect_request(%s)", (exec_id,))
    if not req or req.get("error"):
        return {"detections": {}, "error": (req or {}).get("error", "no detect request")}
    K = np.asarray(req["K"], dtype=float)
    dist = np.asarray(req["dist"], dtype=float).reshape(-1)
    T = np.asarray(req["T_base_cam"], dtype=float)
    tip = np.asarray(req["tip_offset"], dtype=float).reshape(-1)
    detections = {}
    for hold in req.get("holds") or []:
        raw = base64.b64decode(hold["frame_b64"].replace("\n", ""), validate=True)
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        q6 = np.asarray(hold.get("q_meas") or [0, 0, 0, 0, 0, 0], dtype=float)[:6]
        if DRY:
            found = oracle_vision.green_centroid(frame)
        else:
            found = oracle_vision.detect_tip(
                frame, q6, K, dist, T, tip, scale=float(hold.get("grid_scale") or 1.0))
        if found:
            detections[str(hold["index"])] = {
                "uv": found["uv"], "score": found["score"], "detector": found["detector"],
                "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                "grid_scale": round(float(hold.get("grid_scale") or 1.0), 4),
                "frame": hold.get("frame"),
            }
    return {"detections": detections}


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": True, "dry": DRY, "execute": EXECUTE,
                                 "dsn": bool(DSN), "planner": PLANNER})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
                exec_id = body.get("exec_id")
                if not exec_id:
                    self._send(400, {"error": "exec_id is required"})
                    return
                if self.path == "/execute":
                    self._send(200, execute(exec_id))
                elif self.path == "/extract":
                    self._send(200, extract(exec_id))
                elif self.path == "/detect":
                    self._send(200, detect(exec_id))
                else:
                    self._send(404, {"error": "not found"})
            except KeyError as exc:
                self._send(404, {"error": str(exc)})
            except Exception as exc:  # 5xx -> durable retry/dead-letter
                self._send(500, {"error": "%s: %s" % (type(exc).__name__, exc)})

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()
    if STORE is None:
        raise SystemExit("REBOT_PG_DSN is required")
    srv = ThreadingHTTPServer((args.host, args.port), make_handler())
    print("workflow activity on %s:%d (dry=%s execute=%s)" %
          (args.host, args.port, DRY, EXECUTE), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
