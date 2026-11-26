"""Unified isolated activity service for durable pg_durable workflows.

Runs native CV/FFmpeg decode, green-tip detection, optional embedding, and the
at-most-once play/record wrapper in ONE isolated process reachable via `df.http()`:

    GET  /health   Liveness + Postgres connectivity check
    POST /execute  {exec_id}   At-most-once motion claim, MediaMTX record + atomic segment ingest
    POST /extract  {exec_id}   Single-call SQL extraction_plan + bounded seek/decode of target_t
    POST /detect   {exec_id}   Green-tip detection (oracle_vision.detect_tip / dry green_centroid)
    POST /embed    {data_b64}  Optional isolated frame embedding (replaces standalone embed_service)
"""
from __future__ import annotations

import argparse
import base64
import binascii
import json
import math
import os
import signal
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
APP = ROOT / "app"
for _path in (str(HERE), str(APP)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import embed as embed_mod  # noqa: E402
import mediamtx  # noqa: E402
import oracle_vision  # noqa: E402
from store import SubstrateStore  # noqa: E402

DSN = os.environ.get("REBOT_PG_DSN", "")
WORKSPACE = os.environ.get("REBOT_PG_WORKSPACE", "default")
PLANNER = os.environ.get("REBOT_PLANNER", "http://planner:8000").rstrip("/")
PLANNER_TOKEN = os.environ.get("REBOT_PLANNER_TOKEN", "")
ACTIVITY_TOKEN = os.environ.get("REBOT_ACTIVITY_TOKEN", "")
MEDIAMTX_API = os.environ.get("REBOT_MEDIAMTX_API", "http://mediamtx:9997").rstrip("/")
RECORDINGS = Path(os.environ.get("REBOT_RECORDINGS", str(ROOT / "work" / "recordings")))
CAMERA = os.environ.get("REBOT_CAMERA", "rtsp://mediamtx:8554/phone")
DRY = os.environ.get("REBOT_WORKFLOW_DRY", "0") == "1"
EXECUTE = os.environ.get("REBOT_EXECUTE", "0") == "1"
GRASP_VERIFY = os.environ.get("REBOT_GRASP_VERIFY", "1") == "1"
KP = os.environ.get("REBOT_KP", "50,2:150,3:150")
KD = os.environ.get("REBOT_KD", "3,2:10,3:10,4:5,5:4,6:4,7:4")
PLAY_TIMEOUT = int(float(os.environ.get("REBOT_PLAY_TIMEOUT", "900")))
MAX_BODY_BYTES = 32 * 1024 * 1024
_EDGE_TOL = 1e-6

STORE = SubstrateStore(DSN) if DSN else None
_RECORDER_LOCK = threading.Lock()
_EMBED_LOCK = threading.Lock()
_EMBEDDER = embed_mod.HistogramEmbedder()


class NotFoundError(Exception):
    """Domain 404 (distinct from internal KeyError which must be 500)."""


class BadRequestError(Exception):
    """Domain 400 (malformed payload; never triggers durable retries)."""


def _json(sql, args=()):
    return STORE.scalar(sql, args)


def _http_json(url, body, timeout=360, method="POST", token=""):
    data = json.dumps(body, allow_nan=False).encode("utf-8") if body is not None else None
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
        raise RuntimeError("cannot reach %s: %s" % (url, error)) from error


def _video_duration(path: Path) -> float:
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


def _ingest_recording(exec_id: str, segments, play_wall: float) -> dict:
    """Ingest all MediaMTX segments atomically in ONE SQL call, then clean spool."""
    blobs: list[bytes] = []
    durations: list[float] = []
    first_start = None
    paths: list[Path] = []
    for segment in segments:
        p = Path(segment.path)
        blobs.append(p.read_bytes())
        dur = _video_duration(p)
        durations.append(dur if dur > 0 else 10.0)
        if first_start is None:
            first_start = float(segment.start)
        paths.append(p)
    t_offset = float(play_wall - (first_start if first_start is not None else play_wall))
    result = STORE.ingest_execution_segments(exec_id, blobs, durations, t_offset, codec="mp4")
    for p in paths:
        try:
            p.unlink()
        except OSError:
            pass
    return result


def _stop_and_ingest(exec_id: str, recorder, record_wall: float | None, play_wall: float,
                     recording_error: str | None) -> dict | None:
    if recorder is None:
        return {"error": recording_error or "recording unavailable"}
    try:
        recorder.disable()
        time.sleep(0.5)
        # mediamtx.Recorder.segments() already subtracts 2.0 s tolerance internally.
        segments = recorder.segments(since=record_wall) if record_wall else []
        if segments:
            return _ingest_recording(exec_id, segments, play_wall)
        return None
    except Exception as exc:  # noqa: BLE001 - evidence loss is reported, not fatal
        return {"error": str(exc)}


def _grab_camera_frame(camera_url: str) -> np.ndarray | None:
    """Grab one live BGR frame from CAMERA without disrupting recording."""
    if not camera_url:
        return None
    cap = cv2.VideoCapture(camera_url)
    try:
        if not cap.isOpened():
            return None
        ok, frame = cap.read()
        return frame if ok and frame is not None else None
    except Exception:  # noqa: BLE001
        return None
    finally:
        cap.release()


def _plan_grasp_check_spec(plan_id: str, workspace: str) -> dict | None:
    """Inspect a pick-and-place plan + active rig to schedule a post-retract grasp check."""
    if STORE is None or not GRASP_VERIFY or not CAMERA:
        return None
    try:
        raw = STORE.read_plan_blob(plan_id)
        if not raw:
            return None
        plan = json.loads(raw)
    except Exception:  # noqa: BLE001
        return None
    cap = plan.get("capture") if isinstance(plan.get("capture"), dict) else {}
    is_seq = bool(cap.get("sequence") or plan.get("sequence"))
    targets = cap.get("targets") or plan.get("targets") or []
    if not is_seq or len(targets) < 2:
        return None
    pick = targets[0]
    place = targets[1]
    t_pick_end = pick.get("hold_t_end")
    t_place_start = place.get("hold_t_start")
    point = pick.get("point")
    if t_pick_end is None or t_place_start is None or not point or len(point) < 3:
        return None
    span = float(t_place_start) - float(t_pick_end)
    if span < 1.5:
        return None
    # Check right after vertical retract finishes (~45% into transit, capped at +2.5 s)
    # so the gripper has lifted clear of the pick spot with >5 s of transit left to save.
    t_check = float(t_pick_end) + min(2.5, max(1.2, 0.45 * span))
    with STORE.conn() as c:
        row = c.execute(
            "SELECT r.T_base_cam, c.K, c.dist, c.image_size "
            "FROM world.shared_rig r JOIN world.calibration c ON c.calib_fingerprint = r.calib_fingerprint "
            "WHERE r.workspace_id = %s AND r.status = 'active' "
            "ORDER BY r.fitted DESC NULLS LAST LIMIT 1",
            (workspace,),
        ).fetchone()
    if not row:
        return None
    q0 = [0.0] * 6
    q_check = None
    wps = plan.get("waypoints") or []
    if wps and isinstance(wps[0].get("q"), list) and len(wps[0]["q"]) >= 6:
        q0 = [float(v) for v in wps[0]["q"][:6]]
        best_wp = min(wps, key=lambda w: abs(float(w.get("t", 0.0)) - t_check))
        if isinstance(best_wp.get("q"), list) and len(best_wp["q"]) >= 6:
            q_check = [float(v) for v in best_wp["q"][:6]]
    return {
        "t_check": t_check,
        "point": [float(v) for v in point[:3]],
        "q0": q0,
        "q_check": q_check,
        "K": np.asarray(row["k"], dtype=float),
        "dist": np.asarray(row["dist"], dtype=float).reshape(-1),
        "T_base_cam": np.asarray(row["t_base_cam"], dtype=float),
        "image_size": list(row["image_size"]),
    }


def _start_grasp_guard(spec: dict | None, pre_frame: np.ndarray | None,
                       play_wall: float, done_event: threading.Event,
                       result_out: dict) -> threading.Thread | None:
    """Background post-retract visual grasp check: calls POST /stop on clear miss."""
    if spec is None or pre_frame is None:
        return None

    def _watch():
        wait_s = max(0.0, (play_wall + spec["t_check"]) - time.time())
        if done_event.wait(wait_s):
            return
        post_frame = _grab_camera_frame(CAMERA)
        if post_frame is None or done_event.is_set():
            return
        try:
            pre_up, turn, scale = oracle_vision.orient_frame(
                pre_frame, spec["q0"], spec["K"], spec["dist"], spec["T_base_cam"], spec["image_size"])
            post_up, _t2, _s2 = oracle_vision.orient_frame(
                post_frame, spec["q0"], spec["K"], spec["dist"], spec["T_base_cam"],
                spec["image_size"], hint_turn=turn if turn else None)
            import calib as calib_mod
            K_up = calib_mod.scale_K(
                spec["K"], (int(spec["image_size"][0]), int(spec["image_size"][1])),
                (int(pre_up.shape[1]), int(pre_up.shape[0])))
            pick_uv = oracle_vision._project_point(spec["point"], K_up, spec["dist"], spec["T_base_cam"])
            verdict = oracle_vision.verify_grasp(
                pre_up, post_up, pick_uv, scale=scale,
                post_q6=spec.get("q_check"), K=K_up, dist=spec["dist"], T_base_cam=spec["T_base_cam"])
            result_out.update(verdict)
            if verdict.get("grasped") is False and verdict.get("confidence") == "high" and not done_event.is_set():
                try:
                    _http_json(PLANNER + "/stop", {}, timeout=10, token=PLANNER_TOKEN)
                    result_out["aborted_transit"] = True
                except Exception as exc:  # noqa: BLE001
                    result_out["stop_error"] = str(exc)
        except Exception as exc:  # noqa: BLE001
            result_out["error"] = str(exc)

    th = threading.Thread(target=_watch, daemon=True)
    th.start()
    return th


def execute(exec_id: str) -> dict:
    if not _json("SELECT execution_claim_play(%s)", (exec_id,)):
        ex = _json("SELECT execution_get(%s)", (exec_id,))
        if ex is None:
            raise NotFoundError("unknown execution %s" % exec_id)
        return {"ok": True, "claimed": False, "executed": False,
                "status": ex.get("status"), "exec_id": exec_id}

    ex = _json("SELECT execution_get(%s)", (exec_id,))
    plan_id = ex["plan_id"]
    if DRY:
        if EXECUTE:
            _json("SELECT execution_fail(%s, %s)",
                  (exec_id, "dry mode is refused when REBOT_EXECUTE=1"))
            return {"ok": False, "claimed": True, "executed": False,
                    "error": "dry mode is refused when REBOT_EXECUTE=1"}
        tel = _json("SELECT execution_dry_telemetry(%s,%s)", (plan_id, KP))
        return {"ok": True, "claimed": True, "executed": True, "dry": True,
                "exec_id": exec_id, "telemetry": tel}

    # Serialize global MediaMTX recorder ownership across concurrent HTTP threads.
    with _RECORDER_LOCK:
        recorder = None
        record_wall = None
        recording_error = None
        guard_done = threading.Event()
        guard_result: dict = {}
        guard_thread = None
        try:
            grasp_spec = _plan_grasp_check_spec(plan_id, WORKSPACE)
            pre_frame = _grab_camera_frame(CAMERA) if grasp_spec is not None else None
            path_name = mediamtx.path_from_camera(CAMERA) or "phone"
            try:
                recorder = mediamtx.Recorder(MEDIAMTX_API, path=path_name, recordings=RECORDINGS)
                existing_paths = {s.path for s in recorder.segments()}
                recorder.enable()
                record_wall = time.time()
                # Wait up to 2.5 s for MediaMTX to lock onto the first H.264 IDR
                # keyframe and open segment 0 before arm motion starts, so t_offset >= 0.
                deadline = record_wall + 2.5
                while time.time() < deadline:
                    if any(s.path not in existing_paths for s in recorder.segments(since=record_wall)):
                        break
                    time.sleep(0.1)
            except Exception as exc:  # noqa: BLE001
                recorder = None
                record_wall = None
                recording_error = str(exc)

            play_wall = time.time()
            guard_thread = _start_grasp_guard(grasp_spec, pre_frame, play_wall, guard_done, guard_result)
            # Leave 30 s headroom inside df.http's PLAY_TIMEOUT for recorder teardown + ingest.
            gateway_timeout = max(30, PLAY_TIMEOUT - 30)
            status, payload = _http_json(
                PLANNER + "/execute",
                {"id": plan_id, "kp": KP, "kd": KD},
                timeout=gateway_timeout, token=PLANNER_TOKEN,
            )
            guard_done.set()
            if guard_thread is not None:
                guard_thread.join(timeout=2.0)
            # Ingest recording even on abort so the oracle has camera evidence for fault attribution!
            recording = _stop_and_ingest(exec_id, recorder, record_wall, play_wall, recording_error)
            recorder = None
            has_recording = isinstance(recording, dict) and int(recording.get("segments") or 0) > 0

            if status != 200 or not payload.get("ok") or payload.get("executed") is not True:
                return {"ok": False, "claimed": True, "executed": False,
                        "error": payload.get("error", "the gateway refused the play"),
                        "telemetry": payload.get("telemetry"),
                        "grasp_check": guard_result or None,
                        "recording": recording, "recording_error": recording_error,
                        "no_recording": not has_recording}
            return {"ok": True, "claimed": True, "executed": True,
                    "exec_id": exec_id, "telemetry": payload.get("telemetry"),
                    "grasp_check": guard_result or None,
                    "recording": recording, "recording_error": recording_error,
                    "no_recording": not has_recording}
        except Exception as exc:
            _json("SELECT execution_fail(%s, %s)", (exec_id, "%s: %s" % (type(exc).__name__, exc)))
            raise
        finally:
            guard_done.set()
            if recorder is not None:
                try:
                    recorder.disable()
                except Exception:  # noqa: BLE001
                    pass


def _frame_time(pos_ms: float, idx: int, fps: float) -> float:
    if pos_ms and pos_ms > 0:
        return pos_ms / 1000.0
    if fps and fps > 0:
        return idx / fps
    return 0.0


def decode_best_frame_for_window(fragments: list[dict], w0: float, w1: float,
                                 target_t: float) -> tuple[float, np.ndarray] | None:
    """Decode only the single frame closest to SQL-computed `target_t` within [w0, w1].

    Keeps at most ONE BGR frame in RAM (vs ~370 MB buffering an entire 2 s window)
    and isolates corrupt fragments without killing unrelated holds.
    """
    best: tuple[float, float, np.ndarray] | None = None  # (dist, t, frame)
    for frag in fragments:
        path = None
        cap = None
        try:
            fd, path = tempfile.mkstemp(suffix=".mp4")
            with os.fdopen(fd, "wb") as fh:
                fh.write(frag["data"])
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                continue
            fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
            t0 = float(frag["t_start"])
            idx = 0
            # Seek near w0 when the window starts well into the fragment.
            rel_start = max(0.0, w0 - t0 - 0.5)
            if rel_start > 1.0 and fps > 0:
                cap.set(cv2.CAP_PROP_POS_MSEC, rel_start * 1000.0)
                pos_after = cap.get(cv2.CAP_PROP_POS_MSEC) or 0.0
                idx = int(round((pos_after / 1000.0) * fps))
            while True:
                pos_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                ok, frame = cap.read()
                if not ok:
                    break
                t = t0 + _frame_time(pos_ms, idx, fps)
                if w0 - _EDGE_TOL <= t <= w1 + _EDGE_TOL:
                    dist = abs(t - target_t)
                    if best is None or dist < best[0]:
                        best = (dist, t, frame)
                idx += 1
                if t > w1 + 0.25:
                    break
        finally:
            if cap is not None:
                cap.release()
            if path and os.path.exists(path):
                os.unlink(path)
    return (best[1], best[2]) if best is not None else None


def extract(exec_id: str) -> dict:
    """Item 2: 1 DB fetch (`extraction_plan` + needed fragments) + bounded single-frame decode."""
    try:
        plan, by_seq = STORE.extraction_plan_with_fragments(exec_id)
    except Exception as exc:
        if "unknown execution" in str(exc):
            raise NotFoundError(str(exc)) from exc
        raise
    if not plan.get("episode_id"):
        return {"holds": [], "missed": [], "error": "no episode"}
    windows = plan.get("windows") or []
    if not by_seq:
        return {"holds": [], "missed": [w.get("index") for w in windows],
                "error": "no recording fragments"}

    hold_metas, jpeg_blobs, missed = [], [], []
    for window in windows:
        idx = window.get("index")
        frags = [by_seq[int(s)] for s in (window.get("fragments") or []) if int(s) in by_seq]
        if not frags:
            missed.append(idx)
            continue
        target_t = float(window.get("target_t") if window.get("target_t") is not None else window["midpoint"])
        picked = decode_best_frame_for_window(frags, float(window["w0"]), float(window["w1"]), target_t)
        if picked is None:
            missed.append(idx)
            continue
        best_t, frame = picked
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            missed.append(idx)
            continue
        h, w = frame.shape[:2]
        score = window.get("score")
        if score is not None and not math.isfinite(float(score)):
            score = None
        hold_metas.append({
            "index": idx, "t": round(float(best_t), 6), "score": score,
            "width": int(w), "height": int(h),
        })
        jpeg_blobs.append(buf.tobytes())
    if hold_metas:
        persisted = STORE.store_execution_stills(exec_id, hold_metas, jpeg_blobs)
        return {
            "stored": int(persisted.get("stored") or 0),
            "holds": persisted.get("holds") or [],
            "missed": missed,
        }
    return {"stored": 0, "holds": [], "missed": missed}


def detect(exec_id: str) -> dict:
    try:
        req, by_sha = STORE.detect_request_with_stills(exec_id)
    except Exception as exc:
        if "unknown execution" in str(exc):
            raise NotFoundError(str(exc)) from exc
        raise
    if not req or req.get("error"):
        return {"detections": {}, "error": (req or {}).get("error", "no detect request")}
    K = dist = T = tip = calib_size = None
    if not DRY:
        K = np.asarray(req["K"], dtype=float)
        dist = np.asarray(req["dist"], dtype=float).reshape(-1)
        T = np.asarray(req["T_base_cam"], dtype=float)
        tip = np.asarray(req["tip_offset"], dtype=float).reshape(-1)
        calib_size = req.get("image_size")
    detections = {}
    upright_by_index: dict[int, tuple[np.ndarray, float]] = {}
    q6_by_index: dict[int, np.ndarray] = {}
    cached_turn: int | None = None
    for hold in req.get("holds") or []:
        sha = hold.get("frame")
        raw = by_sha.get(sha) if sha else None
        if raw is None and hold.get("frame_b64"):
            raw = base64.b64decode(hold["frame_b64"].replace("\n", ""), validate=True)
        if not raw:
            continue
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        scale = float(hold.get("grid_scale") or 1.0)
        if DRY:
            found = oracle_vision.green_centroid(frame)
        else:
            q_raw = hold.get("q_meas") or [0, 0, 0, 0, 0, 0]
            q6 = np.asarray([0.0 if v is None else float(v) for v in q_raw[:6]], dtype=float)
            if not np.all(np.isfinite(q6)):
                continue
            frame, turn, scale = oracle_vision.orient_frame(
                frame, q6, K, dist, T, calib_size, hint_turn=cached_turn)
            if turn in (1, 3):
                cached_turn = turn
            upright_by_index[int(hold["index"])] = (frame, scale)
            q6_by_index[int(hold["index"])] = q6
            K_use = K
            if calib_size and len(calib_size) >= 2 and (
                int(frame.shape[1]) != int(calib_size[0]) or int(frame.shape[0]) != int(calib_size[1])
            ):
                import calib as calib_mod
                K_use = calib_mod.scale_K(K, (int(calib_size[0]), int(calib_size[1])),
                                          (int(frame.shape[1]), int(frame.shape[0])))
            found = oracle_vision.detect_tip(
                frame, q6, K_use, dist, T, tip, scale=scale)
        if found:
            detections[str(hold["index"])] = {
                "uv": [float(found["uv"][0]), float(found["uv"][1])],
                "score": float(found["score"]),
                "detector": found["detector"],
                "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                "grid_scale": round(float(scale), 4),
                "frame": sha,
            }

    # Offline post-grasp verification on pick-and-place sequences: compare the
    # unobstructed pre-pick still (Hold 1) against the post-transit still (Hold 3)
    # around the first pick target pixel.
    grasp_verification = None
    targets = req.get("targets") or []
    if not DRY and req.get("sequence") and len(targets) >= 2 and 1 in upright_by_index and 3 in upright_by_index:
        pick_pt = targets[0].get("point")
        if pick_pt and len(pick_pt) >= 3:
            pre_img, pre_scale = upright_by_index[1]
            post_img, _post_scale = upright_by_index[3]
            K_use = K
            if calib_size and len(calib_size) >= 2 and (
                int(pre_img.shape[1]) != int(calib_size[0]) or int(pre_img.shape[0]) != int(calib_size[1])
            ):
                import calib as calib_mod
                K_use = calib_mod.scale_K(K, (int(calib_size[0]), int(calib_size[1])),
                                          (int(pre_img.shape[1]), int(pre_img.shape[0])))
            pick_uv = oracle_vision._project_point(pick_pt, K_use, dist, T)
            if pick_uv is not None:
                grasp_verification = oracle_vision.verify_grasp(
                    pre_img, post_img, pick_uv, scale=pre_scale,
                    post_q6=q6_by_index.get(3), K=K_use, dist=dist, T_base_cam=T)

    out = {"detections": detections}
    if grasp_verification is not None:
        out["grasp_verification"] = grasp_verification
    return out


def embed_frame(body: dict) -> dict:
    still_sha = body.get("still_sha")
    if isinstance(still_sha, str) and still_sha:
        if STORE is None:
            raise BadRequestError("still_sha requires REBOT_PG_DSN")
        try:
            raw = STORE.get_blob(still_sha)
        except KeyError as exc:
            raise NotFoundError("unknown blob %s" % still_sha) from exc
    else:
        data_b64 = body.get("data_b64")
        if not isinstance(data_b64, str) or not data_b64:
            raise BadRequestError("still_sha or data_b64 is required")
        try:
            raw = base64.b64decode(data_b64, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise BadRequestError("invalid base64 data_b64") from exc
    with _EMBED_LOCK:
        vec = _EMBEDDER.embed(raw)
        return {
            "ok": True,
            "model_id": _EMBEDDER.model_id,
            "name": _EMBEDDER.name,
            "version": _EMBEDDER.version,
            "dim": _EMBEDDER.dim,
            "vector": vec,
        }


def _db_ok() -> bool:
    if STORE is None:
        return False
    try:
        return STORE.scalar("SELECT 1") == 1
    except Exception:  # noqa: BLE001
        return False


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, obj: dict):
            body = json.dumps(obj, allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _authorized(self) -> bool:
            if not ACTIVITY_TOKEN:
                return True
            return self.headers.get("X-Rebot-Token", "") == ACTIVITY_TOKEN

        def _read_json(self) -> dict:
            raw_len = self.headers.get("Content-Length", "0")
            try:
                length = int(raw_len)
            except ValueError as exc:
                raise BadRequestError("invalid Content-Length") from exc
            if length < 0 or length > MAX_BODY_BYTES:
                raise BadRequestError("payload size %d out of bounds" % length)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise BadRequestError("body is not valid JSON: %s" % exc) from exc
            if not isinstance(parsed, dict):
                raise BadRequestError("JSON body must be an object")
            return parsed

        def do_GET(self):
            if self.path == "/health":
                ok = _db_ok() if DSN else True
                self._send(200 if ok else 503, {
                    "ok": ok, "dry": DRY, "execute": EXECUTE,
                    "dsn": bool(DSN), "db": ok, "planner": PLANNER,
                })
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if not self._authorized():
                self._send(401, {"error": "unauthorized"})
                return
            try:
                body = self._read_json()
                if self.path == "/embed":
                    self._send(200, embed_frame(body))
                    return
                exec_id = body.get("exec_id")
                if not isinstance(exec_id, str) or not exec_id:
                    raise BadRequestError("exec_id is required")
                if self.path == "/execute":
                    self._send(200, execute(exec_id))
                elif self.path == "/extract":
                    self._send(200, extract(exec_id))
                elif self.path == "/detect":
                    self._send(200, detect(exec_id))
                else:
                    self._send(404, {"error": "not found"})
            except BadRequestError as exc:
                self._send(400, {"error": str(exc)})
            except NotFoundError as exc:
                self._send(404, {"error": str(exc)})
            except Exception as exc:  # 5xx -> durable retry/dead-letter
                self._send(500, {"error": "%s: %s" % (type(exc).__name__, exc)})

        def log_message(self, fmt, *args):
            pass

    return Handler


class _ActivityServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()
    if STORE is None:
        raise SystemExit("REBOT_PG_DSN is required")
    srv = _ActivityServer((args.host, args.port), make_handler())

    def _shutdown(_sig, _frame):
        threading.Thread(target=srv.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    print("workflow activity on %s:%d (dry=%s execute=%s)" %
          (args.host, args.port, DRY, EXECUTE), flush=True)
    try:
        srv.serve_forever()
    finally:
        srv.server_close()
        STORE.close()


if __name__ == "__main__":
    main()
