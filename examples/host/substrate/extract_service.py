"""Isolated hold-frame extraction activity (spec §2 crash-isolation invariant).

Video decode is native (cv2's FFmpeg backend) and crash-prone on malformed input,
so it must never run in a postmaster child nor in the capture/edge gateway process.
This service runs as a SEPARATE process; pg_durable reaches it via df.http(), so a
decode crash fails the workflow step (retry/dead-letter) and leaves the cluster and
the arm gateway untouched. Same shape as embed_service.py.

The pure decision logic (clock alignment + stillness selection) is in extract.py;
this module only does the native decode and JPEG encode around it.

    python extract_service.py --port 8766

POST /extract
  {
    "fragments": [{"seq":0,"t_start":0.0,"t_end":6.0,"data_b64":"<mp4 bytes>"}, ...],
    "samples":   [{"cmd":[7],"meas":[7],"t":0.0}, ...],   # telemetry, trajectory clock
    "windows":   [{"index":1,"t_start":1.0,"t_end":1.6}, ...],
    "t_offset":  0.0,          # video_t = traj_t + t_offset (recording vs play start)
    "jpeg_quality": 92
  }
  -> {"holds":[{"index","t","score","frame_b64","width","height"}, ...],
      "missed":[index, ...]}   # windows with no decodable frame (reported, not guessed)
GET /health -> {"ok": true}

Bad input -> 4xx; decode/internal failure -> 5xx (the df.http durable retry path).
Only frames inside each hold window are retained, so memory is bounded by the
window, not the whole recording.
"""
from __future__ import annotations

import argparse
import base64
import binascii
import json
import os
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import extract


def _frame_time(pos_ms, idx, fps):
    """Episode-relative offset of a decoded frame within its fragment.

    Prefer the container timestamp (POS_MSEC); fall back to idx/fps when the
    backend reports 0 (some synthetic/raw streams), so selection stays exact.
    """
    if pos_ms and pos_ms > 0:
        return pos_ms / 1000.0
    if fps and fps > 0:
        return idx / fps
    return 0.0


def decode_window_frames(fragments, w0, w1):
    """Decode the fragments covering [w0, w1], keeping only in-window frames.

    `fragments` are dicts with `t_start` and raw `data` bytes. Returns a list of
    `(episode_time, frame)` in time order. Native decode -- isolated here.
    """
    frames = []
    for frag in fragments:
        path = None
        cap = None
        try:
            fd, path = tempfile.mkstemp(suffix=".mp4")
            with os.fdopen(fd, "wb") as fh:
                fh.write(frag["data"])
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                raise ValueError("cannot open fragment seq=%s" % frag.get("seq"))
            fps = cap.get(cv2.CAP_PROP_FPS)
            t0 = float(frag["t_start"])
            idx = 0
            while True:
                pos_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                ok, frame = cap.read()
                if not ok:
                    break
                t = t0 + _frame_time(pos_ms, idx, fps)
                if w0 - extract._EDGE_TOL <= t <= w1 + extract._EDGE_TOL:
                    frames.append((t, frame))
                idx += 1
                if t > w1 + extract._EDGE_TOL:
                    break  # past the window; no need to decode the rest
        finally:
            if cap is not None:
                cap.release()
            if path and os.path.exists(path):
                os.unlink(path)
    frames.sort(key=lambda tf: tf[0])
    return frames


def extract_holds(req):
    """Pure-ish orchestration: decode per window, select the stillest frame, encode.

    Returns the response dict. Raises on malformed input (caller maps to 4xx/5xx).
    """
    fragments = req["fragments"]
    samples = req["samples"]
    windows = req["windows"]
    t_offset = float(req.get("t_offset", 0.0))
    quality = max(1, min(100, int(req.get("jpeg_quality", 92))))
    # Decode base64 fragment payloads once.
    by_seq = {}
    for frag in fragments:
        by_seq[frag["seq"]] = {"seq": frag["seq"], "t_start": float(frag["t_start"]),
                               "t_end": float(frag["t_end"]),
                               "data": base64.b64decode(frag["data_b64"], validate=True)}
    plan = extract.plan_extraction(windows, list(by_seq.values()), t_offset)
    holds, missed = [], []
    for window, entry in zip(windows, plan):
        frags = [by_seq[s] for s in entry["fragments"] if s in by_seq]
        if not frags:
            missed.append(window.get("index"))
            continue
        frames = decode_window_frames(frags, entry["w0"], entry["w1"])
        choice = extract.select_hold_frame([t for t, _ in frames], samples, window, t_offset)
        if choice is None or not frames:
            missed.append(window.get("index"))
            continue
        best_t = choice["t"]
        _, frame = min(frames, key=lambda tf: abs(tf[0] - best_t))
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            missed.append(window.get("index"))
            continue
        h, w = frame.shape[:2]
        holds.append({"index": window.get("index"), "t": best_t, "score": choice["score"],
                      "frame_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
                      "width": w, "height": h})
    return {"holds": holds, "missed": missed}


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj, allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": True})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/extract":
                self._send(404, {"error": "not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if length < 0 or length > 32 * 1024 * 1024:
                    raise ValueError("invalid content length")
                req = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(req, dict):
                    raise TypeError("body must be object")
                for key in ("fragments", "samples", "windows"):
                    if key not in req:
                        raise KeyError(key)
            except (KeyError, TypeError, ValueError, binascii.Error, json.JSONDecodeError):
                self._send(400, {"error": "expected JSON {fragments, samples, windows}"})
                return
            try:
                result = extract_holds(req)
            except (KeyError, TypeError, ValueError, binascii.Error) as e:
                self._send(400, {"error": str(e)})
                return
            except Exception as e:  # decode failure -> 5xx -> durable retry
                self._send(500, {"error": str(e)})
                return
            self._send(200, result)

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.host, args.port), make_handler())
    print("extract activity listening on %s:%d" % (args.host, args.port), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()