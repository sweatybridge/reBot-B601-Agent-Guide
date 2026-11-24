"""Isolated embedding activity service (spec §2 crash-isolation invariant).

Native inference (CLIP/ONNX) must never run in a postmaster child: a deterministic
native crash would reset the cluster and the durable retry would replay it. This
service runs as a SEPARATE process; pg_durable reaches it via df.http(), so a crash
fails the workflow step (retry/dead-letter) and leaves the cluster untouched.

The default embedder is the deterministic HistogramEmbedder (offline testing).
A production model implements embed.Embedder and is selected here — the workflow
and the database side do not change.

    python embed_service.py --port 8765

POST /embed  {"data_b64": "<base64 bytes>"}
  -> {"model_id", "name", "version", "dim", "vector": [...]}
GET  /health -> {"ok": true, "model_id": ...}

Bad input returns 4xx; internal failures return 5xx (df.http fails the node on 5xx,
which is the durable retry/dead-letter path).
"""
from __future__ import annotations

import argparse
import base64
import binascii
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from embed import Embedder, HistogramEmbedder


def make_handler(embedder: Embedder, store=None):
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, obj: dict):
            body = json.dumps(obj, allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": True, "model_id": embedder.model_id})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/embed":
                self._send(404, {"error": "not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if length < 0 or length > 32 * 1024 * 1024:
                    raise ValueError("invalid content length")
                req = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(req, dict):
                    raise TypeError("body must be object")
                if isinstance(req.get("still_sha"), str) and req["still_sha"]:
                    if store is None:
                        raise ValueError("still_sha requires REBOT_PG_DSN")
                    try:
                        data = store.get_blob(req["still_sha"])
                    except KeyError as exc:
                        self._send(404, {"error": "blob not found: %s" % exc})
                        return
                else:
                    data = base64.b64decode(req["data_b64"], validate=True)
            except (KeyError, TypeError, ValueError, binascii.Error, json.JSONDecodeError):
                self._send(400, {"error": "expected JSON {still_sha} or {data_b64: base64}"})
                return
            try:
                with lock:
                    vec = embedder.embed(data)
            except Exception as e:  # inference failure -> 5xx -> durable retry
                self._send(500, {"error": str(e)})
                return
            self._send(200, {
                "model_id": embedder.model_id, "name": embedder.name,
                "version": embedder.version, "dim": embedder.dim, "vector": vec,
            })

        def log_message(self, fmt, *args):  # quiet by default
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    dsn = os.environ.get("REBOT_PG_DSN", "")
    store = None
    if dsn:
        from store import SubstrateStore
        store = SubstrateStore(dsn)
    embedder = HistogramEmbedder()  # production: plug the CLIP/ONNX Embedder here
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(embedder, store=store))
    print("embed activity listening on %s:%d (model %s)" % (args.host, args.port, embedder.model_id), flush=True)
    try:
        srv.serve_forever()
    finally:
        if store is not None:
            store.close()


if __name__ == "__main__":
    main()