#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The arm gateway: pose, plan, execute -- one HTTP service, no web framework.

Runs inside the Pinocchio image (see Dockerfile.pinocchio). It exists so the
viewers can turn a tap into motion without the host needing Python, OpenCV or
shell access: the phone posts a tapped frame to the capture server, the capture
server posts the target here, and this container talks to the board.

The board side is `rebot-edge serve` (see examples/edge-rust), so there is no
ssh, no scp and no key on the host:

    GET  /pose on the edge   <- the `rebot-edge dump` JSON
    POST /play on the edge   <- the trajectory JSON, played (or --check'd)

Why stdlib `http.server`: the image has pinocchio and numpy and nothing else, and
a lab arm does one plan at a time. A framework would be a dependency for four
routes.

    GET  /health              what is configured, and whether the edge answers
    GET  /pose                proxy of the edge's pose
    POST /plan                {target, pose?, options?} -> /app/out/<id>.json
    POST /execute             {id, kp?, kd?} -> POST /play on the edge

`/execute` refuses unless REBOT_EXECUTE=1. The file it sends was produced by the
same `plan()` the CLI uses, and the edge re-checks limits and steps before any
frame is sent -- this service adds convenience, not a new way to bypass the gate.
"""

from __future__ import annotations

import json
import logging
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import plan_to_target as planner  # noqa: E402

OUT_DIR = Path(os.environ.get("REBOT_OUT", "/app/out"))
EDGE_URL = os.environ.get("REBOT_EDGE_URL", "http://raspberrypi:8081").rstrip("/")
EDGE_TOKEN = os.environ.get("REBOT_EDGE_TOKEN", "")
TOKEN = os.environ.get("REBOT_PLANNER_TOKEN", "")
EXECUTE = os.environ.get("REBOT_EXECUTE", "0") == "1"
KP = os.environ.get("REBOT_KP", "50,2:150,3:150")
KD = os.environ.get("REBOT_KD", "3,2:10,3:10,4:5,5:4,6:4,7:4")
MAX_AGE_S = float(os.environ.get("REBOT_PLAN_MAX_AGE", "600"))

PLAN_ID = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{6}$")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                    datefmt="%H:%M:%S")


class GatewayError(Exception):
    """Reported to the caller with a status code."""

    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def edge_request(method, path, body=None, timeout=30):
    """One call to `rebot-edge serve`; returns (status, payload)."""
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    if EDGE_TOKEN:
        headers["X-Rebot-Token"] = EDGE_TOKEN
    request = urllib.request.Request(EDGE_URL + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as answer:
            return answer.status, json.loads(answer.read() or b"{}")
    except urllib.error.HTTPError as error:
        try:
            payload = json.loads(error.read() or b"{}")
        except (ValueError, OSError):
            payload = {"ok": False, "error": "the edge returned %s" % error.code}
        return error.code, payload
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise GatewayError("cannot reach the edge at %s: %s" % (EDGE_URL, error), 502) from error


def dump_pose():
    status, payload = edge_request("GET", "/pose", timeout=20)
    if status != 200:
        raise GatewayError(payload.get("error", "the edge refused the pose dump"), 502)
    if "ids" not in payload or "q" not in payload:
        raise GatewayError("the edge's pose has no ids/q", 502)
    return payload


def plan_one(body):
    target = body.get("target")
    if not isinstance(target, dict):
        raise GatewayError("'target' must be the locate_target.py object")
    pose = body.get("pose")
    if pose is None:
        pose = dump_pose()
    elif not isinstance(pose, dict) or "ids" not in pose:
        raise GatewayError("'pose' must be a rebot-edge dump when supplied")

    options = planner.Options()
    for name, value in (body.get("options") or {}).items():
        if not hasattr(options, name):
            raise GatewayError("unknown option %r" % name)
        setattr(options, name, value)
    options.quiet = True

    started = time.time()
    try:
        payload, summary = planner.plan(pose, target, options)
    except planner.PlanError as error:
        raise GatewayError(str(error), 422) from error

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plan_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]
    path = OUT_DIR / ("%s.json" % plan_id)
    path.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")

    logging.info("plan %s: %d waypoints, %.2f s, %.2f s to compute",
                 plan_id, summary["waypoints"], summary["duration_s"], time.time() - started)
    summary["compute_s"] = round(time.time() - started, 2)
    return {"ok": True, "id": plan_id, "path": str(path), "bytes": path.stat().st_size,
            "summary": summary, "execute": EXECUTE}


def execute_one(body):
    if not EXECUTE:
        raise GatewayError("execution is disabled on this host (REBOT_EXECUTE=1 to allow it)", 403)
    plan_id = str(body.get("id") or "")
    if not PLAN_ID.match(plan_id):
        raise GatewayError("'id' must be a plan id")
    path = OUT_DIR / ("%s.json" % plan_id)
    if not path.is_file():
        raise GatewayError("no plan %s on disk" % plan_id, 404)
    age = time.time() - path.stat().st_mtime
    if age > MAX_AGE_S:
        raise GatewayError("plan %s is %.0f s old (limit %.0f s): plan again" % (plan_id, age, MAX_AGE_S),
                           409)

    trajectory = json.loads(path.read_text(encoding="utf-8"))
    query = urllib.parse.urlencode({
        "kp": str(body.get("kp") or KP),
        "kd": str(body.get("kd") or KD),
    })
    status, payload = edge_request("POST", "/play?" + query, body=trajectory, timeout=600)
    if status != 200:
        raise GatewayError(payload.get("error", "the edge refused the trajectory"), 502)

    logging.info("edge played %s: executed=%s", plan_id, payload.get("executed"))
    return {
        "ok": True,
        "id": plan_id,
        "executed": bool(payload.get("executed", True)),
        "log": str(payload.get("output", ""))[-4000:],
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "rebot-arm-gateway/1"

    def log_message(self, format, *args):  # noqa: A002 - stdlib signature
        logging.info("%s %s", self.address_string(), format % args)

    def _authorised(self):
        if not TOKEN:
            return True
        return self.headers.get("X-Rebot-Token", "") == TOKEN

    def _body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        if length > 1_000_000:
            raise GatewayError("body too large", 413)
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise GatewayError("body is not JSON: %s" % error) from error

    def _send(self, status, payload):
        blob = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(blob)))
        self.end_headers()
        self.wfile.write(blob)

    def do_GET(self):  # noqa: N802 - stdlib signature
        try:
            if self.path == "/health":
                edge = {"url": EDGE_URL, "ok": False}
                try:
                    status, payload = edge_request("GET", "/health", timeout=3)
                    edge["ok"] = status == 200
                    for key in ("version", "can_up", "allow_execute", "playing"):
                        edge[key] = payload.get(key)
                except GatewayError as error:
                    edge["error"] = str(error)
                self._send(200, {
                    "ok": True,
                    "execute": EXECUTE,
                    "edge": edge,
                    "out": str(OUT_DIR),
                    "plans": len(list(OUT_DIR.glob("*.json"))) if OUT_DIR.is_dir() else 0,
                })
            elif self.path == "/pose":
                self._send(200, {"ok": True, "pose": dump_pose()})
            else:
                self._send(404, {"ok": False, "error": "no such route"})
        except GatewayError as error:
            self._send(error.status, {"ok": False, "error": str(error)})

    def do_POST(self):  # noqa: N802 - stdlib signature
        try:
            if not self._authorised():
                raise GatewayError("missing or wrong X-Rebot-Token", 401)
            if self.path == "/plan":
                self._send(200, plan_one(self._body()))
            elif self.path == "/execute":
                self._send(200, execute_one(self._body()))
            else:
                self._send(404, {"ok": False, "error": "no such route"})
        except GatewayError as error:
            logging.warning("%s -> %s: %s", self.path, error.status, error)
            self._send(error.status, {"ok": False, "error": str(error)})
        except Exception as error:  # noqa: BLE001 - never kill the server on one request
            logging.exception("unhandled error on %s", self.path)
            self._send(500, {"ok": False, "error": "%s: %s" % (type(error).__name__, error)})


def main():
    port = int(os.environ.get("REBOT_PLANNER_PORT", "8000"))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    logging.info("arm gateway on :%d, edge=%s, execute=%s, edge_token=%s",
                 port, EDGE_URL, "on" if EXECUTE else "off (REBOT_EXECUTE=1 to enable)",
                 "set" if EDGE_TOKEN else "none")
    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
