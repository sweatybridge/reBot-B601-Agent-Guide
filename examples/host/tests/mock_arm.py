#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A stand-in for the arm side: the gateway the capture server calls.

The eval needs a *plan* to validate task completion, and the bench is not always
powered. This is the smallest thing that answers the two routes `/sequence` uses,
in the same shapes `planner_server.py` returns, so the real capture server, the
real validation and the real sequence payload are still in the loop:

    POST /plan     {targets, tip_offset_m, gripper, options} -> {ok, id, summary, execute}
    POST /execute  {id} -> {ok, id, executed, log}
    GET  /pose     -> {ok, pose}          (the gateway's read, for completeness)
    GET  /mock/state   how many plans and executes were asked for, and for what

It can also refuse the first N plans with a realistic error, which is how the
agent's "read the error, fix the marker, try again" loop gets tested:

    python tests/mock_arm.py --port 18080 --fail-first 1
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# The gateway's own floor. A refused plan says something an operator could act on.
IK_REFUSAL = ("IK failed at target 1: the tool tip cannot reach "
              "[%.3f, %.3f, %.3f] m with the tool pointing down")


class State:
    """What the mock has been asked to do, and how it is told to misbehave."""

    def __init__(self, fail_first: int = 0, fail_mode: str = "ik",
                 fail_execute: bool = False, execute: bool = True):
        self.fail_first = int(fail_first)
        self.fail_mode = fail_mode
        self.fail_execute = bool(fail_execute)
        self.execute = bool(execute)
        self.plans = 0
        self.executes = 0
        self.refusals = 0
        self.last_targets = []
        self.last_plan = ""
        self.lock = threading.Lock()

    def snapshot(self) -> dict:
        with self.lock:
            return {"ok": True, "plans": self.plans, "executes": self.executes,
                    "refusals": self.refusals, "last_targets": self.last_targets,
                    "last_plan": self.last_plan}

    def plan_refusal(self, tip) -> str:
        if self.fail_mode == "below-floor":
            return ("tip z %.3f m is below the minimum %.3f m" % (float(tip[2]), 0.10))
        if self.fail_mode == "unreachable":
            return "target 1 is %.2f m from the base axis, past the 0.75 m reach" % (
                (float(tip[0]) ** 2 + float(tip[1]) ** 2) ** 0.5)
        return IK_REFUSAL % (float(tip[0]), float(tip[1]), float(tip[2]))


def plan_payload(body: dict, state: State):
    """The gateway's answer, in its shape. Refusal first, so it is testable."""
    targets = body.get("targets") or []
    if not targets:
        return 400, {"ok": False, "error": "'targets' must be a non-empty list"}
    tips = [target.get("tip") for target in targets]
    for tip in tips:
        if not isinstance(tip, list) or len(tip) != 3:
            return 400, {"ok": False, "error": "a target has no [x, y, z] tip"}
    with state.lock:
        state.plans += 1
        state.last_targets = tips
        refuse = state.refusals < state.fail_first
        if refuse:
            state.refusals += 1
        markers = len(tips)
    if refuse:
        return 422, {"ok": False, "error": state.plan_refusal(tips[0]),
                     "stage": "plan"}
    plan_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]
    with state.lock:
        state.last_plan = plan_id
    return 200, {
        "ok": True, "id": plan_id, "path": "mock/%s.json" % plan_id, "bytes": 4096,
        "execute": state.execute,
        "summary": {
            "markers": markers,
            "waypoints": 6 * markers,
            "duration_s": round(2.5 * markers, 1),
            "worst_ik_m": 0.0007,
            "peak_tau": 3.1,
            "target": tips[0],
            "compute_s": 0.01,
        },
    }


def execute_payload(body: dict, state: State):
    plan_id = str(body.get("id") or "")
    if not state.execute:
        return 403, {"ok": False, "error": "execution is disabled on this host"}
    if not plan_id:
        return 400, {"ok": False, "error": "'id' must be a plan id"}
    with state.lock:
        state.executes += 1
    if state.fail_execute:
        return 502, {"ok": False, "error": "mock edge: the trajectory was refused"}
    return 200, {"ok": True, "id": plan_id, "executed": True,
                 "log": "mock edge: played %s" % plan_id}


class Handler(BaseHTTPRequestHandler):
    server_version = "rebot-mock-arm/1"
    state: State = None            # set by serve()

    def log_message(self, fmt, *args):  # noqa: A002 - stdlib signature
        pass

    def _send(self, status: int, body: dict):
        raw = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return {}

    def do_GET(self):  # noqa: N802 - stdlib signature
        if self.path == "/mock/state":
            self._send(200, self.state.snapshot())
            return
        if self.path.rstrip("/") == "/pose":
            self._send(200, {"ok": True, "pose": {"ids": [1, 2, 3, 4, 5, 6, 7],
                                                  "q": [0.0] * 7}})
            return
        self._send(404, {"ok": False, "error": "no such route"})

    def do_POST(self):  # noqa: N802 - stdlib signature
        path = self.path.split("?")[0].rstrip("/")
        body = self._body()
        if path == "/mock/state":
            self._send(200, self.state.snapshot())
            return
        if path == "/plan":
            status, answer = plan_payload(body, self.state)
        elif path == "/execute":
            status, answer = execute_payload(body, self.state)
        elif path == "/move":
            status, answer = 200, {"ok": True, "id": "mock-move", "moved": True,
                                   "log": "mock edge: moved"}
        else:
            status, answer = 404, {"ok": False, "error": "no such route"}
        self._send(status, answer)


def serve(port: int = 0, host: str = "127.0.0.1", **options):
    """Start the mock on a thread; returns (server, url, state)."""
    state = State(**options)
    handler = type("BoundHandler", (Handler,), {"state": state})
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = "http://%s:%d" % (host, server.server_address[1])
    return server, url, state


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="A mock arm gateway for the agent eval.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--fail-first", type=int, default=0,
                        help="refuse this many plans before succeeding")
    parser.add_argument("--fail-mode", default="ik",
                        choices=["ik", "below-floor", "unreachable"])
    parser.add_argument("--fail-execute", action="store_true")
    parser.add_argument("--no-execute", action="store_true",
                        help="report execution as disabled, as REBOT_EXECUTE=0 does")
    args = parser.parse_args(argv)
    server, url, state = serve(args.port, args.host, fail_first=args.fail_first,
                               fail_mode=args.fail_mode,
                               fail_execute=args.fail_execute,
                               execute=not args.no_execute)
    print("mock arm gateway on %s" % url)
    print("  POST /plan  POST /execute  GET /pose  GET /mock/state")
    if args.fail_first:
        print("  refusing the first %d plan(s) with a %s error"
              % (args.fail_first, args.fail_mode))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
