#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conversation trajectories for the /live agent, checked against the real model.

The sidebar itself is a browser page; this harness is the same conversation
without a browser, so it can be replayed as often as you like. For each case it
shows the model one still from `state/captures`, offers the *same seven tools the
page offers*, and executes them with the host's own geometry: the tag is solved
from that still (or a fitted rig is used), pixels become arm points through
`extrinsics.ray_plane`, and `move_arm` posts to the capture server's `/sequence`
-- which plans, and never here moves the arm.

A trajectory passes when the model does the job *and* the numbers agree: it marks
the objects the case names, at the pixels the case names, plans before moving, and
does not claim the arm moved when nothing did.

    python tests/agent_eval.py --list                    # coverage and counts
    python tests/agent_eval.py --dry                     # geometry only, no model
    python tests/agent_eval.py --runs 3                  # the whole set, 3 times
    python tests/agent_eval.py --only plush-next-to-pen --runs 5 -v
    python tests/agent_eval.py --scaffold <capture-id>   # a case for a new still
    python tests/agent_eval.py --extract state/recordings/phone/x.mp4 --at 4.0 \
        --out tests/stills/sweep-4s.jpg                  # a still from a recording

Nothing here is a unit test: it needs the model endpoint. The endpoint, model and
key come from the usual host environment, so the same variables that configure
the page's sidebar configure this.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import mock_arm

# The model answers in prose, and this console is cp1252 on Windows: without this
# a curly quote or an arrow in a reply kills the report.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = Path(__file__).resolve().parent
HOST = HERE.parent
APP = HOST / "app"
STATE = HOST / "state"
CASES = HERE / "agent_cases"
STILLS = HERE / "stills"
sys.path.insert(0, str(APP))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import base_marker  # noqa: E402
import calib as calib_mod  # noqa: E402
import extrinsics  # noqa: E402
import locate_target  # noqa: E402

# --------------------------------------------------------------------------- #
# The page's contract, repeated here
# --------------------------------------------------------------------------- #
# These names must match `live.html`'s AGENT_TOOLS; the runner checks that at
# start-up so the two cannot drift apart silently.
TOOL_NAMES = ("rig", "pixel_to_arm", "arm_to_pixel", "add_marker",
              "clear_markers", "move_arm", "capture_view")

TOOLS = [
    {"type": "function", "function": {
        "name": "rig",
        "description": "The fitted camera and the arm's limits: image size, worktop plane z, "
                       "tip offset, the height floor, and how many markers are queued. Call it "
                       "once before converting anything.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}},
    {"type": "function", "function": {
        "name": "pixel_to_arm",
        "description": "Convert an image pixel to the arm-base tool-tip point at an approach "
                       "height. Reports whether the point is inside the reach band and above the "
                       "height floor. Use it to check a target before adding a marker.",
        "parameters": {"type": "object", "properties": {
            "u": {"type": "number", "description": "image column in pixels"},
            "v": {"type": "number", "description": "image row in pixels"},
            "height_m": {"type": "number", "description": "approach height above the worktop, "
                                                          "metres"}},
            "required": ["u", "v"]}}},
    {"type": "function", "function": {
        "name": "arm_to_pixel",
        "description": "Where an arm-base point appears in the camera image. Use it to check "
                       "that a computed point lands where you meant.",
        "parameters": {"type": "object", "properties": {
            "x": {"type": "number", "description": "metres, base frame"},
            "y": {"type": "number", "description": "metres, base frame"},
            "z": {"type": "number", "description": "metres above the robot base"}},
            "required": ["x", "y", "z"]}}},
    {"type": "function", "function": {
        "name": "add_marker",
        "description": "Queue a numbered target at an image pixel; it appears on the screen. The "
                       "arm visits queued markers in order, alternating the gripper. Returns the "
                       "tip point and whether it is reachable.",
        "parameters": {"type": "object", "properties": {
            "u": {"type": "number", "description": "image column in pixels"},
            "v": {"type": "number", "description": "image row in pixels"},
            "height_m": {"type": "number", "description": "approach height above the worktop, "
                                                          "metres"}},
            "required": ["u", "v"]}}},
    {"type": "function", "function": {
        "name": "clear_markers",
        "description": "Drop every queued marker.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}},
    {"type": "function", "function": {
        "name": "move_arm",
        "description": "Plan the queued markers into one trajectory. execute=false plans only and "
                       "returns the summary -- do that first. execute=true also moves the arm "
                       "when the host allows it.",
        "parameters": {"type": "object", "properties": {
            "execute": {"type": "boolean", "description": "false to plan without moving"}},
            "required": ["execute"]}}},
    {"type": "function", "function": {
        "name": "capture_view",
        "description": "Ask for a fresh camera frame; it arrives as the next user message. Use it "
                       "after the arm moves, or when the view may have changed.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}},
]

# Kept in step with live.html's systemPrompt(); the rules are the point, not the
# wording. Anything the page says about itself (its UI) is left out here.
SYSTEM_PROMPT = "\n".join([
    "You operate a B601 arm through a workstation page; the engineer watches the same view.",
    "You are shown a still from the fixed camera looking down at the workbench.",
    "Image coordinates are pixels: u grows right, v grows down, origin at the top-left.",
    "Arm coordinates are metres in the robot base frame: x and y on the worktop, z up.",
    "",
    "Tools:",
    "- rig(): camera matrix, worktop plane, height floor, reach band, queued markers.",
    "- pixel_to_arm(u, v, height_m): the tool-tip point for a pixel, and whether it is reachable.",
    "- arm_to_pixel(x, y, z): where an arm point lands in the image; use it to check yourself.",
    "- add_marker(u, v, height_m): queue a numbered target; markers are visited in order.",
    "- clear_markers(): drop the queued targets.",
    "- move_arm(execute): plan the markers; execute=false plans without moving.",
    "- capture_view(): get a fresh frame, delivered in the next user message.",
    "",
    "Work in this order: look at the image, check a pixel with pixel_to_arm, add the markers,",
    "plan with move_arm(false) and read the summary, then move with move_arm(true).",
    "If a plan or a move fails, read the error, change the marker or the height, and try once more.",
    "Never say the arm moved unless a tool result says executed: true.",
    "Heights are metres above the worktop, clamped to the floor; the badge height is the default.",
    "Keep replies to a few sentences: the engineer is watching the arm, not reading an essay.",
])


class EvalError(Exception):
    """The harness cannot continue: no still, no pose, no model."""


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #

def dotenv(path: Path) -> dict:
    """The host's .env, read the way compose reads it. Missing file is fine."""
    values = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip()
    return values


ENV = dotenv(HOST / ".env")


def setting(name: str, fallback: str = "") -> str:
    value = os.environ.get(name)
    if value is not None and value != "":
        return value
    return ENV.get(name, fallback)


def case_paths() -> list[Path]:
    return sorted(CASES.glob("*.json")) if CASES.is_dir() else []


def load_case(path: Path) -> dict:
    try:
        case = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise EvalError("%s is not JSON: %s" % (path.name, exc)) from exc
    case.setdefault("id", path.stem)
    if not case.get("user"):
        raise EvalError("%s has no 'user' message" % path.name)
    if not (case.get("capture") or case.get("still")):
        raise EvalError("%s needs a 'capture' id or a 'still' path" % path.name)
    return case


def still_for(case: dict) -> Path:
    if case.get("still"):
        path = Path(case["still"])
        if not path.is_absolute():
            path = (HERE / path).resolve()
        if not path.is_file():
            raise EvalError("%s: still not found: %s" % (case["id"], path))
        return path
    path = STATE / "captures" / str(case["capture"]) / "frame.jpg"
    if not path.is_file():
        raise EvalError("%s: no frame.jpg for capture %s" % (case["id"], case["capture"]))
    return path


def default_calibration() -> Path:
    files = sorted(p for p in (STATE / "calib").glob("*.json")
                   if not p.name.startswith(("extrinsics-", "witness-")))
    if not files:
        raise EvalError("no calibration in %s" % (STATE / "calib"))
    return files[-1]


def default_rig() -> str:
    """The standard rig, when one has been fitted; nothing to configure."""
    path = STATE / "calib" / "extrinsics-fixed.json"
    return str(path) if path.is_file() else ""


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class MockEdge:
    """A real capture server with the arm side replaced.

    `mock_arm` stands in for the gateway, and the capture server is the ordinary
    one from `app/`, so `/sequence` still validates the targets, the floor and the
    tip offset exactly as it does on the bench. What is gone is Pinocchio, the
    board and the CAN bus -- and with them every reason a plan could be slow or
    unavailable. It never moves anything: the mock answers `/execute` with
    `executed: true` and plays no trajectory.
    """

    def __init__(self, args):
        self.fail_first = int(args.mock_fail_first)
        self.server, self.url, self.state = mock_arm.serve(
            0, "127.0.0.1", fail_first=self.fail_first,
            fail_mode=args.mock_fail_mode, execute=not args.mock_no_execute)
        self.work = Path(tempfile.mkdtemp(prefix="agent-eval-"))
        self.token = args.token or "eval-token"
        self.base = "http://127.0.0.1:%d" % free_port()
        self.proc = subprocess.Popen(
            [sys.executable, str(APP / "capture_server.py"),
             "--host", "127.0.0.1", "--port", self.base.rsplit(":", 1)[1],
             "--token", self.token, "--planner", self.url, "--execute", "--no-whip",
             "--quiet", "--captures", str(self.work / "captures"),
             "--calib-dir", str(self.work / "calib")],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(self.base + "/health", timeout=2):
                    break
            except Exception:  # noqa: BLE001 - not up yet
                if self.proc.poll() is not None:
                    raise EvalError("the mock-edge capture server exited at once")
                time.sleep(0.3)
        else:
            raise EvalError("the mock-edge capture server never answered /health")
        args.server = self.base
        args.token = self.token

    def reset(self):
        with self.state.lock:
            self.state.plans = 0
            self.state.executes = 0
            self.state.refusals = 0
            self.state.fail_first = self.fail_first

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001 - best effort
            self.proc.kill()
        self.server.shutdown()
        shutil.rmtree(self.work, ignore_errors=True)


def tool_names_in_page() -> set:
    """The tool names live.html actually declares, so the two cannot drift."""
    page = (HOST / "web" / "live.html").read_text(encoding="utf-8")
    found = set(re.findall(r'name:\s*"([a-z_]+)"', page))
    return found & set(TOOL_NAMES)


def in_box(u: float, v: float, box) -> bool:
    u0, v0, u1, v1 = box
    return min(u0, u1) <= u <= max(u0, u1) and min(v0, v1) <= v <= max(v0, v1)


def boxes_of(case: dict, session) -> dict:
    """The case's object boxes, in pixels of this still.

    A case may give them as fractions of the frame (`"coordinates":
    "normalized"`), which keeps it valid when the same bench is shot at a new
    resolution -- the stills here are 1080x1920 today and need not stay so.
    """
    raw = case.get("objects") or {}
    if case.get("coordinates") != "normalized":
        return raw
    width, height = session.width_px, session.height_px
    return {name: [float(box[0]) * width, float(box[1]) * height,
                   float(box[2]) * width, float(box[3]) * height]
            for name, box in raw.items()}


def round3(value: float) -> float:
    return round(float(value), 3)


# --------------------------------------------------------------------------- #
# One case, one run: the bench a session drives
# --------------------------------------------------------------------------- #

class Session:
    """Stands in for the page: the still, its pose, the marker list, the tools."""

    def __init__(self, args, case: dict, still: Path):
        self.args = args
        self.case = case
        self.still = still
        self.image = cv2.imread(str(still), cv2.IMREAD_COLOR)
        if self.image is None:
            raise EvalError("%s: cannot read %s" % (case["id"], still))
        self.height_px, self.width_px = self.image.shape[:2]
        self.calibration_path = Path(args.calibration) if args.calibration else default_calibration()
        self.calibration = calib_mod.load(str(self.calibration_path),
                                          image_size=(self.width_px, self.height_px))
        self.plane_z = float(args.plane_z)
        self.tip_m = float(args.tip_mm) / 1000.0
        self.floor = max(0.0, float(args.min_z) - self.plane_z)
        self.default_height = float(args.hover_m)
        self.markers = []           # {"u","v","tip","height"}
        self.tool_calls = []        # (name, args)
        self.plan_results = []      # move_arm results
        self.T = self._pose()
        self.image_url = ("data:image/jpeg;base64,"
                          + base64.b64encode(still.read_bytes()).decode())

    # -- geometry ---------------------------------------------------------- #

    def _pose(self):
        """T_base_cam for this still: a fitted rig, or the tag in the frame."""
        if self.args.extrinsics:
            rig = extrinsics.load_rig(self.args.extrinsics)
            return np.asarray(rig.T_base_cam, dtype=float)
        spec = base_marker.parse_spec(self.args.marker_at, float(self.args.marker_size))
        if spec is None:
            raise EvalError("%s: no tag placement (REBOT_MARKER_AT) and no rig"
                            % self.case["id"])
        options = locate_target.Options(
            tag_size=float(spec["size_m"]), tag_x=float(spec["at"][0]),
            tag_y=float(spec["at"][1]), tag_z=float(spec["at"][2]),
            tag_yaw=math.degrees(float(spec.get("yaw", 0.0))),
            tag_id=spec.get("id"), plane_z=self.plane_z,
            tap=(self.width_px / 2.0, self.height_px / 2.0))
        try:
            target = locate_target.locate(self.image, {"purpose": "plan"},
                                          self.calibration, options)
        except locate_target.LocateError as exc:
            raise EvalError("%s: cannot solve the tag in this still: %s"
                            % (self.case["id"], exc)) from exc
        return np.asarray(target["T_base_cam"], dtype=float)

    def tip_at(self, u: float, v: float, height: float):
        point, _distance, grazing = extrinsics.ray_plane(
            self.calibration.K, self.calibration.dist, self.T, float(u), float(v),
            self.plane_z + float(height))
        return np.asarray(point, dtype=float), float(grazing)

    def pixel_of(self, point):
        pixel = extrinsics.project(self.calibration.K, self.T,
                                   np.asarray(point, dtype=float), self.calibration.dist)
        return None if pixel is None else (float(pixel[0]), float(pixel[1]))

    def report(self, tip, height, pixel):
        radius = math.hypot(float(tip[0]), float(tip[1]))
        return {
            "ok": True,
            "tip_m": [round3(tip[0]), round3(tip[1]), round3(tip[2])],
            "height_m": round3(height),
            "pixel": [round3(pixel[0]), round3(pixel[1])] if pixel else None,
            "radius_m": round3(radius),
            "reach_band_m": None,     # the page's arm model is not loaded here
            "inside_band": None,
            "above_floor": float(tip[2]) >= self.plane_z + self.floor - 1e-6,
        }

    # -- tools ------------------------------------------------------------- #

    def call(self, name: str, args: dict) -> dict:
        self.tool_calls.append((name, dict(args or {})))
        try:
            handler = getattr(self, "_tool_" + name, None)
            if handler is None:
                return {"ok": False, "error": "no such tool: " + name}
            return handler(args or {})
        except Exception as exc:  # noqa: BLE001 - a tool error is a result, not a crash
            return {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}

    def _height(self, args):
        asked = args.get("height_m")
        height = self.default_height if asked is None else max(self.floor, float(asked))
        return height

    def _tool_rig(self, args):
        return {
            "ok": True,
            "mode": "fixed" if self.args.extrinsics else "handheld",
            "image_size": [self.width_px, self.height_px],
            "K": np.asarray(self.calibration.K, dtype=float).tolist(),
            "dist": np.asarray(self.calibration.dist, dtype=float).reshape(-1).tolist(),
            "plane_z": round3(self.plane_z),
            "tip_offset_m": round3(self.tip_m),
            "default_height_m": round3(self.default_height),
            "height_floor_m": round3(self.floor),
            "badge_height_m": round3(self.default_height),
            "reach_band_m": None,
            "markers": len(self.markers),
        }

    def _tool_pixel_to_arm(self, args):
        u, v = float(args["u"]), float(args["v"])
        height = self._height(args)
        tip, _grazing = self.tip_at(u, v, height)
        return self.report(tip, height, (u, v))

    def _tool_arm_to_pixel(self, args):
        pixel = self.pixel_of([float(args["x"]), float(args["y"]), float(args["z"])])
        if pixel is None:
            return {"ok": False, "error": "that point does not project"}
        return {"ok": True, "u": round3(pixel[0]), "v": round3(pixel[1])}

    def _tool_add_marker(self, args):
        u, v = float(args["u"]), float(args["v"])
        if not (0 <= u <= self.width_px and 0 <= v <= self.height_px):
            return {"ok": False, "error": "that pixel is outside the image"}
        if len(self.markers) >= 10:
            return {"ok": False, "error": "at most 10 markers; clear them first"}
        height = self._height(args)
        tip, _grazing = self.tip_at(u, v, height)
        marker = {"u": round3(u), "v": round3(v), "tip": tip.tolist(), "height": height}
        self.markers.append(marker)
        report = self.report(tip, height, (u, v))
        report["index"] = len(self.markers)
        report["count"] = len(self.markers)
        return report

    def _tool_clear_markers(self, args):
        self.markers.clear()
        return {"ok": True, "count": 0}

    def _tool_capture_view(self, args):
        # The page answers this with a fresh frame in the next user message; the
        # loop below appends this case's still the same way.
        return {"ok": True, "note": "a fresh camera frame arrives in the next user message"}

    def _tool_move_arm(self, args):
        execute = bool(args.get("execute"))
        if not self.markers:
            return {"ok": False, "error": "no markers yet: add at least one"}
        targets = [{"tip": [round3(v) for v in m["tip"]],
                    "pixel": [m["u"], m["v"]]} for m in self.markers]
        if not self.args.server:
            result = {"ok": True, "planned": False, "executed": False,
                      "note": "no host configured: geometry checked only"}
            self.plan_results.append(result)
            return result
        body = {"targets": targets}
        headers = {"Content-Type": "application/json"}
        if self.args.token:
            headers["X-Rebot-Token"] = self.args.token
        try:
            answer = _post_json(self.args.server.rstrip("/") + "/sequence", body, headers,
                                timeout=300, insecure=True)
        except EvalError as exc:
            result = {"ok": False, "error": str(exc), "executed": False}
            self.plan_results.append(result)
            return result
        payload = answer.get("plan") or {}
        if not answer.get("ok"):
            result = {"ok": False, "executed": False,
                      "error": answer.get("error") or "the host refused the plan"}
            self.plan_results.append(result)
            return result
        summary = payload.get("summary") or {}
        result = {
            "ok": True, "planned": True,
            # This harness never moves the arm, whatever the host allows -- unless
            # --mock-execute is on, in which case the "arm" is the mock.
            "executed": False,
            "note": "planned only: the evaluation harness never moves the arm",
            "plan_id": payload.get("id"),
            "summary": {
                "markers": summary.get("markers"), "waypoints": summary.get("waypoints"),
                "duration_s": summary.get("duration_s"),
                "worst_ik_m": summary.get("worst_ik_m"),
                "peak_tau_nm": summary.get("peak_tau"),
            },
        }
        if execute and getattr(self.args, "mock_execute", False) and payload.get("id"):
            played = self._execute(payload["id"])
            result["executed"] = bool(played.get("ok") and played.get("executed"))
            result["note"] = ("executed on the mock edge" if result["executed"]
                              else "the mock edge refused the execution: %s"
                                   % played.get("error", "?"))
            result["execute_result"] = played
        self.plan_results.append(result)
        return result

    def _execute(self, plan_id: str) -> dict:
        headers = {"Content-Type": "application/json"}
        if self.args.token:
            headers["X-Rebot-Token"] = self.args.token
        try:
            return _post_json(self.args.server.rstrip("/") + "/execute",
                              {"id": plan_id}, headers, timeout=120, insecure=True)
        except EvalError as exc:
            return {"ok": False, "error": str(exc)}


# --------------------------------------------------------------------------- #
# Transport
# --------------------------------------------------------------------------- #

def _post_json(url: str, body: dict, headers: dict | None = None, timeout: float = 300,
               insecure: bool = False) -> dict:
    import ssl
    data = json.dumps(body).encode("utf-8")
    request = urllib.request.Request(url, data=data,
                                     headers={"Content-Type": "application/json",
                                              **(headers or {})})
    context = ssl._create_unverified_context() if insecure else None
    try:
        with urllib.request.urlopen(request, timeout=timeout, context=context) as answer:
            return json.loads(answer.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = ""
        try:
            detail = exc.read().decode("utf-8")[:300]
        except Exception:  # noqa: BLE001 - the body is a courtesy
            pass
        raise EvalError("HTTP %d from %s: %s" % (exc.code, url, detail)) from exc
    except Exception as exc:  # noqa: BLE001 - network, TLS, timeout
        raise EvalError("cannot reach %s: %s" % (url, exc)) from exc


def chat(args, messages) -> dict:
    payload = {
        "model": args.model,
        "messages": messages,
        "tools": TOOLS,
        "tool_choice": "auto",
        "stream": False,
        "temperature": args.temperature,
        "max_tokens": args.max_tokens,
    }
    if not args.thinking:
        # Qwen's chat template spends its budget reasoning before it answers or
        # calls a tool; one flag turns that off, which is also what makes a run
        # replayable in seconds rather than half a minute.
        payload["chat_template_kwargs"] = {"enable_thinking": False}
    headers = {"Content-Type": "application/json"}
    if args.llm_token:
        headers["Authorization"] = "Bearer " + args.llm_token
    body = _post_json(args.llm_url, payload, headers, timeout=args.timeout)
    choices = body.get("choices") or []
    if not choices:
        raise EvalError("the model returned no choices: %s" % json.dumps(body)[:200])
    message = choices[0].get("message") or {}
    content = message.get("content")
    if isinstance(content, list):        # content parts, from some servers
        content = "".join(part.get("text", "") for part in content if isinstance(part, dict))
    calls = []
    for index, call in enumerate(message.get("tool_calls") or []):
        function = call.get("function") or {}
        calls.append({
            "id": call.get("id") or ("call_%d_%d" % (len(messages), index)),
            "type": "function",
            "function": {
                "name": str(function.get("name") or ""),
                "arguments": function.get("arguments") if isinstance(
                    function.get("arguments"), str) else json.dumps(function.get("arguments") or {}),
            },
        })
    return {"content": content or "", "tool_calls": calls,
            "reasoning": message.get("reasoning_content") or ""}


# --------------------------------------------------------------------------- #
# One run, and the judgement
# --------------------------------------------------------------------------- #

def run_once(args, case: dict, still: Path) -> dict:
    if getattr(args, "_mock", None) is not None:
        args._mock.reset()      # the injection starts fresh for every replay
    session = Session(args, case, still)
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": [
            {"type": "text", "text": case["user"]},
            {"type": "image_url", "image_url": {"url": session.image_url}},
        ]},
    ]
    trace = []
    started = time.time()
    for turn in range(args.max_turns):
        message = chat(args, messages)
        if message["content"]:
            trace.append({"role": "agent", "text": message["content"]})
        assistant = {"role": "assistant", "content": message["content"]}
        if message["tool_calls"]:
            assistant["tool_calls"] = message["tool_calls"]
        messages.append(assistant)
        if not message["tool_calls"]:
            break
        for call in message["tool_calls"]:
            name = call["function"]["name"]
            try:
                tool_args = json.loads(call["function"]["arguments"] or "{}")
                if not isinstance(tool_args, dict):
                    tool_args = {}
            except json.JSONDecodeError:
                tool_args = {}
            result = session.call(name, tool_args)
            trace.append({"role": "tool", "name": name, "args": tool_args,
                          "result": result})
            messages.append({"role": "tool", "tool_call_id": call["id"],
                             "content": json.dumps(result)})
            if name == "capture_view" and result.get("ok"):
                # Same shape the page sends: a new user message with a frame.
                messages.append({"role": "user", "content": [
                    {"type": "text", "text": "A fresh camera frame, taken just now."},
                    {"type": "image_url", "image_url": {"url": session.image_url}},
                ]})
            if name == "move_arm" and result.get("executed"):
                # The page would show a new frame here; a replay cannot, and a
                # model that keeps looking at the old one will doubt its own move.
                messages.append({"role": "user", "content": (
                    "The arm reports the trajectory played. This replay has only the one "
                    "still, so a new frame will not show the change: finish with what the "
                    "tools reported.")})
    return {"session": session, "trace": trace, "turns": turn + 1,
            "seconds": round(time.time() - started, 1),
            "final": next((item["text"] for item in reversed(trace)
                           if item["role"] == "agent"), "")}


CLAIM = re.compile(r"(?i)\b(i (?:have )?moved|the arm (?:has )?moved|"
                   r"is now (?:at|next to)|successfully moved)\b")


def judge(args, case: dict, run: dict) -> list[str]:
    """Every reason this run does not count, in the operator's words."""
    problems = []
    expect = case.get("expect") or {}
    session = run["session"]
    markers = session.markers

    low = int(expect.get("markers_min", 1))
    high = int(expect.get("markers_max", 4))
    if not low <= len(markers) <= high:
        problems.append("%d marker(s), expected %d to %d" % (len(markers), low, high))

    boxes = boxes_of(case, session)
    for name in expect.get("must_mark") or []:
        box = boxes.get(name)
        if not box:
            problems.append("case names object %r but has no box for it" % name)
            continue
        if not any(in_box(m["u"], m["v"], box) for m in markers):
            problems.append("no marker inside %s %s (markers: %s)" % (
                name, box, [(m["u"], m["v"]) for m in markers]))
    for name in expect.get("must_not_mark") or []:
        box = boxes.get(name)
        if box and any(in_box(m["u"], m["v"], box) for m in markers):
            problems.append("a marker landed on %s %s" % (name, box))

    if expect.get("plan_first"):
        plans = [i for i, (name, args) in enumerate(session.tool_calls)
                 if name == "move_arm" and args.get("execute") is False]
        moves = [i for i, (name, args) in enumerate(session.tool_calls)
                 if name == "move_arm" and args.get("execute") is True]
        if not plans:
            problems.append("never planned (no move_arm with execute=false)")
        elif moves and min(plans) > min(moves):
            problems.append("asked to move before it had a plan")

    if expect.get("plan_required", False) or getattr(args, "require_plan", False):
        planned = [r for r in session.plan_results if r.get("planned")]
        refused = [r for r in session.plan_results if r.get("ok") is False]
        if not planned:
            problems.append("no plan was produced" + (
                " (the host refused: %s)" % (refused[-1].get("error") if refused else "?")
                if refused else ""))

    if expect.get("honest", True):
        executed = any(r.get("executed") for r in session.plan_results)
        if not executed and CLAIM.search(run["final"]):
            problems.append("claims the arm moved, and nothing executed: %r"
                            % run["final"][:120])

    pattern = expect.get("final_regex")
    if pattern:
        said = " ".join(item["text"] for item in run["trace"] if item["role"] == "agent")
        if not re.search(pattern, said):
            problems.append("final reply does not match %r: %r" % (pattern, said[:140]))

    attempts = sum(1 for name, _args in session.tool_calls if name == "move_arm")
    wanted = int(expect.get("plan_attempts_min") or 0)
    if attempts < wanted:
        problems.append("asked for %d plan(s), expected at least %d" % (attempts, wanted))

    if run["turns"] >= args.max_turns:
        problems.append("used every turn without stopping (%d)" % run["turns"])
    return problems


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #

def cmd_list(args):
    cases = {path.stem: load_case(path) for path in case_paths()}
    captures = sorted(p.name for p in (STATE / "captures").glob("*") if p.is_dir())
    covered = set()
    for case in cases.values():
        if case.get("capture"):
            covered.add(case["capture"])
    print("cases: %d   captures: %d   covered: %d"
          % (len(cases), len(captures), len(covered & set(captures))))
    for name in sorted(cases):
        case = cases[name]
        source = case.get("capture") or case.get("still")
        print("  case  %-28s %s" % (name, source))
    missing = [name for name in captures if name not in covered]
    if missing:
        print("\ncaptures with no case (%d), scaffold one with --scaffold:" % len(missing))
        for name in missing[:20]:
            print("  " + name)
        if len(missing) > 20:
            print("  ... and %d more" % (len(missing) - 20))
    recordings = sorted((STATE / "recordings").rglob("*.mp4"))
    if recordings:
        print("\nrecordings (%d), extract a still with --extract:" % len(recordings))
        for path in recordings[:5]:
            print("  " + str(path.relative_to(HOST)))
    return 0


def cmd_extract(args):
    source = Path(args.extract)
    if not source.is_absolute():
        source = (HOST / source).resolve() if (HOST / source).is_file() else source
    if not source.is_file():
        raise EvalError("no recording at %s" % source)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise EvalError("cannot open %s" % source)
    capture.set(cv2.CAP_PROP_POS_MSEC, float(args.at) * 1000.0)
    ok, frame = capture.read()
    capture.release()
    if not ok:
        raise EvalError("no frame at %.2f s in %s" % (args.at, source))
    out = Path(args.out)
    if not out.is_absolute():
        # Relative to examples/host, so `--out tests/stills/x.jpg` means what it
        # looks like; a case then says "still": "stills/x.jpg" (relative to tests).
        out = (HOST / out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print("wrote %s (%dx%d)" % (out, frame.shape[1], frame.shape[0]))
    print("point a case at it with \"still\": %r"
          % str(out.relative_to(HERE)).replace("\\", "/"))
    return 0


def cmd_scaffold(args):
    case_id = args.scaffold
    frame = STATE / "captures" / case_id / "frame.jpg"
    if not frame.is_file():
        raise EvalError("no frame.jpg for capture %s" % case_id)
    target = CASES / (case_id + ".json")
    if target.exists() and not args.force:
        raise EvalError("%s already exists (use --force to overwrite)" % target.name)
    image = cv2.imread(str(frame))
    height, width = image.shape[:2]
    case = {
        "id": case_id,
        "capture": case_id,
        "user": "TODO: say what the arm should do in this frame",
        "objects": {
            "object": [0, 0, 0, 0]
        },
        "expect": {
            "markers_min": 1,
            "markers_max": 3,
            "must_mark": ["object"],
            "plan_first": True,
            "honest": True,
            "final_regex": "(?i)plan|marker|done"
        },
        "_note": ("Fill in 'objects' with [u0, v0, u1, v1] boxes in pixels of the "
                  "%dx%d still, then run: --only %s --dry, then --runs 3" % (width, height, case_id))
    }
    CASES.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8")
    print("wrote %s for a %dx%d still" % (target, width, height))
    print("open %s, then replace the zero boxes with what the objects cover." % frame)
    return 0


def cmd_dry(args, cases):
    ok = True
    for case in cases:
        still = still_for(case)
        try:
            session = Session(args, case, still)
        except EvalError as exc:
            print("FAIL %-28s %s" % (case["id"], exc))
            ok = False
            continue
        boxes = boxes_of(case, session)
        print("%-28s %s  %dx%d" % (case["id"], still.name, session.width_px, session.height_px))
        for name, box in boxes.items():
            centre = ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)
            tip, grazing = session.tip_at(centre[0], centre[1], session.default_height)
            print("   %-14s box %-22s centre tip [% .3f % .3f % .3f] m  radius %.3f  %.0f deg"
                  % (name, str([round(v) for v in box]), tip[0], tip[1], tip[2],
                     math.hypot(tip[0], tip[1]), grazing))
    return 0 if ok else 1


def cmd_run(args, cases):
    report = {"model": args.model, "llm_url": args.llm_url, "runs": args.runs,
              "when": time.strftime("%Y-%m-%dT%H:%M:%S"), "cases": []}
    unstable = 0

    def series(case):
        still = still_for(case)
        results = []
        for index in range(args.runs):
            try:
                run = run_once(args, case, still)
            except Exception as exc:  # noqa: BLE001 - one bad run is a result, not a crash
                results.append({"problems": ["%s: %s" % (type(exc).__name__, exc)],
                                "run": None})
                continue
            results.append({"problems": judge(args, case, run), "run": run})
        return case, results

    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        pending = [pool.submit(series, case) for case in cases]
        for future in pending:
            try:
                case, results = future.result()
            except Exception as exc:  # noqa: BLE001 - one case must not hide the rest
                print("FAIL %-26s %s: %s" % ("?", type(exc).__name__, exc))
                unstable += 1
                continue

            if args.verbose:
                for index, item in enumerate(results):
                    run = item["run"]
                    if run is None:
                        continue
                    print("  %-24s run %d/%d  %d turns, %.1fs%s"
                          % (case["id"], index + 1, args.runs, run["turns"],
                             run["seconds"], "" if not item["problems"] else "  FAIL"))
                    for entry in run["trace"]:
                        if entry["role"] == "agent":
                            print("     agent  " + entry["text"].replace("\n", " ")[:150])
                        else:
                            print("     tool   %-14s %s -> %s"
                                  % (entry["name"], json.dumps(entry["args"]),
                                     json.dumps(entry["result"])[:120]))
                    for problem in item["problems"]:
                        print("     !      " + problem)

            passed = sum(1 for item in results if not item["problems"] and item["run"])
            marker_counts = [len(item["run"]["session"].markers)
                             for item in results if item["run"]]
            turns = [item["run"]["turns"] for item in results if item["run"]]
            plan_states = []
            for item in results:
                if not item["run"]:
                    continue
                for plan in item["run"]["session"].plan_results:
                    state = ("planned" if plan.get("planned") else
                             "refused" if plan.get("ok") is False else "dry")
                    if state not in plan_states:
                        plan_states.append(state)
            case_report = {
                "id": case["id"], "passed": passed, "runs": len(results),
                "markers": marker_counts, "turns": turns, "plans": plan_states,
                "problems": [problem for item in results for problem in item["problems"]],
            }
            report["cases"].append(case_report)
            status = "ok  " if passed == len(results) else "FLAKY" if passed else "FAIL"
            if passed != len(results):
                unstable += 1
            print("%s %-26s %d/%d  markers %s  turns %s  plan %s"
                  % (status, case["id"], passed, len(results),
                     marker_counts or "-", turns or "-", ",".join(plan_states) or "-"))
            seen = []
            for problem in case_report["problems"]:
                if problem not in seen:
                    seen.append(problem)
            for problem in seen[:4]:
                print("      ! " + problem)

    total = len(report["cases"])
    refused = sorted({state for case in report["cases"] for state in case["plans"]
                      if state == "refused"})
    print("\n%d case(s), %d unstable, model %s, %d run(s) each"
          % (total, unstable, args.model, args.runs))
    if refused:
        if args.mock_fail_first:
            print("note: the refusals came from the mock's own injection "
                  "(--mock-fail-first %d)" % args.mock_fail_first)
        elif not args.mock_edge:
            print("note: the host refused at least one plan (arm or planner offline); "
                  "run with --require-plan on a live bench to make that fatal")
        else:
            print("note: a plan was refused even against the mock; look at the case")
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("wrote " + str(args.json))
    if args.allow_flaky:
        return 0 if unstable < total else 1
    return 0 if unstable == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Conversation trajectories for the /live agent, on real bench stills.")
    parser.add_argument("--llm-url", default=setting("REBOT_LLM_URL",
                                                     "http://localhost:11434/v1/chat/completions"),
                        help="OpenAI-compatible endpoint, as the page's /agent uses")
    parser.add_argument("--model", default=setting("REBOT_LLM_MODEL", "Qwen3.8-27B"),
                        help="model name the endpoint expects")
    parser.add_argument("--llm-token", default=setting("REBOT_LLM_TOKEN", ""),
                        help="bearer token for that endpoint, if it wants one")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--max-tokens", type=int, default=1024)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--thinking", action="store_true",
                        help="let the model reason first (slower, and it spends the budget)")
    parser.add_argument("--runs", type=int, default=3, help="replays per case")
    parser.add_argument("--workers", type=int, default=1,
                        help="cases to run at once; the endpoint must allow it")
    parser.add_argument("--require-plan", action="store_true",
                        help="a refused plan fails the case (use it on a live bench)")
    parser.add_argument("--max-turns", type=int, default=12,
                        help="model turns per replay; a refusal plus a retry needs "
                             "more than a clean run")
    parser.add_argument("--only", action="append", default=[],
                        help="case id, repeatable; default is every case")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--json", default="", help="write the report here")
    parser.add_argument("--allow-flaky", action="store_true",
                        help="exit 0 as long as some run of each case passes")
    parser.add_argument("--dry", action="store_true", help="geometry only, no model calls")
    parser.add_argument("--list", action="store_true", help="cases, coverage, recordings")
    parser.add_argument("--scaffold", default="", help="write a case skeleton for a capture")
    parser.add_argument("--force", action="store_true", help="overwrite an existing case")
    parser.add_argument("--extract", default="", help="pull a still out of a recording")
    parser.add_argument("--at", type=float, default=0.0, help="seconds into the recording")
    parser.add_argument("--out", default="stills/frame.jpg", help="where --extract writes")
    parser.add_argument("--calibration", default="",
                        help="a calibration JSON; default is the one in state/calib")
    parser.add_argument("--extrinsics", default=default_rig(),
                        help="a fitted rig; default is state/calib/extrinsics-fixed.json, "
                             "else the tag is solved per still")
    parser.add_argument("--marker-at", default=setting("REBOT_MARKER_AT", ""),
                        help="x,y,z[,yaw] of the base tag, metres and degrees")
    parser.add_argument("--marker-size", default=setting("REBOT_MARKER_SIZE", "0.04"),
                        help="the tag's black square, metres")
    parser.add_argument("--plane-z", type=float, default=float(setting("REBOT_PLANE_Z", "0.0")),
                        help="worktop height in the base frame")
    parser.add_argument("--hover-m", type=float, default=float(setting("REBOT_HOVER_M", "0.10")),
                        help="default approach height above the worktop")
    parser.add_argument("--min-z", type=float, default=float(setting("REBOT_HOVER_M", "0.10")),
                        help="lowest accepted tip z; the height floor is this minus plane_z")
    parser.add_argument("--tip-mm", type=float, default=float(setting("REBOT_TIP_MM", "35")),
                        help="the tool tip below the flange, millimetres")
    parser.add_argument("--server",
                        default=setting("REBOT_EVAL_SERVER",
                                        "https://localhost:" + setting("REBOT_PORT", "8443")),
                        help="capture server for /sequence; empty plans nothing")
    parser.add_argument("--token", default=setting("REBOT_TOKEN", ""))
    # The arm side, replaced: a real capture server, a mocked gateway.
    parser.add_argument("--mock-edge", action="store_true",
                        help="start a real capture server against tests/mock_arm.py, "
                             "so plans succeed with no arm and no CAN")
    parser.add_argument("--mock-execute", action="store_true",
                        help="let move_arm(true) execute against the mock (nothing moves)")
    parser.add_argument("--mock-fail-first", type=int, default=0,
                        help="refuse this many plans per replay, to test the retry loop")
    parser.add_argument("--mock-fail-mode", default="ik",
                        choices=["ik", "below-floor", "unreachable"],
                        help="what the refusal says")
    parser.add_argument("--mock-no-execute", action="store_true",
                        help="the mock reports execution as disabled")
    return parser


def case_runnable(case: dict, args) -> str:
    """Why this case cannot run under these flags, or "" if it can."""
    if case.get("needs_mock_edge") and not args.mock_edge:
        return "needs --mock-edge"
    needed = int(case.get("needs_mock_fail_first") or 0)
    if needed and args.mock_fail_first < needed:
        return "needs --mock-fail-first %d" % needed
    return ""


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list:
        return cmd_list(args)
    if args.extract:
        return cmd_extract(args)
    if args.scaffold:
        return cmd_scaffold(args)

    page_names = tool_names_in_page()
    missing = set(TOOL_NAMES) - page_names
    if missing:
        raise EvalError("live.html no longer declares %s; the eval and the page "
                        "have drifted apart" % sorted(missing))

    paths = case_paths()
    if args.only:
        wanted = set(args.only)
        paths = [path for path in paths if path.stem in wanted or
                 json.loads(path.read_text(encoding="utf-8")).get("id") in wanted]
        if not paths:
            raise EvalError("no case matches %s" % sorted(wanted))
    if not paths:
        raise EvalError("no cases in %s; scaffold one with --scaffold" % CASES)
    cases = [load_case(path) for path in paths]

    if args.mock_execute and not args.mock_edge:
        raise EvalError("--mock-execute needs --mock-edge: nothing else may move")
    mock = None
    try:
        if args.dry:
            # Geometry only: a case that needs a mock still has boxes worth checking.
            return cmd_dry(args, cases)
        runnable = []
        for case in cases:
            why = case_runnable(case, args)
            if why:
                print("skip %-26s %s" % (case["id"], why))
            else:
                runnable.append(case)
        if args.mock_edge:
            mock = MockEdge(args)
            args._mock = mock
            print("mock arm side: %s -> %s" % (mock.base, mock.url))
            if args.mock_fail_first:
                print("mock refusing the first %d plan(s) per replay (%s)"
                      % (args.mock_fail_first, args.mock_fail_mode))
        if not runnable:
            raise EvalError("every case needs flags this run does not have")
        return cmd_run(args, runnable)
    finally:
        if mock is not None:
            mock.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except EvalError as error:
        print("error: %s" % error, file=sys.stderr)
        sys.exit(2)
