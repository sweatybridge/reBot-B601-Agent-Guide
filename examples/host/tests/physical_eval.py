#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Physical-arm-in-the-loop evaluation driven through the exact /live HTTP API.

Drives the running capture server using the same endpoints and tool loop as
`web/live.html`:

    GET  /health, /reach.json, /pose
    POST /agent, /sequence, /execute, /trace
    GET  /trace/{sid}/events

The actor LLM is in the loop on every trial: it sees a live annotated camera
frame (pixel grid + queued marker pins), calls pixel_to_arm / add_marker /
capture_view / move_arm, self-adjusts height when out of range, plans through
real Pinocchio, and (in Tier 1/2) moves the physical B601 arm.

Three safety tiers:
    --tier 0  (default) Live camera + real /agent + real /sequence, NEVER executes.
    --tier 1  Touch-free live execution (enforces raised --min-height-m, default 0.10 m).
    --tier 2  Full closed-loop pick-and-place (requires --allow-execute AND host
              REBOT_EXECUTE=1 AND edge --allow-execute).

Examples:
    # Tier 0: live camera + real Pinocchio planning, no arm motion
    python tests/physical_eval.py --suite tests/physical_suites/pingpong-example.json --tier 0

    # Tier 2: 3 ping-pong cycles with physical arm + automatic 6-stage funnel grading
    python tests/physical_eval.py --suite tests/physical_suites/pingpong-example.json \
        --tier 2 --allow-execute --cycles 3 --json out/physical-report.json

    # Ad-hoc prompt on live bench (Tier 0)
    python tests/physical_eval.py --prompt "Put a marker on the red plush toy and plan it."
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import ssl
import sys
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = Path(__file__).resolve().parent
HOST = HERE.parent
APP = HOST / "app"
sys.path.insert(0, str(APP))
sys.path.insert(0, str(HERE))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import agent_eval  # noqa: E402
import calib as calib_mod  # noqa: E402
import camera as camera_mod  # noqa: E402
import extrinsics  # noqa: E402
import locate_target  # noqa: E402
import rebot_ik  # noqa: E402


class PhysicalEvalError(Exception):
    """Setup, pre-flight, or transport failure."""


class AgentTurnError(Exception):
    """A failed /agent call carrying server-side retry metadata."""

    def __init__(self, message: str, retryable: bool = False, reason: str = "",
                 finish_reason: str = ""):
        super().__init__(message)
        self.retryable = bool(retryable)
        self.reason = str(reason or "")
        self.finish_reason = str(finish_reason or "")


# --------------------------------------------------------------------------- #
# HTTP client matching live.html's api() contract
# --------------------------------------------------------------------------- #

def new_session_id() -> str:
    """UTCstamp-hex6 matching live.html's newSessionId()."""
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]


def utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())


class BenchClient:
    """Thin HTTPS client talking to capture_server.py with live.html headers."""

    def __init__(self, base_url: str, token: str = "", insecure: bool = True):
        self.base = base_url.rstrip("/")
        self.token = token
        self.context = ssl._create_unverified_context() if insecure else None

    def request_json(self, method: str, path: str, body: dict | None = None,
                     session_id: str = "", timeout: float = 120.0) -> tuple[int, dict]:
        headers = {"Cache-Control": "no-store"}
        if self.token:
            headers["X-Rebot-Token"] = self.token
        if session_id:
            headers["X-Rebot-Session"] = session_id
        data = None
        if body is not None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(self.base + path, data=data, headers=headers,
                                     method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout, context=self.context) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                return resp.status, (json.loads(raw) if raw.strip() else {})
        except urllib.error.HTTPError as exc:
            raw = ""
            try:
                raw = exc.read().decode("utf-8", errors="replace")
            except Exception:  # noqa: BLE001
                pass
            try:
                payload = json.loads(raw) if raw.strip() else {}
            except json.JSONDecodeError:
                payload = {"ok": False, "error": raw[:300] or ("HTTP %d" % exc.code)}
            return exc.code, payload
        except Exception as exc:  # noqa: BLE001
            raise PhysicalEvalError("cannot reach %s%s: %s" % (self.base, path, exc)) from exc

    def health(self) -> dict:
        status, data = self.request_json("GET", "/health", timeout=15.0)
        if status != 200 or not data.get("ok"):
            raise PhysicalEvalError("/health failed (%d): %s" % (status, data.get("error") or data))
        return data

    def reach(self) -> dict:
        status, data = self.request_json("GET", "/reach.json", timeout=30.0)
        if status != 200 or not data.get("ok"):
            raise PhysicalEvalError("/reach.json failed (%d): %s"
                                    % (status, data.get("error") or data))
        return data

    def pose(self) -> dict | None:
        status, data = self.request_json("GET", "/pose", timeout=15.0)
        if status != 200:
            return None
        pose = data.get("pose") if isinstance(data.get("pose"), dict) else data
        if not isinstance(pose, dict) or "q" not in pose:
            return None
        return pose

    def trace(self, session_id: str, events: list[dict]) -> None:
        if not session_id or not events:
            return
        try:
            self.request_json("POST", "/trace",
                              {"session": session_id, "events": events},
                              session_id=session_id, timeout=15.0)
        except Exception:  # noqa: BLE001 - tracing is additive (§8.4)
            pass

    def trace_events(self, session_id: str) -> list[dict]:
        status, data = self.request_json("GET", "/trace/%s/events" % session_id,
                                         session_id=session_id, timeout=15.0)
        if status != 200:
            return []
        return list(data.get("events") or [])


# --------------------------------------------------------------------------- #
# Camera source + stream_turn geometry matching web/geometry.js + web/locate.js
# --------------------------------------------------------------------------- #

class FrameSource:
    """Provides live BGR frames from camera.py or static file(s) for smoke tests."""

    def __init__(self, camera_spec: str = "", still_paths: list[str] | None = None):
        self.camera_spec = str(camera_spec or "").strip()
        self.still_paths = [Path(p) for p in (still_paths or []) if str(p).strip()]
        self._still_index = 0
        self._eye = None
        if self.still_paths:
            for path in self.still_paths:
                if not path.is_file():
                    raise PhysicalEvalError("still not found: %s" % path)
        elif self.camera_spec:
            self._eye = camera_mod.open_source(self.camera_spec)
        else:
            raise PhysicalEvalError(
                "no frame source configured: pass --camera <rtsp/http/device:0> "
                "(or set REBOT_CAMERA) or --still <path>")

    def read(self) -> np.ndarray:
        if self.still_paths:
            idx = min(self._still_index, len(self.still_paths) - 1)
            self._still_index += 1
            frame = cv2.imread(str(self.still_paths[idx]), cv2.IMREAD_COLOR)
            if frame is None:
                raise PhysicalEvalError("cannot read still %s" % self.still_paths[idx])
            return frame
        return self._eye.read()

    def close(self) -> None:
        if self._eye is not None:
            try:
                self._eye.close()
            except Exception:  # noqa: BLE001
                pass
            self._eye = None


def map_pixel(u: float, v: float, size: tuple[int, int], turns: int) -> tuple[float, float]:
    """Clockwise quarter-turn pixel map matching web/geometry.js mapPixel."""
    w, h = float(size[0]), float(size[1])
    x, y = float(u), float(v)
    for _ in range(int(round(turns)) % 4):
        x, y = h - y, x
        w, h = h, w
    return x, y


def shape_turn(calib_size: tuple[int, int], frame_size: tuple[int, int]) -> int:
    cw, ch = float(calib_size[0]), float(calib_size[1])
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if not (cw > 0 and ch > 0 and fw > 0 and fh > 0):
        return -1
    same = abs((cw / ch) / (fw / fh) - 1.0) <= 0.01
    swapped = abs((cw / ch) / (fh / fw) - 1.0) <= 0.01 and abs(cw / ch - ch / cw) > 0.01
    if swapped:
        return 3
    return 0 if same else -1


def resolve_turn(reach_doc: dict, frame_size: tuple[int, int]) -> int:
    known = (reach_doc.get("stream_turn") or {}).get("turns")
    if known is not None:
        return int(round(float(known))) % 4
    calib_size = tuple((reach_doc.get("calibration") or {}).get("image_size") or (0, 0))
    guessed = shape_turn(calib_size, frame_size)
    return guessed if guessed >= 0 else 0


class RigGeometry:
    """Exact browser-equivalent ray/plane and projection math from /reach.json."""

    def __init__(self, reach_doc: dict, frame_size: tuple[int, int]):
        cal = reach_doc.get("calibration") or {}
        if not cal.get("K") or not reach_doc.get("T_base_cam"):
            raise PhysicalEvalError("host has no fixed rig fitted (/reach.json missing T_base_cam)")
        self.calib_size = (int(cal["image_size"][0]), int(cal["image_size"][1]))
        self.frame_size = (int(frame_size[0]), int(frame_size[1]))
        self.K_native = np.asarray(cal["K"], dtype=float)
        self.dist = np.asarray(cal.get("dist") or [0.0] * 5, dtype=float).reshape(-1)
        self.T = np.asarray(reach_doc["T_base_cam"], dtype=float)
        self.turns = resolve_turn(reach_doc, self.frame_size)
        self.plane_z = float(reach_doc.get("plane_z", 0.0))
        self.tip_m = float(reach_doc.get("tip_m", 0.035))
        self.hover_m = float(reach_doc.get("hover_m", 0.04))
        self.rig_fingerprint = (reach_doc.get("rig") or {}).get("fingerprint")

    def _scaled_K(self, width: int, height: int) -> np.ndarray:
        sx = width / float(self.calib_size[0])
        sy = height / float(self.calib_size[1])
        K = self.K_native.copy()
        K[0, 0] *= sx
        K[0, 2] *= sx
        K[1, 1] *= sy
        K[1, 2] *= sy
        return K

    def tip_at(self, u: float, v: float, height: float) -> tuple[np.ndarray, float]:
        fw, fh = self.frame_size
        if self.turns:
            ru, rv = map_pixel(u, v, (fw, fh), self.turns)
            uw, uh = (fh, fw) if (self.turns % 2) else (fw, fh)
            at_u = ru * self.calib_size[0] / float(uw)
            at_v = rv * self.calib_size[1] / float(uh)
            K = self.K_native
        else:
            at_u, at_v = float(u), float(v)
            K = self._scaled_K(fw, fh)
        point, _dist, grazing = extrinsics.ray_plane(
            K, self.dist, self.T, at_u, at_v, self.plane_z + float(height))
        return np.asarray(point, dtype=float), float(grazing)

    def pixel_of(self, point) -> tuple[float, float] | None:
        fw, fh = self.frame_size
        K = self.K_native if (self.turns % 2) else self._scaled_K(fw, fh)
        px = locate_target.project(K, self.T, np.asarray(point, dtype=float), self.dist)
        if px is None:
            return None
        pu, pv = float(px[0]), float(px[1])
        if not self.turns:
            return pu, pv
        inv = (4 - self.turns) % 4
        if self.turns % 2:
            ru, rv = map_pixel(pu, pv, self.calib_size, inv)
            return (ru * (fw / float(self.calib_size[1])),
                    rv * (fh / float(self.calib_size[0])))
        return map_pixel(pu, pv, (fw, fh), inv)


# --------------------------------------------------------------------------- #
# Witness / camera-bump drift check between trials
# --------------------------------------------------------------------------- #

def camera_drift_px(baseline_bgr: np.ndarray, current_bgr: np.ndarray) -> float:
    """Measure global frame shift (pixels) via phase correlation on border strips.

    Focuses on the outer frame margins where the bench/background sits rather
    than the center where objects move, catching camera knocks cheaply.
    """
    if baseline_bgr is None or current_bgr is None:
        return 0.0
    if baseline_bgr.shape[:2] != current_bgr.shape[:2]:
        return 999.0
    g0 = cv2.cvtColor(baseline_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g1 = cv2.cvtColor(current_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    h, w = g0.shape
    # Downscale for speed and noise robustness, then scale shift back.
    scale = min(1.0, 640.0 / max(w, h))
    if scale < 1.0:
        g0 = cv2.resize(g0, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        g1 = cv2.resize(g1, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    win = cv2.createHanningWindow((g0.shape[1], g0.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(g0, g1, win)
    if response < 0.05:
        return 0.0
    return math.hypot(dx, dy) / scale


# --------------------------------------------------------------------------- #
# LiveBenchSession: mirrors web/live.html's tool loop against /agent + /sequence + /execute
# --------------------------------------------------------------------------- #

_READ_ONLY_TOOLS = {"rig", "pixel_to_arm", "arm_to_pixel"}
_ALLOWED_ARGS = {
    "rig": (),
    "pixel_to_arm": ("u", "v", "height_m"),
    "arm_to_pixel": ("x", "y", "z"),
    "add_marker": ("u", "v", "height_m"),
    "add_via": ("u", "v", "height_m"),
    "clear_markers": (),
    "move_arm": ("execute",),
    "capture_view": (),
}


def _retry_plan_for(reason: str) -> dict:
    if reason in ("length_in_reasoning", "length_in_tool_call", "repetition_loop"):
        return {
            "disable_thinking": True,
            "max_tokens_bump": True,
            "retry_note": (
                "Your previous response was cut off before emitting a tool call or reply. "
                "Respond directly without long reasoning: either emit the next tool call now "
                "or give a concise 1-2 sentence reply."
            ),
        }
    if reason == "malformed_tool_call":
        return {
            "disable_thinking": True,
            "max_tokens_bump": False,
            "retry_note": (
                "Your previous tool call had invalid JSON arguments. Emit a single valid "
                "JSON object matching the tool schema."
            ),
        }
    if reason == "context_overflow":
        return {
            "disable_thinking": False,
            "max_tokens_bump": False,
            "compact": True,
            "retry_note": (
                "Earlier turns were compacted to fit the context window. Continue from the "
                "latest frame and messages."
            ),
        }
    return {
        "disable_thinking": True,
        "max_tokens_bump": False,
        "retry_note": (
            "Your previous turn produced no text or tool call. Either call the next tool "
            "now or reply concisely to the user."
        ),
    }


def _is_real_user_turn(message: dict) -> bool:
    if not isinstance(message, dict) or message.get("role") != "user":
        return False
    content = message.get("content")
    if isinstance(content, list):
        for part in content:
            if isinstance(part, dict) and part.get("type") == "text":
                if part.get("text") == "A fresh camera frame, taken just now.":
                    return False
                break
    return True


class LiveBenchSession:
    """Drives one physical trial through capture_server.py's exact /live API."""

    def __init__(self, client: BenchClient, frames: FrameSource, health: dict,
                 reach_doc: dict, tier: int, min_height_m: float,
                 allow_execute: bool, max_turns: int = 12,
                 request_timeout: float = 300.0):
        self.client = client
        self.frames = frames
        self.health = health
        self.reach_doc = reach_doc
        self.tier = int(tier)
        self.allow_execute = bool(allow_execute and tier >= 1)
        self.max_turns = int(max_turns)
        self.request_timeout = float(request_timeout)

        self.session_id = new_session_id()
        self.frame_before = self.frames.read()
        self.current_frame = self.frame_before
        h_px, w_px = self.frame_before.shape[:2]
        self.width_px, self.height_px = w_px, h_px
        self.geom = RigGeometry(reach_doc, (w_px, h_px))

        min_z = float(health.get("min_z", self.geom.plane_z + self.geom.hover_m))
        host_floor = max(0.0, min_z - self.geom.plane_z)
        # Tier 1 enforces a raised touch-free floor.
        self.floor = max(host_floor, float(min_height_m)) if self.tier == 1 else host_floor
        self.default_height = max(self.floor, float(health.get("hover_m", self.geom.hover_m)))
        self.badge_height = self.default_height

        agent_info = health.get("agent") or {}
        self.context_tokens = int(agent_info.get("context_tokens") or 0)
        self.max_tokens = int(agent_info.get("max_tokens") or 2048)
        self.prompt_tokens = 0
        self.context_note = ""

        self.markers: list[dict] = []
        self.tool_calls: list[tuple[str, dict]] = []
        self.tool_results: list[tuple[str, dict, dict]] = []
        self.plan_results: list[dict] = []
        self.execute_results: list[dict] = []
        self.retries: list[dict] = []
        self.history: list[dict] = []
        self.trace_log: list[dict] = []
        self._pseq = 0
        self._pending_trace: list[dict] = []

        self._emit_trace("session_start", {
            "source": "physical_eval",
            "tier": self.tier,
            "width": self.width_px,
            "height": self.height_px,
            "rig_fingerprint": self.geom.rig_fingerprint,
        })

    # -- trace forwarding -------------------------------------------------- #

    def _emit_trace(self, event_type: str, fields: dict) -> None:
        self._pseq += 1
        event = dict(fields or {})
        event["type"] = event_type
        event["pseq"] = self._pseq
        event["t"] = utc_now()
        self._pending_trace.append(event)
        if len(self._pending_trace) >= 16:
            self.flush_trace()

    def flush_trace(self) -> None:
        if not self._pending_trace:
            return
        batch = self._pending_trace
        self._pending_trace = []
        self.client.trace(self.session_id, batch)

    # -- framing & context ------------------------------------------------- #

    def _system_prompt(self, retry_note: str = "") -> str:
        prompt = agent_eval.SYSTEM_PROMPT.replace(
            "looking down at the workbench.",
            "looking down at the workbench (%dx%d pixels)." % (self.width_px, self.height_px),
            1,
        )
        if self.context_note:
            prompt += "\n\n" + self.context_note
        if retry_note:
            prompt += "\n\n[Retry note: " + retry_note + "]"
        return prompt

    def _frame_data_url(self, image: np.ndarray) -> str:
        annotated = agent_eval.annotate_frame(image, self.markers, project_fn=self.geom.pixel_of)
        ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            raise PhysicalEvalError("failed to encode camera frame as JPEG")
        return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")

    def _estimate_tokens(self) -> int:
        chars = len(self._system_prompt()) + len(json.dumps(agent_eval.TOOLS))
        images = 0
        for msg in self.history:
            content = msg.get("content")
            if isinstance(content, str):
                chars += len(content)
            elif isinstance(content, list):
                for part in content:
                    if not isinstance(part, dict):
                        continue
                    if part.get("type") == "text":
                        chars += len(str(part.get("text") or ""))
                    elif part.get("type") == "image_url":
                        images += 1
            if msg.get("tool_calls"):
                chars += len(json.dumps(msg["tool_calls"]))
        return int(math.ceil(chars / 3.5)) + images * 1200

    def _prune_frames(self, limit: int = 4) -> int:
        refs = []
        for idx, msg in enumerate(self.history):
            content = msg.get("content")
            if not isinstance(content, list):
                continue
            for at, part in enumerate(content):
                if isinstance(part, dict) and part.get("type") == "image_url":
                    refs.append((idx, at))
        dropped = 0
        while len(refs) > limit:
            idx, at = refs.pop(0)
            self.history[idx]["content"][at] = {"type": "text", "text": "(older frame dropped)"}
            dropped += 1
        return dropped

    def _compact_history(self, target_tokens: int, reason: str = "context_tokens") -> int:
        dropped_frames = self._prune_frames(1)
        dropped_msgs = 0
        while self._estimate_tokens() > target_tokens:
            next_user = next(
                (i for i in range(1, len(self.history)) if _is_real_user_turn(self.history[i])),
                -1,
            )
            if next_user <= 0:
                break
            del self.history[:next_user]
            dropped_msgs += next_user
        if dropped_msgs or dropped_frames:
            self.prompt_tokens = 0
            self._emit_trace("compaction", {
                "dropped_messages": dropped_msgs,
                "dropped_frames": dropped_frames,
                "reason": reason,
            })
        if dropped_msgs:
            self.context_note = (
                "Earlier turns were trimmed to fit the model context; "
                "continue from the latest messages and the current frame."
            )
        return dropped_msgs + dropped_frames

    def _prepare_context(self) -> None:
        if not self.context_tokens:
            return
        reserve = self.max_tokens if self.max_tokens > 0 else 2048
        budget = max(1024, self.context_tokens - reserve - 256)
        current = max(self._estimate_tokens(), self.prompt_tokens)
        if current > budget * 0.85:
            self._compact_history(int(budget * 0.6))

    # -- reach & tool execution (mirrors live.html) ------------------------ #

    def _clamp_height(self, asked: float) -> float:
        snapped = round(float(asked) / 0.005) * 0.005
        return max(self.floor, min(max(self.floor, 0.60), snapped))

    def _probe_height(self, args: dict) -> float:
        asked = args.get("height_m")
        return self.badge_height if asked is None else self._clamp_height(float(asked))

    def _commit_height(self, args: dict) -> float:
        asked = args.get("height_m")
        if asked is not None:
            self.badge_height = self._clamp_height(float(asked))
        return self.badge_height

    def _point_reach(self, tip, band):
        x, y, z = float(tip[0]), float(tip[1]), float(tip[2])
        radius = math.hypot(x, y)
        above_floor = z >= self.geom.plane_z + self.floor - 1e-6
        if band is None:
            return radius, above_floor, False, ("no_band_at_height" if above_floor else "below_floor")
        az_deg = abs(math.degrees(math.atan2(y, x)))
        in_sector = az_deg <= float(band["sector_deg"]) + 0.5
        in_radius = band["inner_m"] - 1e-6 <= radius <= band["outer_m"] + 1e-6
        inside = in_radius and in_sector
        reason = None
        if not above_floor:
            reason = "below_floor"
        elif not in_sector:
            reason = "outside_sector"
        elif radius < band["inner_m"] - 1e-6:
            reason = "too_close"
        elif radius > band["outer_m"] + 1e-6:
            reason = "too_far"
        return radius, above_floor, inside, reason

    def _find_reachable_height(self, tip, pixel) -> dict:
        max_h = 0.45
        candidates = []
        h = self.floor
        while h <= max_h + 1e-9:
            candidates.append(round(h, 3))
            h += 0.02

        x0, y0 = float(tip[0]), float(tip[1])
        r0 = math.hypot(x0, y0)
        az0 = abs(math.degrees(math.atan2(y0, x0)))
        vhits = []
        for cand in candidates:
            band = agent_eval.compute_reach_band(self.geom.plane_z, cand, self.geom.tip_m)
            if band and az0 <= float(band["sector_deg"]) + 0.5 \
                    and band["inner_m"] - 1e-6 <= r0 <= band["outer_m"] + 1e-6:
                vhits.append(cand)
        if vhits:
            min_h = vhits[0]
            fine = round(min_h - 0.01, 3)
            if fine >= self.floor - 1e-9:
                band = agent_eval.compute_reach_band(self.geom.plane_z, fine, self.geom.tip_m)
                if band and az0 <= float(band["sector_deg"]) + 0.5 \
                        and band["inner_m"] - 1e-6 <= r0 <= band["outer_m"] + 1e-6:
                    min_h = fine
            proj = self.geom.pixel_of([x0, y0, self.geom.plane_z + min_h])
            if proj and 0 <= proj[0] <= self.width_px and 0 <= proj[1] <= self.height_px:
                return {
                    "reachable_heights_m": [agent_eval.round3(min_h), agent_eval.round3(vhits[-1])],
                    "suggested_height_m": agent_eval.round3(min_h),
                    "suggested_pixel": [agent_eval.round3(proj[0]), agent_eval.round3(proj[1])],
                }

        if pixel is not None:
            rhits = []
            for cand in candidates:
                band = agent_eval.compute_reach_band(self.geom.plane_z, cand, self.geom.tip_m)
                if band is None:
                    continue
                try:
                    probe, _ = self.geom.tip_at(pixel[0], pixel[1], cand)
                except Exception:  # noqa: BLE001
                    continue
                if self._point_reach(probe, band)[2]:
                    rhits.append(cand)
            if rhits:
                min_h = rhits[0]
                fine = round(min_h - 0.01, 3)
                if fine >= self.floor - 1e-9:
                    band = agent_eval.compute_reach_band(self.geom.plane_z, fine, self.geom.tip_m)
                    if band is not None:
                        try:
                            probe, _ = self.geom.tip_at(pixel[0], pixel[1], fine)
                            if self._point_reach(probe, band)[2]:
                                min_h = fine
                        except Exception:  # noqa: BLE001
                            pass
                return {
                    "reachable_heights_m": [agent_eval.round3(min_h), agent_eval.round3(rhits[-1])],
                    "suggested_height_m": agent_eval.round3(min_h),
                    "suggested_pixel": [agent_eval.round3(pixel[0]), agent_eval.round3(pixel[1])],
                }

        return {"reachable_heights_m": None, "suggested_height_m": None, "suggested_pixel": None}

    def _tip_report(self, tip, height: float, pixel) -> dict:
        band = agent_eval.compute_reach_band(self.geom.plane_z, height, self.geom.tip_m)
        radius, above_floor, inside_band, reason = self._point_reach(tip, band)
        report = {
            "ok": True,
            "tip_m": [agent_eval.round3(tip[0]), agent_eval.round3(tip[1]),
                      agent_eval.round3(tip[2])],
            "height_m": agent_eval.round3(height),
            "pixel": [agent_eval.round3(pixel[0]), agent_eval.round3(pixel[1])] if pixel else None,
            "radius_m": agent_eval.round3(radius),
            "reach_band_m": band,
            "inside_band": inside_band,
            "above_floor": above_floor,
        }
        if reason:
            report["reason"] = reason
        if not inside_band or not above_floor:
            report.update(self._find_reachable_height(tip, pixel))
        return report

    def call_tool(self, raw_name: str, args: dict) -> dict:
        name = str(raw_name or "").strip()
        if name.startswith("functions."):
            name = name[len("functions."):]
        clean_args = dict(args or {})
        self.tool_calls.append((name, clean_args))
        self._emit_trace("tool_call", {"name": name, "args": clean_args})
        t0 = time.time()
        try:
            allowed = _ALLOWED_ARGS.get(name)
            if allowed is None:
                result = {"ok": False, "error": "no such tool: " + name}
            else:
                extra = [k for k in clean_args if k not in allowed]
                if extra:
                    result = {
                        "ok": False,
                        "error": "unknown parameter(s) [%s] for %s%s" % (
                            ", ".join(extra), name,
                            ("; expected: " + ", ".join(allowed)) if allowed else "; takes no arguments"),
                    }
                else:
                    result = getattr(self, "_tool_" + name)(clean_args)
        except Exception as exc:  # noqa: BLE001
            result = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}
        ms = int(round((time.time() - t0) * 1000))
        self._emit_trace("tool_result", {"name": name, "args": clean_args,
                                         "result": result, "ms": ms})
        self.tool_results.append((name, clean_args, result))
        return result

    def _tool_rig(self, args: dict) -> dict:
        return {
            "ok": True,
            "mode": "fixed",
            "image_size": [self.width_px, self.height_px],
            "K": self.geom.K_native.tolist(),
            "dist": self.geom.dist.tolist(),
            "plane_z": agent_eval.round3(self.geom.plane_z),
            "tip_offset_m": agent_eval.round3(self.geom.tip_m),
            "default_height_m": agent_eval.round3(self.default_height),
            "height_floor_m": agent_eval.round3(self.floor),
            "badge_height_m": agent_eval.round3(self.badge_height),
            "reach_band_m": agent_eval.compute_reach_band(
                self.geom.plane_z, self.badge_height, self.geom.tip_m),
            "markers": len(self.markers),
            "vias": sum(len(m.get("via") or []) for m in self.markers),
        }

    def _tool_pixel_to_arm(self, args: dict) -> dict:
        u, v = float(args["u"]), float(args["v"])
        height = self._probe_height(args)
        tip, _ = self.geom.tip_at(u, v, height)
        return self._tip_report(tip, height, (u, v))

    def _tool_arm_to_pixel(self, args: dict) -> dict:
        px = self.geom.pixel_of([float(args["x"]), float(args["y"]), float(args["z"])])
        if px is None:
            return {"ok": False, "error": "that point does not project"}
        return {"ok": True, "u": agent_eval.round3(px[0]), "v": agent_eval.round3(px[1])}

    def _tool_add_marker(self, args: dict) -> dict:
        u, v = float(args["u"]), float(args["v"])
        if not (0 <= u <= self.width_px and 0 <= v <= self.height_px):
            return {"ok": False, "error": "that pixel is outside the video frame"}
        if len(self.markers) >= 10:
            return {"ok": False, "error": "at most 10 markers; clear them first"}
        height = self._commit_height(args)
        tip, _ = self.geom.tip_at(u, v, height)
        report = self._tip_report(tip, height, (u, v))
        self.markers.append({
            "u": agent_eval.round3(u), "v": agent_eval.round3(v),
            "tip": tip.tolist(), "height": height,
            "inside_band": report.get("inside_band"),
        })
        report["index"] = len(self.markers)
        report["count"] = len(self.markers)
        self._emit_trace("marker_solved", {
            "u": agent_eval.round3(u), "v": agent_eval.round3(v),
            "height_m": report["height_m"], "tip_m": report["tip_m"],
            "turns": self.geom.turns,
            "rig_fingerprint": self.geom.rig_fingerprint,
            "inside_band": report["inside_band"], "above_floor": report["above_floor"],
        })
        return report

    def _tool_add_via(self, args: dict) -> dict:
        u, v = float(args["u"]), float(args["v"])
        if not (0 <= u <= self.width_px and 0 <= v <= self.height_px):
            return {"ok": False, "error": "that pixel is outside the video frame"}
        if not self.markers:
            return {"ok": False, "error": "place a marker first: vias attach to the last marker",
                    "count": 0}
        owner = self.markers[-1]
        vias = owner.setdefault("via", [])
        if len(vias) >= 5:
            return {"ok": False, "error": "5 vias per marker is the limit",
                    "count": len(self.markers)}
        height = self._commit_height(args)
        tip, _ = self.geom.tip_at(u, v, height)
        vias.append(tip.tolist())
        report = self._tip_report(tip, height, (u, v))
        report["index"] = len(self.markers)
        report["via"] = len(vias)
        report["count"] = len(self.markers)
        return report

    def _tool_clear_markers(self, args: dict) -> dict:
        self.markers.clear()
        return {"ok": True, "count": 0}

    def _tool_capture_view(self, args: dict) -> dict:
        return {"ok": True, "note": "a fresh frame arrives in the next user message"}

    def _tool_move_arm(self, args: dict) -> dict:
        raw_execute = args.get("execute")
        if not isinstance(raw_execute, bool):
            return {
                "ok": False,
                "error": "execute must be a boolean (true or false), got %r" % (raw_execute,),
                "markers": len(self.markers),
            }
        will_execute = raw_execute is True
        if not self.markers:
            return {"ok": False, "error": "no markers yet: add at least one", "markers": 0}

        # Tier 1 safety gate: enforce raised height floor on all targets.
        if self.tier == 1:
            for idx, m in enumerate(self.markers, start=1):
                if float(m["tip"][2]) < self.geom.plane_z + self.floor - 1e-6:
                    return {
                        "ok": False,
                        "error": "tier-1 touch-free floor: marker %d height %.3f m < %.3f m"
                                 % (idx, float(m["tip"][2]) - self.geom.plane_z, self.floor),
                        "markers": len(self.markers),
                    }

        targets = []
        for m in self.markers:
            entry = {
                "tip": [agent_eval.round3(v) for v in m["tip"]],
                "pixel": [m["u"], m["v"]],
            }
            if m.get("via"):
                entry["via"] = [[agent_eval.round3(v) for v in pt] for pt in m["via"]]
            targets.append(entry)

        status, answer = self.client.request_json(
            "POST", "/sequence", {"targets": targets},
            session_id=self.session_id, timeout=300.0)
        if status != 200 or not answer.get("ok"):
            err = answer.get("error") or ("sequence failed (%d)" % status)
            out = {"ok": False, "executed": False, "error": err, "markers": len(self.markers)}
            self.plan_results.append(out)
            self._emit_trace("sequence_result", {
                "plan_id": None, "targets": targets, "summary": None,
                "executed": False, "error": err,
            })
            return out

        plan = answer.get("plan") or {}
        plan_id = plan.get("id")
        summary = plan.get("summary") or {}
        out = {
            "ok": True,
            "planned": True,
            "executed": False,
            "plan_id": plan_id,
            "markers": len(self.markers),
            "summary": {
                "target_m": summary.get("target"),
                "waypoints": summary.get("waypoints", 0),
                "duration_s": float(summary.get("duration_s") or 0.0),
                "worst_ik_m": summary.get("worst_ik_m"),
                "peak_tau_nm": summary.get("peak_tau"),
            },
            "note": "planned only; nothing moved",
        }
        if not will_execute:
            self.plan_results.append(out)
            self._emit_trace("sequence_result", {
                "plan_id": plan_id, "targets": targets, "summary": summary,
                "executed": False, "error": None,
            })
            return out

        if not self.allow_execute:
            out["note"] = "execution disabled in physical_eval (tier %d); trajectory planned only" % self.tier
            self.plan_results.append(out)
            self._emit_trace("sequence_result", {
                "plan_id": plan_id, "targets": targets, "summary": summary,
                "executed": False, "error": None,
            })
            return out

        if not answer.get("execute"):
            out["note"] = "execution is off on the host (REBOT_EXECUTE=0)"
            self.plan_results.append(out)
            self._emit_trace("sequence_result", {
                "plan_id": plan_id, "targets": targets, "summary": summary,
                "executed": False, "error": None,
            })
            return out

        ex_status, ex_body = self.client.request_json(
            "POST", "/execute", {"id": plan_id},
            session_id=self.session_id, timeout=360.0)
        self.execute_results.append({"plan_id": plan_id, "status": ex_status, "body": ex_body})
        if ex_status != 200 or ex_body.get("ok") is False:
            err = ex_body.get("error") or ("execute failed (%d)" % ex_status)
            fail_out = {"ok": False, "executed": False, "plan_id": plan_id,
                        "error": err, "markers": len(self.markers)}
            self.plan_results.append(fail_out)
            self._emit_trace("sequence_result", {
                "plan_id": plan_id, "targets": targets, "summary": summary,
                "executed": False, "error": err,
            })
            return fail_out

        out["executed"] = True
        out["note"] = "arm finished: %s" % plan_id
        self.plan_results.append(out)
        self._emit_trace("sequence_result", {
            "plan_id": plan_id, "targets": targets, "summary": summary,
            "executed": True, "error": None,
        })
        return out

    # -- /agent call + auto-retry loop (mirrors live.html runAgent) -------- #

    def _post_agent(self, retry_opts: dict) -> dict:
        self._prepare_context()
        body = {
            "messages": [{"role": "system",
                          "content": self._system_prompt(retry_opts.get("retry_note", ""))}]
                        + self.history,
            "tools": agent_eval.TOOLS,
            "stream": False,
        }
        if retry_opts.get("disable_thinking"):
            body["disable_thinking"] = True
        if retry_opts.get("max_tokens_bump"):
            body["max_tokens_bump"] = True
        status, data = self.client.request_json(
            "POST", "/agent", body, session_id=self.session_id,
            timeout=self.request_timeout)
        if status != 200 or not data.get("ok"):
            raise AgentTurnError(
                str(data.get("error") or ("/agent failed (%d)" % status)),
                retryable=bool(data.get("retryable")),
                reason=str(data.get("reason") or ""),
                finish_reason=str(data.get("finish_reason") or ""),
            )
        return {
            "message": data.get("message") or {},
            "finish_reason": str(data.get("finish_reason") or ""),
            "complete": data.get("complete") is not False,
            "usage": data.get("usage") or {},
        }

    def run(self, user_prompt: str) -> dict:
        started = time.time()
        first_url = self._frame_data_url(self.frame_before)
        self.history.append({
            "role": "user",
            "content": [
                {"type": "text", "text": user_prompt},
                {"type": "image_url", "image_url": {"url": first_url}},
            ],
        })
        self._emit_trace("user_message", {
            "text": user_prompt, "width": self.width_px, "height": self.height_px,
            "frame": first_url,
        })

        total_retries = 0
        last_read_sig = ""
        fatal_error = ""
        exhausted = False
        turns_used = 0

        for step in range(self.max_turns):
            turns_used = step + 1
            turn = None
            retry_opts: dict = {}
            for attempt in range(3):
                try:
                    turn = self._post_agent(retry_opts)
                    break
                except AgentTurnError as err:
                    if err.retryable and attempt < 2 and total_retries < 3:
                        total_retries += 1
                        plan = _retry_plan_for(err.reason)
                        self.retries.append({"step": step + 1, "attempt": attempt + 1,
                                             "reason": err.reason, "error": str(err)})
                        self._emit_trace("turn_retry", {
                            "step": step + 1, "attempt": attempt + 1,
                            "reason": err.reason or "error",
                            "disable_thinking": bool(plan.get("disable_thinking")),
                        })
                        if plan.get("compact"):
                            target = max(1024, int((self.prompt_tokens or 4096) * 0.5))
                            if not self._compact_history(target, "context_overflow"):
                                fatal_error = str(err)
                                break
                        retry_opts = plan
                        continue
                    fatal_error = str(err)
                    break
            if fatal_error or turn is None:
                break

            usage = turn.get("usage") or {}
            if isinstance(usage.get("prompt_tokens"), (int, float)):
                self.prompt_tokens = int(usage["prompt_tokens"])
            msg = turn.get("message") or {}
            content_text = str(msg.get("content") or "").strip()
            raw_calls = list(msg.get("tool_calls") or [])
            if content_text:
                self.trace_log.append({"role": "agent", "text": content_text})

            assistant: dict = {"role": "assistant", "content": content_text}
            if raw_calls:
                assistant["tool_calls"] = []
                for idx, call in enumerate(raw_calls):
                    fn = call.get("function") or {}
                    name = str(fn.get("name") or "").strip()
                    if name.startswith("functions."):
                        name = name[len("functions."):]
                    args_str = fn.get("arguments")
                    if not isinstance(args_str, str):
                        args_str = json.dumps(args_str or {}, ensure_ascii=False)
                    assistant["tool_calls"].append({
                        "id": call.get("id") or ("call_%d_%d" % (step, idx)),
                        "type": "function",
                        "function": {"name": name, "arguments": args_str},
                    })
            self.history.append(assistant)

            if not raw_calls:
                self._emit_trace("task_end", {
                    "final_text": content_text, "turns": turns_used, "exhausted": False,
                })
                break

            wants_frame = False
            for call in assistant["tool_calls"]:
                tname = call["function"]["name"]
                targs = agent_eval._parse_tool_args_lenient(call["function"]["arguments"])
                sig = (tname + "|" + json.dumps(targs, sort_keys=True)) if tname in _READ_ONLY_TOOLS else ""
                is_dup = bool(sig and sig == last_read_sig)
                if tname not in _READ_ONLY_TOOLS:
                    last_read_sig = ""
                else:
                    last_read_sig = sig

                if is_dup:
                    result = {
                        "ok": False,
                        "error": "duplicate call: you already called %s with these exact arguments; "
                                 "use the previous result or change u, v, or height_m" % tname,
                    }
                    self.tool_calls.append((tname, targs))
                    self.tool_results.append((tname, targs, result))
                else:
                    result = self.call_tool(tname, targs)
                self.trace_log.append({"role": "tool", "name": tname, "args": targs,
                                       "result": result})
                self.history.append({
                    "role": "tool",
                    "tool_call_id": call["id"],
                    "content": json.dumps(result, ensure_ascii=False),
                })
                if tname == "capture_view" and result.get("ok"):
                    wants_frame = True

            if wants_frame:
                self.current_frame = self.frames.read()
                url = self._frame_data_url(self.current_frame)
                self.history.append({
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "A fresh camera frame, taken just now."},
                        {"type": "image_url", "image_url": {"url": url}},
                    ],
                })
                self._emit_trace("frame_delivered", {
                    "width": self.current_frame.shape[1],
                    "height": self.current_frame.shape[0],
                    "reason": "capture_view",
                    "frame": url,
                })
                self._prune_frames(4)
        else:
            exhausted = True
            last_text = next((e["text"] for e in reversed(self.trace_log)
                              if e["role"] == "agent"), "")
            self._emit_trace("task_end", {
                "final_text": last_text, "turns": turns_used, "exhausted": True,
            })

        self.flush_trace()
        final_text = next((e["text"] for e in reversed(self.trace_log)
                           if e["role"] == "agent"), "")
        return {
            "session_id": self.session_id,
            "turns": turns_used,
            "seconds": round(time.time() - started, 2),
            "retries": len(self.retries),
            "retry_details": self.retries,
            "fatal_error": fatal_error,
            "exhausted": exhausted,
            "final": final_text,
        }


# --------------------------------------------------------------------------- #
# 6-stage automatic physical grader
# --------------------------------------------------------------------------- #

MOTOR7_CLOSED = float(rebot_ik.GRIPPER["motor7_closed"])


def _fetch_telemetry(planner_url: str, planner_token: str, plan_id: str) -> dict | None:
    if not planner_url or not plan_id:
        return None
    url = planner_url.rstrip("/") + "/telemetry/" + plan_id
    headers = {"X-Rebot-Token": planner_token} if planner_token else {}
    req = urllib.request.Request(url, headers=headers, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception:  # noqa: BLE001
        return None


def grade_grasp_telemetry(execute_results: list[dict], planner_url: str = "",
                          planner_token: str = "") -> dict:
    """Stage 5: infer physical grasp from motor-7 telemetry (closed on air vs object)."""
    if not execute_results:
        return {"checked": False, "ok": None, "reason": "no_execution"}
    last = execute_results[-1]
    body = last.get("body") or {}
    plan_id = last.get("plan_id") or ""
    telemetry = body.get("telemetry") or _fetch_telemetry(planner_url, planner_token, plan_id)
    if not isinstance(telemetry, dict):
        return {"checked": False, "ok": None, "reason": "no_telemetry"}
    if telemetry.get("abort"):
        return {"checked": True, "ok": False, "reason": "edge_abort",
                "abort": telemetry.get("abort")}
    if telemetry.get("gripper_stall"):
        return {"checked": True, "ok": True, "reason": "gripper_stall_detected"}
    samples = telemetry.get("samples") or []
    m7_vals = []
    for sample in samples:
        q = sample.get("q_meas") or sample.get("q")
        if isinstance(q, list) and len(q) >= 7:
            m7_vals.append(float(q[6]))
    if not m7_vals:
        return {"checked": False, "ok": None, "reason": "no_motor7_samples"}
    min_m7 = min(m7_vals)
    # If the gripper closed all the way to its closed stop without stalling,
    # it closed on air (missed the object or grabbed too high).
    closed_on_air = min_m7 <= MOTOR7_CLOSED + 0.06
    return {
        "checked": True,
        "ok": not closed_on_air,
        "min_motor7_rad": round(min_m7, 3),
        "reason": "closed_on_air" if closed_on_air else "held_object_width",
    }


def _zone_mask(shape: tuple[int, int], geom: RigGeometry, zone: dict) -> np.ndarray | None:
    h, w = shape
    mask = np.zeros((h, w), dtype=np.uint8)
    if "box" in zone and len(zone["box"]) == 4:
        u0, v0, u1, v1 = [int(round(x)) for x in zone["box"]]
        cv2.rectangle(mask, (max(0, min(u0, u1)), max(0, min(v0, v1))),
                      (min(w - 1, max(u0, u1)), min(h - 1, max(v0, v1))), 255, -1)
        return mask if int(mask.sum()) > 0 else None
    if "xy_m" in zone:
        cx, cy = float(zone["xy_m"][0]), float(zone["xy_m"][1])
        rad = float(zone.get("radius_m", 0.08))
        z = float(zone.get("z_m", geom.plane_z + 0.02))
        pts = []
        for deg in range(0, 360, 15):
            a = math.radians(deg)
            px = geom.pixel_of([cx + rad * math.cos(a), cy + rad * math.sin(a), z])
            if px is not None:
                pts.append([int(round(px[0])), int(round(px[1]))])
        if len(pts) >= 3:
            cv2.fillPoly(mask, [np.asarray(pts, dtype=np.int32)], 255)
            return mask if int(mask.sum()) > 0 else None
    return None


def _hsv_count(bgr: np.ndarray, mask: np.ndarray, hsv_spec: dict) -> int:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s_lo, s_hi = hsv_spec.get("s", [60, 255])
    v_lo, v_hi = hsv_spec.get("v", [40, 255])
    h_ranges = hsv_spec.get("h_ranges")
    if not h_ranges:
        h_lo, h_hi = hsv_spec.get("h", [0, 179])
        h_ranges = [[h_lo, h_hi]]
    hit = np.zeros(mask.shape, dtype=np.uint8)
    for lo, hi in h_ranges:
        m = cv2.inRange(hsv, np.array([lo, s_lo, v_lo], dtype=np.uint8),
                        np.array([hi, s_hi, v_hi], dtype=np.uint8))
        hit = cv2.bitwise_or(hit, m)
    hit = cv2.bitwise_and(hit, mask)
    return int(np.count_nonzero(hit))


def grade_outcome_vision(frame_before: np.ndarray, frame_after: np.ndarray,
                         geom: RigGeometry, trial: dict, zones: dict) -> dict:
    """Stage 6 (deterministic): compare pre- and post-park frames in start/target zones."""
    from_name = trial.get("from_zone")
    to_name = trial.get("to_zone")
    if not from_name and not to_name:
        return {"checked": False, "ok": None, "reason": "no_zones_configured"}
    if frame_before is None or frame_after is None:
        return {"checked": False, "ok": None, "reason": "missing_frames"}

    diff = cv2.absdiff(cv2.cvtColor(frame_before, cv2.COLOR_BGR2GRAY),
                       cv2.cvtColor(frame_after, cv2.COLOR_BGR2GRAY))
    hsv_spec = trial.get("object_hsv")
    diff_thresh = float(trial.get("zone_diff_threshold", 12.0))
    details: dict = {}
    checks = []

    if from_name and from_name in zones:
        m_from = _zone_mask(diff.shape, geom, zones[from_name])
        if m_from is not None:
            from_diff = float(cv2.mean(diff, mask=m_from)[0])
            details["from_diff"] = round(from_diff, 2)
            if hsv_spec:
                before_px = _hsv_count(frame_before, m_from, hsv_spec)
                after_px = _hsv_count(frame_after, m_from, hsv_spec)
                details["from_hsv_before_px"] = before_px
                details["from_hsv_after_px"] = after_px
                checks.append(before_px > 50 and after_px < before_px * 0.5)
            else:
                checks.append(from_diff >= diff_thresh)

    if to_name and to_name in zones:
        m_to = _zone_mask(diff.shape, geom, zones[to_name])
        if m_to is not None:
            to_diff = float(cv2.mean(diff, mask=m_to)[0])
            details["to_diff"] = round(to_diff, 2)
            if hsv_spec:
                before_px = _hsv_count(frame_before, m_to, hsv_spec)
                after_px = _hsv_count(frame_after, m_to, hsv_spec)
                details["to_hsv_before_px"] = before_px
                details["to_hsv_after_px"] = after_px
                checks.append(after_px > max(100, before_px * 1.5))
            else:
                checks.append(to_diff >= diff_thresh)

    if not checks:
        return {"checked": False, "ok": None, "reason": "zones_outside_frame", **details}
    ok = all(checks)
    return {"checked": True, "ok": ok,
            "reason": "zones_verified" if ok else "object_not_moved_to_target", **details}


def grade_outcome_vlm(client: BenchClient, session_id: str,
                      frame_before: np.ndarray, frame_after: np.ndarray,
                      prompt: str) -> dict:
    """Optional Stage 6 VLM-as-judge on (before, after, prompt) via /agent."""
    def encode(img):
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii") if ok else ""

    judge_msgs = [
        {"role": "system", "content": (
            "You are a strict robotics evaluation judge. Compare BEFORE and AFTER images "
            "from a fixed camera and reply ONLY with JSON: "
            '{"moved": true/false, "at_target": true/false, "reason": "short explanation"}.'
        )},
        {"role": "user", "content": [
            {"type": "text", "text": "Task instruction: %s\nImage 1 is BEFORE. Image 2 is AFTER." % prompt},
            {"type": "image_url", "image_url": {"url": encode(frame_before)}},
            {"type": "image_url", "image_url": {"url": encode(frame_after)}},
        ]},
    ]
    status, data = client.request_json(
        "POST", "/agent",
        {"messages": judge_msgs, "tools": [], "stream": False, "disable_thinking": True},
        session_id=session_id, timeout=120.0)
    if status != 200 or not data.get("ok"):
        return {"checked": False, "ok": None, "reason": "vlm_judge_failed"}
    text = str((data.get("message") or {}).get("content") or "").strip()
    m = re.search(r"\{.*\}", text, re.DOTALL)
    if not m:
        return {"checked": False, "ok": None, "reason": "vlm_judge_non_json", "raw": text[:160]}
    try:
        verdict = json.loads(m.group(0))
    except json.JSONDecodeError:
        return {"checked": False, "ok": None, "reason": "vlm_judge_bad_json", "raw": text[:160]}
    ok = bool(verdict.get("moved")) and bool(verdict.get("at_target"))
    return {"checked": True, "ok": ok, "moved": bool(verdict.get("moved")),
            "at_target": bool(verdict.get("at_target")),
            "reason": str(verdict.get("reason") or "")}


def poll_oracle_verdict(client: BenchClient, session_id: str,
                        wait_s: float = 8.0) -> dict | None:
    """Wait briefly for the server's post-execute oracle event in /trace/{sid}/events."""
    deadline = time.time() + max(0.0, float(wait_s))
    while True:
        events = client.trace_events(session_id)
        for ev in reversed(events):
            if ev.get("type") == "oracle":
                return ev
        if time.time() >= deadline:
            return None
        time.sleep(0.5)


def grade_trial(session: LiveBenchSession, run_info: dict, trial: dict,
                zones: dict, frame_after: np.ndarray | None,
                oracle_event: dict | None, planner_url: str = "",
                planner_token: str = "", use_vlm_judge: bool = False) -> dict:
    """Evaluate one physical trial across the 6-stage funnel."""
    expect = trial.get("expect") or {}
    stages: dict = {}

    # Stage 1: Protocol & harness
    proto_probs = []
    if run_info.get("fatal_error"):
        proto_probs.append("fatal error: %s" % run_info["fatal_error"])
    if run_info.get("exhausted"):
        proto_probs.append("exhausted max_turns (%d)" % run_info["turns"])
    plans = [i for i, (n, a) in enumerate(session.tool_calls)
             if n == "move_arm" and a.get("execute") is False]
    moves = [i for i, (n, a) in enumerate(session.tool_calls)
             if n == "move_arm" and a.get("execute") is True]
    if expect.get("plan_first", True):
        if not plans:
            proto_probs.append("never planned with move_arm(execute=false)")
        elif moves and min(plans) > min(moves):
            proto_probs.append("called move_arm(true) before move_arm(false)")
    executed_any = any(r.get("executed") for r in session.plan_results)
    if expect.get("honest", True) and not executed_any \
            and agent_eval.CLAIM.search(run_info.get("final") or ""):
        proto_probs.append("claimed the arm moved when executed was false")
    stages["protocol"] = {"ok": not proto_probs, "problems": proto_probs}

    # Stage 2: Targeting & reach
    tgt_probs = []
    markers = session.markers
    low = int(expect.get("markers_min", 1))
    high = int(expect.get("markers_max", 4))
    if not low <= len(markers) <= high:
        tgt_probs.append("%d marker(s), expected %d..%d" % (len(markers), low, high))
    if expect.get("must_be_inside_band", True):
        oob = [i + 1 for i, m in enumerate(markers) if m.get("inside_band") is False]
        if oob:
            tgt_probs.append("marker(s) %s outside reach band" % oob)
    if expect.get("max_height_m") is not None:
        cap = float(expect["max_height_m"])
        high_m = [(i + 1, agent_eval.round3(m["height"])) for i, m in enumerate(markers)
                  if float(m.get("height", 0.0)) > cap + 1e-6]
        if high_m:
            tgt_probs.append("marker height(s) %s > max_height_m %.3f" % (high_m, cap))
    if expect.get("height_adjust_required"):
        saw_out = any(n == "pixel_to_arm" and r.get("inside_band") is False
                      for n, _a, r in session.tool_results)
        raised = any(m.get("inside_band") is True
                     and float(m.get("height", 0.0)) > session.default_height + 1e-4
                     for m in markers)
        if not saw_out:
            tgt_probs.append("expected an out-of-band pixel_to_arm probe")
        elif not raised:
            tgt_probs.append("did not autonomously raise height_m into range")
    stages["targeting"] = {"ok": not tgt_probs, "problems": tgt_probs,
                           "markers": len(markers)}

    # Stage 3: Pinocchio plan
    planned = [r for r in session.plan_results if r.get("planned")]
    plan_probs = []
    if not planned:
        refused = [r.get("error") for r in session.plan_results if r.get("ok") is False]
        plan_probs.append("no valid plan produced" + (": %s" % refused[-1] if refused else ""))
    else:
        worst_ik = (planned[-1].get("summary") or {}).get("worst_ik_m")
        if worst_ik is not None and float(worst_ik) > 0.015:
            plan_probs.append("worst_ik_m %.4f m > 0.015 m" % float(worst_ik))
    stages["plan"] = {"ok": not plan_probs, "problems": plan_probs,
                      "plan_id": planned[-1].get("plan_id") if planned else None}

    # Stage 4: Physical oracle (Tier 1/2)
    infra_excluded = False
    if session.tier == 0 or not executed_any:
        stages["oracle"] = {"checked": False, "ok": None, "reason": "not_executed"}
    else:
        ex_errors = [e["body"].get("error") for e in session.execute_results
                     if e["status"] != 200 or (e["body"] or {}).get("ok") is False]
        if ex_errors:
            stages["oracle"] = {"checked": True, "ok": False, "reason": "execute_failed",
                                "error": ex_errors[-1]}
            infra_excluded = True
        elif oracle_event:
            verdict = oracle_event.get("verdict")
            layer = (verdict.get("layer") if isinstance(verdict, dict) else None) or ""
            ok = (verdict == "ok") if isinstance(verdict, str) else bool(
                isinstance(verdict, dict) and verdict.get("ok", layer in ("", "none", "ok")))
            if layer in ("camera-arm", "arm-motor", "execution"):
                infra_excluded = True
            stages["oracle"] = {"checked": True, "ok": ok, "verdict": verdict, "layer": layer}
        else:
            stages["oracle"] = {"checked": False, "ok": None, "reason": "no_oracle_event"}

    # Stage 5: Motor-7 grasp check (Tier 2)
    if session.tier >= 2 and executed_any:
        stages["grasp"] = grade_grasp_telemetry(
            session.execute_results, planner_url=planner_url, planner_token=planner_token)
    else:
        stages["grasp"] = {"checked": False, "ok": None, "reason": "not_applicable"}

    # Stage 6: Before/after outcome check (Tier 2)
    if session.tier >= 2 and executed_any and frame_after is not None:
        outcome = grade_outcome_vision(
            session.frame_before, frame_after, session.geom, trial, zones)
        if use_vlm_judge and not outcome.get("checked"):
            outcome = grade_outcome_vlm(
                session.client, session.session_id,
                session.frame_before, frame_after, trial["prompt"])
        stages["outcome"] = outcome
    else:
        stages["outcome"] = {"checked": False, "ok": None, "reason": "not_applicable"}

    # Overall verdict: every checked stage must pass.
    checked_results = [s["ok"] for s in stages.values() if s.get("ok") is not None]
    passed = bool(checked_results) and all(checked_results)
    return {
        "passed": passed,
        "infra_excluded": infra_excluded,
        "executed": executed_any,
        "stages": stages,
    }


# --------------------------------------------------------------------------- #
# Suite expansion & CLI runner
# --------------------------------------------------------------------------- #

def load_trials(args) -> tuple[dict, list[dict]]:
    if args.prompt:
        return {}, [{
            "id": "adhoc-1",
            "prompt": args.prompt,
            "expect": {"plan_first": True, "honest": True, "must_be_inside_band": True},
        }]
    if not args.suite:
        raise PhysicalEvalError("pass --suite <path.json> or --prompt <text>")
    path = Path(args.suite)
    if not path.is_file():
        raise PhysicalEvalError("suite file not found: %s" % path)
    suite = json.loads(path.read_text(encoding="utf-8"))
    zones = suite.get("zones") or {}
    base_seq = list(suite.get("trials") or suite.get("sequence") or [])
    if not base_seq:
        raise PhysicalEvalError("suite %s has no 'trials' or 'sequence'" % path.name)
    trials = []
    for cycle in range(1, max(1, int(args.cycles)) + 1):
        for idx, item in enumerate(base_seq, start=1):
            t = dict(item)
            base_id = t.get("id") or ("step-%d" % idx)
            t["id"] = "%s-c%d" % (base_id, cycle) if int(args.cycles) > 1 else base_id
            if not t.get("prompt"):
                raise PhysicalEvalError("trial %s has no 'prompt'" % t["id"])
            trials.append(t)
    return zones, trials


def summarize_report(results: list[dict], tier: int, model: str) -> dict:
    stage_names = ("protocol", "targeting", "plan", "oracle", "grasp", "outcome")
    funnel = {}
    for name in stage_names:
        checked = [r for r in results if r["grade"]["stages"][name].get("ok") is not None]
        passed = [r for r in checked if r["grade"]["stages"][name]["ok"] is True]
        funnel[name] = {"passed": len(passed), "checked": len(checked)}

    llm_trials = [r for r in results if not r["grade"]["infra_excluded"]]
    llm_passed = [r for r in llm_trials if r["grade"]["passed"]]
    infra_excluded = len(results) - len(llm_trials)
    return {
        "when": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "tier": tier,
        "model": model,
        "total_trials": len(results),
        "llm_trials": len(llm_trials),
        "llm_passed": len(llm_passed),
        "llm_pass_rate": round(len(llm_passed) / len(llm_trials), 4) if llm_trials else 0.0,
        "infra_excluded": infra_excluded,
        "total_retries": sum(r["run"]["retries"] for r in results),
        "avg_turns": round(sum(r["run"]["turns"] for r in results) / len(results), 2) if results else 0.0,
        "avg_seconds": round(sum(r["run"]["seconds"] for r in results) / len(results), 2) if results else 0.0,
        "funnel": funnel,
        "trials": results,
    }


def run_suite(args) -> int:
    if args.tier >= 2 and not args.allow_execute:
        raise PhysicalEvalError(
            "Tier 2 executes full pick-and-place on physical hardware; "
            "pass --allow-execute explicitly to confirm")
    if args.tier == 1 and not args.allow_execute:
        raise PhysicalEvalError(
            "Tier 1 executes touch-free motion on physical hardware; "
            "pass --allow-execute explicitly to confirm")

    zones, trials = load_trials(args)
    client = BenchClient(args.server, token=args.token, insecure=True)
    health = client.health()
    if not (health.get("agent") or {}).get("configured"):
        raise PhysicalEvalError("host has no LLM configured (set REBOT_LLM_URL on capture_server)")
    if not health.get("rig"):
        raise PhysicalEvalError("host has no fixed rig fitted (/health rig is null)")
    if args.tier >= 1 and not health.get("execute"):
        raise PhysicalEvalError("host has REBOT_EXECUTE=0; cannot run Tier %d" % args.tier)

    reach_doc = client.reach()
    still_list = [s.strip() for s in args.still.split(",") if s.strip()] if args.still else None
    frames = FrameSource(camera_spec=args.camera, still_paths=still_list)

    out_dir = Path(args.out_dir) if args.out_dir else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    baseline_frame = None
    consecutive_phys_fails = 0
    model_name = (health.get("agent") or {}).get("model") or "unknown"

    print("physical_eval: server=%s  model=%s  tier=%d  trials=%d"
          % (args.server, model_name, args.tier, len(trials)))
    try:
        for index, trial in enumerate(trials, start=1):
            # Pre-flight health & pose check before every trial.
            h_now = client.health()
            if not h_now.get("ok") or not h_now.get("rig"):
                raise PhysicalEvalError("pre-flight failed before trial %s: host not ready"
                                        % trial["id"])
            pose_before = None
            if not args.skip_pose_check:
                pose_before = client.pose()
                if pose_before is None and args.tier >= 1:
                    raise PhysicalEvalError("pre-flight failed: /pose unavailable before %s"
                                            % trial["id"])

            session = LiveBenchSession(
                client=client, frames=frames, health=h_now, reach_doc=reach_doc,
                tier=args.tier, min_height_m=args.min_height_m,
                allow_execute=args.allow_execute, max_turns=args.max_turns,
                request_timeout=args.timeout,
            )

            # Camera-knock interlock across trials.
            if baseline_frame is None:
                baseline_frame = session.frame_before
                drift = 0.0
            else:
                drift = camera_drift_px(baseline_frame, session.frame_before)
                if drift > float(args.max_drift_px):
                    raise PhysicalEvalError(
                        "interlock: camera drifted %.2f px (> %.2f px limit) before trial %s; "
                        "stopping batch" % (drift, args.max_drift_px, trial["id"]))

            run_info = session.run(trial["prompt"])

            frame_after = None
            oracle_event = None
            executed_any = any(r.get("executed") for r in session.plan_results)
            if executed_any:
                if args.settle_s > 0:
                    time.sleep(float(args.settle_s))
                frame_after = frames.read()
                oracle_event = poll_oracle_verdict(client, session.session_id,
                                                   wait_s=args.oracle_wait_s)

            grade = grade_trial(
                session=session, run_info=run_info, trial=trial, zones=zones,
                frame_after=frame_after, oracle_event=oracle_event,
                planner_url=args.planner, planner_token=args.planner_token,
                use_vlm_judge=args.vlm_judge,
            )

            if out_dir is not None:
                cv2.imwrite(str(out_dir / ("%02d-%s-before.jpg" % (index, trial["id"]))),
                            session.frame_before)
                if frame_after is not None:
                    cv2.imwrite(str(out_dir / ("%02d-%s-after.jpg" % (index, trial["id"]))),
                                frame_after)

            record = {
                "index": index,
                "id": trial["id"],
                "prompt": trial["prompt"],
                "session_id": session.session_id,
                "camera_drift_px": round(drift, 2),
                "pose_before": (pose_before.get("q") if isinstance(pose_before, dict) else None),
                "run": run_info,
                "grade": grade,
            }
            results.append(record)

            tag = "EXCL" if grade["infra_excluded"] else ("PASS" if grade["passed"] else "FAIL")
            st = grade["stages"]
            short_labels = (
                ("protocol", "proto"), ("targeting", "tgt"), ("plan", "plan"),
                ("oracle", "orc"), ("grasp", "grp"), ("outcome", "out"),
            )
            funnel_str = " ".join(
                "%s=%s" % (lbl, ("ok" if st[k]["ok"] is True else
                                 "FAIL" if st[k]["ok"] is False else "-"))
                for k, lbl in short_labels
            )
            print("[%2d/%2d] %-4s %-22s %s  turns=%d retries=%d %.1fs  sid=%s"
                  % (index, len(trials), tag, trial["id"], funnel_str,
                     run_info["turns"], run_info["retries"], run_info["seconds"],
                     session.session_id))
            if args.verbose:
                for k, v in st.items():
                    if v.get("ok") is False:
                        print("         ! %s: %s" % (k, v.get("problems") or v.get("reason") or v))

            # Safety interlocks on physical execution failures.
            if grade["stages"]["oracle"].get("reason") == "execute_failed" \
                    or grade["stages"]["grasp"].get("reason") == "edge_abort":
                print("INTERLOCK: edge execution error/abort; halting remaining trials")
                break
            if args.tier >= 1 and not grade["passed"]:
                consecutive_phys_fails += 1
                if consecutive_phys_fails >= int(args.max_consecutive_fails):
                    print("INTERLOCK: %d consecutive failures; halting remaining trials"
                          % consecutive_phys_fails)
                    break
            else:
                consecutive_phys_fails = 0
    finally:
        frames.close()

    report = summarize_report(results, tier=args.tier, model=model_name)
    print("\n--- Physical Eval Funnel ---")
    for name, counts in report["funnel"].items():
        if counts["checked"]:
            print("  %-10s %d / %d" % (name, counts["passed"], counts["checked"]))
    print("LLM pass rate: %d/%d (%.1f%%)  infra_excluded=%d  retries=%d  avg=%.1fs"
          % (report["llm_passed"], report["llm_trials"], report["llm_pass_rate"] * 100.0,
             report["infra_excluded"], report["total_retries"], report["avg_seconds"]))

    if args.json:
        out_path = Path(args.json)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("wrote %s" % out_path)

    return 0 if report["llm_trials"] > 0 and report["llm_passed"] == report["llm_trials"] else 1


def build_parser() -> argparse.ArgumentParser:
    env = agent_eval.ENV
    def env_get(key: str, default: str = "") -> str:
        val = os.environ.get(key)
        return val if (val is not None and val != "") else env.get(key, default)

    parser = argparse.ArgumentParser(
        description="Physical-arm-in-the-loop LLM eval driven via /live HTTP APIs.")
    parser.add_argument("--server",
                        default=env_get("REBOT_EVAL_SERVER",
                                        "https://localhost:" + env_get("REBOT_PORT", "8443")),
                        help="capture_server base URL")
    parser.add_argument("--token", default=env_get("REBOT_TOKEN", ""),
                        help="X-Rebot-Token for capture_server")
    parser.add_argument("--planner", default="",
                        help="optional planner_server URL to fetch /telemetry/{plan_id}")
    parser.add_argument("--planner-token", default=env_get("REBOT_PLANNER_TOKEN", ""))
    parser.add_argument("--camera", default=env_get("REBOT_CAMERA", ""),
                        help="live camera source (rtsp://..., http://.../snapshot.jpg, device:0)")
    parser.add_argument("--still", default="",
                        help="comma-separated still image path(s) instead of live camera (for smoke tests)")
    parser.add_argument("--suite", default="", help="path to physical eval suite JSON")
    parser.add_argument("--prompt", default="", help="run a single ad-hoc prompt without a suite file")
    parser.add_argument("--cycles", type=int, default=1,
                        help="number of full cycles through a ping-pong suite")
    parser.add_argument("--tier", type=int, choices=(0, 1, 2), default=0,
                        help="0=plan-only, 1=touch-free hover execute, 2=full pick-and-place")
    parser.add_argument("--allow-execute", action="store_true",
                        help="required explicit safety confirmation for Tier 1 and Tier 2")
    parser.add_argument("--min-height-m", type=float, default=0.10,
                        help="raised touch-free floor for Tier 1 (metres above worktop)")
    parser.add_argument("--max-turns", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--settle-s", type=float, default=1.0,
                        help="seconds to wait after /execute before grabbing post-move frame")
    parser.add_argument("--oracle-wait-s", type=float, default=6.0,
                        help="seconds to poll /trace/{sid}/events for the server's oracle event")
    parser.add_argument("--max-drift-px", type=float, default=3.0,
                        help="interlock: abort batch if camera shifts more than this many pixels")
    parser.add_argument("--max-consecutive-fails", type=int, default=2,
                        help="interlock: halt Tier 1/2 batch after N consecutive trial failures")
    parser.add_argument("--skip-pose-check", action="store_true",
                        help="skip /pose pre-flight check (useful for Tier 0 when edge is offline)")
    parser.add_argument("--vlm-judge", action="store_true",
                        help="fall back to VLM-as-judge on (before, after) frames when no zones are set")
    parser.add_argument("--out-dir", default="",
                        help="optional directory to save per-trial before/after JPEGs")
    parser.add_argument("--json", default="", help="write full JSON report to this path")
    parser.add_argument("--verbose", "-v", action="store_true")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return run_suite(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except PhysicalEvalError as exc:
        print("error: %s" % exc, file=sys.stderr)
        sys.exit(2)
