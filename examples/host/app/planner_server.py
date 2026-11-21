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
    GET  /telemetry/{id}      the persisted playback log of one execution
    POST /plan                {targets, tip_offset_m, gripper, pose?, options?}
    POST /execute             {id, kp?, kd?} -> POST /play on the edge
    POST /move                {q, seconds?, gripper?} -> joint-space move, no IK
    POST /fk                  {q: [7 floats]} -> tool tip and joint origins, read-only

The compact planner contract is tip-space only: `targets` is a list of
`{tip: [x, y, z]}` objects (optional `pixel`/`tap`/`capture` provenance),
`tip_offset_m` is the physical flange-to-tip distance for the whole request, and
`gripper` is `"none"` (one target, no gripper motion) or `"alternate"`
(close/open through a marker sequence). `hover`, `hover_m` and `tip_m` are
deprecated; `min_z` defaults to the default hover height and the capture server
sends its value with every request. The old `/plan-sequence` route answers 410.

`/execute` refuses unless REBOT_EXECUTE=1. The file it sends was produced by the
same `plan()` the CLI uses, and the edge re-checks limits and steps before any
frame is sent -- this service adds convenience, not a new way to bypass the gate.
"""

from __future__ import annotations

import json
import hmac
import logging
import math
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

import numpy as np  # noqa: E402

import plan_to_target as planner  # noqa: E402
import rebot_ik  # noqa: E402
import sweep  # noqa: E402
import traj  # noqa: E402

ROOT = HERE.parent
OUT_DIR = Path(os.environ.get("REBOT_OUT", str(ROOT / "state" / "out")))
# The edge's structured playback logs, one per execution (§5.2). The capture
# server's oracle reads them back after /execute; the 4000-char stdout tail in
# the response stays for humans.
TELEMETRY_DIR = Path(os.environ.get("REBOT_TELEMETRY", str(ROOT / "state" / "telemetry")))
# The learned sag residual maps (§7.3b), keyed by rig fingerprint. The capture
# server's /identify/sag fits them; /move applies the one the active rig names.
CALIB_DIR = Path(os.environ.get("REBOT_CALIB", str(ROOT / "state" / "calib")))
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


def new_id() -> str:
    """One id scheme for plans and moves: stamp-hex6, like the capture server."""
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]


def save_telemetry(entry_id, telemetry):
    """Persist the edge's playback log; §8.4: a full disk never fails a move.

    The block is stored exactly as the edge sent it (samples already thinned to
    the wire budget, hold windows intact); GET /telemetry/{id} reads it back.
    """
    if not isinstance(telemetry, dict):
        return None
    try:
        TELEMETRY_DIR.mkdir(parents=True, exist_ok=True)
        path = TELEMETRY_DIR / ("%s.json" % entry_id)
        path.write_text(json.dumps(telemetry, separators=(",", ":")), encoding="utf-8")
        return path
    except OSError as error:
        logging.warning("cannot write telemetry for %s: %s", entry_id, error)
        return None


def read_telemetry(entry_id):
    path = TELEMETRY_DIR / ("%s.json" % entry_id)
    if not path.is_file():
        raise GatewayError("no telemetry %s on disk" % entry_id, 404)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise GatewayError("telemetry %s cannot be read: %s" % (entry_id, error), 500) from error


def fk_one(body):
    """Forward kinematics of a measured pose: the tool tip and the joint origins.

    Read-only and offline-safe: rebot_ik is pure numpy (no Pinocchio), so the
    capture server can ask for FK of telemetry joints without an arm or a
    planner image. Motor 7 is the gripper and not part of the chain; it is
    accepted and ignored, exactly as the player treats it.
    """
    q = body.get("q")
    if not isinstance(q, (list, tuple)) or len(q) != 7:
        raise GatewayError("'q' must be seven joint angles (motor 7 is the gripper)")
    try:
        values = [float(value) for value in q]
    except (TypeError, ValueError) as error:
        raise GatewayError("'q' must be numeric") from error
    if not all(math.isfinite(value) for value in values):
        raise GatewayError("'q' must be finite")
    tool, links = rebot_ik.forward(np.asarray(values[:6], dtype=float), with_links=True)
    return {
        "ok": True,
        "tip_m": [float(v) for v in tool[:3, 3]],
        # link1..link6 origins plus gripper_end: seven points, the last is the tip.
        "joint_points": [[float(v) for v in frame[:3, 3]] for frame in links],
    }


def request_tip_offset(body):
    value = body.get("tip_offset_m")
    if value is None:
        raise GatewayError("'tip_offset_m' is required (the flange-to-tip offset)")
    try:
        offset = float(value)
    except (TypeError, ValueError) as error:
        raise GatewayError("'tip_offset_m' must be a number in metres") from error
    if not math.isfinite(offset) or offset < 0:
        raise GatewayError("'tip_offset_m' must be finite and >= 0")
    return offset


def request_options(body):
    """Parse the small, tip-only option set; hover is gone."""
    raw = body.get("options") or {}
    for name in ("hover", "hover_m", "tip_m"):
        if name in raw:
            raise GatewayError(
                "%s is deprecated in the planner API; send tip points and "
                "'tip_offset_m' at the top level" % name)
    options = planner.Options()
    for name, value in raw.items():
        if not hasattr(options, name):
            raise GatewayError("unknown option %r" % name)
        # Validate against the field's own type so a wrong JSON type is a 4xx
        # here, not a TypeError deep in planning that surfaces as a 500.
        default = getattr(options, name)
        if isinstance(default, bool):
            if not isinstance(value, bool):
                raise GatewayError("option %r must be a boolean" % name)
        elif isinstance(default, float):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise GatewayError("option %r must be a number" % name)
            value = float(value)
        elif isinstance(default, str):
            if not isinstance(value, str):
                raise GatewayError("option %r must be a string" % name)
        elif isinstance(default, dict):
            if not isinstance(value, dict):
                raise GatewayError("option %r must be an object" % name)
        elif default is None:
            # Optional numeric option (close_width_mm): a number or null.
            if value is not None and (isinstance(value, bool)
                                      or not isinstance(value, (int, float))):
                raise GatewayError("option %r must be a number or null" % name)
            value = None if value is None else float(value)
        setattr(options, name, value)
    options.hover = True
    options.hover_m = 0.0
    options.quiet = True
    return options


def compact_targets(body):
    """`targets: [{tip:[x,y,z], pixel?, tap?, capture?}]` -> internal targets."""
    items = body.get("targets")
    if not isinstance(items, list) or not items:
        raise GatewayError("'targets' must be a non-empty list of tip targets")
    if len(items) > 20:
        raise GatewayError("a sequence is limited to 20 targets")
    targets = []
    for index, item in enumerate(items, start=1):
        if not isinstance(item, dict):
            raise GatewayError("target %d is not an object" % index)
        tip = item.get("tip")
        if not isinstance(tip, list) or len(tip) != 3:
            raise GatewayError("target %d has no [x, y, z] tip" % index)
        try:
            point = [float(value) for value in tip]
        except (TypeError, ValueError) as error:
            raise GatewayError("target %d has a non-numeric tip" % index) from error
        if not all(math.isfinite(value) for value in point):
            raise GatewayError("target %d has a non-finite tip" % index)
        target = {"format": "rebot-target", "version": 1,
                  "point_frame": "tip", "point": point}
        for key in ("pixel", "tap", "capture"):
            if item.get(key) is not None:
                target[key] = item[key]
        targets.append(target)
    gripper = body.get("gripper")
    if gripper not in ("none", "alternate"):
        raise GatewayError("'gripper' must be \"none\" or \"alternate\"")
    return targets, gripper


def plan_targets(body):
    """One planning route for one or many tip points."""
    targets, gripper = compact_targets(body)
    pose = body.get("pose")
    if pose is None:
        pose = dump_pose()
    elif not isinstance(pose, dict) or "ids" not in pose:
        raise GatewayError("'pose' must be a rebot-edge dump when supplied")

    options = request_options(body)
    options.tip_m = request_tip_offset(body)
    # The same sag policy as /move (§7.3b): model compensation at the gains the
    # trajectory will be played at, plus the learned residual where a keyed map
    # exists for the active rig.
    options.kp_spec = str(body.get("kp") or KP)
    options.calib_dir = str(CALIB_DIR)

    # Every hover plan -- single tap or marker sequence -- drives the same
    # Cartesian approach, which passes close to the reach boundary mid-leg:
    # there the IK stalls a few mm short with the orientation already solved.
    # The arm's repeatability is ~23 mm, so a 2 mm acceptance band rejects poses
    # the hardware could not hold anyway. Floor the band at 10 mm for both.
    options.ik_pos_tol = max(float(options.ik_pos_tol), 0.010)
    started = time.time()
    try:
        if gripper == "none":
            if len(targets) != 1:
                raise GatewayError('gripper "none" takes exactly one target')
            payload, summary = planner.plan(pose, targets[0], options)
            label = "plan"
        else:
            payload, summary = planner.plan_sequence(pose, targets, options)
            label = "sequence"
    except planner.PlanError as error:
        raise GatewayError(str(error), 422) from error

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plan_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]
    path = OUT_DIR / ("%s.json" % plan_id)
    path.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")

    if label == "sequence":
        tap_heights = summary.get("tap_heights_m") or []
        if tap_heights and all(value is not None for value in tap_heights):
            logging.info("sequence %s: %d markers, tap heights [%s] mm, %d waypoints, "
                         "%.2f s, %.2f s to compute",
                         plan_id, summary["markers"],
                         ", ".join("%.0f" % (float(height) * 1000) for height in tap_heights),
                         summary["waypoints"], summary["duration_s"],
                         time.time() - started)
        else:
            logging.info("sequence %s: %d markers, %d waypoints, %.2f s, %.2f s to compute",
                         plan_id, summary["markers"], summary["waypoints"],
                         summary["duration_s"], time.time() - started)
    else:
        logging.info("plan %s: tip offset %.0f mm, %d waypoints, %.2f s, "
                     "%.2f s to compute",
                     plan_id, options.tip_m * 1000, summary["waypoints"],
                     summary["duration_s"], time.time() - started)

    summary["compute_s"] = round(time.time() - started, 2)
    return {"ok": True, "id": plan_id, "path": str(path), "bytes": path.stat().st_size,
            "summary": summary, "execute": EXECUTE}


def move_one(body):
    """Move the arm to a joint pose, in joint space, through the same gate.

    This exists for the automatic calibration sweep: it has to put the arm in
    eight specific poses without a human pressing a button eight times. It is
    not a general motion API -- there is no Cartesian target here, so there is
    no IK to fail, no singularity to cross, and no path that a collision check
    has not already approved against the capsule model.
    """
    if not EXECUTE:
        raise GatewayError("execution is disabled on this host (REBOT_EXECUTE=1 to allow it)", 403)
    # Moves are taped too (§5.2): the calibration sweeps are executions, and
    # their holds are the oracle's measurement points just like a plan's.
    # A caller that measures those holds afterwards needs to know the id the
    # telemetry lands under, so it may name the move itself (§7.4 coverage).
    requested_id = str(body.get("id") or "")
    if requested_id and not PLAN_ID.match(requested_id):
        raise GatewayError("'id' must look like %s, got %r" % (PLAN_ID.pattern, requested_id))
    move_id = requested_id or new_id()
    requested = body.get("targets")
    if requested is None:
        q = body.get("q")
        if not isinstance(q, (list, tuple)) or len(q) != 6:
            raise GatewayError("'q' must be six joint angles, or 'targets' a list of them")
        requested = [q]
    if not isinstance(requested, (list, tuple)) or not requested:
        raise GatewayError("'targets' must be a non-empty list")

    targets = []
    for item in requested:
        if not isinstance(item, (list, tuple)) or len(item) != 6:
            raise GatewayError("every target must be six joint angles")
        pose = np.asarray([float(v) for v in item], dtype=float)
        if not np.all(np.isfinite(pose)):
            raise GatewayError("every target must be finite")
        if not sweep.command_ok(pose):
            raise GatewayError("a target is outside the joint limits: [%s]"
                               % ", ".join("%+.3f" % v for v in pose), 422)
        if not sweep.within_limits(pose, margin=0.02):
            # The park pose sits on j2/j3's zero limit by design; commanding it is
            # fine, but a pose pressed against a limit is worth a line in the log.
            logging.info("a target is within 0.02 rad of a limit: [%s]",
                         ", ".join("%+.3f" % v for v in pose))
        targets.append(pose)

    current = dump_pose()
    if not isinstance(current.get("q"), list) or len(current["q"]) < 6:
        raise GatewayError("the edge's pose has no q", 502)
    q_from = np.asarray([float(v) for v in current["q"]], dtype=float)
    if q_from.size < 7:
        q_from = np.append(q_from, 0.0)
    # A torque-off joint can sag a few mrad past its limit at park (j2 rests just
    # below 0). Command the clamped start so every waypoint stays in the envelope;
    # the edge's align leg ramps from the measured pose to it. Only a small sag is
    # clamped -- a large excursion is a fault symptom, and validate must refuse it
    # rather than silently ramp the arm back from an abnormal pose. The task path
    # clamps its start the same way (plan_to_target).
    lower, upper = rebot_ik.LIMITS[:, 0], rebot_ik.LIMITS[:, 1]
    excursion = float(np.max(np.maximum(lower - q_from[:6], q_from[:6] - upper)))
    if 0.0 < excursion <= 0.02:
        logging.info("start pose sagged %.1f mrad past a limit: clamped to the envelope",
                     1000.0 * excursion)
        q_from[:6] = np.clip(q_from[:6], lower, upper)
    gripper = float(body["gripper"]) if "gripper" in body else float(q_from[6])

    # Startup peel, shared with plan_to_target's unpark: the parked arm rests j4's
    # hinge on the j2 casing, and a wrist-driving first leg stalls motor 4 against
    # that contact. Always prepend the elbow lift when starting from rest so the arm
    # breaks contact before any such leg -- every /move from rest peels, exactly as
    # the task planner does. A caller already off the stops gets no extra leg.
    lift = sweep.unpark_pose(q_from[:6])
    unparked = not np.allclose(lift, q_from[:6], atol=1e-9)
    if unparked:
        targets.insert(0, lift)

    # Every leg, not just the first: a chain is only as safe as its worst
    # interpolation, and the segments between poses are new ground. It is checked
    # on the *targets*, because that is where the arm will actually be: the
    # gravity compensation below only raises the commands to get it there.
    chain = [q_from[:6]] + targets
    for index, (start, end) in enumerate(zip(chain, chain[1:]), start=1):
        if not sweep.path_safe(start, end):
            raise GatewayError(
                "leg %d of the move dips within a link radius of the worktop; move the arm "
                "clear first" % index, 422)

    # A player that cannot reach the bus still exits 0, so the move "succeeds"
    # and the arm never leaves its pose. Ask the board before sending anything.
    health_status, health = edge_request("GET", "/health", timeout=5)
    if health_status == 200 and health.get("can_up") is False:
        raise GatewayError(
            "the board's CAN channel is down (%s): bring it up before moving"
            % (health.get("channel") or "can0"), 503)

    # The controller has no integral term, so a joint under load rests tau_g/kp
    # away from its command. Command the offset so the arm *holds* the poses the
    # table was validated at -- it is the geometry that matters, not the number
    # in the file, and a 6 deg set-down is 60 mm at the tool.
    kp_spec = str(body.get("kp") or KP)
    kp = sweep.kp_vector(kp_spec)
    commands = []
    applied = []
    for index, target in enumerate(targets):
        # The learned residual term (§7.3b), when a validated map exists for the
        # active rig and these gains; zeros otherwise. aim() clips the total.
        extra = sweep.active_sag_extra(target, kp_spec, CALIB_DIR)
        command = sweep.aim(target, kp, extra_sag=extra)
        clipped = np.clip(command, rebot_ik.LIMITS[:, 0], rebot_ik.LIMITS[:, 1])
        if not np.allclose(clipped, command, atol=1e-9):
            logging.warning("pose %d is at a joint limit: %.0f mrad of sag cannot be "
                            "compensated", index + 1,
                            1000.0 * float(np.max(np.abs(command - clipped))))
        commands.append(clipped)
        applied.append(round(1000.0 * float(np.max(np.abs(clipped - target))), 1))

    chain = [q_from[:6]] + commands
    wanted = [sweep.seconds_for(start, end) for start, end in zip(chain, chain[1:])]
    given = body.get("seconds")
    if unparked and isinstance(given, (list, tuple)):
        # The caller's per-leg seconds describe its original targets; the peel leg
        # the gateway prepended takes the policy duration.
        given = [wanted[0]] + list(given)
    if given is None:
        seconds = list(wanted)
    elif isinstance(given, (list, tuple)):
        if len(given) != len(wanted):
            raise GatewayError("'seconds' must be one per leg (%d, got %d)"
                               % (len(wanted), len(given)))
        seconds = [max(value, float(given[index])) for index, value in enumerate(wanted)]
    else:
        seconds = [max(value, float(given)) for value in wanted]
    # A caller may ask for a slower leg, never a quicker one: the policy is the
    # floor, so even the error-path return to park cannot snap back.
    seconds = [max(sweep.MIN_SECONDS, min(sweep.MAX_SECONDS, value)) for value in seconds]
    # The board returns the arm to `q_from` when the file ends, so a hold at each
    # pose is the only window in which that pose physically exists -- the
    # calibration sweep picks its frames out of exactly those windows.
    hold = max(0.0, min(30.0, float(body.get("hold") or 0.0)))
    limits = [[float(lo), float(hi)] for lo, hi in rebot_ik.LIMITS] + [[-0.1, 5.0]]
    points = [np.append(pose, gripper) for pose in chain]
    try:
        if len(targets) == 1:
            trajectory = traj.joint_move(q_from, points[-1], seconds[0], limits=limits,
                                         hold_s=hold,
                                         label="calibration move to [%s]"
                                               % ", ".join("%+.2f" % v for v in targets[0]))
        else:
            trajectory = traj.joint_sequence(points, seconds, hold_s=hold, limits=limits,
                                             label="%d poses, one pass" % len(targets))
    except ValueError as error:
        raise GatewayError(str(error), 422) from error

    # Persist the trajectory beside the telemetry it is about to produce, exactly
    # as /plan persists a plan. The oracle reads the hold windows out of this
    # file, so a move's holds are measurable the same way a plan's are (§5.2) --
    # and a dry run lets the caller read the REAL windows before the arm moves,
    # which is what schedules the coverage session's hold grabs (§7.4).
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / ("%s.json" % move_id)
    path.write_text(json.dumps(trajectory, separators=(",", ":")), encoding="utf-8")
    if body.get("dry_run"):
        return {"ok": True, "id": move_id, "dry_run": True, "path": str(path),
                "bytes": path.stat().st_size, "poses": len(targets),
                "seconds": [round(float(value), 3) for value in seconds], "hold": hold,
                "waypoints": len(trajectory["waypoints"]),
                "duration_s": trajectory.get("duration_s")}

    query = urllib.parse.urlencode({
        "kp": str(body.get("kp") or KP),
        "kd": str(body.get("kd") or KD),
    })
    total = sum(seconds) + hold * len(targets)
    status, payload = edge_request("POST", "/play?" + query, body=trajectory,
                                   timeout=max(600.0, 4.0 * total))
    if status != 200:
        # The edge attaches the playback log even on abort (§5.1): the abort
        # timestamp and the per-sample cmd/meas pairs are exactly the evidence
        # a fault needs. Persist it before raising, or the record dies with the
        # error and the fault becomes undiagnosable.
        save_telemetry(move_id, payload.get("telemetry"))
        logging.warning("edge aborted move %s: %s", move_id, payload.get("error"))
        raise GatewayError(payload.get("error", "the edge refused the move"), 502)

    # The arm is back at q_from now: that is the board's one-shot semantics, not
    # a failure. Whether the move *happened* is the sweep's question, and it
    # answers it from the poses recorded during the holds.
    logging.info("moved through %d pose(s), %.1f s of motion, held %.1f s each, "
                 "sag compensation %s mrad", len(targets), sum(seconds), hold, applied)
    telemetry = payload.get("telemetry")
    save_telemetry(move_id, telemetry)
    answer = {"ok": True, "id": move_id, "seconds": round(sum(seconds), 2), "hold": hold,
              "poses": len(targets), "sag_mrad": applied, "pose": dump_pose()}
    if telemetry is not None:
        answer["telemetry"] = telemetry
    return answer


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
        # Same rule as /move: an aborted playback's log is evidence, not noise.
        save_telemetry(plan_id, payload.get("telemetry"))
        logging.warning("edge aborted plan %s: %s", plan_id, payload.get("error"))
        raise GatewayError(payload.get("error", "the edge refused the trajectory"), 502)

    logging.info("edge played %s: executed=%s", plan_id, payload.get("executed"))
    # The full log as the edge sent it goes to disk for the oracle; the same
    # (already downsampled) block rides in the response next to executed/log.
    telemetry = payload.get("telemetry")
    save_telemetry(plan_id, telemetry)
    answer = {
        "ok": True,
        "id": plan_id,
        "executed": bool(payload.get("executed", True)),
        "log": str(payload.get("output", ""))[-4000:],
    }
    if telemetry is not None:
        answer["telemetry"] = telemetry
    return answer


class Handler(BaseHTTPRequestHandler):
    server_version = "rebot-arm-gateway/1"

    def log_message(self, format, *args):  # noqa: A002 - stdlib signature
        logging.info("%s %s", self.address_string(), format % args)

    def _authorised(self):
        if not TOKEN:
            return True
        # Constant-time compare: a timing side channel on the token is a real
        # risk for a server that can move the arm.
        return hmac.compare_digest(self.headers.get("X-Rebot-Token", ""), TOKEN)

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
                    "telemetry": str(TELEMETRY_DIR),
                    "plans": len(list(OUT_DIR.glob("*.json"))) if OUT_DIR.is_dir() else 0,
                })
            elif self.path == "/pose":
                self._send(200, {"ok": True, "pose": dump_pose()})
            elif self.path.startswith("/telemetry/"):
                # GET is open on the other routes, but a playback log carries
                # the whole motion record: the token is checked here explicitly.
                if not self._authorised():
                    raise GatewayError("missing or wrong X-Rebot-Token", 401)
                entry_id = self.path[len("/telemetry/"):]
                if not PLAN_ID.match(entry_id):
                    raise GatewayError("not a plan or move id", 404)
                self._send(200, {"ok": True, "id": entry_id,
                                 "telemetry": read_telemetry(entry_id)})
            else:
                self._send(404, {"ok": False, "error": "no such route"})
        except GatewayError as error:
            self._send(error.status, {"ok": False, "error": str(error)})

    def do_POST(self):  # noqa: N802 - stdlib signature
        try:
            if not self._authorised():
                raise GatewayError("missing or wrong X-Rebot-Token", 401)
            if self.path == "/plan":
                self._send(200, plan_targets(self._body()))
            elif self.path == "/plan-sequence":
                raise GatewayError(
                    "/plan-sequence is deprecated; POST /plan with targets and gripper", 410)
            elif self.path == "/execute":
                self._send(200, execute_one(self._body()))
            elif self.path == "/move":
                self._send(200, move_one(self._body()))
            elif self.path == "/fk":
                self._send(200, fk_one(self._body()))
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
