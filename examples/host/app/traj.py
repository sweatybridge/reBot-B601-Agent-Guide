#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Trajectory payload assembly, plus a local copy of the player's validation.

`plan_to_target.py` runs in the Pinocchio container; this module is pure numpy so
the same payload can be built and checked on the host, which is what
`selftest.py` does. The checks mirror `examples/edge-rust/src/main.rs` (monotonic
time, limits, the 0.5 rad step, `tau` on every waypoint or none) so a file that
passes here is not going to be refused by `rebot-edge --check` for a schema
reason. The player still re-checks everything: this is a convenience, not a
replacement for the gate.
"""

from __future__ import annotations

import math

import numpy as np

FORMAT = "rebot-joint-trajectory"
VERSION = 1
IDS = [1, 2, 3, 4, 5, 6, 7]
MAX_JOINT_STEP = 0.5      # rad, between consecutive waypoints
LIMIT_TOL = 0.002          # rad, encoders read a few mrad past a stop at rest
MIT_POSITION_LIMIT = 4.0 * math.pi
MIN_HZ, MAX_HZ = 5.0, 200.0


def waypoint(t, q, tau=None):
    # Six decimals is microseconds: enough for the player's scheduler, and it
    # keeps the files readable instead of 0.03333333333333333.
    return {"t": round(float(t), 6), "q": [float(v) for v in q],
            "tau": [float(v) for v in tau] if tau is not None else None}


def build(waypoints, control_hz, limits, steps, extra=None, hold_start=None, hold_end=None):
    """Assemble the JSON the edge player reads, with no NaNs and no None tau."""
    cleaned = []
    for point in waypoints:
        entry = {"t": float(point["t"]), "q": [float(v) for v in point["q"]]}
        if point.get("tau") is not None:
            entry["tau"] = [float(v) for v in point["tau"]]
        cleaned.append(entry)

    payload = {
        "format": FORMAT,
        "version": VERSION,
        "created": _now(),
        "control_hz": float(control_hz),
        "ids": list(IDS),
        "limits": [[float(lo), float(hi)] for lo, hi in limits] if limits else None,
        "steps": steps,
        "duration_s": float(cleaned[-1]["t"]) if cleaned else 0.0,
        "waypoints": cleaned,
    }
    if hold_start is not None and hold_end is not None:
        # Same 1e-6 as the waypoint times: these are seconds, not a checksum.
        payload["hold_start_s"] = round(float(hold_start), 6)
        payload["hold_end_s"] = round(float(hold_end), 6)
    if extra:
        payload.update(extra)
    payload = {key: value for key, value in payload.items() if value is not None}
    validate(payload)
    return payload


def _now():
    import time
    return time.strftime("%Y-%m-%dT%H:%M:%S")


def joint_move(q_from, q_to, seconds, control_hz=30.0, hold_s=0.4, limits=None, label=None):
    """A joint-space move the player can replay, with a hold at the end.

    Used by the arm gateway's `/move` so a calibration sweep does not need a
    Cartesian plan: joint interpolation cannot fail to find a solution, cannot
    pass through a singularity, and the hold at the end is what makes a capture
    land on a still arm. No `tau` -- the player accepts a file without it, and a
    slow move at the vendor gains does not need gravity feedforward.
    """
    q_from = np.asarray(q_from, dtype=float).reshape(-1)
    q_to = np.asarray(q_to, dtype=float).reshape(-1)
    if q_from.shape != q_to.shape or q_from.size != len(IDS):
        raise ValueError("a joint move needs %d angles at both ends, got %s -> %s"
                         % (len(IDS), q_from.shape, q_to.shape))
    if not np.all(np.isfinite(q_from)) or not np.all(np.isfinite(q_to)):
        raise ValueError("joint angles must be finite")
    travel = float(np.max(np.abs(q_to - q_from)))
    if travel < 1e-6:
        seconds = max(0.2, float(seconds))
    hz = float(control_hz)
    count = max(2, int(round(max(0.2, float(seconds)) * hz)))
    period = 1.0 / hz

    waypoints = [waypoint(0.0, q_from)]
    time_now = 0.0
    for index in range(1, count + 1):
        time_now += period
        fraction = 0.5 - 0.5 * math.cos(math.pi * index / count)
        waypoints.append(waypoint(time_now, q_from + (q_to - q_from) * fraction))
    for _ in range(max(1, int(round(float(hold_s) * hz)))):
        time_now += period
        waypoints.append(waypoint(time_now, q_to))

    try:
        return build(waypoints, hz, limits,
                     label or ("joint move %.2f rad over %.1f s" % (travel, seconds)))
    except ValueError as error:
        raise ValueError("%s (slow the move down: %.2f rad over %.1f s is %.2f rad per "
                         "waypoint, the limit is %.2f)"
                         % (error, travel, seconds, travel / max(1, count) * math.pi / 2,
                            MAX_JOINT_STEP)) from error


def joint_sequence(poses, seconds, hold_s=0.4, control_hz=30.0, limits=None, label=None):
    """One file that walks through `poses` in order, holding at each.

    The player returns the arm to where the move started when the file ends, so a
    single file is one round trip for a whole sweep instead of one per pose: the
    arm goes out, visits every pose with a hold, and comes back once. That is both
    quicker (no return leg between poses) and kinder (one enable/disable cycle).

    `seconds` is one duration per leg, so `len(seconds) == len(poses) - 1`; the
    caller computes them from the same speed policy as a single move.
    """
    points = [np.asarray(pose, dtype=float).reshape(-1) for pose in poses]
    if len(points) < 2:
        raise ValueError("a sequence needs at least two poses, got %d" % len(points))
    durations = [float(value) for value in seconds] if np.ndim(seconds) else \
        [float(seconds)] * (len(points) - 1)
    if len(durations) != len(points) - 1:
        raise ValueError("%d durations for %d legs" % (len(durations), len(points) - 1))
    hz = float(control_hz)
    period = 1.0 / hz
    hold = max(1, int(round(max(0.0, float(hold_s)) * hz)))

    waypoints = [waypoint(0.0, points[0])]
    now = 0.0
    for index in range(1, len(points)):
        start, end = points[index - 1], points[index]
        count = max(2, int(round(max(0.2, durations[index - 1]) * hz)))
        for step in range(1, count + 1):
            now += period
            fraction = 0.5 - 0.5 * math.cos(math.pi * step / count)
            waypoints.append(waypoint(now, start + (end - start) * fraction))
        for _ in range(hold):
            now += period
            waypoints.append(waypoint(now, end))

    return build(waypoints, hz, limits,
                 label or ("%d poses, one pass" % len(points)))


def validate(payload):
    """Raise ValueError with the first thing the player would refuse."""
    if payload.get("format") != FORMAT:
        raise ValueError("format must be %r, got %r" % (FORMAT, payload.get("format")))
    ids = [int(v) for v in payload.get("ids") or []]
    if not ids or any(not 1 <= v <= 7 for v in ids) or len(set(ids)) != len(ids):
        raise ValueError("ids must be unique motor IDs 1..7, got %r" % (ids,))

    waypoints = payload.get("waypoints") or []
    if len(waypoints) < 2:
        raise ValueError("need at least two waypoints, got %d" % len(waypoints))

    has_tau = [point.get("tau") is not None for point in waypoints]
    if any(has_tau) and not all(has_tau):
        raise ValueError("'tau' must be on every waypoint or none")

    times = []
    for index, point in enumerate(waypoints):
        if len(point.get("q") or []) != len(ids):
            raise ValueError("waypoint %d has %d values for %d motors"
                             % (index, len(point.get("q") or []), len(ids)))
        if not np.all(np.isfinite(point["q"])):
            raise ValueError("waypoint %d has a non-finite joint angle" % index)
        if point.get("tau") is not None and len(point["tau"]) != len(ids):
            raise ValueError("waypoint %d has %d tau values for %d motors"
                             % (index, len(point["tau"]), len(ids)))
        times.append(float(point["t"]))
    if not all(np.isfinite(times)):
        raise ValueError("a waypoint time is not finite")
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("waypoint times are not monotonic")

    limits = payload.get("limits")
    if limits and len(limits) == len(ids):
        for joint, (lo, hi) in enumerate(limits):
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
                raise ValueError("limits[%d] is not a valid [lower, upper] pair" % joint)
            column = [point["q"][joint] for point in waypoints]
            if min(column) < lo - LIMIT_TOL or max(column) > hi + LIMIT_TOL:
                raise ValueError("joint %d leaves its limit [%.3f, %.3f]: %.3f..%.3f"
                                 % (ids[joint], lo, hi, min(column), max(column)))

    for index, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
        step = max(abs(x - y) for x, y in zip(a["q"], b["q"]))
        if step > MAX_JOINT_STEP:
            raise ValueError("waypoints %d->%d jump %.3f rad, above the %.2f sanity limit"
                             % (index, index + 1, step, MAX_JOINT_STEP))

    for index, point in enumerate(waypoints):
        for joint, q in enumerate(point["q"]):
            if abs(q) > MIT_POSITION_LIMIT + LIMIT_TOL:
                raise ValueError("joint %d at waypoint %d is %+.3f rad, outside the MIT range"
                                 % (ids[joint], index, q))

    hz = float(payload.get("control_hz") or 20.0)
    if not math.isfinite(hz):
        raise ValueError("control_hz is not finite")

    return {
        "waypoints": len(waypoints),
        "duration_s": times[-1] - times[0],
        "peak_tau": max((abs(v) for point in waypoints if point.get("tau")
                         for v in point["tau"]), default=0.0),
        "max_step_rad": max((max(abs(x - y) for x, y in zip(a["q"], b["q"]))
                             for a, b in zip(waypoints, waypoints[1:])), default=0.0),
    }
