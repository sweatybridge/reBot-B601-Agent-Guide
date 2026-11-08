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
    return {"t": float(t), "q": [float(v) for v in q],
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
        payload["hold_start_s"] = float(hold_start)
        payload["hold_end_s"] = float(hold_end)
    if extra:
        payload.update(extra)
    payload = {key: value for key, value in payload.items() if value is not None}
    validate(payload)
    return payload


def _now():
    import time
    return time.strftime("%Y-%m-%dT%H:%M:%S")


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
        if point["tau"] is not None and len(point["tau"]) != len(ids):
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
        "peak_tau": max((abs(v) for point in waypoints if point["tau"]
                         for v in point["tau"]), default=0.0),
        "max_step_rad": max((max(abs(x - y) for x, y in zip(a["q"], b["q"]))
                             for a, b in zip(waypoints, waypoints[1:])), default=0.0),
    }
