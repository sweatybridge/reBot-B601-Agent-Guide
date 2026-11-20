#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Trajectory payload assembly, plus a local copy of the player's validation.

`plan_to_target.py` runs in the Pinocchio container; this module is pure numpy so
the same payload can be built and checked on the host, which is what
`selftest.py` does. The checks mirror `examples/edge-rust/src/main.rs` (monotonic
time, limits, the 0.5 rad step, `tau` on every waypoint or none, `v` likewise)
so a file that passes here is not going to be refused by `rebot-edge --check`
for a schema reason. The player still re-checks everything: this is a
convenience, not a replacement for the gate.

`build` also derives the per-waypoint velocity feedforward `v` from the q/t
series (central difference, one-sided at the ends). The MIT frame's kd term
damps toward the commanded velocity: with v=0 it brakes the motion itself and
the arm pulses at the waypoint rate. Deriving it here means every producer --
joint moves, sequences, Cartesian plans -- carries it, with no per-caller opt-in.
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

# Velocity ramp at each end of a leg, seconds. The profile is trapezoid: short
# linear velocity ramps into a constant-velocity cruise. The cosine ease this
# replaces has sine velocity tails that scale with leg duration -- on a 25 s leg
# the P-only drive spent ~2.5 s at each end below its stiction breakaway speed
# (~0.2 rad/s at kp=50) and moved in stick-slip segments. The trapezoid crosses
# that band in ramp * v_breakaway / v_cruise regardless of leg length; 0.15 s
# keeps the ramp-end stall under ~0.2 s while ramp acceleration stays modest.
RAMP_S = 0.15


def _ramp_ratio(duration, ramp):
    return min(0.5, max(1e-9, float(ramp) / max(float(duration), 1e-9)))


def ease_fraction(u, duration, ramp=RAMP_S):
    """Fraction of a leg's travel at normalized time u, trapezoid velocity.

    Legs shorter than 2*ramp degenerate to a triangle (no cruise); both shapes
    start and end at rest. `duration` is the leg's scheduled time in seconds.
    """
    r = _ramp_ratio(duration, ramp)
    vc = 1.0 / (1.0 - r)
    u = max(0.0, min(1.0, float(u)))
    if u <= r:
        return vc * u * u / (2.0 * r)
    if u >= 1.0 - r:
        return 1.0 - vc * (1.0 - u) ** 2 / (2.0 * r)
    return vc * (u - 0.5 * r)


def ease_velocity(u, duration, ramp=RAMP_S):
    """d(ease_fraction)/dt in 1/s; multiply by the joint travel for rad/s."""
    r = _ramp_ratio(duration, ramp)
    vc = 1.0 / (1.0 - r)
    u = max(0.0, min(1.0, float(u)))
    duration = max(float(duration), 1e-9)
    if u <= r:
        return vc * u / (r * duration)
    if u >= 1.0 - r:
        return vc * (1.0 - u) / (r * duration)
    return vc / duration


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

    # Velocity feedforward, derived once for every producer: central difference
    # inside the series, one-sided at the ends. Shape-agnostic -- trapezoid legs
    # and IK legs alike; the difference is exact to O(period^2) on the smooth
    # parts and smears the ramp/cruise acceleration corner over at most one tick.
    # Holds come out exactly zero. A repeated timestamp (degenerate, still
    # schema-legal) gets zero rather than a division by zero.
    if len(cleaned) >= 2:
        for index, entry in enumerate(cleaned):
            if index == 0:
                ahead, behind = cleaned[1], entry
            elif index == len(cleaned) - 1:
                ahead, behind = entry, cleaned[-2]
            else:
                ahead, behind = cleaned[index + 1], cleaned[index - 1]
            dt = ahead["t"] - behind["t"]
            # A sample identical to either neighbour is at rest at that instant
            # (the end of a leg, every hold): the straddling difference
            # would otherwise smear half a tick of motion into the hold.
            q = np.asarray(entry["q"])
            at_rest = any(
                float(np.max(np.abs(q - np.asarray(other["q"])))) <= 1e-9
                for other in (cleaned[index - 1] if index > 0 else None,
                              cleaned[index + 1] if index + 1 < len(cleaned) else None)
                if other is not None)
            if dt <= 1e-9 or at_rest:
                entry["v"] = [0.0] * len(entry["q"])
            else:
                entry["v"] = [round((a - b) / dt, 6)
                              for a, b in zip(ahead["q"], behind["q"])]

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
    slow move at the vendor gains does not need gravity feedforward. `build`
    still derives the velocity feedforward `v`, so the kd term does not brake
    the move itself.
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
        fraction = ease_fraction(index / count, count / hz)
        waypoints.append(waypoint(time_now, q_from + (q_to - q_from) * fraction))
    for _ in range(max(1, int(round(float(hold_s) * hz)))):
        time_now += period
        waypoints.append(waypoint(time_now, q_to))

    try:
        return build(waypoints, hz, limits,
                     label or ("joint move %.2f rad over %.1f s" % (travel, seconds)))
    except ValueError as error:
        raise ValueError("%s (slow the move down: %.2f rad over %.1f s is %.2f rad per "
                         "waypoint at cruise, the limit is %.2f)"
                         % (error, travel, seconds,
                            travel / max(1.0, count - RAMP_S * hz),
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
            fraction = ease_fraction(step / count, count / hz)
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
    has_v = [point.get("v") is not None for point in waypoints]
    if any(has_v) and not all(has_v):
        raise ValueError("'v' must be on every waypoint or none")

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
        if point.get("v") is not None:
            if len(point["v"]) != len(ids):
                raise ValueError("waypoint %d has %d v values for %d motors"
                                 % (index, len(point["v"]), len(ids)))
            if not np.all(np.isfinite(point["v"])):
                raise ValueError("waypoint %d has a non-finite velocity" % index)
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

    # The velocity must agree with the step gate: the fastest motion 0.5 rad
    # per waypoint permits at this rate is MAX_JOINT_STEP * hz (the player
    # clamps hz to [MIN_HZ, MAX_HZ] first, so mirror that). A v that outruns
    # the q series is corrupt, and the MIT frame would silently clip it.
    # The 0.1% slack absorbs the six-decimal rounding of waypoint times.
    if any(has_v):
        ceiling = MAX_JOINT_STEP * max(MIN_HZ, min(MAX_HZ, hz)) * 1.001 + 1e-6
        for index, point in enumerate(waypoints):
            for joint, value in enumerate(point["v"]):
                if abs(value) > ceiling:
                    raise ValueError("joint %d at waypoint %d commands %+.3f rad/s, above the "
                                     "%.3f rad/s the step gate permits at %.0f Hz"
                                     % (ids[joint], index, value, ceiling, hz))

    return {
        "waypoints": len(waypoints),
        "duration_s": times[-1] - times[0],
        "peak_tau": max((abs(v) for point in waypoints if point.get("tau")
                         for v in point["tau"]), default=0.0),
        "peak_v": max((abs(v) for point in waypoints if point.get("v")
                       for v in point["v"]), default=0.0),
        "max_step_rad": max((max(abs(x - y) for x, y in zip(a["q"], b["q"]))
                             for a, b in zip(waypoints, waypoints[1:])), default=0.0),
    }
