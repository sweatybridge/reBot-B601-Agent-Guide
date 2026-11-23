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

import rebot_ik  # single source for the motion tuning (world.arm_config)

FORMAT = "rebot-joint-trajectory"
VERSION = 1
IDS = [1, 2, 3, 4, 5, 6, 7]
MAX_JOINT_STEP = rebot_ik.MOTION["max_joint_step"]   # rad, between consecutive waypoints
LIMIT_TOL = rebot_ik.MOTION["limit_tol"]             # rad, encoders read a few mrad past a stop at rest
MIT_POSITION_LIMIT = 4.0 * math.pi
MIN_HZ, MAX_HZ = 5.0, 200.0
# The default stream rate every producer inherits. 50 Hz is measured on the
# reference board: 0 of 1100 ticks late, 20.0/20.0/21.8 ms, peak follow error
# 0.0048 rad (edge-rust README's rate table) -- finer ticks than 30 Hz at no
# timing cost. The player is timestamp-driven and accepts 5..200 Hz, so a file
# planned at another rate still plays; this is only the default.
DEFAULT_HZ = 50.0

# Velocity ramp at each end of a leg, seconds. The profile is an S-ramp
# trapezoid: velocity rises as a smoothstep into a constant-velocity cruise,
# so acceleration starts and ends at zero -- no acceleration step at
# rest->ramp->cruise for the P-only drive to feel as a torque kick. The
# smoothstep's area is 1/2, exactly the linear ramp's, so the ramp covers the
# same distance in the same time and the cruise speed (travel / (T - ramp))
# is unchanged. The cosine ease this replaces has sine velocity tails that
# scale with leg duration -- on a 25 s leg the drive spent ~2.5 s at each end
# below its stiction breakaway speed (~0.2 rad/s at kp=50) and moved in
# stick-slip segments. The S-ramp leaves rest quadratically (a hair slower
# than linear at the very start) but crosses the breakaway band in well under
# the 0.2 s stall budget; 0.15 s keeps the whole ramp short.
# Single source: rebot_ik.MOTION (world.arm_config motion group).
RAMP_S = rebot_ik.MOTION["ramp_s"]


def _ramp_ratio(duration, ramp):
    return min(0.5, max(1e-9, float(ramp) / max(float(duration), 1e-9)))


def ease_fraction(u, duration, ramp=None):
    """Fraction of a leg's travel at normalized time u, S-ramp trapezoid.

    Velocity rises as a smoothstep (3x^2 - 2x^3) over the ramp: acceleration
    is zero at rest and at cruise, continuous in between. The smoothstep's
    area is 1/2, the linear ramp's, so ramp distance and cruise timing are
    unchanged. Legs shorter than 2*ramp degenerate to a smooth bell (no
    cruise); every shape starts and ends at rest. `duration` is the leg's
    scheduled time in seconds.
    """
    r = _ramp_ratio(duration, RAMP_S if ramp is None else ramp)
    vc = 1.0 / (1.0 - r)
    u = max(0.0, min(1.0, float(u)))
    if u <= r:
        x = u / r
        return vc * r * (x ** 3 - 0.5 * x ** 4)
    if u >= 1.0 - r:
        y = (1.0 - u) / r
        return 1.0 - vc * r * (y ** 3 - 0.5 * y ** 4)
    return vc * (u - 0.5 * r)


def ease_velocity(u, duration, ramp=None):
    """d(ease_fraction)/dt in 1/s; multiply by the joint travel for rad/s."""
    r = _ramp_ratio(duration, RAMP_S if ramp is None else ramp)
    vc = 1.0 / (1.0 - r)
    u = max(0.0, min(1.0, float(u)))
    duration = max(float(duration), 1e-9)
    if u <= r:
        x = u / r
        return vc * (3.0 * x * x - 2.0 * x ** 3) / duration
    if u >= 1.0 - r:
        y = (1.0 - u) / r
        return vc * (3.0 * y * y - 2.0 * y ** 3) / duration
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


def joint_move(q_from, q_to, seconds, control_hz=DEFAULT_HZ, hold_s=0.4, limits=None, label=None):
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


def joint_sequence(poses, seconds, hold_s=0.4, control_hz=DEFAULT_HZ, limits=None, label=None):
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


# Corner fillet radius at via knots: joint-space arc cut from each adjacent
# segment. A quadratic Bezier tangent to both segments rounds the corner, so
# the direction change spreads over the fillet instead of one tick. The path
# stays within radius/2 of the knot -- a few mm at the tip for typical turns,
# below the arm's ~23 mm repeatability -- and at the 0.5 rad/s cruise cap the
# centripetal budget s^2/r is a few rad/s^2 spread over ticks, not a spike.
CORNER_RADIUS_RAD = 0.05
CHAIN_LINE_SAMPLES = 40    # per straight portion: the path_safe gate's density
CHAIN_FILLET_SAMPLES = 16


def _chain_knots(poses):
    points = [np.asarray(p, dtype=float).reshape(-1) for p in poses]
    if len(points) < 2:
        raise ValueError("a chain needs at least two poses, got %d" % len(points))
    size = points[0].size
    if any(p.size != size for p in points):
        raise ValueError("chain poses must all have the same joint count")
    if not all(np.all(np.isfinite(p)) for p in points):
        raise ValueError("chain poses must be finite")
    return points


def _chain_path(poses, radius=CORNER_RADIUS_RAD):
    """Dense samples + cumulative arc of the filleted knot path.

    Straight portions are sampled at the capsule gate's own density and
    interpolated exactly; each interior knot gets a tangent quadratic fillet
    whose radius is clamped to half the shorter adjacent segment so fillets
    never overlap. A degenerate corner (zero radius or a zero-length segment)
    keeps the sharp knot.
    """
    points = _chain_knots(poses)
    lengths = [float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:])]
    samples = [points[0]]
    cursor = points[0]
    for index in range(1, len(points) - 1):
        knot = points[index]
        span_in, span_out = lengths[index - 1], lengths[index]
        r = min(float(radius), span_in / 2.0, span_out / 2.0)
        if r > 1e-9 and span_in > 1e-12 and span_out > 1e-12:
            u_in = (knot - points[index - 1]) / span_in
            u_out = (points[index + 1] - knot) / span_out
            a = knot - u_in * r
            b = knot + u_out * r
            for step in range(1, CHAIN_LINE_SAMPLES + 1):
                samples.append(cursor + (a - cursor) * (step / CHAIN_LINE_SAMPLES))
            for step in range(1, CHAIN_FILLET_SAMPLES + 1):
                t = step / CHAIN_FILLET_SAMPLES
                samples.append((1.0 - t) ** 2 * a
                               + 2.0 * t * (1.0 - t) * knot
                               + t ** 2 * b)
            cursor = b
        else:
            for step in range(1, CHAIN_LINE_SAMPLES + 1):
                samples.append(cursor + (knot - cursor) * (step / CHAIN_LINE_SAMPLES))
            cursor = knot
    for step in range(1, CHAIN_LINE_SAMPLES + 1):
        samples.append(cursor + (points[-1] - cursor) * (step / CHAIN_LINE_SAMPLES))
    samples = np.asarray(samples)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(samples, axis=0), axis=1))])
    return samples, arc


def chain_samples(poses, radius=CORNER_RADIUS_RAD):
    """Dense samples of the filleted path, for the worktop capsule gate: a
    rounded corner leaves the chord near the knot, so the gate must see the
    fillets, not just the straight segments."""
    samples, _arc = _chain_path(poses, radius)
    return samples


def chain_travel(poses, radius=CORNER_RADIUS_RAD):
    """The furthest-travelling joint's path length along the filleted path, rad.

    A joint that reverses direction at a via travels more than its endpoint
    delta; the speed policy must budget the whole path, corner cuts included.
    """
    samples, _arc = _chain_path(poses, radius)
    return float(np.max(np.sum(np.abs(np.diff(samples, axis=0)), axis=0)))


def chain_series(poses, seconds, control_hz=DEFAULT_HZ, radius=CORNER_RADIUS_RAD):
    """Joint poses for one continuous leg through `poses`, tick by tick.

    Straight joint-space segments with tangent quadratic fillets at the
    interior knots, under ONE S-ramp trapezoid time law over the whole path's
    arc length: start at rest at poses[0] (already the caller's last waypoint),
    ramp up over RAMP_S, cruise through every via without stopping, ramp down
    to rest at poses[-1]. The fillets spread each direction change over ticks;
    a near-180 deg reversal still turns through zero speed -- unavoidable.

    Returns `count` poses, the last exactly poses[-1]. Ticks on the straight
    portions lie on the segments exactly; fillets are sampled at
    CHAIN_FILLET_SAMPLES. Knots are passed within radius/2 (the fillet's
    corner cut), the discrete ticks straddle within half a tick of travel.
    """
    samples, arc = _chain_path(poses, radius)
    total = float(arc[-1])
    hz = float(control_hz)
    count = max(2, int(round(max(0.2, float(seconds)) * hz)))
    duration = count / hz
    out = []
    for index in range(1, count + 1):
        target = ease_fraction(index / count, duration) * total
        slot = int(np.searchsorted(arc, target, side="right")) - 1
        slot = max(0, min(slot, len(arc) - 2))
        span = arc[slot + 1] - arc[slot]
        weight = 0.0 if span <= 1e-12 else (target - arc[slot]) / span
        out.append(samples[slot] + (samples[slot + 1] - samples[slot]) * weight)
    out[-1] = samples[-1].copy()
    return out


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
