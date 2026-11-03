#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline cartesian planner for the reBot B601-RS.

HOST DEVICE - runs on a workstation, needs only numpy (+ rebot_ik.py next to it).
It turns a cartesian move into a joint-space trajectory file; the edge device
never runs IK, it plays the file back with play_trajectory.py at a fixed rate.

    # 1. on the edge device: read the current joint angles (read-only, no torque)
    python3 dump_joints.py > current.json
    # ...copy current.json to the workstation

    # 2. on the workstation: raise the tool 10 cm, hold 1 s, come back
    python3 plan_cartesian.py --joints current.json --delta 0,0,0.10 \
        --duration 5 --pause 1.0 --hz 20 --out trajectories/raise10cm.json

    # 3. copy the trajectory back, then on the edge device (dry run first)
    python3 play_trajectory.py ../trajectories/raise10cm.json
    python3 play_trajectory.py ../trajectories/raise10cm.json --execute

Why offline: one damped-least-squares solve costs ~1.3 s on a Pi Zero W, so a
closed-loop 20 Hz controller is impossible there. Precomputing the joint path
turns the Pi's job into "send the next joint vector on time", which it can do.

The trajectory is planned as a straight line in cartesian space: every waypoint
is an independent IK solve warm-started from the previous one, and any waypoint
that fails to converge aborts the whole plan (no silently wrong paths).
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import rebot_ik as k  # noqa: E402

FORMAT = "rebot-joint-trajectory"
VERSION = 1

# See plan_sequence.py: encoders read a few mrad past a mechanical stop at rest.
LIMIT_TOL = 0.002


def ease_in_out(t):
    return 0.5 * (1.0 - math.cos(math.pi * min(max(t, 0.0), 1.0)))


def load_joints(path):
    """Accept dump_joints.py output, a plain list, or {'q': [...]}."""
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, list):
        q = data
    elif isinstance(data, dict) and "q" in data:
        q = data["q"]
    else:
        raise SystemExit("%s: expected a joint list or an object with 'q'" % path)
    q = np.asarray(q, dtype=float)
    if q.shape == (k.DOF + 1,):
        # dump_joints.py often includes motor 7; the arm chain ignores it.
        q = q[:k.DOF]
    if q.shape != (k.DOF,):
        raise SystemExit("%s: expected %d joint angles for motors 1..%d (a 7th is allowed "
                         "and ignored), got %s" % (path, k.DOF, k.DOF, q.shape))
    return q


def parse_segments(args):
    """Cartesian path as a list of x,y,z deltas, applied in order.

    --path "0,0,0.25;0.10,0,0"  -> up 25 cm, then forward 10 cm
    --delta 0,0,0.10            -> single segment (kept for compatibility)
    """
    if args.path:
        segments = []
        for part in args.path.split(";"):
            part = part.strip()
            if not part:
                continue
            values = [float(v) for v in part.split(",")]
            if len(values) != 3:
                raise SystemExit("--path: '%s' is not x,y,z" % part)
            segments.append(np.array(values, dtype=float))
        if not segments:
            raise SystemExit("--path is empty")
        return segments

    delta = np.array([float(v) for v in args.delta.split(",")], dtype=float)
    if delta.shape != (3,):
        raise SystemExit("--delta expects x,y,z in metres, e.g. 0,0,0.10")
    return [delta]


def plan(args):
    q0 = load_joints(args.joints)
    if not k.within_limits(q0, margin=-LIMIT_TOL):
        raise SystemExit("start pose is outside the URDF joint limits: %s" % q0)

    segments = parse_segments(args)

    pose0 = k.forward(q0)
    total_delta = np.sum(segments, axis=0)
    target = k.translation_target(pose0, total_delta)

    print("Offline cartesian plan")
    print("  start joints : %s" % ", ".join("%+.4f" % v for v in q0))
    print("  start tcp    : [%+.4f %+.4f %+.4f] m" % tuple(pose0[:3, 3]))
    offset = np.zeros(3)
    for index, segment in enumerate(segments, start=1):
        offset = offset + segment
        print("  segment %d    : [%+.3f %+.3f %+.3f] m -> [%+.4f %+.4f %+.4f] m"
              % (index, segment[0], segment[1], segment[2], *(pose0[:3, 3] + offset)))
    print("  profile      : %.1f s per leg, %.1f s hold, %d Hz" % (args.duration, args.pause, args.hz))
    print("  IK tolerance : %.1f mm / %.2f deg" % (args.tol_mm, math.degrees(args.tol_rad)))

    if np.linalg.norm(total_delta) < 1e-9:
        raise SystemExit("the path goes nowhere, nothing to plan")

    # Build legs as (label, start_offset, end_offset, duration). Offsets are
    # cumulative cartesian displacements from the start pose.
    offsets = [np.zeros(3)]
    for segment in segments:
        offsets.append(offsets[-1] + segment)

    legs = []
    for index in range(len(segments)):
        legs.append(("seg%d" % (index + 1), offsets[index], offsets[index + 1], args.duration))
        if args.corner_hold > 0 and index < len(segments) - 1:
            legs.append(("corner%d" % (index + 1), offsets[index + 1], offsets[index + 1],
                         args.corner_hold))
    if args.pause > 0:
        legs.append(("hold", offsets[-1], offsets[-1], args.pause))
    if not args.no_return:
        for index in range(len(segments) - 1, -1, -1):
            legs.append(("back%d" % (index + 1), offsets[index + 1], offsets[index], args.duration))

    waypoints = []
    q = q0.copy()
    worst_residual = 0.0
    t = 0.0
    step = 1.0 / args.hz

    waypoints.append({"t": 0.0, "q": [float(v) for v in q0]})

    for label, p_start, p_end, leg_duration in legs:
        if label == "hold":
            hold_start = t
        n = max(1, int(round(leg_duration * args.hz)))
        for i in range(1, n + 1):
            frac = ease_in_out(i / n)
            offset = p_start + (p_end - p_start) * frac
            leg_target = k.translation_target(pose0, offset)
            seed = q
            q, info = k.ik(leg_target, seed, tol_pos=args.tol_mm / 1000.0,
                           tol_rot=args.tol_rad, margin=args.margin, anchor=q0)
            if not info["ok"]:
                raise SystemExit(
                    "IK failed on %s sample %d/%d: pos_err=%.4f m rot_err=%.3f rad.\n"
                    "The target is probably out of reach or blocked by a joint limit.\n"
                    "Try a smaller --delta or a different direction."
                    % (label, i, n, info.get("pos_err", float("nan")),
                       info.get("rot_err", float("nan"))))
            worst_residual = max(worst_residual, info["pos_err"])
            t += step
            waypoints.append({"t": round(t, 4), "q": [float(v) for v in q]})
        print("  %-6s leg: %3d waypoints, t = %.2f s" % (label, n, t))
        if label == "hold":
            hold_end = t

    # Safety checks the Pi cannot do for us.
    qs = np.array([w["q"] for w in waypoints])
    if not np.all((qs >= k.LIMITS[:, 0]) & (qs <= k.LIMITS[:, 1])):
        raise SystemExit("planned path violates joint limits")
    steps = np.diff(qs, axis=0)
    max_rate = float(np.max(np.abs(steps)) * args.hz) if len(steps) else 0.0
    if max_rate > args.max_joint_speed:
        raise SystemExit(
            "fastest joint needs %.2f rad/s, above --max-joint-speed %.2f.\n"
            "Lengthen --duration (e.g. %.1f s) or reduce --delta."
            % (max_rate, args.max_joint_speed, args.duration * max_rate / args.max_joint_speed))

    reached = k.forward(qs[-1])
    print("\n  peak joint speed : %.3f rad/s" % max_rate)
    print("  worst IK residual: %.3f mm" % (1000 * worst_residual))
    print("  end tcp          : [%+.4f %+.4f %+.4f] m"
          % tuple(reached[:3, 3]))
    print("  end vs start     : %.2f mm" % (1000 * np.linalg.norm(reached[:3, 3] - pose0[:3, 3])))
    print("  total duration   : %.2f s (%d waypoints)" % (waypoints[-1]["t"], len(waypoints)))

    return {
        "format": FORMAT,
        "version": VERSION,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "urdf": k.URDF_URL,
        "control_hz": args.hz,
        "ids": list(range(1, k.DOF + 1)),
        "limits": [[float(lo), float(hi)] for lo, hi in k.LIMITS],
        "delta_xyz": [float(v) for v in total_delta],
        "segments_xyz": [[float(v) for v in segment] for segment in segments],
        "start_tcp": [float(v) for v in pose0[:3, 3]],
        "end_tcp": [float(v) for v in reached[:3, 3]],
        "peak_joint_speed": max_rate,
        "worst_ik_residual_m": worst_residual,
        "duration_s": waypoints[-1]["t"],
        "hold_start_s": hold_start,
        "hold_end_s": hold_end,
        "waypoints": waypoints,
    }


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Plan a cartesian move as a joint-space trajectory.")
    parser.add_argument("--joints", required=True,
                        help="joint state JSON from dump_joints.py (or a --q value)")
    parser.add_argument("--delta", default="0,0,0.10",
                        help="single cartesian move x,y,z in metres (default: 0,0,0.10 = up 10 cm)")
    parser.add_argument("--path", default="",
                        help="multi-segment path, e.g. '0,0,0.25;0.10,0,0' (overrides --delta)")
    parser.add_argument("--corner-hold", type=float, default=0.5,
                        help="seconds to pause at each intermediate corner (default 0.5)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="seconds per leg (default 5.0)")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="seconds to hold at the far point (default 1.0)")
    parser.add_argument("--hz", type=int, default=20,
                        help="waypoint rate; the Pi replays at this rate (default 20)")
    parser.add_argument("--margin", type=float, default=0.02,
                        help="joint-limit margin in rad while solving (default 0.02)")
    parser.add_argument("--tol-mm", type=float, default=0.5, help="IK position tolerance (default 0.5)")
    parser.add_argument("--tol-rad", type=float, default=0.005, help="IK orientation tolerance (default 0.005)")
    parser.add_argument("--max-joint-speed", type=float, default=0.8,
                        help="refuse plans faster than this (rad/s, default 0.8)")
    parser.add_argument("--no-return", action="store_true", help="stop at the far point instead of returning")
    parser.add_argument("--out", default="trajectory.json", help="output file")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    trajectory = plan(args)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(trajectory, handle, separators=(",", ":"))
    print("\nwrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
