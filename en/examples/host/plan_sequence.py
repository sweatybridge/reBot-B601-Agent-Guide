#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compose an arm + gripper sequence into one 7-motor joint trajectory.

HOST DEVICE - runs on a workstation (numpy + rebot_ik.py next to it). The arm
legs go through IK (cartesian straight lines); the gripper legs are direct
motor-7 angles. Everything lands in a single trajectory file that
play_trajectory.py (../edge/) replays unchanged, so the Pi only ever sees joint
angles.

Steps are written as `;`-separated actions:

    move:x,y,z:seconds          cartesian delta in base_link (x forward, z up)
    movej:x,y,z:JOINT:DELTA:s   same, with one joint rotating during the leg
    home:seconds                back to the start position, keeping orientation
    homej:seconds               joint-space return to the start pose (mirror of movej)
    joint:JOINT:DELTA:seconds   rotate a single joint, arm otherwise still
    grip:open:seconds           open to max
    grip:close:seconds          back to the closed stop
    grip:to:CM:seconds          close to a jaw width in cm
    hold:seconds                hold everything still

Example - open, diagonal transfer while the wrist tilts, grip at 6 cm, hold,
return with the wrist untilting, open, close:

    python3 plan_sequence.py --joints current.json --out ../trajectories/seq.json \
        --hz 30 --steps "grip:open:2.5; movej:0.20,0,0.30:4:0.1:5; \
                         grip:to:6:2.5; hold:2; homej:5; grip:open:2.5; grip:close:2.5"

A joint cannot be prescribed independently of the tool pose on a 6-DOF arm:
`movej` therefore interpolates in joint space and the tool path bows (the script
prints how far). A sequential "translate home, then untilt" is not reachable at
all - holding the tilt at the home position needs joints 2/3 past their stops.

Gripper calibration (measured on the reference arm): the jaws shut at about
-0.015 rad, not 0, and the rad-per-cm scale is 0.412, derived from Seeed's
0..5 rad closed-to-open range spanning the ~12 cm of finger travel in the URDF.
Correct both numbers once you have a ruler reading.
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
MOTOR7_CLOSED = -0.015
MOTOR7_OPEN = 4.90
RAD_PER_CM = 0.412
MOTOR7_LIMITS = (-0.10, 5.0)

# Encoders read a few mrad past a mechanical stop, so a strict comparison
# rejects poses the arm is demonstrably sitting in (joint 3 reads -0.0012 rad
# against a 0 rad lower limit when the arm is at rest).
LIMIT_TOL = 0.002


def ease_in_out(t):
    return 0.5 * (1.0 - math.cos(math.pi * min(max(t, 0.0), 1.0)))


def cm_to_rad(cm):
    return MOTOR7_CLOSED + cm * RAD_PER_CM


def max_off_line(points, start_xyz, end_xyz):
    """Largest distance from the straight line between two endpoints."""
    direction = np.asarray(end_xyz, dtype=float) - np.asarray(start_xyz, dtype=float)
    length = float(np.linalg.norm(direction))
    if length < 1e-9:
        return 0.0
    unit = direction / length
    worst = 0.0
    for point in points:
        rel = point - np.asarray(start_xyz, dtype=float)
        worst = max(worst, float(np.linalg.norm(rel - unit * (rel @ unit))))
    return worst


def load_state(path, start_gripper):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    q = data["q"] if isinstance(data, dict) else data
    if len(q) not in (6, 7):
        raise SystemExit("%s: expected 6 or 7 joint values, got %d" % (path, len(q)))
    gripper = float(q[6]) if len(q) == 7 else start_gripper
    return np.asarray(q[:6], dtype=float), gripper


def parse_steps(spec):
    steps = []
    for raw in spec.split(";"):
        fields = [f.strip() for f in raw.split(":") if f.strip()]
        if not fields:
            continue
        kind = fields[0].lower()
        try:
            if kind == "move":
                delta = np.array([float(v) for v in fields[1].split(",")], dtype=float)
                if delta.shape != (3,):
                    raise ValueError
                steps.append(("move", delta, float(fields[2])))
            elif kind == "grip":
                action = fields[1].lower()
                if action == "open":
                    steps.append(("grip", MOTOR7_OPEN, float(fields[2])))
                elif action == "close":
                    steps.append(("grip", MOTOR7_CLOSED, float(fields[2])))
                elif action == "to":
                    steps.append(("grip", cm_to_rad(float(fields[2])), float(fields[3])))
                else:
                    raise ValueError
            elif kind == "movej":
                # cartesian move with a joint rotating at the same time: solve IK
                # for the destination, offset that joint, then interpolate the
                # whole leg in joint space. A 6-DOF arm cannot hold a straight
                # tool line and an independent joint angle at once, so the tool
                # path becomes a gentle arc - the script reports how far off the
                # straight line it strays.
                delta_xyz = np.array([float(v) for v in fields[1].split(",")], dtype=float)
                joint_index = int(fields[2]) - 1
                if delta_xyz.shape != (3,) or not 0 <= joint_index < 6:
                    raise ValueError
                steps.append(("movej", (delta_xyz, joint_index, float(fields[3])),
                              float(fields[4])))
            elif kind == "joint":
                joint_index = int(fields[1]) - 1
                if not 0 <= joint_index < 6:
                    raise ValueError
                steps.append(("joint", (joint_index, float(fields[2])), float(fields[3])))
            elif kind == "home":
                steps.append(("home", None, float(fields[1])))
            elif kind == "homej":
                steps.append(("homej", None, float(fields[1])))
            elif kind == "hold":
                steps.append(("hold", None, float(fields[1])))
            else:
                raise ValueError
        except (IndexError, ValueError):
            raise SystemExit("cannot parse step '%s' - see the docstring for the grammar" % raw)
    if not steps:
        raise SystemExit("no steps given")
    return steps


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compose arm + gripper moves into one trajectory.")
    parser.add_argument("--joints", required=True, help="dump_joints.py output (include motor 7)")
    parser.add_argument("--steps", required=True, help="actions, see docstring")
    parser.add_argument("--hz", type=int, default=20)
    parser.add_argument("--margin", type=float, default=0.02, help="joint-limit margin for IK")
    parser.add_argument("--tol-mm", type=float, default=0.5)
    parser.add_argument("--tol-rad", type=float, default=0.005)
    parser.add_argument("--gripper-kp-note", default="", help=argparse.SUPPRESS)
    parser.add_argument("--out", default="sequence.json")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    q, gripper = load_state(args.joints, MOTOR7_CLOSED)
    if not k.within_limits(q, margin=-LIMIT_TOL):
        raise SystemExit("start pose is outside the joint limits: %s" % q)

    steps = parse_steps(args.steps)
    pose = k.forward(q)
    q_start = q.copy()          # the limit band must always allow returning here
    offset = np.zeros(3)
    t = 0.0
    step_time = 1.0 / args.hz
    waypoints = [{"t": 0.0, "q": [float(v) for v in q] + [gripper]}]
    hold_start = hold_end = None

    print("Arm + gripper sequence")
    print("  start joints : %s  gripper %+.3f rad" % (
        ", ".join("%+.4f" % v for v in q), gripper))
    print("  start tcp    : [%+.4f %+.4f %+.4f] m" % tuple(pose[:3, 3]))

    home_position = pose[:3, 3].copy()

    for index, (kind, value, duration) in enumerate(steps, start=1):
        n = max(1, int(round(duration * args.hz)))
        if kind in ("move", "home"):
            # Targets are relative to the pose the arm is actually in now, not to
            # the pose it started from: a joint-space step in between (e.g. a
            # wrist rotation) moves the tool, and an absolute target would yank
            # it back to the old corner on the next cartesian leg.
            pose_now = k.forward(q)
            leg_start = pose_now[:3, 3].copy()
            if kind == "home":
                leg_end = home_position.copy()
            else:
                leg_end = leg_start + value
            for i in range(1, n + 1):
                frac = ease_in_out(i / n)
                point = leg_start + (leg_end - leg_start) * frac
                target = k.pose(point, pose_now[:3, :3])
                q, info = k.ik(target, q, tol_pos=args.tol_mm / 1000.0,
                               tol_rot=args.tol_rad, margin=args.margin, anchor=q_start)
                if not info["ok"]:
                    raise SystemExit(
                        "IK failed on step %d (%s) at %.0f%%: pos_err=%.2f mm. "
                        "The waypoint is out of reach or blocked by a joint limit."
                        % (index, kind, 100.0 * i / n, 1000 * info.get("pos_err", float("nan"))))
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
            now = k.forward(q)[:3, 3]
            print("  %-2d %-9s -> tcp [%+.3f %+.3f %+.3f] m, t=%.2f s"
                  % (index, kind, now[0], now[1], now[2], t))
        elif kind == "movej":
            delta_xyz, joint_index, joint_delta = value
            pose_now = k.forward(q)
            leg_start_xyz = pose_now[:3, 3].copy()
            leg_end_xyz = leg_start_xyz + delta_xyz
            q_end, info = k.ik(k.pose(leg_end_xyz, pose_now[:3, :3]), q,
                               tol_pos=args.tol_mm / 1000.0, tol_rot=args.tol_rad,
                               margin=args.margin, anchor=q_start)
            if not info["ok"]:
                raise SystemExit("IK failed for the destination of step %d: pos_err=%.2f mm"
                                 % (index, 1000 * info.get("pos_err", float("nan"))))
            q_end = q_end.copy()
            q_end[joint_index] += joint_delta
            if not k.within_limits(q_end, margin=-LIMIT_TOL):
                raise SystemExit("step %d: joint %d would end outside its limits"
                                 % (index, joint_index + 1))

            leg_from = q.copy()
            track = []
            for i in range(1, n + 1):
                q = leg_from + (q_end - leg_from) * ease_in_out(i / n)
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
                track.append(k.forward(q)[:3, 3])
            # Report how far the tool strayed from the straight line it aimed at:
            # rotating a joint during the leg necessarily bends the path.
            deviation = 0.0
            direction = leg_end_xyz - leg_start_xyz
            length = float(np.linalg.norm(direction))
            if length > 1e-9:
                unit = direction / length
                for point in track:
                    rel = point - leg_start_xyz
                    deviation = max(deviation, float(np.linalg.norm(rel - unit * (rel @ unit))))
            tip = k.forward(q)[:3, 3]
            print("  %-2d %-9s -> j%d %+0.3f, tcp [%+.3f %+.3f %+.3f] m, "
                  "off-line %.1f mm, t=%.2f s"
                  % (index, kind, joint_index + 1, q[joint_index], tip[0], tip[1], tip[2],
                     1000 * deviation, t))
        elif kind == "homej":
            # Joint-space return, the mirror of movej: every joint travels back to
            # the start pose over the leg. A sequential "translate home, then
            # untwist" is not possible on this arm - reaching the home position
            # with the wrist still tilted needs joint 2/3 past their 0 rad stops
            # (IK lands 36 mm short), so the tilt has to come off during the leg.
            leg_from = q.copy()
            leg_start_xyz = k.forward(q)[:3, 3].copy()
            q_end = q_start.copy()
            leg_end_xyz = k.forward(q_end)[:3, 3].copy()
            track = []
            for i in range(1, n + 1):
                q = leg_from + (q_end - leg_from) * ease_in_out(i / n)
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
                track.append(k.forward(q)[:3, 3])
            tip = k.forward(q)[:3, 3]
            print("  %-2d %-9s -> j4 %+0.3f, tcp [%+.3f %+.3f %+.3f] m, off-line %.1f mm, t=%.2f s"
                  % (index, kind, q[3], tip[0], tip[1], tip[2],
                     1000 * max_off_line(track, leg_start_xyz, leg_end_xyz), t))
        elif kind == "joint":
            joint_index, delta = value
            start_angle = q[joint_index]
            for i in range(1, n + 1):
                q[joint_index] = start_angle + delta * ease_in_out(i / n)
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
            now = k.forward(q)[:3, 3]
            print("  %-2d joint%d %+0.3f rad -> j%d=%+.3f, tcp z %+.3f m, t=%.2f s"
                  % (index, joint_index + 1, delta, joint_index + 1, q[joint_index], now[2], t))
        elif kind == "grip":
            start_g = gripper
            for i in range(1, n + 1):
                gripper = start_g + (value - start_g) * ease_in_out(i / n)
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
            print("  %-2d grip %+.3f -> %+.3f rad, t=%.2f s" % (index, start_g, gripper, t))
        else:
            if hold_start is None:
                hold_start = t
            for _ in range(n):
                t += step_time
                waypoints.append({"t": round(t, 4), "q": [float(v) for v in q] + [gripper]})
            if hold_end is None:
                hold_end = t
            print("  %-2d hold      %.1f s, t=%.2f s" % (index, duration, t))

    qs = np.array([w["q"][:6] for w in waypoints])
    if not np.all((qs >= k.LIMITS[:, 0] - LIMIT_TOL) & (qs <= k.LIMITS[:, 1] + LIMIT_TOL)):
        over = np.where(np.any((qs < k.LIMITS[:, 0] - LIMIT_TOL) |
                               (qs > k.LIMITS[:, 1] + LIMIT_TOL), axis=0))[0]
        raise SystemExit("planned arm path violates joint limits on joint(s) %s"
                         % ", ".join(str(i + 1) for i in over))
    gs = np.array([w["q"][6] for w in waypoints])
    if not np.all((gs >= MOTOR7_LIMITS[0]) & (gs <= MOTOR7_LIMITS[1])):
        raise SystemExit("planned gripper path outside %.2f..%.2f rad" % MOTOR7_LIMITS)

    max_rate = float(np.max(np.abs(np.diff(qs, axis=0))) * args.hz) if len(qs) > 1 else 0.0
    reached = k.forward(qs[-1])
    print("\n  peak arm speed : %.3f rad/s" % max_rate)
    print("  end tcp        : [%+.4f %+.4f %+.4f] m" % tuple(reached[:3, 3]))
    print("  end vs start   : %.1f mm" % (1000 * np.linalg.norm(reached[:3, 3] - pose[:3, 3])))
    print("  duration       : %.2f s (%d waypoints)" % (t, len(waypoints)))

    data = {
        "format": FORMAT,
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "control_hz": args.hz,
        "ids": list(range(1, 8)),
        "limits": [[float(lo), float(hi)] for lo, hi in k.LIMITS] + [list(MOTOR7_LIMITS)],
        "steps": args.steps,
        "duration_s": t,
        "hold_start_s": hold_start,
        "hold_end_s": hold_end,
        "waypoints": waypoints,
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(data, handle, separators=(",", ":"))
    print("\nwrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
