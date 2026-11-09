#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plan the 30 Hz transfer with Seeed's Pinocchio stack and write our JSON.

HOST - runs inside the Pinocchio container (see Dockerfile.pinocchio); the ARM
comes from Seeed's `reBotArm_control_py`, the report format is this repo's
`rebot-joint-trajectory`. The only thing the edge device needs is the JSON.

What is actually theirs here: the URDF, the model loading, the IK solver and
`compute_generalized_gravity` (Pinocchio RNEA). What is ours: the sequence timing
and the gripper legs, which are direct motor-7 angles, not IK.

The output adds one field this repo's player understands: `tau`, the gravity
torque of the planned pose, per waypoint. That is a *model* feedforward, sent in
the MIT frame's torque field; it generalises to other trajectories, unlike the
measured trim, but it inherits the model's 30-40 % error at folded poses.

    docker run --rm -v "$PWD:/work" rebot-pino \
        python /work/examples/host/plan_with_pinocchio.py \
        --joints /work/current.json --out /work/seq_pino.json
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import pinocchio as pin

from reBotArm_control_py.dynamics import compute_generalized_gravity
from reBotArm_control_py.kinematics.forward_kinematics import compute_fk
from reBotArm_control_py.kinematics.inverse_kinematics import (
    pos_rot_to_se3,
    solve_ik,
)
from reBotArm_control_py.kinematics.robot_model import (
    get_arm_joint_count,
    get_end_effector_frame_id,
    load_robot_model,
    pad_q_for_model,
)
from reBotArm_control_py.trajectory import IKParams

from plan_to_target import PlanError, posture
import traj

MOTOR7_OPEN = 4.90
MOTOR7_CLOSED = -0.015


def ease(x):
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, x)))


def load_dump(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    try:
        return posture(data)
    except PlanError as error:
        raise SystemExit("%s: %s" % (path, error)) from error


def main(argv=None):
    parser = argparse.ArgumentParser(description="Plan a trajectory with Pinocchio.")
    parser.add_argument("--joints", required=True, help="rebot-edge dump output")
    parser.add_argument("--out", required=True, help="trajectory JSON to write")
    parser.add_argument("--urdf", default="", help="override the URDF path")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--dx", type=float, default=0.20)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.30)
    parser.add_argument("--lift-s", type=float, default=5.0)
    parser.add_argument("--return-s", type=float, default=5.0)
    parser.add_argument("--grip-s", type=float, default=2.5)
    parser.add_argument("--hold-s", type=float, default=2.0)
    parser.add_argument("--peel-x", type=float, default=0.0,
                        help="move the tool this far forward while the jaws open")
    parser.add_argument("--peel-z", type=float, default=0.0,
                        help="and this far up during the same leg (not needed: the joint sequence does it)")
    parser.add_argument("--peel-j2", type=float, default=0.0,
                        help="shoulder lift (deg) during the peel; 0 is enough, the "
                             "elbow carries it")
    parser.add_argument("--peel-j3", type=float, default=8.0,
                        help="elbow rotation (deg) during the peel - the binding stop")
    parser.add_argument("--peel-j4", type=float, default=0.0,
                        help="wrist rotation (deg) during the peel; 0 is enough. "
                             "Wrist+shoulder without the elbow return a -18 mm dip")
    parser.add_argument("--peel-together", action="store_true",
                        help="move all three at once instead of in sequence")
    parser.add_argument("--peel-blend", type=float, default=0.4,
                        help="fraction of the lift over which the wrist returns to flat")
    parser.add_argument("--payload", type=float, default=0.0,
                        help="mass (kg) held at the tool frame; folded into tau")
    parser.add_argument("--no-return", action="store_true",
                        help="stop at the hold instead of coming home")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    model = load_robot_model(args.urdf or None)
    frame = get_end_effector_frame_id(model)
    n_arm = get_arm_joint_count(model, frame)
    if n_arm != 6:
        raise SystemExit("expected 6 arm joints, model has %d" % n_arm)
    if args.payload > 0.0:
        # Attach the mass at the tool frame, so compute_generalized_gravity
        # includes it. Verified against the URDF model: 1 kg adds
        # j2 -2.77, j3 +5.08, j4 +2.84 Nm at the parked pose.
        tip = model.frames[frame]
        model.appendBodyToJoint(
            tip.parentJoint,
            pin.Inertia(args.payload, np.zeros(3), np.zeros((3, 3))),
            tip.placement,
        )
    q_start, gripper_start = load_dump(args.joints)
    # The arm rests a few mrad past a stop; the planner and the player both work
    # inside the URDF limits, and the align ramp covers the difference.
    # lowerPositionLimit/upperPositionLimit are indexed by q, and their helper
    # indexes them by joint id, which Pinocchio 4 numbers differently - read the
    # arrays directly.
    lower = np.asarray(model.lowerPositionLimit[:n_arm], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:n_arm], dtype=float)
    clamped = np.clip(q_start, lower, upper)
    if not np.allclose(clamped, q_start):
        print("  start pose  : clamped to the joint limits: %s -> %s"
              % (np.round(q_start, 4), np.round(clamped, 4)))
        q_start = clamped
    q = pad_q_for_model(model, q_start, n_arm)
    _, R0, T0 = compute_fk(model, q)
    p0 = T0[:3, 3].copy()
    delta = np.array([args.dx, args.dy, args.dz])
    ik_params = IKParams(max_iter=300, tolerance=1e-6, damping=1e-8, step_size=1.0)

    print("Pinocchio plan")
    print("  pinocchio   : %s" % pin.__version__)
    print("  start tcp   : [%+.4f %+.4f %+.4f] m" % tuple(p0))
    print("  target      : [%+.4f %+.4f %+.4f] m (orientation held)" % tuple(p0 + delta))
    if args.payload > 0.0:
        print("  payload     : %.2f kg at the tool frame" % args.payload)

    def ik_to(point, seed, rotation=None):
        result = solve_ik(model, model.createData(), frame,
                          pos_rot_to_se3(point, R0 if rotation is None else rotation), seed,
                          ik_params, controlled_joints=n_arm)
        if not result.success:
            raise SystemExit("IK failed at [%.3f %.3f %.3f]: %s"
                             % (point[0], point[1], point[2],
                                getattr(result, "message", "no solution")))
        return pad_q_for_model(model, result.q, n_arm)

    def blend_rotation(a, b, fraction):
        """Rotate from `a` towards `b` by `fraction` of the shortest arc."""
        m = a.T @ b
        angle = float(np.arccos(np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0)))
        if angle < 1e-9:
            return a.copy()
        axis = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
        axis = axis / (2.0 * np.sin(angle))
        k = np.array([[0.0, -axis[2], axis[1]],
                      [axis[2], 0.0, -axis[0]],
                      [-axis[1], axis[0], 0.0]])
        step = angle * fraction
        return a @ (np.eye(3) + np.sin(step) * k + (1.0 - np.cos(step)) * k @ k)

    period = 1.0 / args.hz
    waypoints = []
    t = 0.0

    def append(q_model, gripper):
        gravity = compute_generalized_gravity(model=model, q=q_model)
        waypoints.append({
            "t": round(t, 6),
            "q": [float(v) for v in q_model[:n_arm]] + [float(gripper)],
            # One torque per motor, in `ids` order; the gripper carries none.
            "tau": [float(v) for v in gravity[:n_arm]] + [0.0],
        })

    # Leg 1: open the jaws and peel. The parked pose rests on the j2 and j3
    # lower stops (both measured within 7 mrad of 0.000), so the way off is a
    # sequence, one stop at a time: the wrist rotation first (it opens the fold
    # without fighting a stop), then the shoulder, then the elbow. A cartesian
    # peel (peel_x/peel_z) can be layered on top but is not needed.
    peel = np.array([args.peel_x, 0.0, args.peel_z])
    n = max(1, int(round(args.grip_s * args.hz)))
    start_q = q.copy()
    q_peel = q.copy()
    arms = [(3, np.radians(args.peel_j4)), (1, np.radians(args.peel_j2)),
            (2, np.radians(args.peel_j3))]
    if args.peel_together:
        windows = [(0.0, 1.0)] * 3
    else:
        windows = [(0.0, 0.4), (0.4, 0.7), (0.7, 1.0)]
    for i in range(1, n + 1):
        t += period
        raw = i / n
        q_peel = start_q.copy()
        for phase, (index, amplitude) in enumerate(arms):
            lo, hi = windows[phase]
            span = max(1e-6, hi - lo)
            local = ease(min(1.0, max(0.0, (raw - lo) / span)))
            q_peel[index] = start_q[index] + amplitude * local
        append(q_peel, gripper_start + (MOTOR7_OPEN - gripper_start) * ease(raw))
    T_peel = compute_fk(model, q_peel)[2]
    peel_start = T_peel[:3, 3].copy()
    R_peel = T_peel[:3, :3].copy()

    # Leg 2: straight cartesian lift, orientation blended from the peeled wrist
    # back to flat over the first part of the leg, so j4 returns smoothly.
    end_point = p0 + delta
    travel = end_point - peel_start
    n = max(1, int(round(args.lift_s * args.hz)))
    seed = q_peel.copy()
    for i in range(1, n + 1):
        t += period
        fraction = ease(i / n)
        rotation = blend_rotation(R_peel, R0, min(1.0, fraction / max(1e-6, args.peel_blend)))
        q = ik_to(peel_start + travel * fraction, seed, rotation)
        seed = q.copy()
        append(q, MOTOR7_OPEN)

    # Leg 3: close to 6 cm at the top.
    n = max(1, int(round(args.grip_s * args.hz)))
    for i in range(1, n + 1):
        t += period
        append(q, MOTOR7_OPEN + (2.457 - MOTOR7_OPEN) * ease(i / n))

    # Leg 4: hold.
    n = max(1, int(round(args.hold_s * args.hz)))
    hold_start = t
    for i in range(1, n + 1):
        t += period
        append(q, 2.457)
    hold_end = t

    # Leg 5: retrace to the start pose.
    if not args.no_return:
        n = max(1, int(round(args.return_s * args.hz)))
        seed = q.copy()
        for i in range(1, n + 1):
            t += period
            q = ik_to(p0 + delta * (1.0 - ease(i / n)), seed)
            seed = q.copy()
            append(q, 2.457)
        n = max(1, int(round(args.grip_s * args.hz)))
        for i in range(1, n + 1):
            t += period
            append(q, 2.457 + (MOTOR7_OPEN - 2.457) * ease(i / n))
        for i in range(1, n + 1):
            t += period
            append(q, MOTOR7_OPEN + (MOTOR7_CLOSED - MOTOR7_OPEN) * ease(i / n))

    # One payload builder for both planners: traj.build owns the schema, the
    # duration and the hold window, so this file cannot drift from the player's
    # expectations (or from plan_to_target.py).
    payload = traj.build(
        waypoints, args.hz,
        [[float(lo), float(hi)] for lo, hi in zip(lower, upper)] + [[-0.1, 5.0]],
        ("pinocchio: open, lift %.2f/%.2f/%.2f m with world orientation held, "
         "grip 6 cm, hold %.1f s, return" % (args.dx, args.dy, args.dz, args.hold_s)),
        extra={
            "dynamics": {
                "source": "reBotArm_control_py.compute_generalized_gravity (Pinocchio RNEA)",
                "tau": "gravity torque of the planned pose, per waypoint",
            },
        },
        hold_start=hold_start,
        hold_end=hold_end,
    )
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, separators=(",", ":"))

    tau = np.array([wp["tau"] for wp in waypoints])
    print("  waypoints   : %d over %.2f s" % (len(waypoints), t))
    print("  peak |tau|  : %s N.m" % np.round(np.abs(tau).max(axis=0), 2))
    print("  end tcp     : %s" % np.round(compute_fk(model, q)[2][:3, 3], 4))
    print("  wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
