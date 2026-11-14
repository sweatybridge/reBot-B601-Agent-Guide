#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plan a phone-located target with Seeed's Pinocchio stack.

Two callers share this module:

* `main()` -- the CLI, inside the Pinocchio container or via compose:
  `docker compose --profile plan run --rm planner --joints ... --target ... --out ...`
* `planner_server.py` -- the long-running arm gateway the viewers talk to when
  someone taps the video. It calls `plan()` directly, so a tap and a shell
  command produce byte-identical trajectories.

The motion is deliberately simple and repeatable:

    start pose -> unpark (joint space) -> pre-grasp above the target -> descend
    -> close -> lift -> back to the start pose -> open -> park

The orientation is the start pose's, rotated so the chosen tool axis points at
the table (`--tool-axis`, see `--inspect`), then yawed by `--yaw`. Holding the
start pose's orientation is the default because it is the one the arm is known
to reach from; a top-down grasp needs `--tool-axis z-` or whichever local axis
the URDF points out of the gripper.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sweep  # noqa: E402
import traj  # noqa: E402

MOTOR7_OPEN = 4.90
MOTOR7_CLOSED = -0.015
# Halfway between open and closed: the gesture the marker sequence makes on
# odd-numbered markers.
MOTOR7_HALF = (MOTOR7_OPEN + MOTOR7_CLOSED) / 2.0
TOOL_AXES = {
    "x+": (0, 1.0), "x-": (0, -1.0),
    "y+": (1, 1.0), "y-": (1, -1.0),
    "z+": (2, 1.0), "z-": (2, -1.0),
}


class PlanError(Exception):
    """The target cannot be planned; the message is meant for the operator."""


@dataclass
class Options:
    hz: float = 30.0
    approach: float = 0.06
    approach_s: float = 3.0
    descend_s: float = 1.5
    grasp_q: float = 2.45
    open_q: float = MOTOR7_OPEN
    grasp_s: float = 1.2
    hold_s: float = 0.6
    lift: float = 0.10
    lift_s: float = 1.5
    return_s: float = 3.0
    # Hover instead of pick: go to `hover_m` above the target and stop there (and
    # come back, because the board ends every move where it began). A tap that
    # only marks a place on the table wants this, not a grasp sequence.
    hover: bool = False
    hover_m: float = 0.10
    # Where the tool tip sits below `gripper_end`, in metres. The planner aims the
    # *frame*; the tip is this far along the tool axis, so the flange has to be
    # lifted by it for the tip to reach the requested height. Fixed mode can take
    # this from the rig; handheld has only a typed number.
    tip_m: float = 0.0
    tool_axis: str = "current"
    yaw: float = 0.0
    payload: float = 0.0
    min_z: float = 0.002
    max_z: float = 0.95
    max_reach: float = 0.75
    no_return: bool = False
    unpark_j2: float = 0.0
    unpark_j3: float = 8.0
    unpark_s: float = 2.0
    ik_pos_tol: float = 0.002
    ik_angle_tol: float = 0.5
    urdf: str = ""
    quiet: bool = False
    extra: dict = field(default_factory=dict)


def ease(x):
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, x)))


def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_between(a, b):
    """Smallest rotation taking unit vector `a` onto unit vector `b`."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    cross = np.cross(a, b)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(a @ b, -1.0, 1.0))
    if sine < 1e-9:
        if cosine > 0:
            return np.eye(3)
        # 180 deg: any axis perpendicular to `a` works; -I is not a rotation.
        helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, helper)
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    axis = cross / sine
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    angle = math.atan2(sine, cosine)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def work_orientation(R0, tool_axis, yaw_deg):
    """Rotate the start orientation so `tool_axis` points down, then yaw about base z."""
    if tool_axis == "current":
        return R0.copy()
    index, sign = TOOL_AXES[tool_axis]
    local = np.zeros(3)
    local[index] = sign
    approach = R0 @ local
    aligned = rotation_between(approach, np.array([0.0, 0.0, -1.0])) @ R0
    return rotz(yaw_deg) @ aligned


def blend_rotation(a, b, fraction):
    """Rotate from `a` towards `b` by `fraction` of the shortest arc."""
    m = a.T @ b
    angle = float(np.arccos(np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-9:
        return a.copy()
    axis = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    axis = axis / (2.0 * math.sin(angle))
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    step = angle * fraction
    return a @ (np.eye(3) + math.sin(step) * skew + (1.0 - math.cos(step)) * (skew @ skew))


def posture(dump):
    """Validate a `rebot-edge dump` payload and split it into arm and gripper."""
    ids = [int(i) for i in dump["ids"]]
    q = [float(v) for v in dump["q"]]
    if ids[:6] != [1, 2, 3, 4, 5, 6]:
        raise PlanError("expected motor ids 1..6 first, got %s" % (ids,))
    gripper = q[6] if len(q) >= 7 else MOTOR7_CLOSED
    return np.array(q[:6], dtype=float), float(gripper)


def validate_target_meta(target):
    if target.get("format") != "rebot-target":
        raise PlanError("not a rebot-target file (found %r)" % (target.get("format"),))
    point = target.get("point")
    if not isinstance(point, list) or len(point) != 3:
        raise PlanError("target 'point' must be [x, y, z]")
    return np.array([float(v) for v in point])


def inspect_frame(R0, p0):
    print("end-effector at the dumped pose: [%+.4f %+.4f %+.4f] m" % tuple(p0))
    print("orientation, per local axis:")
    for name, (index, sign) in TOOL_AXES.items():
        axis = R0 @ (np.eye(3)[index] * sign)
        angle = math.degrees(math.acos(max(-1.0, min(1.0, axis @ np.array([0.0, 0.0, -1.0])))))
        print("  %-3s -> [%+.3f %+.3f %+.3f] in base, %5.1f deg from straight down"
              % (name, axis[0], axis[1], axis[2], angle))
    print("\npick the axis that points at the table as --tool-axis, then replan")


def plan(dump, target, options: Options, log: Callable[[str], None] = print):
    """Build the trajectory payload for one target. Returns (payload, summary)."""
    model = load_robot_model(options.urdf or None)
    frame = get_end_effector_frame_id(model)
    n_arm = get_arm_joint_count(model, frame)
    if n_arm != 6:
        raise PlanError("expected 6 arm joints, model has %d" % n_arm)

    if options.payload > 0.0:
        tip = model.frames[frame]
        model.appendBodyToJoint(
            tip.parentJoint,
            pin.Inertia(options.payload, np.zeros(3), np.zeros((3, 3))),
            tip.placement,
        )

    q_start, gripper_start = posture(dump)
    lower = np.asarray(model.lowerPositionLimit[:n_arm], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:n_arm], dtype=float)
    clamped = np.clip(q_start, lower, upper)
    if not np.allclose(clamped, q_start):
        log("  start pose  : clamped to the joint limits: %s -> %s"
            % (np.round(q_start, 4), np.round(clamped, 4)))
        q_start = clamped
    q = pad_q_for_model(model, q_start, n_arm)
    _, R0, T0 = compute_fk(model, q)
    p0 = T0[:3, 3].copy()
    p_start = p0.copy()

    point = validate_target_meta(target)
    radius = float(np.linalg.norm(point[:2]))
    # With a hover the arm never descends to the table; judge the height it will
    # actually reach, or every tap on the worktop would be refused as "into it".
    judged = point + np.array([0.0, 0.0, float(options.hover_m) if options.hover else 0.0])
    if judged[2] < options.min_z:
        raise PlanError("target z %.4f is below --min-z %.4f: refusing to plan into the table"
                        % (judged[2], options.min_z))
    if judged[2] > options.max_z:
        raise PlanError("target z %.3f is above --max-z %.3f" % (judged[2], options.max_z))
    if radius > options.max_reach:
        raise PlanError("target is %.3f m from the base axis, above --max-reach %.2f"
                        % (radius, options.max_reach))

    R_work = work_orientation(R0, options.tool_axis, options.yaw)
    ik_params = IKParams(max_iter=300, tolerance=1e-6, damping=1e-8, step_size=1.0)
    worst = {"residual": 0.0, "where": ""}

    def ik_to(place, rotation, seed, label):
        result = solve_ik(model, model.createData(), frame,
                          pos_rot_to_se3(place, rotation), seed, ik_params,
                          controlled_joints=n_arm)
        solution = pad_q_for_model(model, result.q, n_arm) if result.q is not None else None

        # The solver's own success flag is stricter than the task needs: near a
        # reach boundary it can return a pose a fraction of a millimetre off and
        # still call it a failure. Judge the pose that was actually reached.
        position_error, angle_error = 1e9, 1e9
        if solution is not None:
            _, reached_rotation, reached = compute_fk(model, solution)
            position_error = float(np.linalg.norm(reached[:3, 3] - place))
            cosine = (np.trace(reached_rotation.T @ rotation) - 1.0) / 2.0
            angle_error = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

        accepted = (solution is not None
                    and (result.success
                         or (position_error <= options.ik_pos_tol
                             and angle_error <= options.ik_angle_tol)))
        if not accepted:
            raise PlanError(
                "IK failed at [%.3f %.3f %.3f] m (%s): %s; reached %.4f m / %.2f deg off. "
                "The target is probably out of reach from this pose: try a smaller --yaw, "
                "a larger --approach, or start from a pose closer to the workspace."
                % (place[0], place[1], place[2], label,
                   getattr(result, "message", "no solution") or "no solution",
                   position_error, angle_error))

        if position_error > worst["residual"]:
            worst["residual"] = position_error
            worst["where"] = label
        for index in range(n_arm):
            if solution[index] < lower[index] - traj.LIMIT_TOL or solution[index] > upper[index] + traj.LIMIT_TOL:
                raise PlanError("IK returned joint %d = %.3f rad at %s, outside its limits [%.3f, %.3f]"
                                % (index + 1, solution[index], label, lower[index], upper[index]))
        return solution

    period = 1.0 / options.hz
    waypoints = []
    state = {"t": 0.0}

    def append(q_model, gripper=None):
        if gripper is None:
            gripper = waypoints[-1]["q"][6] if waypoints else gripper_start
        gravity = compute_generalized_gravity(model=model, q=q_model)
        waypoints.append(traj.waypoint(
            state["t"], list(q_model[:n_arm]) + [float(gripper)],
            [float(v) for v in gravity[:n_arm]] + [0.0]))

    def joint_leg(base_q, delta, seconds, label):
        """Move in joint space, one stop at a time -- the peel from the guide.

        The parked pose rests on the j2/j3 lower stops, and a Cartesian IK that
        starts exactly on a stop often refuses to move at all. Lifting the elbow
        in joint space first is how plan_with_pinocchio.py gets off the stop.
        """
        base = np.array(base_q[:n_arm], dtype=float)
        count = max(1, int(round(seconds * options.hz)))
        for index in range(1, count + 1):
            state["t"] += period
            fraction = ease(index / count)
            target_q = base + np.array([delta.get(k, 0.0) for k in range(n_arm)]) * fraction
            append(pad_q_for_model(model, target_q, n_arm))
        if not options.quiet:
            log("  %-11s: %d waypoints" % (label, count))
        return pad_q_for_model(model, np.array(waypoints[-1]["q"][:n_arm], dtype=float), n_arm)

    def cartesian(start, end, rotation_from, rotation_to, seconds, blend_fraction, seed, label):
        count = max(1, int(round(seconds * options.hz)))
        for index in range(1, count + 1):
            state["t"] += period
            fraction = ease(index / count)
            place = start + (end - start) * fraction
            rotation = blend_rotation(rotation_from, rotation_to,
                                      min(1.0, fraction / max(1e-6, blend_fraction)))
            seed = ik_to(place, rotation, seed, "%s %d/%d" % (label, index, count))
            append(seed)
        return seed

    def gripper_leg(target_angle, seconds):
        count = max(1, int(round(seconds * options.hz)))
        current = waypoints[-1]["q"][6]
        for index in range(1, count + 1):
            state["t"] += period
            angle = current + (target_angle - current) * ease(index / count)
            waypoints.append(traj.waypoint(
                state["t"], waypoints[-1]["q"][:6] + [angle], waypoints[-1]["tau"]))

    def hold(seconds):
        count = max(1, int(round(seconds * options.hz)))
        for _ in range(count):
            state["t"] += period
            waypoints.append(traj.waypoint(state["t"], waypoints[-1]["q"], waypoints[-1]["tau"]))

    seed = q.copy()
    pre = point + np.array([0.0, 0.0, options.approach])
    unpark = {1: math.radians(options.unpark_j2), 2: math.radians(options.unpark_j3)}
    unparked = any(abs(value) > 1e-9 for value in unpark.values())
    if unparked:
        seed = joint_leg(q.copy(), unpark, options.unpark_s, "unpark")
        _, R0, T_unparked = compute_fk(model, seed)
        p0 = T_unparked[:3, 3].copy()
        R_work = work_orientation(R0, options.tool_axis, options.yaw)

    if options.hover:
        # One leg out, a hold, and the way back. The tap meant "this place": the
        # arm visits it and stops, with no descend, grip or lift.
        hover_point = point + np.array([0.0, 0.0, float(options.hover_m) + float(options.tip_m)])
        seed = cartesian(p0, hover_point, R0, R_work, options.approach_s, 0.4, seed, "hover")
        hold_start = state["t"]
        hold(options.hold_s)
        hold_end = state["t"]
        if not options.no_return:
            cartesian(hover_point, p0, R_work, R0, options.return_s, 0.4, seed, "return")
    else:
        seed = cartesian(p0, pre, R0, R_work, options.approach_s, 0.4, seed, "approach")
        seed = cartesian(pre, point, R_work, R_work, options.descend_s, 1.0, seed, "descend")
        gripper_leg(options.grasp_q, options.grasp_s)
        hold_start = state["t"]
        hold(options.hold_s)
        hold_end = state["t"]
        seed = cartesian(point, point + np.array([0.0, 0.0, options.lift]),
                         R_work, R_work, options.lift_s, 1.0, seed, "lift")
        if not options.no_return:
            seed = cartesian(point + np.array([0.0, 0.0, options.lift]), p0,
                             R_work, R0, options.return_s, 0.4, seed, "return")
            gripper_leg(options.open_q, options.grasp_s)
            if unparked:
                joint_leg(np.array(waypoints[-1]["q"][:n_arm], dtype=float),
                          {k: -v for k, v in unpark.items()}, options.unpark_s, "park")

    provenance = {
        "capture": target.get("capture"),
        "frame": target.get("frame"),
        "target_source": target.get("source"),
        "target_point": [float(v) for v in point],
        "target_tag_reproj_px": (target.get("tag") or {}).get("reproj_px"),
        "calib_fingerprint": target.get("calib_fingerprint"),
        # A fixed camera: which rig, how good, and whether its mount still held
        # at capture time. The tag path has `target_tag_reproj_px` instead.
        "rig": target.get("rig"),
        "witness": target.get("witness"),
    }
    if options.hover:
        steps = ("tap target: hover %.0f mm above [%.3f %.3f %.3f]%s"
                 % (options.hover_m * 1000, point[0], point[1], point[2],
                    "" if options.no_return else ", then return"))
    else:
        steps = ("tap target: approach %.0f mm above [%.3f %.3f %.3f], descend, grip %.2f rad, "
                 "lift %.0f mm%s" % (options.approach * 1000, point[0], point[1], point[2],
                                     options.grasp_q, options.lift * 1000,
                                     "" if options.no_return else ", return and open"))
    payload = traj.build(
        waypoints, options.hz,
        [[float(lo), float(hi)] for lo, hi in zip(lower, upper)] + [[-0.1, 5.0]],
        steps,
        extra={
            "capture": provenance,
            "tool_axis": options.tool_axis,
            "yaw_deg": options.yaw,
            "dynamics": {
                "source": "reBotArm_control_py.compute_generalized_gravity (Pinocchio RNEA)",
                "tau": "gravity torque of the planned pose, per waypoint",
            },
        },
        hold_start=hold_start if hold_end > hold_start else None,
        hold_end=hold_end if hold_end > hold_start else None,
    )

    stats = traj.validate(payload)
    summary = {
        "start_tcp": [float(v) for v in p_start],
        "unpark_tcp": [float(v) for v in p0] if unparked else None,
        "target": [float(v) for v in point],
        "radius": radius,
        "tool_axis": options.tool_axis,
        "yaw_deg": options.yaw,
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
        "worst_ik_m": worst["residual"],
        "worst_ik_at": worst["where"] or None,
        "peak_tau": stats["peak_tau"],
        "max_step_rad": stats["max_step_rad"],
        "unparked": unparked,
    }
    return payload, summary


def plan_sequence(dump, targets, options: Options, log: Callable[[str], None] = print):
    """Hover through a list of tapped points, in order, as one trajectory.

    This is the multi-marker form of the hover plan: each target is visited at
    `hover_m` above the table, the arm holds there while the gripper moves, and
    the pass finishes back at the pose it started from. The gripper starts fully
    open; it half-closes at odd-numbered markers and opens at even-numbered ones.
    The motion is the same cosine-eased Cartesian leg the single-tap plan uses,
    so the speeds and the edge's step checks are unchanged.
    """
    if not isinstance(targets, (list, tuple)) or not targets:
        raise PlanError("a marker sequence needs at least one target")
    if len(targets) > 20:
        raise PlanError("a marker sequence is limited to 20 targets")

    model = load_robot_model(options.urdf or None)
    frame = get_end_effector_frame_id(model)
    n_arm = get_arm_joint_count(model, frame)
    if n_arm != 6:
        raise PlanError("expected 6 arm joints, model has %d" % n_arm)

    if options.payload > 0.0:
        tip = model.frames[frame]
        model.appendBodyToJoint(
            tip.parentJoint,
            pin.Inertia(options.payload, np.zeros(3), np.zeros((3, 3))),
            tip.placement,
        )

    q_start, gripper_start = posture(dump)
    lower = np.asarray(model.lowerPositionLimit[:n_arm], dtype=float)
    upper = np.asarray(model.upperPositionLimit[:n_arm], dtype=float)
    q_start = np.clip(q_start, lower, upper)
    q = pad_q_for_model(model, q_start, n_arm)
    _, R0, T0 = compute_fk(model, q)
    p0 = T0[:3, 3].copy()
    p_start = p0.copy()

    # Validate everything before the first move: a bad marker halfway through a
    # sequence must not leave the arm somewhere unexpected.
    points = []
    for index, target in enumerate(targets, start=1):
        point = validate_target_meta(target)
        radius = float(np.linalg.norm(point[:2]))
        judged = point + np.array([0.0, 0.0, float(options.hover_m)])
        if judged[2] < options.min_z:
            raise PlanError("marker %d: target z %.4f is below --min-z %.4f"
                            % (index, judged[2], options.min_z))
        if judged[2] > options.max_z:
            raise PlanError("marker %d: target z %.3f is above --max-z %.3f"
                            % (index, judged[2], options.max_z))
        if radius > options.max_reach:
            raise PlanError("marker %d: target is %.3f m from the base axis, above "
                            "--max-reach %.2f" % (index, radius, options.max_reach))
        points.append(point)

    # With `--tool-axis current` the single-tap plan holds the start
    # orientation, including its yaw. A marker sequence can do better: the
    # gripper yaw follows the marker's direction from the base, which is the
    # free-yaw envelope the reach overlay draws. `z-` is the reference arm's
    # down axis at the parked pose; an explicit axis is used as given.
    sequence_axis = "z-" if options.tool_axis == "current" else options.tool_axis
    ik_params = IKParams(max_iter=300, tolerance=1e-6, damping=1e-8, step_size=1.0)
    worst = {"residual": 0.0, "where": ""}

    def ik_to(place, rotation, seed, label):
        result = solve_ik(model, model.createData(), frame,
                          pos_rot_to_se3(place, rotation), seed, ik_params,
                          controlled_joints=n_arm)
        solution = pad_q_for_model(model, result.q, n_arm) if result.q is not None else None

        position_error, angle_error = 1e9, 1e9
        if solution is not None:
            _, reached_rotation, reached = compute_fk(model, solution)
            position_error = float(np.linalg.norm(reached[:3, 3] - place))
            cosine = (np.trace(reached_rotation.T @ rotation) - 1.0) / 2.0
            angle_error = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

        accepted = (solution is not None
                    and (result.success
                         or (position_error <= options.ik_pos_tol
                             and angle_error <= options.ik_angle_tol)))
        if not accepted:
            raise PlanError(
                "IK failed at [%.3f %.3f %.3f] m (%s): %s; reached %.4f m / %.2f deg off. "
                "The marker is probably out of reach from this pose."
                % (place[0], place[1], place[2], label,
                   getattr(result, "message", "no solution") or "no solution",
                   position_error, angle_error))

        if position_error > worst["residual"]:
            worst["residual"] = position_error
            worst["where"] = label
        for index in range(n_arm):
            if solution[index] < lower[index] - traj.LIMIT_TOL or solution[index] > upper[index] + traj.LIMIT_TOL:
                raise PlanError("IK returned joint %d = %.3f rad at %s, outside its limits [%.3f, %.3f]"
                                % (index + 1, solution[index], label, lower[index], upper[index]))
        return solution

    period = 1.0 / options.hz
    waypoints = []
    state = {"t": 0.0}

    def append(q_model, gripper):
        gravity = compute_generalized_gravity(model=model, q=q_model)
        waypoints.append(traj.waypoint(
            state["t"], list(q_model[:n_arm]) + [float(gripper)],
            [float(v) for v in gravity[:n_arm]] + [0.0]))

    def hold(seconds, gripper):
        count = max(1, int(round(seconds * options.hz)))
        for _ in range(count):
            state["t"] += period
            append(pad_q_for_model(model, np.array(waypoints[-1]["q"][:n_arm], dtype=float),
                                   n_arm), gripper)

    def gripper_leg(target, seconds):
        count = max(1, int(round(seconds * options.hz)))
        current = waypoints[-1]["q"][6]
        for index in range(1, count + 1):
            state["t"] += period
            angle = current + (target - current) * ease(index / count)
            waypoints.append(traj.waypoint(
                state["t"], waypoints[-1]["q"][:6] + [angle], waypoints[-1]["tau"]))

    def joint_leg(base_q, delta, seconds, label, gripper):
        base = np.array(base_q[:n_arm], dtype=float)
        count = max(1, int(round(seconds * options.hz)))
        for index in range(1, count + 1):
            state["t"] += period
            fraction = ease(index / count)
            target_q = base + np.array([delta.get(k, 0.0) for k in range(n_arm)]) * fraction
            append(pad_q_for_model(model, target_q, n_arm), gripper)
        if not options.quiet:
            log("  %-11s: %d waypoints" % (label, count))
        return pad_q_for_model(model, np.array(waypoints[-1]["q"][:n_arm], dtype=float), n_arm)

    def cartesian(start, end, rotation_from, rotation_to, seconds, blend_fraction, seed,
                  label, gripper):
        count = max(1, int(round(seconds * options.hz)))
        for index in range(1, count + 1):
            state["t"] += period
            fraction = ease(index / count)
            place = start + (end - start) * fraction
            rotation = blend_rotation(rotation_from, rotation_to,
                                      min(1.0, fraction / max(1e-6, blend_fraction)))
            seed = ik_to(place, rotation, seed, "%s %d/%d" % (label, index, count))
            append(seed, gripper)
        return seed

    # The edge starts from the arm's current gripper angle, so a closed gripper
    # would jump open on the first waypoint. Hold still and ramp it open first.
    gripper_start = float(gripper_start)
    if abs(gripper_start - MOTOR7_OPEN) > 1e-6:
        state["t"] += period
        append(q.copy(), gripper_start)
        gripper_leg(MOTOR7_OPEN, max(0.8, options.hold_s))
    gripper = MOTOR7_OPEN

    seed = q.copy()
    unpark = {1: math.radians(options.unpark_j2), 2: math.radians(options.unpark_j3)}
    unparked = any(abs(value) > 1e-9 for value in unpark.values())
    if unparked:
        seed = joint_leg(q.copy(), unpark, options.unpark_s, "unpark", gripper)
        _, R0, T_unparked = compute_fk(model, seed)
        p0 = T_unparked[:3, 3].copy()
    start_seed = seed.copy()

    current_place = p0.copy()
    current_rotation = R0.copy()
    yaw_degrees = []
    for index, point in enumerate(points, start=1):
        place = point + np.array([0.0, 0.0, float(options.hover_m) + float(options.tip_m)])
        yaw_deg = math.degrees(math.atan2(float(point[1]), float(point[0])))
        yaw_degrees.append(round(yaw_deg, 3))
        R_marker = work_orientation(R0, sequence_axis, yaw_deg)
        if index == 1:
            # The first approach is the one the single-tap plan is known to
            # handle: from the arm's pose to the first marker.
            seed = cartesian(current_place, place, current_rotation, R_marker,
                             options.approach_s, 0.4, seed, "marker 1", gripper)
        else:
            # Between markers, a straight Cartesian line can leave the annulus
            # the arm can reach with the tool down. Solve the marker pose, then
            # interpolate joint space; the capsule check still guards the table.
            try:
                target = ik_to(place, R_marker, seed, "marker %d" % index)
            except PlanError:
                target = ik_to(place, R_marker, start_seed, "marker %d" % index)
            current_q = np.array(waypoints[-1]["q"][:n_arm], dtype=float)
            if not sweep.path_safe(current_q, target[:n_arm]):
                raise PlanError(
                    "the leg from marker %d to marker %d dips within a link radius "
                    "of the worktop; space the markers further apart or move the arm "
                    "clear" % (index - 1, index))
            seed = joint_leg(
                current_q,
                {joint: target[joint] - current_q[joint] for joint in range(n_arm)},
                options.approach_s, "marker %d" % index, gripper)
        hold(0.25, gripper)
        next_gripper = MOTOR7_HALF if index % 2 == 1 else MOTOR7_OPEN
        if abs(next_gripper - gripper) > 1e-6:
            gripper_leg(next_gripper, max(0.8, options.hold_s))
        gripper = next_gripper
        hold(0.25, gripper)
        current_place = place
        current_rotation = R_marker

    # Return in joint space. A Cartesian return to the parked pose is the one
    # leg where IK can fail on the last millimetres (the parked pose sits on
    # two stops), and a return is not a precision move. The capsule check still
    # refuses a leg that would cross the worktop, exactly like /move does.
    q_last = np.array(waypoints[-1]["q"][:n_arm], dtype=float)
    q_home = np.array(q_start[:n_arm], dtype=float)
    if not sweep.path_safe(q_last, q_home):
        raise PlanError("the return from marker %d to the start pose dips within a "
                        "link radius of the worktop; move the arm clear and retry"
                        % len(points))
    joint_leg(q_last, {index: q_home[index] - q_last[index] for index in range(n_arm)},
              options.return_s, "return", gripper)

    steps = ("%d markers: hover each in order; gripper open, half-close on odd "
             "markers, open on even; return" % len(points))
    provenance = {
        "sequence": True,
        "targets": [
            {"point": [float(v) for v in point],
             "capture": targets[index].get("capture"),
             "source": targets[index].get("source"),
             "tap": targets[index].get("tap")}
            for index, point in enumerate(points)
        ],
        "tool_axis": sequence_axis,
        "yaw_deg": options.yaw,
        "marker_yaw_deg": yaw_degrees,
        "gripper": {"open": MOTOR7_OPEN, "half": MOTOR7_HALF,
                    "closed": MOTOR7_CLOSED},
    }
    payload = traj.build(
        waypoints, options.hz,
        [[float(lo), float(hi)] for lo, hi in zip(lower, upper)] + [[-0.1, 5.0]],
        steps,
        extra={
            "capture": provenance,
            "tool_axis": sequence_axis,
            "yaw_deg": options.yaw,
            "marker_yaw_deg": yaw_degrees,
            "dynamics": {
                "source": "reBotArm_control_py.compute_generalized_gravity (Pinocchio RNEA)",
                "tau": "gravity torque of the planned pose, per waypoint",
            },
        },
    )
    stats = traj.validate(payload)
    summary = {
        "markers": len(points),
        "targets": [[float(v) for v in point] for point in points],
        "radii": [float(np.linalg.norm(point[:2])) for point in points],
        "start_tcp": [float(v) for v in p_start],
        "waypoints": stats["waypoints"],
        "duration_s": stats["duration_s"],
        "worst_ik_m": worst["residual"],
        "worst_ik_at": worst["where"] or None,
        "peak_tau": stats["peak_tau"],
        "max_step_rad": stats["max_step_rad"],
        "unparked": unparked,
    }
    return payload, summary


def describe_sequence(summary):
    lines = ["Marker sequence: %d markers, gripper open -> half/odd -> open/even"
             % summary["markers"]]
    for index, (point, radius) in enumerate(zip(summary["targets"], summary["radii"]),
                                            start=1):
        lines.append("  marker %2d : [%+.4f %+.4f %+.4f] m (%.3f m from the base axis)"
                     % (index, point[0], point[1], point[2], radius))
    lines += [
        "  waypoints  : %d over %.2f s" % (summary["waypoints"], summary["duration_s"]),
        "  worst IK   : %.4f m at %s" % (summary["worst_ik_m"],
                                          summary["worst_ik_at"] or "-"),
        "  peak |tau| : %.2f N.m" % summary["peak_tau"],
        "  max step   : %.3f rad" % summary["max_step_rad"],
    ]
    return "\n".join(lines)


def describe(summary):
    lines = ["Pinocchio plan for a tapped target",
             "  start tcp   : [%+.4f %+.4f %+.4f] m" % tuple(summary["start_tcp"])]
    if summary["unparked"]:
        lines.append("  unpark tcp  : [%+.4f %+.4f %+.4f] m" % tuple(summary["unpark_tcp"]))
    lines += [
        "  target      : [%+.4f %+.4f %+.4f] m (%.3f m from the base axis)"
        % (*summary["target"], summary["radius"]),
        "  orientation : %s%s" % (summary["tool_axis"],
                                  "" if summary["yaw_deg"] == 0 else
                                  ", yawed %+.0f deg" % summary["yaw_deg"]),
        "  waypoints   : %d over %.2f s" % (summary["waypoints"], summary["duration_s"]),
        "  worst IK    : %.4f m at %s" % (summary["worst_ik_m"], summary["worst_ik_at"] or "-"),
        "  peak |tau|  : %.2f N.m" % summary["peak_tau"],
        "  max step    : %.3f rad" % summary["max_step_rad"],
    ]
    return "\n".join(lines)


def add_arguments(parser):
    parser.add_argument("--joints", required=True, help="rebot-edge dump output")
    parser.add_argument("--target", default="", help="target.json from locate_target.py")
    parser.add_argument("--out", default="", help="trajectory JSON to write")
    parser.add_argument("--urdf", default="", help="override the URDF path")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--approach", type=float, default=0.06,
                        help="pre-grasp height above the target, m")
    parser.add_argument("--approach-s", type=float, default=3.0)
    parser.add_argument("--descend-s", type=float, default=1.5)
    parser.add_argument("--grasp-q", type=float, default=2.45,
                        help="motor 7 angle after closing, rad (6 cm is 2.46)")
    parser.add_argument("--open-q", type=float, default=MOTOR7_OPEN)
    parser.add_argument("--grasp-s", type=float, default=1.2)
    parser.add_argument("--hold-s", type=float, default=0.6)
    parser.add_argument("--lift", type=float, default=0.10, help="lift height after closing, m")
    parser.add_argument("--lift-s", type=float, default=1.5)
    parser.add_argument("--return-s", type=float, default=3.0)
    parser.add_argument("--tool-axis", default="current", choices=sorted(TOOL_AXES) + ["current"],
                        help="local axis that should point at the table (see --inspect)")
    parser.add_argument("--yaw", type=float, default=0.0, help="rotation about base z, deg")
    parser.add_argument("--unpark-j2", type=float, default=0.0,
                        help="joint-space shoulder lift before the Cartesian move, deg")
    parser.add_argument("--unpark-j3", type=float, default=8.0,
                        help="joint-space elbow lift before the Cartesian move, deg "
                             "(the guide's peel; the parked pose sits on this stop)")
    parser.add_argument("--unpark-s", type=float, default=2.0)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--ik-pos-tol", type=float, default=0.002,
                        help="accept an IK result that lands within this distance, m "
                             "(the arm's own repeatability is ~23 mm)")
    parser.add_argument("--ik-angle-tol", type=float, default=0.5,
                        help="and within this orientation error, deg")
    parser.add_argument("--payload", type=float, default=0.0, help="held mass at the tool frame, kg")
    parser.add_argument("--min-z", type=float, default=0.002)
    parser.add_argument("--max-z", type=float, default=0.95)
    parser.add_argument("--max-reach", type=float, default=0.75)
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--inspect", action="store_true", help="print the tool axes and exit")


def options_from_args(args) -> Options:
    return Options(
        hz=args.hz, approach=args.approach, approach_s=args.approach_s,
        descend_s=args.descend_s, grasp_q=args.grasp_q, open_q=args.open_q,
        grasp_s=args.grasp_s, hold_s=args.hold_s, lift=args.lift, lift_s=args.lift_s,
        return_s=args.return_s, tool_axis=args.tool_axis, yaw=args.yaw,
        payload=args.payload, min_z=args.min_z, max_z=args.max_z, max_reach=args.max_reach,
        no_return=args.no_return, unpark_j2=args.unpark_j2, unpark_j3=args.unpark_j3,
        unpark_s=args.unpark_s, ik_pos_tol=args.ik_pos_tol, ik_angle_tol=args.ik_angle_tol,
        urdf=args.urdf, quiet=args.quiet,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description="Plan to a tapped target.")
    add_arguments(parser)
    args = parser.parse_args(argv)

    if args.inspect:
        model = load_robot_model(args.urdf or None)
        frame = get_end_effector_frame_id(model)
        n_arm = get_arm_joint_count(model, frame)
        with open(args.joints, "r", encoding="utf-8") as handle:
            dump = json.load(handle)
        q_start, _ = posture(dump)
        lower = np.asarray(model.lowerPositionLimit[:n_arm], dtype=float)
        upper = np.asarray(model.upperPositionLimit[:n_arm], dtype=float)
        q = pad_q_for_model(model, np.clip(q_start, lower, upper), n_arm)
        _, R0, T0 = compute_fk(model, q)
        inspect_frame(R0, T0[:3, 3])
        return 0

    if not args.target:
        raise SystemExit("--target is required (tap in the viewer, or run locate_target.py)")

    with open(args.joints, "r", encoding="utf-8") as handle:
        dump = json.load(handle)
    with open(args.target, "r", encoding="utf-8") as handle:
        target = json.load(handle)

    try:
        payload, summary = plan(dump, target, options_from_args(args))
    except PlanError as error:
        raise SystemExit(str(error)) from error

    print(describe(summary))

    if args.print_only:
        print("  (--print-only: not written)")
        return 0
    if not args.out:
        raise SystemExit("--out is required to write the trajectory")

    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, separators=(",", ":"))
    print("  wrote       : %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
