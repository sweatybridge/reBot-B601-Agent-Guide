#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inverse kinematics for the reBot B601-RS, pure numpy.

The arm has six revolute joints and a *non-spherical* wrist (the last three axes
do not intersect: the joint origins are 104 mm and 60 mm apart), so there is no
tidy closed form. This module does forward kinematics exactly and solves IK with
damped least squares, respecting the URDF joint limits.

Chain transcribed from Seeed's URDF (no meshes needed, all six joints + the
fixed gripper frame):

    src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf
    https://github.com/Seeed-Projects/reBotArmController_ROS2

Conventions
-----------
* Lengths in metres, angles in radians, right-handed, `base_link` frame.
* `q` is a length-6 vector in motor order: motor 1 -> joint1 ... motor 6 ->
  joint6. Motor 7 drives the gripper and is not part of this chain.
* FK returns the `gripper_end` link, i.e. the tool frame, not `link6`.

Why not Pinocchio: it has no armv6l/armv7l builds, and this robot needs a 6x6
solve, which numpy does instantly.

Gravity helpers (`gravity_torques`, `potential_energy`, `com_positions`)
-----------------------------------------------------------------------
These are **diagnostics only — nothing in the control path calls them.** They
are validated against the potential-energy gradient to 3.5e-9 Nm, but do not
feed them to the motors without reading this: on the reference arm the measured
droop at the top of a 10 cm lift implies ~1.97 Nm on joint 4 (model: 1.67 Nm,
good agreement) yet only ~0.10 Nm and ~0.74 Nm on joints 2 and 3 (model: 1.75
and 5.81 Nm). The upper arm is partly supported at that pose, so injecting the
full model torque there would drive the arm up against whatever holds it.
`torque_ff` in `write_operation_frame` is the place it would go if you calibrate
it per joint, per pose, first.
"""

import numpy as np

URDF_URL = ("https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/main/"
            "src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf")

# (name, origin xyz, origin rpy, axis, lower, upper) straight out of the URDF.
CHAIN = (
    ("joint1", (-0.00034283, -0.00098683, 0.075), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), -2.80, 2.80),
    ("joint2", (0.020343, 0.027237, 0.07), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0), 0.00, 3.14),
    ("joint3", (-0.236, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 0.00, 3.14),
    ("joint4", (0.228, -0.072746, 0.0045), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), -1.57, 1.57),
    ("joint5", (0.087, -0.048, -0.03075), (-1.5708, 0.0, 0.0), (0.0, 0.0, -1.0), -1.57, 1.57),
    ("joint6", (0.0365, 0.0, 0.048), (0.0, 1.5708, 0.0), (0.0, 0.0, -1.0), -3.14, 3.14),
)

# Fixed joint j_gripper_end: link6 -> gripper_end.
TOOL_XYZ = (0.0, 0.0, 0.16621)
TOOL_RPY = (3.1416, -1.5708, 0.0)

LIMITS = np.array([(lo, hi) for _n, _x, _r, _a, lo, hi in CHAIN])
JOINT_NAMES = [row[0] for row in CHAIN]
DOF = len(CHAIN)

GRAVITY = 9.80665

# Link inertials from the same URDF: (link, mass kg, centre of mass in that link
# frame). Used for gravity feedforward; the base does not move and is skipped.
# The gripper fingers (0.0752 kg each) are omitted - they sit ~10 cm from the
# wrist and contribute well under 0.1 Nm.
LINK_INERTIALS = (
    ("link1", 0.4655, (0.00817652, 0.00122584, 0.03314197)),
    ("link2", 1.5520, (-0.11786481, -0.00342226, -0.02715519)),
    ("link3", 1.2520, (0.10342715, -0.06641379, -0.02710687)),
    ("link4", 0.4600, (0.04755445, -0.04411135, -0.03141119)),
    ("link5", 0.2012, (0.01144006, 0.00000479, 0.03804875)),
    ("link6", 0.1000, (0.00001802, 0.00000002, 0.00434806)),
    ("gripper_end", 0.6500, (-0.11275169, 0.00012844, -0.00000573)),
)


# --------------------------------------------------------------------------- #
# Small SO(3)/SE(3) helpers
# --------------------------------------------------------------------------- #
def rpy_to_matrix(rpy):
    """URDF fixed-axis roll/pitch/yaw (R = Rz(yaw) @ Ry(pitch) @ Rx(roll))."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, angle):
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    t = 1.0 - c
    return np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def transform(rotation=None, translation=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def rotation_error(r_current, r_target):
    """Axis-angle of (R_target @ R_current^T), as a 3-vector."""
    r_err = r_target @ r_current.T
    cos_angle = np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([
        r_err[2, 1] - r_err[1, 2],
        r_err[0, 2] - r_err[2, 0],
        r_err[1, 0] - r_err[0, 1],
    ]) / (2.0 * np.sin(angle))
    return axis * angle


# --------------------------------------------------------------------------- #
# Kinematics
# --------------------------------------------------------------------------- #
def forward(q, with_frames=False, with_links=False):
    """Tool pose for joint vector `q`; optionally also the joint frames."""
    q = np.asarray(q, dtype=float)
    if q.shape != (DOF,):
        raise ValueError("expected %d joint angles, got %s" % (DOF, q.shape))

    t = np.eye(4)
    frames = []
    links = []
    for (name, xyz, rpy, axis, lo, hi), angle in zip(CHAIN, q):
        t = t @ transform(rpy_to_matrix(rpy), xyz)
        p = t[:3, 3].copy()
        z = t[:3, :3] @ np.asarray(axis, dtype=float)
        frames.append((p, z))
        t = t @ transform(axis_angle(axis, angle))
        links.append(t.copy())          # after joint i -> link i frame

    t = t @ transform(rpy_to_matrix(TOOL_RPY), TOOL_XYZ)
    links.append(t.copy())              # gripper_end

    if with_links and with_frames:
        return t, frames, links
    if with_links:
        return t, links
    return (t, frames) if with_frames else t


def com_positions(q):
    """World position of every moving link's centre of mass."""
    _tool, links = forward(q, with_links=True)
    points = []
    for index, (_name, mass, com) in enumerate(LINK_INERTIALS):
        frame = links[index]
        p = frame[:3, :3] @ np.asarray(com, dtype=float) + frame[:3, 3]
        points.append((mass, p))
    return points


def potential_energy(q, gravity=GRAVITY):
    """Gravitational potential of the moving links (only its gradient matters)."""
    return sum(mass * gravity * p[2] for mass, p in com_positions(q))


def gravity_torques(q, gravity=GRAVITY):
    """Joint torques needed to hold this pose against gravity.

    tau_g = dU/dq: the motor torque required to stay still, in the same sign
    convention as the joint angles (so it can be fed straight into the MIT
    frame's torque field).
    """
    _tool, frames = forward(q, with_frames=True)
    coms = com_positions(q)
    tau = np.zeros(DOF)
    for j, (p_j, z_j) in enumerate(frames):
        total = 0.0
        # Only links downstream of joint j can be moved by it; anything upstream
        # contributes nothing (and would otherwise add a spurious cross product).
        for index in range(j, len(coms)):
            mass, p_com = coms[index]
            force = np.array([0.0, 0.0, -mass * gravity])
            total += z_j @ np.cross(p_com - p_j, force)
        tau[j] = -total                   # -sum(...) == dU/dq
    return tau


def jacobian(q):
    """Geometric Jacobian (6x6): rows 0-2 linear, 3-5 angular."""
    tool, frames = forward(q, with_frames=True)
    return _jacobian_from_frames(frames, tool[:3, 3])


def _jacobian_from_frames(frames, p_tool):
    j = np.zeros((6, DOF))
    for i, (p_i, z_i) in enumerate(frames):
        j[:3, i] = np.cross(z_i, p_tool - p_i)
        j[3:, i] = z_i
    return j


def pose(position, rotation=None):
    return transform(rotation, position)


def within_limits(q, margin=0.0):
    return bool(np.all(q >= LIMITS[:, 0] + margin) and np.all(q <= LIMITS[:, 1] - margin))


def ik(target, seed, tol_pos=1e-4, tol_rot=1e-3, max_iters=200, damping=0.02,
       max_step=0.15, rot_weight=0.5, margin=0.02, anchor=None):
    """Damped-least-squares IK.

    Returns (q, info) where info reports the residual and whether the solve
    converged. Never returns a solution outside the joint limits: the caller can
    trust the returned `q` or check `info["ok"]` first.

    `rot_weight` balances radians against metres; 0.5 means 1 rad of orientation
    error counts like 0.5 m of position error.
    """
    seed = np.asarray(seed, dtype=float)
    if seed.shape != (DOF,):
        raise ValueError("expected %d joint angles, got %s" % (DOF, seed.shape))
    seed = np.clip(seed, LIMITS[:, 0], LIMITS[:, 1])

    # The margin keeps the solution away from the stops, but it must never push
    # a joint that already sits inside the band: a joint resting at 0.005 rad
    # against a 0 rad limit would otherwise be forced to 0.02 and the solve
    # would chase an unreachable target.
    #
    # `anchor` is the pose the plan started from. Widening the band to include
    # it means a return leg can always get back to where it began, instead of
    # stalling a millimetre short because the margin cut the last few mrad off.
    low = np.minimum(LIMITS[:, 0] + margin, seed)
    high = np.maximum(LIMITS[:, 1] - margin, seed)
    if anchor is not None:
        anchor = np.clip(np.asarray(anchor, dtype=float), LIMITS[:, 0], LIMITS[:, 1])
        low = np.minimum(low, anchor)
        high = np.maximum(high, anchor)

    q = seed.copy()
    target_p = target[:3, 3]
    target_r = target[:3, :3]

    err_norm = np.inf
    for iteration in range(max_iters):
        current, frames = forward(q, with_frames=True)
        pos_err = target_p - current[:3, 3]
        rot_err = rotation_error(current[:3, :3], target_r)
        err_vec = np.concatenate([pos_err, rot_weight * rot_err])
        err_norm = float(np.linalg.norm(pos_err) + rot_weight * np.linalg.norm(rot_err))

        if np.linalg.norm(pos_err) < tol_pos and np.linalg.norm(rot_err) < tol_rot:
            return q, {"ok": True, "iters": iteration, "pos_err": float(np.linalg.norm(pos_err)),
                       "rot_err": float(np.linalg.norm(rot_err))}

        j = _jacobian_from_frames(frames, current[:3, 3])
        # Weight the angular rows so the two blocks are comparable.
        jw = j.copy()
        jw[3:, :] *= rot_weight

        try:
            step = jw.T @ np.linalg.solve(jw @ jw.T + (damping ** 2) * np.eye(6), err_vec)
        except np.linalg.LinAlgError:
            return q, {"ok": False, "iters": iteration, "reason": "singular",
                       "pos_err": float(np.linalg.norm(pos_err)),
                       "rot_err": float(np.linalg.norm(rot_err))}

        largest = float(np.max(np.abs(step)))
        if largest > max_step:
            step *= max_step / largest

        previous = err_norm
        for _attempt in range(6):                 # back off if the step made it worse
            candidate = np.clip(q + step, low, high)
            trial = forward(candidate)
            trial_err = float(np.linalg.norm(target_p - trial[:3, 3])
                              + rot_weight * np.linalg.norm(rotation_error(trial[:3, :3], target_r)))
            if trial_err < previous or _attempt == 5:
                q = candidate
                break
            step *= 0.5

    current = forward(q)
    return q, {
        "ok": False,
        "iters": max_iters,
        "reason": "max iterations",
        "pos_err": float(np.linalg.norm(target_p - current[:3, 3])),
        "rot_err": float(np.linalg.norm(rotation_error(current[:3, :3], target_r))),
    }


def translation_target(current_pose, delta_xyz, keep_orientation=True):
    """Target pose = current pose shifted by `delta_xyz` in the base frame."""
    target = np.array(current_pose, dtype=float, copy=True)
    target[:3, 3] = target[:3, 3] + np.asarray(delta_xyz, dtype=float)
    if not keep_orientation:
        target[:3, :3] = np.eye(3)
    return target
