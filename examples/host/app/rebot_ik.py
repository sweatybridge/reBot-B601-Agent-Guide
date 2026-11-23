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
`gravity_torques` is the model the gravity demo feeds to the motors; the other
two are diagnostics. It agrees with the potential-energy gradient to 4e-9 Nm.
On the reference arm a hold run at the top of a 10 cm lift measured 0.86 / 7.46
/ 1.85 Nm on joints 2-4 against 1.40 / 6.22 / 1.65 from this model - close
enough to be useful, not close enough to skip a per-pose scale.

One thing to get right before comparing: the MIT frame scales kp and torque by
the *motor model* (RS-06 on joints 1-3, RS-00 on 4-6), and sending rs-00
everywhere makes a kp * droop estimate ten times too small. The edge scripts
take `--model-spec` for this and default to that split. `torque_ff` in
`write_operation_frame` is where the model torque goes.
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

# Per-link capsule radii for the worktop clearance gate (sweep.path_safe). Single
# source: sweep.RADII and witness.LINK_RADII alias this (were three copies).
LINK_RADII = (0.055, 0.055, 0.050, 0.045, 0.035, 0.030, 0.020)
CLEARANCE = 0.012          # m above the worktop for a link's outer surface
LINK_RADIUS_MAX = 0.85     # any link centre, for the interpolation check

# Gripper (motor 7) calibration. Single source: plan_to_target / plan_with_pinocchio
# alias MOTOR7_OPEN/CLOSED/GRIPPER_RAD_PER_MM from this (were three copies).
GRIPPER = {"motor7_open": 4.90, "motor7_closed": -0.015, "rad_per_mm": 0.0412}

# Motion tuning (traj/sweep speed policy + gates). Single source: traj.MAX_JOINT_STEP/
# LIMIT_TOL/RAMP_S and sweep.DEFAULT_MIN_Z/PEAK_SPEED/MIN_CRUISE_SPEED/MIN_SECONDS/
# MAX_SECONDS/GRIPPER_PEAK_SPEED/SAG_LIMIT_RAD alias this. Matches the Rust cores'
# motion constants (rebot_arm_config) and the world.arm_config seed bit-for-bit.
MOTION = {
    "default_min_z": 0.04, "sag_limit_rad": 0.15, "limit_tol": 0.002, "max_joint_step": 0.5,
    "ramp_s": 0.15, "min_cruise_speed": 0.25, "min_seconds": 0.3, "max_seconds": 25.0,
    "peak_speed": 0.5, "gripper_peak_speed": 3.0,
}

# Hand-eye fit sanity gates (extrinsics.MIN_POSES/MAX_CAMERA_M/MAX_TIP_M alias this;
# matches calib.rs and the world.arm_config calibration group bit-for-bit).
CALIBRATION = {"min_poses": 6, "max_camera_m": 5.0, "max_tip_m": 0.20}

# Oracle attribution thresholds (oracle.PIXEL_ERROR_MM etc. alias this; matches
# oracle_attribute's required config arg and the world.arm_config oracle group
# bit-for-bit). CMD_TOL/HOLD_TIME_TOL stay in oracle.py (numeric epsilons).
ORACLE = {
    "pixel_error_mm": 10.0, "tip_error_mm": 8.0, "hold_error_rad": 0.030,
    "witness_drift_px": 2.0, "witness_scale_tol": 0.05, "sag_correlation": 0.8,
    "tip_vector_coherence": 0.8, "zero_bias_spread_rad": 0.005, "reach_fraction": 0.9,
    "placement_xy_mm": 10.0, "placement_z_tol_mm": 5.0, "ik_residual_mm": 10.0,
}

# Learning thresholds (identify.RIG_MAX_AGE_DAYS etc. alias this; matches
# rebot_fit_sag_map / accuracy_cell_update's required config arg and the
# world.arm_config learning group bit-for-bit). Arrays stay lists; the count
# gates stay ints.
LEARNING = {
    "rig_max_age_days": 30.0, "tag_growth_px": 2.0, "ewma_alpha": 0.3,
    "marker_min_usable": 3, "marker_spread_max_mm": 6.0, "holdout_max_px": 3.0,
    "tag_prior_depth": 0.01, "sag_min_holds": 30, "sag_huber_rad": 0.010,
    "sag_validate_max_mrad": 15.0, "sag_validate_ratio": 2.0,
    "sag_span_radius_m": 0.10, "sag_span_height_m": 0.05, "accuracy_min_n": 3,
    "accuracy_radius_edges": [0.10, 0.20, 0.30, 0.40, 0.50, 0.60],
    "accuracy_height_edges": [0.00, 0.10, 0.20, 0.30, 0.40], "accuracy_sectors": 8,
    "coverage_max_poses": 12, "coverage_hold_s": 2.0, "coverage_perturbations": 24,
    "coverage_max_reach_m": 0.75,
}
_LEARNING_INT_KEYS = ("marker_min_usable", "sag_min_holds", "accuracy_min_n",
                      "accuracy_sectors", "coverage_max_poses", "coverage_perturbations")
_LEARNING_ARRAY_KEYS = ("accuracy_radius_edges", "accuracy_height_edges")


def _learning_doc(src):
    """Coerce a learning group to the canonical doc shape (ints/floats/lists)."""
    out = {}
    for k, v in src.items():
        if k in _LEARNING_INT_KEYS:
            out[k] = int(v)
        elif k in _LEARNING_ARRAY_KEYS:
            out[k] = [float(x) for x in v]
        else:
            out[k] = float(v)
    return out


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


# --------------------------------------------------------------------------- #
# Canonical arm config (world.arm_config). rebot_ik is the single Python source
# for the physical-model constants; the table is the system of record in PG mode.
# `motion` tuning lives in traj/sweep and is a separate consolidation group.
# --------------------------------------------------------------------------- #
def default_config():
    """The physical-model config built from the current constants (file-mode default)."""
    return {
        "gravity": float(GRAVITY),
        "limits": [[float(lo), float(hi)] for lo, hi in LIMITS],
        "link_inertials": [[n, float(m), [float(c) for c in com]] for n, m, com in LINK_INERTIALS],
        "link_radii": [float(r) for r in LINK_RADII],
        "tool": {"xyz": [float(v) for v in TOOL_XYZ], "rpy": [float(v) for v in TOOL_RPY]},
        "gripper": {k: float(v) for k, v in GRIPPER.items()},
        "collision": {"clearance": float(CLEARANCE), "link_radius_max": float(LINK_RADIUS_MAX)},
        "motion": {k: float(v) for k, v in MOTION.items()},
        "calibration": {"min_poses": int(CALIBRATION["min_poses"]),
                        "max_camera_m": float(CALIBRATION["max_camera_m"]),
                        "max_tip_m": float(CALIBRATION["max_tip_m"])},
        "oracle": {k: float(v) for k, v in ORACLE.items()},
        "learning": _learning_doc(LEARNING),
    }


def apply_config(cfg):
    """Override the physical-model + motion constants from a world.arm_config doc, and
    re-sync the modules that alias them (sweep/witness radii, planner gripper, traj/
    sweep motion). The seed is bit-identical to the defaults, so this is a no-op until
    the table is edited."""
    global LIMITS, LINK_INERTIALS, TOOL_XYZ, TOOL_RPY, GRAVITY
    global LINK_RADII, CLEARANCE, LINK_RADIUS_MAX, GRIPPER, CHAIN, MOTION, CALIBRATION, ORACLE, LEARNING
    import sys
    if "limits" in cfg:
        LIMITS = np.asarray(cfg["limits"], dtype=float)
        CHAIN = tuple((n, x, r, a, float(lo), float(hi))
                      for (n, x, r, a, _lo, _hi), (lo, hi) in zip(CHAIN, cfg["limits"]))
    if "link_inertials" in cfg:
        LINK_INERTIALS = tuple((n, float(m), tuple(float(c) for c in com))
                               for n, m, com in cfg["link_inertials"])
    if "tool" in cfg:
        TOOL_XYZ = tuple(float(v) for v in cfg["tool"]["xyz"])
        TOOL_RPY = tuple(float(v) for v in cfg["tool"]["rpy"])
    if "gravity" in cfg:
        GRAVITY = float(cfg["gravity"])
    if "link_radii" in cfg:
        LINK_RADII = tuple(float(v) for v in cfg["link_radii"])
    if "collision" in cfg:
        CLEARANCE = float(cfg["collision"]["clearance"])
        LINK_RADIUS_MAX = float(cfg["collision"]["link_radius_max"])
    if "gripper" in cfg:
        GRIPPER = {k: float(v) for k, v in cfg["gripper"].items()}
    if "motion" in cfg:
        MOTION = {k: float(v) for k, v in cfg["motion"].items()}
    if "calibration" in cfg:
        CALIBRATION = {"min_poses": int(cfg["calibration"]["min_poses"]),
                       "max_camera_m": float(cfg["calibration"]["max_camera_m"]),
                       "max_tip_m": float(cfg["calibration"]["max_tip_m"])}
    if "oracle" in cfg:
        ORACLE = {k: float(v) for k, v in cfg["oracle"].items()}
    if "learning" in cfg:
        LEARNING = _learning_doc(cfg["learning"])
    # Re-sync aliases bound at import time in already-loaded consumers.
    for name in ("sweep", "witness"):
        mod = sys.modules.get(name)
        if mod is not None:
            if hasattr(mod, "RADII"):
                mod.RADII = LINK_RADII
            if hasattr(mod, "LINK_RADII"):
                mod.LINK_RADII = LINK_RADII
            if hasattr(mod, "CLEARANCE"):
                mod.CLEARANCE = CLEARANCE
            if hasattr(mod, "LINK_RADIUS_MAX"):
                mod.LINK_RADIUS_MAX = LINK_RADIUS_MAX
    for name in ("plan_to_target", "plan_with_pinocchio"):
        mod = sys.modules.get(name)
        if mod is not None and hasattr(mod, "MOTOR7_OPEN"):
            mod.MOTOR7_OPEN = GRIPPER["motor7_open"]
            mod.MOTOR7_CLOSED = GRIPPER["motor7_closed"]
            mod.GRIPPER_RAD_PER_MM = GRIPPER["rad_per_mm"]
    # Motion tuning aliases (traj/sweep). Their default-arg functions resolve these
    # globals at call time, so re-syncing here propagates a table edit.
    for name, keys in (
        ("traj", {"MAX_JOINT_STEP": "max_joint_step", "LIMIT_TOL": "limit_tol", "RAMP_S": "ramp_s"}),
        ("sweep", {"DEFAULT_MIN_Z": "default_min_z", "PEAK_SPEED": "peak_speed",
                   "MIN_CRUISE_SPEED": "min_cruise_speed", "MIN_SECONDS": "min_seconds",
                   "MAX_SECONDS": "max_seconds", "GRIPPER_PEAK_SPEED": "gripper_peak_speed",
                   "SAG_LIMIT_RAD": "sag_limit_rad"}),
    ):
        mod = sys.modules.get(name)
        if mod is not None:
            for attr, key in keys.items():
                if hasattr(mod, attr):
                    setattr(mod, attr, MOTION[key])
    ext = sys.modules.get("extrinsics")
    if ext is not None:
        if hasattr(ext, "MIN_POSES"):
            ext.MIN_POSES = CALIBRATION["min_poses"]
        if hasattr(ext, "MAX_CAMERA_M"):
            ext.MAX_CAMERA_M = CALIBRATION["max_camera_m"]
        if hasattr(ext, "MAX_TIP_M"):
            ext.MAX_TIP_M = CALIBRATION["max_tip_m"]
    orc = sys.modules.get("oracle")
    if orc is not None:
        for attr, key in (
            ("PIXEL_ERROR_MM", "pixel_error_mm"), ("TIP_ERROR_MM", "tip_error_mm"),
            ("HOLD_ERROR_RAD", "hold_error_rad"), ("WITNESS_DRIFT_PX", "witness_drift_px"),
            ("WITNESS_SCALE_TOL", "witness_scale_tol"), ("SAG_CORRELATION", "sag_correlation"),
            ("TIP_VECTOR_COHERENCE", "tip_vector_coherence"),
            ("ZERO_BIAS_SPREAD_RAD", "zero_bias_spread_rad"), ("REACH_FRACTION", "reach_fraction"),
            ("PLACEMENT_XY_MM", "placement_xy_mm"), ("PLACEMENT_Z_TOL_MM", "placement_z_tol_mm"),
            ("IK_RESIDUAL_MM", "ik_residual_mm"),
        ):
            if hasattr(orc, attr):
                setattr(orc, attr, ORACLE[key])
    ident = sys.modules.get("identify")
    if ident is not None:
        for attr, key in (
            ("RIG_MAX_AGE_DAYS", "rig_max_age_days"), ("TAG_GROWTH_PX", "tag_growth_px"),
            ("EWMA_ALPHA", "ewma_alpha"), ("MARKER_MIN_USABLE", "marker_min_usable"),
            ("MARKER_SPREAD_MAX_MM", "marker_spread_max_mm"), ("HOLDOUT_MAX_PX", "holdout_max_px"),
            ("TAG_PRIOR_DEPTH", "tag_prior_depth"), ("SAG_MIN_HOLDS", "sag_min_holds"),
            ("SAG_HUBER_RAD", "sag_huber_rad"), ("SAG_VALIDATE_MAX_MRAD", "sag_validate_max_mrad"),
            ("SAG_VALIDATE_RATIO", "sag_validate_ratio"), ("SAG_SPAN_RADIUS_M", "sag_span_radius_m"),
            ("SAG_SPAN_HEIGHT_M", "sag_span_height_m"), ("ACCURACY_MIN_N", "accuracy_min_n"),
            ("ACCURACY_SECTORS", "accuracy_sectors"), ("COVERAGE_MAX_POSES", "coverage_max_poses"),
            ("COVERAGE_HOLD_S", "coverage_hold_s"),
            ("COVERAGE_PERTURBATIONS", "coverage_perturbations"),
            ("COVERAGE_MAX_REACH_M", "coverage_max_reach_m"),
        ):
            if hasattr(ident, attr):
                setattr(ident, attr, LEARNING[key])
        if hasattr(ident, "ACCURACY_RADIUS_EDGES"):
            ident.ACCURACY_RADIUS_EDGES = tuple(LEARNING["accuracy_radius_edges"])
        if hasattr(ident, "ACCURACY_HEIGHT_EDGES"):
            ident.ACCURACY_HEIGHT_EDGES = tuple(LEARNING["accuracy_height_edges"])
