#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The fixed pose sweep an automatic calibration drives the arm through.

Eight poses, chosen once and checked in code rather than by hand:

* every pose is inside the joint limits with margin, and puts the tool in front
  of the base, DEFAULT_MIN_Z-0.42 m above the worktop, within 0.6 m of the base axis;
* every *leg* between consecutive poses is checked with the capsule model from
  `manuals/_build/under_cabinet_probe.py`: no link centre passes within its own
  radius of the table, at any point of the interpolation, not just at the ends;
* the legs are ordered to keep the largest single-joint move small, and j1 is
  held to +-1.25 rad so the arm does not sweep across the whole cell;
* the tips are spread over the work area and at several heights, and the wrists
  differ, which is what makes the camera pose observable.

The table is not a suggestion -- `check()` re-validates it on every run, so a
changed URDF, a new limit or an edited number fails loudly instead of moving an
arm through the table. Live in a cell where the arm must not cross the sink, or
where the camera sees only half the counter, `SWEEP` can be replaced by another
tuple of the same shape; the checks still apply.

Kept free of OpenCV: `planner_server.py` imports this to vet a `/move` before it
sends anything to the board.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

import rebot_ik
import traj

# Parked pose: on the j2/j3 stops, the transit point before and after the
# sweep. Not an observation -- the camera usually cannot see the tool there.
HOME = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# The startup elbow peel, shared with plan_to_target's unpark (unpark_j3=8 deg).
# At rest the arm's j4 hinge seats on the j2 casing; a wrist-driving first leg
# stalls against that contact (motor 4 faults). Lifting j3 raises the forearm and
# breaks the contact before any such leg. UNPARK_REST_J3 is the j3 angle below
# which the arm is considered resting on the stop.
UNPARK_J3_DEG = 8.0
UNPARK_REST_J3 = 0.05

# Eight poses, ordered. Generated from a seeded search and then checked by
# `check()`; the comment on each line is the tool position it should reach.
POSES = (
    (+0.0816, +1.1339, +1.0326, -0.1623, +1.1340, -0.2144),  # tip [+0.335 +0.157 +0.363]
    (+0.9109, +2.7865, +3.0549, -1.2256, +1.4085, -2.2022),  # tip [+0.472 -0.282 +0.260]
    (-0.8503, +2.5861, +2.8438, -0.8384, -1.4520, -2.3254),  # tip [+0.490 +0.254 +0.337]
    (-0.9401, +0.8726, +0.1201, +0.9488, -1.2936, -2.3712),  # tip [+0.289 +0.066 +0.251]
    (-1.0382, +0.3392, +0.3669, -1.1685, -1.2995, +2.1393),  # tip [+0.209 -0.028 +0.174]
    (+1.0324, +1.3483, +0.1628, +1.1010, -0.2078, +2.4164),  # tip [+0.173 -0.370 +0.167]
    (-0.9777, +2.6814, +2.3747, +1.4330, -1.4816, +2.4998),  # tip [+0.455 +0.315 +0.345]
    (+1.2214, +2.4181, +2.7969, -1.4298, +1.1353, +2.4046),  # tip [+0.333 -0.377 +0.303]
)

# Link capsule radii, the same table the collision probe uses (single source:
# rebot_ik.LINK_RADII; witness.LINK_RADII aliases the same).
RADII = rebot_ik.LINK_RADII

CLEARANCE = rebot_ik.CLEARANCE   # m above the worktop for a link's outer surface
# The single minimum tool-tip height above the worktop, and the default hover
# height. The planner's min_z floor, the capture server's REBOT_HOVER_M default
# and the sweep pose band all derive from this one number so they cannot drift.
# CLEARANCE above is a different quantity (a link's outer surface, not the tip).
DEFAULT_MIN_Z = rebot_ik.MOTION["default_min_z"]
X_MIN_TIP = 0.12
RADIUS_MAX = 0.60          # the tip's distance from the base axis
LINK_RADIUS_MAX = rebot_ik.LINK_RADIUS_MAX   # any link centre, for the interpolation check


class SweepError(Exception):
    """The sweep would hit something, or the arm is not where it must start."""


def joint_points(q):
    """Joint origins plus the tool point, in the base frame. (7, 3)."""
    tool, frames, _links = rebot_ik.forward(np.asarray(q, dtype=float).reshape(6),
                                            with_frames=True, with_links=True)
    return np.array([point for point, _axis in frames] + [tool[:3, 3]])


def within_limits(q, margin=0.0):
    q = np.asarray(q, dtype=float).reshape(6)
    lower, upper = rebot_ik.LIMITS[:, 0], rebot_ik.LIMITS[:, 1]
    return bool(np.all(q >= lower + margin) and np.all(q <= upper - margin))


def command_ok(q):
    """The gateway's accept test for a pose it is asked to move to.

    No margin. `HOME` is (0, 0, 0, 0, 0, 0) and j2/j3's range starts at exactly
    0, so the arm's rest pose lies *on* those limits by construction. A comfort
    margin here rejects the park pose itself, which is how a sweep came to fail
    with "that pose is outside the joint limits: [+0.000, ...]" on its first
    move.
    """
    return within_limits(q)


def unpark_pose(q):
    """The pose after the startup elbow peel, or `q` unchanged if already clear.

    The parked arm rests j4's hinge on the j2 casing; lifting j3 by UNPARK_J3_DEG
    (the same peel plan_to_target applies) raises the forearm off that contact so
    the first wrist-driving leg does not stall motor 4. Only applied from rest
    (j3 below UNPARK_REST_J3); the lift is clamped to j3's upper limit. Pure: the
    planner gateway prepends this as the first leg of every /move from rest.
    """
    q = np.asarray(q, dtype=float).reshape(6).copy()
    if q[2] >= UNPARK_REST_J3:
        return q
    q[2] = min(q[2] + math.radians(UNPARK_J3_DEG), float(rebot_ik.LIMITS[2, 1]))
    return q


def pose_ok(q):
    """One pose: in limits, tip in the work area, all links clear of the table."""
    tip = joint_points(q)[-1]
    return (within_limits(q)
            and X_MIN_TIP < tip[0] < 0.50
            and abs(tip[1]) < 0.38
            and DEFAULT_MIN_Z < tip[2] < 0.42
            and math.hypot(tip[0], tip[1]) < RADIUS_MAX)


def samples_safe(samples):
    """The capsule test on a dense joint-space sample series: no link centre
    passes within its own radius of the worktop, and none leaves the cell's
    radial bound. `path_safe` is this test on a straight-line linspace; a
    curved leg (a via chain) must be gated on its own curve, not its chords.
    """
    for q in samples:
        points = joint_points(np.asarray(q, dtype=float).reshape(6))
        for index in range(1, len(points)):
            start, end = points[index - 1], points[index]
            radius = RADII[min(index, len(RADII) - 1)]
            for fraction in (0.0, 0.5):
                centre = start + (end - start) * fraction
                if centre[2] - radius < CLEARANCE:
                    return False
            if math.hypot(end[0], end[1]) > LINK_RADIUS_MAX:
                return False
    return True


def path_safe(a, b, samples=40):
    """Is the straight joint-space interpolation between two poses clear?

    Straight lines in joint space are convex in the limits, but not in the
    table: the tool can dip on the way even when both ends are high, which is
    exactly what a calibration sweep does as it crosses the work area.
    """
    a = np.asarray(a, dtype=float).reshape(6)
    b = np.asarray(b, dtype=float).reshape(6)
    delta = b - a
    return samples_safe(a + delta * t for t in np.linspace(0.0, 1.0, int(samples)))


def legs(park):
    """(target, seconds) for each leg of the one-pass sweep.

    The whole pass is one trajectory: park, each pose in turn, and the board walks
    it back to park at the end. So the legs are consecutive poses, not a return
    trip each.
    """
    previous = np.asarray(park, dtype=float)
    out = []
    for pose in POSES:
        target = np.asarray(pose, dtype=float)
        out.append((target, seconds_for(previous, target)))
        previous = target
    return out


def sequence():
    """Home, the eight poses, home -- the whole sweep in the order it runs."""
    return [np.asarray(HOME, dtype=float)] + [np.asarray(p, dtype=float) for p in POSES] \
        + [np.asarray(HOME, dtype=float)]


def sweep_seconds(park=None):
    """Total motion time: out to each pose once, the hold at each, and back.

    The board walks the arm back to park at the end of the one-pass trajectory,
    so the return leg is part of the motion the estimate must include.
    """
    source = HOME if park is None else park
    travel = sum(value for _, value in legs(source))
    travel += seconds_for(np.asarray(POSES[-1], dtype=float), np.asarray(source, dtype=float))
    return travel + len(POSES) * HOLD_SECONDS


def check(start=None, verbose=False):
    """Validate the table and, optionally, the path from where the arm is now.

    Returns a list of problems; empty means every pose and every leg passed.
    `start` is the pose the arm is in now, which is checked against HOME only.
    """
    problems = []
    for index, pose in enumerate(POSES, start=1):
        if not within_limits(pose):
            problems.append("pose %d is outside the joint limits" % index)
        elif not pose_ok(pose):
            tip = joint_points(pose)[-1]
            problems.append("pose %d leaves the work area: tip [%.3f %.3f %.3f]"
                            % (index, tip[0], tip[1], tip[2]))

    legs = sequence()
    for index in range(len(legs) - 1):
        travel = math.degrees(float(np.max(np.abs(np.asarray(legs[index + 1])
                                                  - np.asarray(legs[index])))))
        if not path_safe(legs[index], legs[index + 1]):
            problems.append("leg %d (%.0f deg joint travel) passes too close to the table"
                            % (index, travel))

    if start is not None:
        start = np.asarray(start, dtype=float).reshape(6)
        if not within_limits(start):
            problems.append("the arm's current pose is outside the joint limits")
        elif not path_safe(start, np.asarray(HOME, dtype=float)):
            problems.append("the path from the current pose back to the parked pose is not clear; "
                            "park the arm first")
    if verbose and not problems:
        tips = np.array([joint_points(pose)[-1] for pose in POSES])
        print("sweep: 8 poses, tips span [%.2f %.2f %.2f] m, min separation %.3f m"
              % (tips[:, 0].max() - tips[:, 0].min(),
                 tips[:, 1].max() - tips[:, 1].min(),
                 tips[:, 2].max() - tips[:, 2].min(),
                 min(np.linalg.norm(tips[i] - tips[j]) for i in range(8)
                     for j in range(i + 1, 8))))
    return problems


# Motion policy for every leg of the sweep, the return to park included.
#
# Legs use the trapezoid profile (traj.ease_fraction): short velocity ramps into
# a constant cruise, so the cap is calibrated to the profile actually used --
# cruise = travel / (T - ramp), and PEAK_SPEED means what it says. 0.5 rad/s is
# about 29 deg/s. A cruise floor (MIN_CRUISE_SPEED) keeps legs above the drive's
# stiction breakaway (~0.2 rad/s at kp=50) if the maximum clamp would otherwise
# make a huge leg crawl. The old 3 s minimum is gone: it made short legs cruise
# far below breakaway. The ramps provide the gentle start, so MIN_SECONDS is
# just the triangle floor (2 x ramp).
PEAK_SPEED = rebot_ik.MOTION["peak_speed"]         # rad/s
MIN_CRUISE_SPEED = rebot_ik.MOTION["min_cruise_speed"]  # rad/s, above the ~0.2 rad/s stiction breakaway
MIN_SECONDS = rebot_ik.MOTION["min_seconds"]
MAX_SECONDS = rebot_ik.MOTION["max_seconds"]

# The jaw is not the arm: a short-travel axis with real stiction. A fixed-duration
# gripper leg commands whatever speed its travel implies -- the sequence's full
# open (4.9 rad in 0.8 s) asked for 9.6 rad/s and the jaw lagged 42 degrees behind
# even with velocity feedforward. Cap its peak the same way the sweep caps the
# arm's, with its own number: 3 rad/s is brisk but trackable at the vendor gains.
# No MIN_SECONDS floor here -- the caller's own floor (0.8 s, grasp_s) stands.
GRIPPER_PEAK_SPEED = rebot_ik.MOTION["gripper_peak_speed"]   # rad/s

# The board returns the arm to the pose a move started from as soon as the file
# ends -- `shut_down` ramps to `self.start`, the pose at bring-up. So a pose only
# exists while the file is still holding it. The camera records the whole sweep
# and the frames are chosen afterwards, which needs the arm to be *still* at the
# pose for a moment, not to be photographed inside a window: 1.5 s is about 22
# frames at 15 fps, several times any plausible stream latency.
HOLD_SECONDS = 1.5

# The sweep's motion policy. The holds are what let a frame be attributed to a
# pose (`sweep_video.select`), so their length is a motion decision.
HOLD_SECONDS = 1.5

# The player's gain spec: `base,ID:value,ID:value`. The default is 50 everywhere
# except motors 2 and 3 at 150 -- the two that carry the arm.
KP_DEFAULT = "50,2:150,3:150"
SAG_LIMIT_RAD = rebot_ik.MOTION["sag_limit_rad"]
TAU_SAG_SCALE = rebot_ik.MOTION["tau_sag_scale"]

# The tool is a gripper: its tip is a few centimetres from the flange. A fit that
# puts it hundreds of millimetres away has locked onto something else -- a shadow,
# a box edge -- and its camera is nonsense even if the residual looks small.
TIP_LIMIT_M = 0.15


def kp_vector(spec=None):
    """Per-joint kp for motors 1..6, from the player's spec string or dict."""
    if isinstance(spec, dict):
        return np.asarray([float(spec.get(str(m), spec.get(m, 50.0))) for m in range(1, 7)],
                          dtype=float)
    text = str(spec or "").strip()
    if not text or text == "default":
        text = KP_DEFAULT
    base = 50.0
    overrides = {}
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            key, value = part.split(":", 1)
            overrides[int(key)] = float(value)
        else:
            base = float(part)
    return np.asarray([overrides.get(motor, base) for motor in range(1, 7)], dtype=float)


def _fmt_kp_num(v: float) -> str:
    v = float(v)
    return str(int(round(v))) if abs(v - round(v)) < 1e-9 else ("%g" % v)


def format_kp_spec(kp_or_spec) -> str:
    """Canonical kp_key string from a telemetry kp dict or spec string."""
    if kp_or_spec is None:
        return KP_DEFAULT
    if isinstance(kp_or_spec, dict):
        if not kp_or_spec:
            return KP_DEFAULT
        vec = kp_vector(kp_or_spec)
    else:
        text = str(kp_or_spec).strip()
        if not text or text == "default":
            return KP_DEFAULT
        try:
            vec = kp_vector(text)
        except (ValueError, TypeError):
            return text
    # Compact form matching KP_DEFAULT ("50,2:150,3:150") using the mode as base.
    vals = [round(float(v), 6) for v in vec]
    base = max(set(vals), key=vals.count)
    parts = [_fmt_kp_num(base)]
    for idx, val in enumerate(vals, start=1):
        if abs(val - base) > 1e-6:
            parts.append("%d:%s" % (idx, _fmt_kp_num(val)))
    return ",".join(parts)


def kp_keys_match(a, b) -> bool:
    """Compare two kp_key values by their 6-joint gain vector (fallback to text)."""
    if a is None or b is None:
        return True
    sa, sb = str(a).strip(), str(b).strip()
    if sa == sb:
        return True
    try:
        va = kp_vector(a)
        vb = kp_vector(b)
    except (ValueError, TypeError):
        return False
    return bool(np.allclose(va, vb, atol=1e-6, rtol=0.0))


def gravity_sag(q, kp, limit=None):
    """How far each joint settles below its command, in rad.

    The MIT frame has no integral term: at rest the motor's `kp * (q_des - q)`
    has to equal the gravity torque, so the joint sits `tau_g / kp` below where it
    was told to be. `rebot_ik.gravity_torques` is the same URDF model the planner
    uses, so this is a prediction from geometry, not a fitted constant.
    """
    limit = SAG_LIMIT_RAD if limit is None else limit
    q = np.asarray(q, dtype=float).reshape(6)
    kp = np.asarray(kp, dtype=float).reshape(6)
    if np.any(kp <= 0.0):
        raise ValueError("kp must be positive")
    sag = rebot_ik.gravity_torques(q) / kp
    return np.clip(sag, -limit, limit)


def soft_clamp_sag(q, cmd, lower, upper):
    """C2-smoothly saturate additive sag compensation inside joint limits.

    When `q` is well inside `[lower, upper]` (available headroom >= 2*|delta|),
    returns `cmd` unchanged. As `q + delta` approaches a joint limit, saturates
    the excess offset with `tanh` into the remaining headroom so the command
    never forms a C0 hard-clamp corner (eliminating limit-peel jerk spikes).
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    cmd = np.asarray(cmd, dtype=float).reshape(-1)
    lo = np.asarray(lower, dtype=float).reshape(-1)
    hi = np.asarray(upper, dtype=float).reshape(-1)
    out = np.empty_like(cmd)
    for j in range(len(q)):
        qj = float(np.clip(q[j], lo[j], hi[j]))
        delta = float(cmd[j] - qj)
        if delta > 0.0:
            headroom = max(0.0, float(hi[j] - qj))
            half = 0.5 * headroom
            if half <= 1e-12:
                delta_eff = 0.0
            elif delta <= half:
                delta_eff = delta
            else:
                delta_eff = half + half * math.tanh((delta - half) / half)
        elif delta < 0.0:
            headroom = max(0.0, float(qj - lo[j]))
            half = 0.5 * headroom
            ad = -delta
            if half <= 1e-12:
                delta_eff = 0.0
            elif ad <= half:
                delta_eff = delta
            else:
                delta_eff = -(half + half * math.tanh((ad - half) / half))
        else:
            delta_eff = 0.0
        out[j] = float(np.clip(qj + delta_eff, lo[j], hi[j]))
    return out


def aim(q, kp, iterations=3, extra_sag=None, base_scale=1.0, blend_weights=None):
    """The command that makes the arm *hold* `q` instead of settling off it.

    This arm settles as `held = command - sag`, where `sag = tau_g / kp` when
    no torque feedforward is active (`/move`, `base_scale = 1.0`). When MIT
    gravity torque feedforward `tau_ff` IS active (`/plan`, `base_scale =
    TAU_SAG_SCALE`), cold-start compensation scales the model sag to avoid
    double-compensating gravity while smoothly blending into the learned
    residual map (`tau_g/kp + extra_sag`) across `blend_weights`.

    The *total* compensation stays clipped by SAG_LIMIT_RAD, so a bad or stale
    map can never command a large excursion.
    """
    target = np.asarray(q, dtype=float).reshape(6)
    extra = (np.zeros(6) if extra_sag is None else
             np.clip(np.asarray(extra_sag, dtype=float).reshape(6),
                     -SAG_LIMIT_RAD, SAG_LIMIT_RAD))
    weights = None if blend_weights is None else np.clip(
        np.asarray(blend_weights, dtype=float).reshape(6), 0.0, 1.0)
    scale = float(base_scale)

    def _offset(pose):
        model_sag = gravity_sag(pose, kp)
        if weights is None:
            raw = scale * model_sag + extra
        else:
            # extra already carries per-joint weight w_j * (a_j * tau_j + b_j):
            # blend model term from scale*model_sag (w=0) to 1.0*model_sag (w=1).
            raw = ((1.0 - weights) * scale + weights) * model_sag + extra
        return np.clip(raw, -SAG_LIMIT_RAD, SAG_LIMIT_RAD)

    command = target + _offset(target)
    for _ in range(max(0, int(iterations) - 1)):
        command = target + _offset(command)
    return command


# --------------------------------------------------------------------------- #
# The learned sag residual map (§7.3b): keying and application.
#
# identify.py FITS the map from telemetry; these pure functions decide whether
# a fitted map may be trusted for a command. They live here, not in identify,
# because planner_server applies the map on /move and cannot import identify
# (which pulls OpenCV through auto_calibrate, and the planner image has none).
# --------------------------------------------------------------------------- #

def zeros_key(joint_zeros):
    """The joint-zeros identity a sag map is keyed by (a zero change invalidates)."""
    if joint_zeros is None:
        return "none"
    blob = json.dumps([round(float(v), 6) for v in np.asarray(joint_zeros, float)])
    return hashlib.sha256(blob.encode()).hexdigest()[:12]


def sag_map_path(calib_dir, rig_fingerprint):
    return Path(calib_dir) / ("sag-%s.json" % rig_fingerprint)


def _sag_map_matches(map_doc, kp_key, zeros_key="none") -> bool:
    if not isinstance(map_doc, dict):
        return False
    if not kp_keys_match(map_doc.get("kp_key"), kp_key):
        return False
    if str(map_doc.get("zeros_key", "none")) != str(zeros_key):
        return False
    return True


def load_sag_map(calib_dir, rig_fingerprint, kp_key, zeros_key="none"):
    """Load the sag map for this arm's kp and zeros (camera-rig independent)."""
    calib_dir = Path(calib_dir)
    candidates = [sag_map_path(calib_dir, rig_fingerprint), calib_dir / "sag-arm.json"]
    if calib_dir.is_dir():
        others = sorted(
            (p for p in calib_dir.glob("sag-*.json") if p not in candidates),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        candidates.extend(others)
    for path in candidates:
        if not path.is_file():
            continue
        try:
            map_doc = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            continue
        if _sag_map_matches(map_doc, kp_key, zeros_key=zeros_key):
            return map_doc
    return None


SAG_ENVELOPE_TAPER_RAD = 0.25
SAG_TAU_TAPER_NM = 1.0


def _smoothstep_taper(value: float, lo: float, hi: float, taper: float) -> float:
    dist = 0.0
    if value < lo:
        dist = lo - value
    elif value > hi:
        dist = value - hi
    if dist <= 0.0:
        return 1.0
    if dist >= taper or taper <= 0.0:
        return 0.0
    u = dist / taper
    return 1.0 - (3.0 * u * u - 2.0 * u * u * u)


def sag_correction_weights(q, sag_map, kp_key=None, rig_fingerprint=None, zeros_key="none"):
    """Per-joint learned residual and C1 envelope weights `(extra, weights)`.

    Gravity sag is an arm/motor property (independent of camera `rig_fingerprint`),
    keyed by canonical `kp` and `zeros_key`. Each joint is gated independently on
    its own torque envelope `[tau_min[j], tau_max[j]]` (or angle envelope on
    legacy maps), so changing J4 from horizontal to top-down never zeroes the
    learned corrections on J2/J3.
    """
    zeros = np.zeros(6)
    if not sag_map:
        return zeros, zeros
    if kp_key is not None and not kp_keys_match(sag_map.get("kp_key"), kp_key):
        return zeros, zeros
    if str(sag_map.get("zeros_key", "none")) != str(zeros_key):
        return zeros, zeros
    q = np.asarray(q, dtype=float).reshape(6)
    tau = rebot_ik.gravity_torques(q)
    envelope = sag_map.get("envelope") or {}
    tau_min = envelope.get("tau_min")
    tau_max = envelope.get("tau_max")
    use_tau = (isinstance(tau_min, (list, tuple)) and len(tau_min) >= 6
               and isinstance(tau_max, (list, tuple)) and len(tau_max) >= 6)
    q_min = np.asarray(envelope.get("q_min", [-math.inf] * 6), dtype=float)
    q_max = np.asarray(envelope.get("q_max", [math.inf] * 6), dtype=float)
    weights = np.zeros(6)
    extra = np.zeros(6)
    per_motor = sag_map.get("per_motor") or {}
    for j in range(6):
        if use_tau:
            lo, hi = float(tau_min[j]), float(tau_max[j])
            taper = max(SAG_TAU_TAPER_NM, 0.25 * max(0.0, hi - lo))
            w = _smoothstep_taper(float(tau[j]), lo, hi, taper)
        else:
            w = _smoothstep_taper(float(q[j]), float(q_min[j]), float(q_max[j]),
                                  SAG_ENVELOPE_TAPER_RAD)
        weights[j] = w
        motor = per_motor.get(str(j + 1))
        if motor and w > 0.0:
            raw = (float(motor["a"]) * float(tau[j]) + float(motor["b"])) * w
            extra[j] = float(np.clip(raw, -SAG_LIMIT_RAD, SAG_LIMIT_RAD))
    return extra, weights


def sag_correction(q, sag_map, kp_key=None, rig_fingerprint=None, zeros_key="none"):
    """The per-joint additive sag term for aim(), or zeros when untrusted."""
    extra, _weights = sag_correction_weights(
        q, sag_map, kp_key=kp_key, rig_fingerprint=rig_fingerprint, zeros_key=zeros_key)
    return extra


def active_sag_extra(q, kp_spec, calib_dir, rig_name="extrinsics-fixed.json"):
    """The learned term for the live /move path: the active rig's keyed map.

    The rig file beside the map names the fingerprint and zeros the map was
    fitted under; a missing or torn rig file means no map, never an exception
    into the move. The total compensation stays clipped inside aim().
    """
    try:
        rig_doc = json.loads((Path(calib_dir) / rig_name).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return np.zeros(6)
    fingerprint = str(rig_doc.get("fingerprint") or "")
    if not fingerprint:
        return np.zeros(6)
    key = zeros_key(rig_doc.get("joint_zeros"))
    map_doc = load_sag_map(calib_dir, fingerprint, kp_key=kp_spec, zeros_key=key)
    return sag_correction(q, map_doc, kp_key=kp_spec, rig_fingerprint=fingerprint,
                          zeros_key=key)


def sag_compensator(kp_spec, calib_dir, rig_name="extrinsics-fixed.json", with_tau=False):
    """A per-pose command compensator for a whole plan (§7.3b): model sag plus
    the learned residual where a keyed map exists.

    When `with_tau=True` (the trajectory also carries MIT gravity torque
    feedforward `tau`), cold-start compensation uses `TAU_SAG_SCALE` and blends
    smoothly into the learned residual map across each joint's envelope weight.
    """
    kp = kp_vector(kp_spec)
    rig_doc = {}
    if calib_dir:
        try:
            rig_doc = json.loads((Path(calib_dir) / rig_name).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            rig_doc = {}
    fingerprint = str(rig_doc.get("fingerprint") or "")
    key = zeros_key(rig_doc.get("joint_zeros"))
    map_doc = (load_sag_map(calib_dir, fingerprint, kp_key=kp_spec, zeros_key=key)
               if calib_dir and fingerprint else None)
    base_scale = TAU_SAG_SCALE if with_tau else 1.0

    def compensate(q):
        q = np.asarray(q, dtype=float).reshape(6)
        extra, weights = sag_correction_weights(
            q, map_doc, kp_key=kp_spec, rig_fingerprint=fingerprint, zeros_key=key)
        return aim(q, kp, extra_sag=extra, base_scale=base_scale,
                   blend_weights=weights if with_tau else None)

    return compensate


def seconds_for_travel(travel, peak_speed=None, minimum=None, maximum=None):
    """The speed policy on a path length, rad: cruise = travel / (T - ramp).

    The peak cap is travel / peak + ramp. The cruise floor is an upper bound on
    T -- if the maximum clamp would make a huge leg crawl below stiction
    breakaway, the leg is shortened to keep cruise >= MIN_CRUISE_SPEED. The
    caller's minimum is the triangle floor for tiny nudges. A via chain budgets
    its whole curved path here, not just the endpoint delta.
    """
    peak_speed = PEAK_SPEED if peak_speed is None else peak_speed
    minimum = MIN_SECONDS if minimum is None else minimum
    maximum = MAX_SECONDS if maximum is None else maximum
    travel = float(travel)
    peak_time = travel / float(peak_speed) + traj.RAMP_S
    cruise_cap = travel / MIN_CRUISE_SPEED + traj.RAMP_S
    return max(minimum, min(maximum, peak_time, cruise_cap))


def seconds_for(a, b, peak_speed=None, minimum=None, maximum=None):
    """How long a leg should take, from the furthest-travelling joint.

    Trapezoid-native: cruise = travel / (T - ramp), so the peak cap is
    travel / peak + ramp. The cruise floor is an upper bound on T -- if the
    maximum clamp would make a huge leg crawl below stiction breakaway, the
    leg is shortened to keep cruise >= MIN_CRUISE_SPEED. The caller's minimum
    is the triangle floor for tiny nudges.
    """
    peak_speed = PEAK_SPEED if peak_speed is None else peak_speed
    minimum = MIN_SECONDS if minimum is None else minimum
    maximum = MAX_SECONDS if maximum is None else maximum
    travel = float(np.max(np.abs(np.asarray(b, dtype=float) - np.asarray(a, dtype=float))))
    return seconds_for_travel(travel, peak_speed, minimum, maximum)


def gripper_seconds(travel, peak_speed=None):
    """Duration that keeps a cosine-eased gripper leg under the jaw speed cap.

    A floor, never a ceiling: callers take max(their own seconds, this), so a
    caller asking for a slower grasp still gets it. Same cosine-peak inversion
    as `seconds_for`, without the sweep's 3 s minimum -- a 2.90 rad close
    wants 1.5 s at the cap, not three.
    """
    peak_speed = GRIPPER_PEAK_SPEED if peak_speed is None else peak_speed
    travel = abs(float(travel))
    if not math.isfinite(travel):
        return 0.0
    return (math.pi / 2.0) * travel / float(peak_speed)
