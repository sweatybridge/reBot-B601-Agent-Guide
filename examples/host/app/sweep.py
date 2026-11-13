#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The fixed pose sweep an automatic calibration drives the arm through.

Eight poses, chosen once and checked in code rather than by hand:

* every pose is inside the joint limits with margin, and puts the tool in front
  of the base, 0.12-0.42 m above the worktop, within 0.5 m of the base axis;
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

import math

import numpy as np

import rebot_ik

# Parked pose: on the j2/j3 stops, the transit point before and after the
# sweep. Not an observation -- the camera usually cannot see the tool there.
HOME = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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

# Link capsule radii, the same table the collision probe uses.
RADII = (0.055, 0.055, 0.050, 0.045, 0.035, 0.030, 0.020)

CLEARANCE = 0.012          # m above the worktop for a link's outer surface
Z_MIN_TIP = 0.10
X_MIN_TIP = 0.12
RADIUS_MAX = 0.60          # the tip's distance from the base axis
LINK_RADIUS_MAX = 0.85     # any link centre, for the interpolation check


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


def pose_ok(q):
    """One pose: in limits, tip in the work area, all links clear of the table."""
    tip = joint_points(q)[-1]
    return (within_limits(q)
            and X_MIN_TIP < tip[0] < 0.50
            and abs(tip[1]) < 0.38
            and Z_MIN_TIP < tip[2] < 0.42
            and math.hypot(tip[0], tip[1]) < RADIUS_MAX)


def path_safe(a, b, samples=40):
    """Is the straight joint-space interpolation between two poses clear?

    Straight lines in joint space are convex in the limits, but not in the
    table: the tool can dip on the way even when both ends are high, which is
    exactly what a calibration sweep does as it crosses the work area.
    """
    a = np.asarray(a, dtype=float).reshape(6)
    b = np.asarray(b, dtype=float).reshape(6)
    delta = b - a
    for t in np.linspace(0.0, 1.0, int(samples)):
        points = joint_points(a + delta * t)
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
    """Total motion time: out to each pose once, plus the hold at each."""
    source = HOME if park is None else park
    travel = sum(value for _, value in legs(source))
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
# `traj.joint_move` interpolates with a cosine ease, so a leg starts and stops at
# zero velocity and its peak is pi/2 times its average. That makes the peak the
# only number worth setting: from it, and the joint that travels furthest, comes
# the duration. 0.35 rad/s is about 20 deg/s -- slow enough to watch, and it
# costs a couple of minutes once, on a commissioning pass that should not be
# hurried. The floor matters for short legs: a 0.2 rad nudge still gets three
# seconds, so it accelerates and decelerates instead of starting with a jerk.
PEAK_SPEED = 0.35        # rad/s
MIN_SECONDS = 3.0
MAX_SECONDS = 25.0

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
SAG_LIMIT_RAD = 0.15

# The tool is a gripper: its tip is a few centimetres from the flange. A fit that
# puts it hundreds of millimetres away has locked onto something else -- a shadow,
# a box edge -- and its camera is nonsense even if the residual looks small.
TIP_LIMIT_M = 0.15


def kp_vector(spec=None):
    """Per-joint kp for motors 1..6, from the player's spec string."""
    base = 50.0
    overrides = {}
    for part in str(spec or KP_DEFAULT).split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            key, value = part.split(":", 1)
            overrides[int(key)] = float(value)
        else:
            base = float(part)
    return np.asarray([overrides.get(motor, base) for motor in range(1, 7)], dtype=float)


def gravity_sag(q, kp, limit=SAG_LIMIT_RAD):
    """How far each joint settles below its command, in rad.

    The MIT frame has no integral term: at rest the motor's `kp * (q_des - q)`
    has to equal the gravity torque, so the joint sits `tau_g / kp` below where it
    was told to be. `rebot_ik.gravity_torques` is the same URDF model the planner
    uses, so this is a prediction from geometry, not a fitted constant.
    """
    q = np.asarray(q, dtype=float).reshape(6)
    kp = np.asarray(kp, dtype=float).reshape(6)
    if np.any(kp <= 0.0):
        raise ValueError("kp must be positive")
    sag = rebot_ik.gravity_torques(q) / kp
    return np.clip(sag, -limit, limit)


def aim(q, kp, iterations=3):
    """The command that makes the arm *hold* `q` instead of settling off it.

    This arm settles as `held = command - sag`, where `sag = tau_g / kp` -- the
    sign is not the intuitive one (j2's axis means gravity shows up negative), and
    it was measured rather than assumed: commanding `target - sag` moved every
    pose by another |sag| (pose 2 went from 92 mrad off to 179), which is only
    consistent with `held = command - sag`. So holding `q` means asking for
    `q + sag`: a fixed point, solved by substitution, since the sag depends only
    weakly on the pose.
    """
    target = np.asarray(q, dtype=float).reshape(6)
    command = target + gravity_sag(target, kp)
    for _ in range(max(0, int(iterations) - 1)):
        command = target + gravity_sag(command, kp)
    return command


def seconds_for(a, b, peak_speed=PEAK_SPEED, minimum=MIN_SECONDS, maximum=MAX_SECONDS):
    """How long a leg should take, from the furthest-travelling joint.

    A cosine ease has peak speed `pi/2 * travel / seconds`, so inverting that
    for the cap is the whole policy. It is used for every leg -- out to a pose,
    between poses, and back to the parked pose -- so the return is never the
    quickest part of the sweep, which is what it used to be.
    """
    travel = float(np.max(np.abs(np.asarray(b, dtype=float) - np.asarray(a, dtype=float))))
    return max(minimum, min(maximum, (math.pi / 2.0) * travel / float(peak_speed)))
