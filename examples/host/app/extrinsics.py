#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fixed-camera extrinsics: `T_base<-cam` fitted from the arm itself.

The tag in the tap pipeline exists for one reason: a handheld phone has to have
its pose re-measured on every frame. Put the camera on a bracket and that pose
becomes a constant -- and a constant can be measured without printing anything,
because the arm is an object of known geometry standing in the middle of the
scene.

What is measured is the **tool tip**. The operator clicks it in each of 6-10
still frames, with the arm parked in a different pose for each one. The camera
never moves; the arm does, and the joint angles come from `GET /pose` on the
edge, so the tip's position in the base frame is known as a function of `q` up
to one unknown: its offset in `gripper_end`. That offset is fitted, not
measured, which is what makes this a calibration instead of a tape measure.

    unknowns   T_cam<-base (6) + tip offset (3)   [+ joint zeros (6)]
    residuals  one clicked pixel per pose          (2 per pose)
    7 poses    14 residuals for 9 unknowns         (5 degrees of freedom)

Three things this gets right that a single photo does not:

* **Conditioning.** One pose is one point, and a point is not a pose. A sweep in
  position and orientation breaks the twofold planar ambiguity and the weak
  direction a folded arm leaves behind. `weakest` in the fit report says which
  direction the sweep failed to pin down, in millimetres, so a bad sweep is a
  number rather than a surprise later.
* **The tip is an unknown, so no stylus calibration.** Measuring the tip by
  pivoting and then touching known points would work too, but it needs a pivot
  calibration and a contact you can repeat to a millimetre. Clicking the tip in
  the image removes both steps.
* **Robust to a mis-click.** A Huber loss down-weights a click that does not
  agree with the other seven, and the fit report names it. One bad click in a
  set of eight is the most likely failure of this procedure.

The joint zeros are a *diagnostic*, never applied. j1 and j6 are not fitted at
all, because each is an exact gauge: rotating the whole arm about the base z is
the same observation as rotating the camera about the base z, and rotating the
tool about its own axis is the same observation as rotating the tip offset about
that axis. j2-j5 are observable, and a large fitted offset means the arm's zero
calibration is off -- fix that in Studio and refit, rather than bending the rig
to match a wrong model.

Everything here is pure numpy: the estimator runs wherever the images are, not
only in the Pinocchio image. `rebot_ik.forward()` is the kinematic model, so the
extrinsics are only as good as that chain -- see the note in
`docs/phone-camera.md` about what a joint-zero error does to them.

Units: metres and radians throughout; pixels only at the boundary.
"""

from __future__ import annotations

import calendar
import hashlib
import json
import math
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

import rebot_ik

FORMAT = "rebot-extrinsics"
VERSION = 1
MIN_POSES = 6
# Where a bench rig physically is: the camera within a few metres of the arm, the
# tool's tip a few centimetres from the flange. A fit outside this is not a rig.
MAX_CAMERA_M = 5.0
MAX_TIP_M = 0.20

# Free parameters, in the order they appear in the solver's vector. 0-5 are the
# camera pose (`T_cam<-base`), 6-8 the tip in `gripper_end`, 9-14 the joint
# zero offsets in radians. The camera is stored as `T_base<-cam` but solved as
# `T_cam<-base`, because that is the direction that projects.
CAMERA = ("rx", "ry", "rz", "tx", "ty", "tz")
TIP = ("tip_x", "tip_y", "tip_z")
ZEROS = ("j1", "j2", "j3", "j4", "j5", "j6")
NAMES = CAMERA + TIP + ZEROS


class ExtrinsicsError(Exception):
    """The rig cannot be fitted, loaded, or trusted; the message is for the operator."""


# --------------------------------------------------------------------------- #
# SO(3) and SE(3)
# --------------------------------------------------------------------------- #

def skew(w):
    w = np.asarray(w, dtype=float).reshape(3)
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def rodrigues(w):
    """Rotation matrix of a rotation vector (the exp map)."""
    w = np.asarray(w, dtype=float).reshape(3)
    angle = float(np.linalg.norm(w))
    K = skew(w)
    if angle < 1e-12:
        return np.eye(3) + K
    return (np.eye(3) + math.sin(angle) / angle * K
            + (1.0 - math.cos(angle)) / (angle * angle) * (K @ K))


def log_so3(R):
    """Rotation vector of `R` (the log map), valid over the whole branch.

    The axis is the eigenvector of `(R+R^T)/2` for eigenvalue 1, which stays
    well conditioned at 180 deg where the antisymmetric part vanishes.
    """
    R = np.asarray(R, dtype=float)
    values, vectors = np.linalg.eigh((R + R.T) / 2.0)
    axis = vectors[:, int(np.argmax(values))]
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.zeros(3)
    axis = axis / norm

    cosine = min(1.0, max(-1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)

    antisymmetric = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    sine = math.sin(angle)
    if abs(sine) > 0.1:
        # Away from 0 and 180 deg this is the better-conditioned formula.
        candidate = antisymmetric / (2.0 * sine)
        if float(np.linalg.norm(candidate)) > 1e-6:
            axis = candidate / float(np.linalg.norm(candidate))
    if float(antisymmetric @ axis) < 0.0:
        axis = -axis
    return axis * angle


def invert(T):
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def transform(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def rotation_deg(R):
    cosine = min(1.0, max(-1.0, (float(np.trace(R)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def look_at(position, target, up=(0.0, 0.0, 1.0)):
    """`T_base<-cam` for a camera at `position`, aimed at `target`.

    Only used to bootstrap the automatic sweep from the mount's position; the
    fit then moves the camera wherever the arm says it is. Within 20 cm is
    plenty, because the detector searches along the tool axis rather than at
    one projected pixel.
    """
    position = np.asarray(position, dtype=float).reshape(3)
    forward = np.asarray(target, dtype=float).reshape(3) - position
    norm = float(np.linalg.norm(forward))
    if norm < 1e-6:
        raise ExtrinsicsError("look_at: the camera is on top of its target")
    forward = forward / norm
    up = np.asarray(up, dtype=float).reshape(3)
    right = np.cross(up, forward)
    if float(np.linalg.norm(right)) < 1e-6:
        # Aimed straight down: any horizontal axis will do.
        up = np.array([0.0, 1.0, 0.0]) if abs(forward[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        right = np.cross(up, forward)
    right = right / float(np.linalg.norm(right))
    down = np.cross(forward, right)
    T = np.eye(4)
    T[:3, :3] = np.stack([right, down, forward], axis=1)
    T[:3, 3] = position
    return T


def camera_angles(T_base_cam):
    """(looking-down angle, yaw) of the optical axis, both in degrees.

    `down` is positive when the camera looks at the table, `yaw` is measured
    from +x about +z, and both are what an operator can check against the mount.
    """
    axis = np.asarray(T_base_cam, dtype=float)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    down = -math.degrees(math.asin(max(-1.0, min(1.0, float(axis[2])))))
    yaw = math.degrees(math.atan2(float(axis[1]), float(axis[0])))
    return down, yaw


# --------------------------------------------------------------------------- #
# Pinhole camera, with the distortion `calib.py` stores
# --------------------------------------------------------------------------- #

def _coefficients(dist):
    values = list(np.asarray(dist, dtype=float).reshape(-1)) if dist is not None else []
    values += [0.0] * (5 - len(values))
    return values[:5]                       # k1, k2, p1, p2, k3


def distort(x, y, dist):
    k1, k2, p1, p2, k3 = _coefficients(dist)
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return xd, yd


def undistort(xd, yd, dist, iterations=10):
    """Invert `distort` by fixed point; the phone's distortion is mild enough."""
    k1, k2, p1, p2, k3 = _coefficients(dist)
    if not any((k1, k2, p1, p2, k3)):
        return xd, yd
    x, y = xd, yd
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return x, y


def pixel(p_base, K, dist, T_cam_base):
    """Project a base-frame point; returns (u, v) or None when behind the lens."""
    point = np.asarray(p_base, dtype=float).reshape(3)
    T = np.asarray(T_cam_base, dtype=float)
    p_cam = T[:3, :3] @ point + T[:3, 3]
    depth = float(p_cam[2])
    if depth <= 1e-6:
        return None
    x, y = distort(float(p_cam[0]) / depth, float(p_cam[1]) / depth, dist)
    K = np.asarray(K, dtype=float)
    return float(K[0, 0] * x + K[0, 2]), float(K[1, 1] * y + K[1, 2])


def ray_plane(K, dist, T_base_cam, u, v, plane_z):
    """Where the pixel's ray meets the horizontal plane. (point, distance, grazing deg).

    This is the intersection `locate_target.py` has always done, with the
    distortion inverted first: the camera fit uses `dist`, so the ray has to as
    well, or every click away from the principal point is biased outwards.
    """
    K = np.asarray(K, dtype=float)
    xd = (float(u) - K[0, 2]) / K[0, 0]
    yd = (float(v) - K[1, 2]) / K[1, 1]
    x, y = undistort(xd, yd, dist)

    direction_cam = np.array([x, y, 1.0])
    T = np.asarray(T_base_cam, dtype=float)
    origin = T[:3, 3]
    direction = T[:3, :3] @ direction_cam
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        raise ExtrinsicsError("degenerate ray for pixel (%s, %s)" % (u, v))
    direction /= norm

    if abs(float(direction[2])) < 1e-6:
        raise ExtrinsicsError("the ray is parallel to the table plane")
    distance = (float(plane_z) - float(origin[2])) / float(direction[2])
    if distance <= 0.0:
        raise ExtrinsicsError("the ray points away from the table plane")
    point = origin + distance * direction
    grazing = math.degrees(math.asin(min(1.0, abs(float(direction[2])))))
    return point, float(distance), float(grazing)


# --------------------------------------------------------------------------- #
# Observations and the model
# --------------------------------------------------------------------------- #

@dataclass
class Observation:
    """One clicked pose: joint angles, the pixel, and where the frame came from."""

    q: np.ndarray
    uv: tuple
    label: str = "tool"
    capture: str = ""

    def to_dict(self):
        return {"q": [float(v) for v in self.q], "uv": [float(self.uv[0]), float(self.uv[1])],
                "label": self.label, "capture": self.capture}

    @classmethod
    def from_dict(cls, data):
        q = np.asarray(data["q"], dtype=float).reshape(-1)
        if q.size < 6:
            raise ExtrinsicsError("an observation needs 6 joint angles, got %d" % q.size)
        uv = np.asarray(data["uv"], dtype=float).reshape(-1)
        if uv.size != 2:
            raise ExtrinsicsError("an observation needs one (u, v), got %s" % (uv,))
        return cls(q=q[:6], uv=(float(uv[0]), float(uv[1])),
                   label=str(data.get("label") or "tool"),
                   capture=str(data.get("capture") or ""))


@dataclass
class FitOptions:
    """What the solver is allowed to move, and how hard it looks for outliers."""

    refine_tip: bool = True
    refine_zeros: bool = False
    plane_z: float = 0.0
    huber_px: float = 2.0          # 0 disables the robust loss
    max_iterations: int = 120
    max_weak_mm: float = 60.0      # refuse a sweep whose weakest direction is worse
    work_radius: float = 0.55      # m, the grid the predicted error is reported over
    image_size: tuple | None = None


# --------------------------------------------------------------------------- #
# The fit
# --------------------------------------------------------------------------- #

class _Model:
    """Residual evaluation for one observation set, with the FK cache.

    When the joint zeros are held fixed the link poses never change, so the
    forward kinematics runs once per pose instead of once per solver step --
    a large difference on a commissioning sweep.
    """

    def __init__(self, observations, K, dist, options: FitOptions):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.dist = (np.asarray(dist, dtype=float).reshape(-1)
                     if dist is not None else np.zeros(5))
        self.options = options
        self.q = [np.asarray(o.q, dtype=float) for o in observations]
        self.uv = np.array([o.uv for o in observations], dtype=float)
        self.fixed = None
        if not options.refine_zeros:
            self.fixed = []
            for q in self.q:
                T = rebot_ik.forward(q)
                self.fixed.append((T[:3, :3].copy(), T[:3, 3].copy()))

        self.free = list(range(6))
        if options.refine_tip:
            self.free += [6, 7, 8]
        if options.refine_zeros:
            # j2-j5 only. j1 and j6 are unobservable here and are held at zero:
            # rotating the whole arm about the base z (j1) is reproduced exactly
            # by rotating the camera about the base z, and rotating the tool
            # about its own axis (j6) is reproduced exactly by rotating the point
            # offset about that axis. Both are gauges, not measurements -- fitting
            # them sends j6 somewhere absurd and says nothing about the arm.
            self.free += [10, 11, 12, 13]

    def size(self):
        return 2 * len(self.q)

    def unpack(self, theta):
        theta = np.asarray(theta, dtype=float)
        return (rodrigues(theta[0:3]), theta[3:6], theta[6:9],
                theta[9:15] if self.options.refine_zeros else None)

    def tip_in_base(self, index, tip, zeros=None):
        if zeros is None or not np.any(zeros):
            if self.fixed is not None:
                R, p = self.fixed[index]
                return R @ tip + p
            q = self.q[index]
        else:
            q = self.q[index] + zeros
        T = rebot_ik.forward(q)
        return T[:3, :3] @ tip + T[:3, 3]

    def residuals(self, theta):
        R, translation, tip, zeros = self.unpack(theta)
        out = np.empty(self.size(), dtype=float)
        camera = transform(R, translation)
        for i in range(len(self.q)):
            point = self.tip_in_base(i, tip, zeros)
            seen = pixel(point, self.K, self.dist, camera)
            if seen is None:
                out[2 * i:2 * i + 2] = 1e3      # behind the camera: a wall, not a NaN
            else:
                out[2 * i] = seen[0] - self.uv[i, 0]
                out[2 * i + 1] = seen[1] - self.uv[i, 1]
        return out

    def jacobian(self, theta, step=1e-6):
        theta = np.asarray(theta, dtype=float)
        columns = []
        for j in self.free:
            forward = theta.copy()
            back = theta.copy()
            forward[j] += step
            back[j] -= step
            columns.append((self.residuals(forward) - self.residuals(back)) / (2.0 * step))
        return np.column_stack(columns)

    def weights(self, residuals):
        """Huber weights on each pose's residual norm, not each axis.

        A click that disagrees by 6 px has to be discounted as a click, not as
        one axis of a click; the norm is what the operator got wrong.
        """
        weights = np.ones(self.size(), dtype=float)
        delta = float(self.options.huber_px)
        if delta <= 0.0:
            return weights
        norms = np.linalg.norm(np.asarray(residuals).reshape(-1, 2), axis=1)
        scale = np.where(norms > delta, delta / np.maximum(norms, 1e-9), 1.0)
        return np.repeat(scale, 2)


def _normalized_pixels(observations, K, dist):
    K = np.asarray(K, dtype=float).reshape(3, 3)
    out = []
    for observation in observations:
        u, v = observation.uv
        xd = (float(u) - K[0, 2]) / K[0, 0]
        yd = (float(v) - K[1, 2]) / K[1, 1]
        out.append(undistort(xd, yd, dist))
    return np.asarray(out, dtype=float)


def _dlt_pose(world, normalized):
    """Normalized DLT on undistorted coordinates -> (R, t) of `T_cam<-base`.

    The DLT only has to be close enough for Levenberg-Marquardt to take over,
    but it doubles as the degeneracy test: a coplanar sweep makes the projection
    matrix rank-deficient, and that must be an error here rather than a
    converged-looking fit later.
    """
    world = np.asarray(world, dtype=float).reshape(-1, 3)
    normalized = np.asarray(normalized, dtype=float).reshape(-1, 2)
    count = world.shape[0]
    if count < 6:
        raise ExtrinsicsError("a DLT needs at least 6 poses to start from, got %d" % count)

    centre = world.mean(axis=0)
    spread = float(np.mean(np.linalg.norm(world - centre, axis=1)))
    if spread < 1e-6:
        raise ExtrinsicsError("every pose puts the tool at the same point; move the arm between "
                              "captures")
    W = (world - centre) / spread

    image_mean = normalized.mean(axis=0)
    image_spread = float(np.mean(np.linalg.norm(normalized - image_mean, axis=1)))
    if image_spread < 1e-9:
        raise ExtrinsicsError("every click landed on the same pixel; the tip has to move")
    U = (normalized - image_mean) / image_spread

    rows = []
    for (X, Y, Z), (u, v) in zip(W, U):
        rows.append([X, Y, Z, 1.0, 0.0, 0.0, 0.0, 0.0, -u * X, -u * Y, -u * Z, -u])
        rows.append([0.0, 0.0, 0.0, 0.0, X, Y, Z, 1.0, -v * X, -v * Y, -v * Z, -v])
    _u, values, vectors = np.linalg.svd(np.asarray(rows, dtype=float))
    # A general 3x4 projection has 11 degrees of freedom, so the second-smallest
    # singular value is the one that collapses when the sweep is degenerate.
    if values[-2] < 1e-7 * values[0]:
        raise ExtrinsicsError(
            "the poses are degenerate (rank %d of 11): the tool needs to move in 3D, at several "
            "heights and with a range of wrist orientations, not along a flat arc"
            % int(np.sum(values > 1e-7 * values[0])))
    P = vectors[-1].reshape(3, 4)

    # Undo the two normalizations: U = S u_n, W = D^-1 X.
    S = np.array([[image_spread, 0.0, image_mean[0]],
                  [0.0, image_spread, image_mean[1]],
                  [0.0, 0.0, 1.0]])
    D = np.eye(4)
    D[:3, :3] *= spread
    D[:3, 3] = centre
    P = S @ P @ np.linalg.inv(D)

    # A calibrated DLT has a proper rotation, so the physical sign is the one
    # that makes the linear block a rotation rather than a reflection.
    if float(np.linalg.det(P[:, :3])) < 0.0:
        P = -P
    scale = float(np.cbrt(abs(float(np.linalg.det(P[:, :3])))))
    if scale < 1e-12:
        raise ExtrinsicsError("the DLT found a singular projection; check the clicked points")
    P = P / scale

    Q, Rq = np.linalg.qr(P[:, :3].T)          # P^T = Q Rq  ->  P = Rq^T Q^T
    rotation = Rq.T
    upper = Q.T
    signs = np.sign(np.diag(upper))
    signs[signs == 0.0] = 1.0
    rotation = rotation @ np.diag(signs)
    upper = np.diag(signs) @ upper
    translation = P[:, 3] / float(np.mean(np.diag(upper)))

    # Project the 3x3 block onto SO(3): the DLT's block is only nearly so.
    U_, _, V_ = np.linalg.svd(rotation)
    rotation = U_ @ V_
    if float(np.linalg.det(rotation)) < 0.0:
        rotation = U_ @ np.diag([1.0, 1.0, -1.0]) @ V_

    depths = (rotation @ world.T).T + translation
    if float(np.mean(depths[:, 2])) < 0.0:
        raise ExtrinsicsError(
            "the DLT found a mirrored solution: every clicked point is behind the camera. "
            "Check that the clicks are on the tool tip and the poses match the frames")
    return rotation, translation


@dataclass
class FitResult:
    """Everything the report and the rig file need, including what could go wrong."""

    rig: "Rig"
    residuals_px: np.ndarray
    weights: np.ndarray
    covariance: np.ndarray
    free: list
    sigma: dict
    weakest: dict
    predicted: dict
    iterations: int
    converged: bool


def _theta0(observations, tip, K, dist):
    normalized = _normalized_pixels(observations, K, dist)
    world = []
    for observation in observations:
        T = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        world.append(T[:3, :3] @ np.asarray(tip, dtype=float) + T[:3, 3])
    rotation, translation = _dlt_pose(world, normalized)
    theta = np.zeros(15)
    theta[0:3] = log_so3(rotation)
    theta[3:6] = translation
    theta[6:9] = np.asarray(tip, dtype=float)
    return theta


def _levenberg_marquardt(model, theta, options):
    theta = np.asarray(theta, dtype=float).copy()
    residuals = model.residuals(theta)
    weights = model.weights(residuals)
    cost = float(np.sum(weights * residuals * residuals))
    damping = 1e-3
    converged = False
    iterations = 0
    jacobian = model.jacobian(theta)

    for iterations in range(1, options.max_iterations + 1):
        root = np.sqrt(weights)
        weighted_jacobian = jacobian * root[:, None]
        gradient = weighted_jacobian.T @ (residuals * root)
        normal = weighted_jacobian.T @ weighted_jacobian
        diagonal = np.maximum(np.diag(normal), 1e-12)
        system = normal + damping * np.diag(diagonal)
        try:
            step = -np.linalg.solve(system, gradient)
        except np.linalg.LinAlgError:
            step = -np.linalg.lstsq(system, gradient, rcond=None)[0]

        trial = theta.copy()
        trial[model.free] += step
        trial_residuals = model.residuals(trial)
        trial_weights = model.weights(trial_residuals)
        trial_cost = float(np.sum(trial_weights * trial_residuals * trial_residuals))

        if trial_cost < cost:
            improvement = cost - trial_cost
            theta = trial
            residuals = trial_residuals
            weights = trial_weights
            cost = trial_cost
            damping = max(damping / 3.0, 1e-12)
            jacobian = model.jacobian(theta)
            if improvement < 1e-10 * max(1.0, cost) or float(np.linalg.norm(step)) < 1e-11:
                converged = True
                break
        else:
            # A step that cannot improve the cost and is already negligible
            # means the fit has plateaued, which is what convergence looks like
            # on noisy clicks. Calling that "did not converge" would put a
            # scary line in front of an operator holding a 0.3 px fit.
            if float(np.linalg.norm(step)) < 1e-10:
                converged = True
                break
            damping *= 5.0
            if damping > 1e12:
                break
    return theta, residuals, weights, jacobian, iterations, converged


def _covariance(model, residuals, weights, jacobian):
    root = np.sqrt(weights)
    weighted = jacobian * root[:, None]
    normal = weighted.T @ weighted
    dof = model.size() - len(model.free)
    if dof <= 0:
        raise ExtrinsicsError(
            "%d poses give %d residuals for %d unknowns: add at least one more pose"
            % (len(model.q), model.size(), len(model.free)))
    variance = float(np.sum(weights * residuals * residuals)) / dof
    try:
        covariance = variance * np.linalg.inv(normal)
    except np.linalg.LinAlgError:
        covariance = variance * np.linalg.pinv(normal)
    return 0.5 * (covariance + covariance.T), variance


def _conditioning(covariance, free, names=NAMES):
    """Worst 1-sigma direction of the fit, named in the operator's units.

    A sweep can look fine and still leave one direction nearly free: all the
    poses at one height, or the camera almost on the tool axis. The largest
    eigenvalue of the covariance is that direction.
    """
    values, vectors = np.linalg.eigh(covariance)
    index = int(np.argmax(values))
    worst = vectors[:, index]
    order = np.argsort(-np.abs(worst))[:3]
    parts = ["%+.2f*%s" % (worst[i], names[free[i]]) for i in order]
    return {"direction": " ".join(parts), "sigma": math.sqrt(max(0.0, float(values[index]))),
            "sigma_mm": 1000.0 * math.sqrt(max(0.0, float(values[index])))}


def predicted_error(theta, covariance, free, K, dist, plane_z, options: FitOptions):
    """What a click costs, in millimetres, over the work area.

    The pixel is held fixed: the operator taps the true point, so the question is
    where that same pixel lands when the rig is off by its own covariance. This
    is the number the tag path used to report as a reprojection error, in the
    units the plan actually cares about.
    """
    T_cam_base = transform(rodrigues(theta[0:3]), theta[3:6])
    T_base_cam = invert(T_cam_base)
    K = np.asarray(K, dtype=float).reshape(3, 3)

    radius = float(options.work_radius)
    xs = np.linspace(0.30 - radius, 0.30 + radius, 5)
    ys = np.linspace(-radius, radius, 5)
    found = []

    for x in xs:
        for y in ys:
            point = np.array([x, y, float(plane_z)])
            if not 0.12 < float(np.linalg.norm(point[:2])) < max(0.75, radius + 0.2):
                continue
            seen = pixel(point, K, dist, T_cam_base)
            if seen is None:
                continue
            if options.image_size:
                width, height = options.image_size
                if not (0 <= seen[0] <= width and 0 <= seen[1] <= height):
                    continue
            try:
                anchor, _, _ = ray_plane(K, dist, T_base_cam, seen[0], seen[1], plane_z)
            except ExtrinsicsError:
                continue
            gradients = np.zeros((len(free), 3))
            for k, j in enumerate(free):
                step = 1e-4
                moved = np.array(theta, dtype=float)
                moved[j] += step
                camera = invert(transform(rodrigues(moved[0:3]), moved[3:6]))
                try:
                    shifted, _, _ = ray_plane(K, dist, camera, seen[0], seen[1], plane_z)
                except ExtrinsicsError:
                    continue
                gradients[k] = (shifted - anchor) / step
            spread = math.sqrt(max(0.0, float(np.trace(gradients.T @ covariance @ gradients))))
            found.append((float(x), float(y), spread))

    if not found:
        return {"centre_mm": None, "corner_mm": None, "points": 0}
    nearest = min(found, key=lambda item: math.hypot(item[0] - 0.30, item[1]))
    worst = max(found, key=lambda item: item[2])
    return {"centre_mm": round(1000.0 * nearest[2], 2),
            "corner_mm": round(1000.0 * worst[2], 2),
            "corner_at": [round(worst[0], 3), round(worst[1], 3)],
            "points": len(found)}


def fit(observations, K, dist, options: FitOptions | None = None, calib_fingerprint="",
        device="", mount="fixed") -> FitResult:
    """Fit `T_base<-cam` (and the tip offset) to clicked poses."""
    options = options or FitOptions()
    observations = list(observations)
    if len(observations) < MIN_POSES:
        raise ExtrinsicsError(
            "%d poses is not enough; the rig needs at least %d, spread over the work area and "
            "at a couple of heights" % (len(observations), MIN_POSES))

    model = _Model(observations, K, dist, options)
    if model.size() <= len(model.free):
        raise ExtrinsicsError(
            "%d poses give %d residuals for %d unknowns: add %d more poses, or stop refining "
            "the joint zeros" % (len(observations), model.size(), len(model.free),
                                 (len(model.free) - model.size()) // 2 + 1))
    if options.refine_zeros and model.size() - len(model.free) < 5:
        raise ExtrinsicsError(
            "refining the joint zeros fits %d unknowns, so it needs at least 9 poses to leave "
            "a usable covariance; this session has %d. Fit without --refine-zeros, or add poses"
            % (len(model.free), len(observations)))

    # The DLT has to start from object points, and the tip is one of the
    # unknowns, so run it twice: once at the tool frame, once at the fitted tip.
    tip = np.zeros(3)
    theta = None
    residuals = weights = jacobian = None
    iterations, converged = 0, False
    for _pass in range(2):
        theta = _theta0(observations, tip, K, dist)
        (theta, residuals, weights, jacobian, iterations,
         converged) = _levenberg_marquardt(model, theta, options)
        if float(np.linalg.norm(theta[6:9] - tip)) < 0.005:
            break
        tip = theta[6:9].copy()

    covariance, _variance = _covariance(model, residuals, weights, jacobian)
    sigma = {NAMES[j]: math.sqrt(max(0.0, float(covariance[k, k])))
             for k, j in enumerate(model.free)}
    weakest = _conditioning(covariance, model.free)
    predicted = predicted_error(theta, covariance, model.free, K, dist, options.plane_z, options)

    per_pose = np.linalg.norm(residuals.reshape(-1, 2), axis=1)
    rms_px = float(np.sqrt(np.mean(per_pose ** 2)))
    T_base_cam = invert(transform(rodrigues(theta[0:3]), theta[3:6]))

    # Pixel -> millimetre at the mean range: approximate, and labelled as such.
    ranges = [float(np.linalg.norm(T_base_cam[:3, 3] - model.tip_in_base(i, theta[6:9],
                  theta[9:15] if options.refine_zeros else None)))
              for i in range(len(observations))]
    focal = float(np.mean([K[0, 0], K[1, 1]]))
    metres_per_pixel = float(np.mean(ranges)) / focal if focal else 0.0

    if weakest["sigma_mm"] > options.max_weak_mm:
        raise ExtrinsicsError(
            "the sweep leaves %s free to %.0f mm (limit %.0f): move the arm through the "
            "workspace, vary the wrist orientation, and re-click"
            % (weakest["direction"], weakest["sigma_mm"], options.max_weak_mm))

    # A rig on a bench lives within a few metres of the arm, and the tool is a
    # gripper a few centimetres long. A fit outside that is not this scene: the
    # correspondences disagree, and the solver has run away. Said here, before the
    # rig can be written, rather than as a mysterious residual later.
    camera = np.asarray(T_base_cam[:3, 3], dtype=float)
    if float(np.linalg.norm(camera)) > MAX_CAMERA_M:
        raise ExtrinsicsError(
            "the fit ran away: the camera came out %.1f m from the base, where a bracket is "
            "within %.0f m. The detected points do not all belong to one arm"
            % (float(np.linalg.norm(camera)), MAX_CAMERA_M))
    tip_length = float(np.linalg.norm(np.asarray(theta[6:9], dtype=float)))
    if tip_length > MAX_TIP_M:
        raise ExtrinsicsError(
            "the fit put the tip %.0f mm from the flange, but a gripper's tip is a few "
            "centimetres: the points are not all the tool tip" % (1000.0 * tip_length))

    rig = Rig(
        T_base_cam=T_base_cam,
        calib_fingerprint=str(calib_fingerprint),
        image_size=tuple(int(v) for v in (options.image_size or (0, 0))),
        plane_z=float(options.plane_z),
        tip_offset=np.asarray(theta[6:9], dtype=float),
        joint_zeros=(np.asarray(theta[9:15], dtype=float) if options.refine_zeros else None),
        rms_px=rms_px,
        rms_mm=rms_px * metres_per_pixel * 1000.0,
        max_px=float(np.max(per_pose)),
        observations=observations,
        sigma=sigma,
        weakest=weakest,
        predicted=predicted,
        device=str(device),
        mount=str(mount),
        fitted=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        iterations=int(iterations),
        converged=bool(converged),
    )
    return FitResult(rig=rig, residuals_px=per_pose, weights=weights[::2],
                     covariance=covariance, free=list(model.free), sigma=sigma,
                     weakest=weakest, predicted=predicted, iterations=iterations,
                     converged=converged)


# --------------------------------------------------------------------------- #
# The rig file
# --------------------------------------------------------------------------- #

@dataclass
class Rig:
    """`T_base<-cam` plus everything needed to know when it is no longer true."""

    T_base_cam: np.ndarray
    calib_fingerprint: str = ""
    image_size: tuple = (0, 0)
    plane_z: float = 0.0
    tip_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    joint_zeros: np.ndarray | None = None
    rms_px: float = 0.0
    rms_mm: float = 0.0
    max_px: float = 0.0
    observations: list = field(default_factory=list)
    sigma: dict = field(default_factory=dict)
    weakest: dict = field(default_factory=dict)
    predicted: dict = field(default_factory=dict)
    device: str = ""
    mount: str = "fixed"
    fitted: str = ""
    iterations: int = 0
    converged: bool = True
    witness: dict | None = None
    path: Path | None = None

    @property
    def fingerprint(self):
        blob = json.dumps({
            "T": np.round(np.asarray(self.T_base_cam, float), 6).tolist(),
            "tip": np.round(np.asarray(self.tip_offset, float), 6).tolist(),
            "size": [int(v) for v in self.image_size],
            "calib": self.calib_fingerprint,
        }, sort_keys=True)
        return hashlib.sha256(blob.encode()).hexdigest()[:12]

    @property
    def age_days(self):
        if not self.fitted:
            return None
        try:
            fitted = calendar.timegm(time.strptime(self.fitted, "%Y-%m-%dT%H:%M:%SZ"))
        except (ValueError, OverflowError):
            return None
        return max(0.0, (time.time() - fitted) / 86400.0)

    def camera_origin(self):
        return np.asarray(self.T_base_cam, float)[:3, 3].copy()

    def describe(self):
        origin = self.camera_origin()
        down, yaw = camera_angles(self.T_base_cam)
        predicted = self.predicted or {}
        weak = self.weakest or {}
        bits = [
            "camera at [%+.3f %+.3f %+.3f] m, looking %.1f deg down, yaw %+.1f deg"
            % (origin[0], origin[1], origin[2], down, yaw),
            "tip [%+.1f %+.1f %+.1f] mm in gripper_end"
            % tuple(1000.0 * np.asarray(self.tip_offset, float)),
            "%d poses, rms %.2f px / %.1f mm, worst %.2f px%s"
            % (len(self.observations), self.rms_px, self.rms_mm, self.max_px,
               "" if self.converged else ", did NOT converge"),
        ]
        if predicted.get("centre_mm") is not None:
            bits.append("predicted %.1f mm at the centre, %.1f mm at the corner"
                        % (predicted["centre_mm"], predicted["corner_mm"]))
        if weak:
            bits.append("weakest %s (%.1f mm)" % (weak.get("direction", "?"),
                                                  weak.get("sigma_mm", 0.0)))
        if self.joint_zeros is not None:
            bits.append("joint zeros (diagnostic, not applied) %s mrad"
                        % " ".join("%+.1f" % (1000.0 * float(v)) for v in self.joint_zeros))
        if self.age_days is not None:
            bits.append("%.0f days old" % self.age_days)
        return "\n".join(bits)

    def provenance(self):
        """The block that goes into `target.json` and the trajectory."""
        predicted = self.predicted or {}
        return {
            "fingerprint": self.fingerprint,
            "mount": self.mount,
            "method": "tip-sweep",
            "rms_mm": round(float(self.rms_mm), 2),
            "predicted_mm": predicted.get("centre_mm"),
            "poses": len(self.observations),
            "fitted": self.fitted,
            "age_days": None if self.age_days is None else round(float(self.age_days), 1),
            # Where the camera is, so a refit (or the panel's form) can start
            # from it instead of asking for a measurement again.
            "camera_at": [round(float(v), 4) for v in np.asarray(self.T_base_cam, float)[:3, 3]],
        }

    def to_dict(self):
        return {
            "format": FORMAT,
            "version": VERSION,
            "T_base_cam": [[float(v) for v in row] for row in np.asarray(self.T_base_cam, float)],
            "calib_fingerprint": self.calib_fingerprint,
            "image_size": [int(v) for v in self.image_size],
            "plane_z": float(self.plane_z),
            "tip_offset": [float(v) for v in np.asarray(self.tip_offset, float)],
            "joint_zeros": (None if self.joint_zeros is None
                            else [float(v) for v in np.asarray(self.joint_zeros, float)]),
            "method": "tip-sweep",
            "rms_px": float(self.rms_px),
            "rms_mm": float(self.rms_mm),
            "max_px": float(self.max_px),
            "sigma": {str(k): float(v) for k, v in (self.sigma or {}).items()},
            "weakest": self.weakest or {},
            "predicted": self.predicted or {},
            "device": self.device,
            "mount": self.mount,
            "fitted": self.fitted,
            "iterations": int(self.iterations),
            "converged": bool(self.converged),
            "fingerprint": self.fingerprint,
            "witness": self.witness,
            "observations": [o.to_dict() for o in self.observations],
        }

    @classmethod
    def from_dict(cls, data, path=None):
        if data.get("format") != FORMAT:
            raise ExtrinsicsError("not a %s file (found %r)" % (FORMAT, data.get("format")))
        try:
            T = np.asarray(data["T_base_cam"], dtype=float).reshape(4, 4)
        except (KeyError, TypeError, ValueError) as error:
            raise ExtrinsicsError("T_base_cam must be a 4x4 matrix: %s" % error) from error
        if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
            raise ExtrinsicsError("T_base_cam's last row must be [0 0 0 1]")
        if abs(float(np.linalg.det(T[:3, :3])) - 1.0) > 1e-3:
            raise ExtrinsicsError("T_base_cam's rotation is not a rotation (det %.4f)"
                                  % float(np.linalg.det(T[:3, :3])))
        return cls(
            T_base_cam=T,
            calib_fingerprint=str(data.get("calib_fingerprint", "")),
            image_size=tuple(int(v) for v in (data.get("image_size") or (0, 0))),
            plane_z=float(data.get("plane_z", 0.0)),
            tip_offset=np.asarray(data.get("tip_offset", [0.0, 0.0, 0.0]), dtype=float),
            joint_zeros=(None if data.get("joint_zeros") is None
                         else np.asarray(data["joint_zeros"], dtype=float)),
            rms_px=float(data.get("rms_px", 0.0)),
            rms_mm=float(data.get("rms_mm", 0.0)),
            max_px=float(data.get("max_px", 0.0)),
            observations=[Observation.from_dict(item) for item in (data.get("observations") or [])],
            sigma=dict(data.get("sigma") or {}),
            weakest=dict(data.get("weakest") or {}),
            predicted=dict(data.get("predicted") or {}),
            device=str(data.get("device", "")),
            mount=str(data.get("mount", "fixed")),
            fitted=str(data.get("fitted", "")),
            iterations=int(data.get("iterations", 0)),
            converged=bool(data.get("converged", True)),
            witness=data.get("witness"),
            path=Path(path) if path else None,
        )


def load_rig(path):
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ExtrinsicsError("no extrinsics at %s (run calibrate_extrinsics.py --fit)" % path) from error
    except json.JSONDecodeError as error:
        raise ExtrinsicsError("%s is not JSON: %s" % (path, error)) from error
    return Rig.from_dict(data, path=path)


def save_rig(rig: Rig, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rig.path = path
    path.write_text(json.dumps(rig.to_dict(), indent=2) + "\n", encoding="utf-8")
    return path


def leave_one_out(observations, K, dist, options: FitOptions | None = None):
    """Refit without each pose and measure where its click lands.

    This is the only out-of-sample number a commissioning sweep has. It is a
    *diagnostic*: leaving one pose out of a thin design can leave a direction
    weak, so the degeneracy gate that protects a written rig is lifted here --
    the covariance still reports what the smaller set could not pin down.
    """
    options = options or FitOptions()
    options = replace(options, max_weak_mm=1e9)
    values = []
    for index, observation in enumerate(observations):
        subset = observations[:index] + observations[index + 1:]
        try:
            result = fit(subset, K, dist, options)
        except ExtrinsicsError:
            return None
        T = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        point = T[:3, :3] @ result.rig.tip_offset + T[:3, 3]
        seen = pixel(point, K, dist, invert(result.rig.T_base_cam))
        values.append(float("inf") if seen is None else
                      math.hypot(seen[0] - observation.uv[0], seen[1] - observation.uv[1]))
    return values


def verify_pair(calibration, rig):
    """Refuse a rig that was fitted with a different lens, sensor or crop.

    The pose itself does not care about the resolution -- K and the pixels scale
    together -- so a 2x downscale of the same sensor is fine. A different aspect
    ratio is not: the principal point of a crop is not the scaled one, and every
    click would be biased. A different device is never fine.
    """
    if not rig.image_size or int(rig.image_size[0]) <= 0:
        return                                   # hand-written rig: nothing to check
    rig_w, rig_h = float(rig.image_size[0]), float(rig.image_size[1])
    cal_w, cal_h = (float(v) for v in calibration.image_size)
    if abs((rig_w / rig_h) - (cal_w / cal_h)) / (rig_w / rig_h) > 0.01:
        raise ExtrinsicsError(
            "this rig was fitted at %dx%d but the camera is calibrated at %dx%d: the aspect "
            "ratio changed, so recalibrate the extrinsics for this capture size"
            % (rig_w, rig_h, cal_w, cal_h))
    if rig.device and calibration.device and rig.device != calibration.device:
        raise ExtrinsicsError(
            "this rig belongs to %r, the calibration to %r: a different camera cannot reuse it"
            % (rig.device, calibration.device))
