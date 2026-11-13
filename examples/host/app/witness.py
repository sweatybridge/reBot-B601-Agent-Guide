#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Did the camera move? The tag used to answer that for free.

With a tag in the frame, stale extrinsics were impossible: the pose was measured
on the same pixels as the tap. A bolted-down camera gives that up, and a bracket
that gets knocked by a shoulder, a cable or a cleaning cloth biases every plan
afterwards with nothing in the log to show for it.

The witness is the replacement. At calibration time it stores the frame and a
grid of high-contrast patches; on every capture it tracks those patches and
measures how far the *background* moved. The arm is masked out of the patch
selection with the kinematic model, and it moves anyway, so what is left is the
scene behind it -- which only moves if the camera does.

Two things this catches that a timestamp cannot: a knock (a translation or a
rotation of the whole view) and a focus or zoom change (a scale in the
homography). It cannot catch a change in the lens itself; nothing can.

    drift_px  median residual of the tracked background, in pixels
    scale     the homography's scale; 1.00 is the lens it was fitted with
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import rebot_ik

# The same capsule radii the manuals' collision probe uses
# (manuals/_build/under_cabinet_probe.py). Only used to mask the arm out of the
# feature selection, so 5-10 mm of slack is harmless.
LINK_RADII = (0.055, 0.055, 0.050, 0.045, 0.035, 0.030, 0.020)


class WitnessError(Exception):
    """The reference frame is missing or cannot be compared with this capture."""


@dataclass
class Reference:
    """The frame the rig was fitted on, and the patches to track in it."""

    gray: np.ndarray
    points: np.ndarray
    image_size: tuple
    created: str = ""
    path: Path | None = None

    def to_dict(self):
        return {"points": [[float(u), float(v)] for u, v in self.points],
                "image_size": [int(v) for v in self.image_size],
                "created": self.created or time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}


def arm_mask(image_size, rig, q, K, dist, padding=10):
    """A 0/255 mask that blanks the arm, from the same model the fit uses.

    The arm is in every calibration frame and it is the one thing in the scene
    that is *supposed* to move. Masking it keeps the witness patches on the
    worktop and the wall, where a shift means the camera moved.
    """
    width, height = int(image_size[0]), int(image_size[1])
    mask = np.full((height, width), 255, np.uint8)
    if rig is None or q is None:
        return mask
    try:
        _tool_pose, frames, links = rebot_ik.forward(np.asarray(q, dtype=float).reshape(6),
                                                    with_frames=True, with_links=True)
    except (ValueError, TypeError):
        return mask

    import extrinsics as ex

    T_cam_base = ex.invert(np.asarray(rig.T_base_cam, dtype=float))
    points = [np.asarray(p, dtype=float) for p, _z in frames]
    points.append(links[-1][:3, 3])
    projected = []
    for point in points:
        seen = ex.pixel(point, K, dist, T_cam_base)
        projected.append(seen)

    for index in range(1, len(points) - 1):
        start, end = projected[index], projected[index + 1]
        if start is None or end is None:
            continue
        depth = float(np.linalg.norm(points[index] - T_cam_base[:3, 3]))
        radius_px = LINK_RADII[min(index, len(LINK_RADII) - 1)] * float(K[0, 0]) / max(depth, 1e-3)
        thickness = int(max(6, 2.0 * radius_px + 2.0 * padding))
        # `cv2.line` leaves a square cap half a thickness past each endpoint, so
        # a capsule drawn with it is one radius too long at every joint. Round
        # caps make it the capsule the radii describe.
        cv2.line(mask, (int(round(start[0])), int(round(start[1]))),
                 (int(round(end[0])), int(round(end[1]))), 0, thickness)
        for point in (start, end):
            cv2.circle(mask, (int(round(point[0])), int(round(point[1]))),
                       max(3, thickness // 2), 0, -1)
    return mask


def select(gray, count=64, min_distance=28, mask=None, border=24):
    """High-contrast patches, away from the image edge and away from the arm."""
    usable = np.ones_like(gray, dtype=np.uint8) * 255
    if mask is not None:
        usable = cv2.bitwise_and(usable, mask)
    usable[:border, :] = 0
    usable[-border:, :] = 0
    usable[:, :border] = 0
    usable[:, -border:] = 0
    if int(np.count_nonzero(usable)) < 400:
        raise WitnessError("not enough usable scene to pick witness patches (is the arm filling "
                           "the frame?)")
    points = cv2.goodFeaturesToTrack(gray, maxCorners=int(count), qualityLevel=0.01,
                                     minDistance=int(min_distance), mask=usable,
                                     blockSize=7)
    if points is None or len(points) < 8:
        raise WitnessError("only %d witness features found; a textured background is needed"
                           % (0 if points is None else len(points)))
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    # Strongest first, so a reference built with a different cap is a prefix.
    return points


def build(image, rig=None, q=None, K=None, dist=None, count=64, border=24):
    """A reference from a calibration frame, with the arm kept out of it."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = None
    if rig is not None and q is not None and K is not None:
        mask = arm_mask(gray.shape[::-1], rig, q, K, dist)
    points = select(gray, count=count, mask=mask, border=border)
    return Reference(gray=gray.copy(), points=points,
                     image_size=(int(gray.shape[1]), int(gray.shape[0])),
                     created=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))


def save(reference, image_path):
    image_path = Path(image_path)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(image_path), reference.gray, [cv2.IMWRITE_JPEG_QUALITY, 92])
    reference.path = image_path
    return image_path


def load(image_path, points, image_size=None):
    image_path = Path(image_path)
    gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise WitnessError("cannot read the witness frame %s" % image_path)
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(points) < 8:
        raise WitnessError("the witness frame has only %d patches" % len(points))
    if image_size and tuple(int(v) for v in image_size) != (gray.shape[1], gray.shape[0]):
        raise WitnessError("the witness frame is %dx%d but records %s"
                           % (gray.shape[1], gray.shape[0], image_size))
    return Reference(gray=gray, points=points,
                     image_size=(int(gray.shape[1]), int(gray.shape[0])),
                     path=image_path)


def check(reference: Reference, image, max_drift_px=2.0):
    """How far the background moved since the reference, in pixels.

    Returns a dict rather than raising for a normal drift: the caller decides
    whether 1.2 px is a warning or a refusal. A *failed* comparison (nothing
    tracked, a different image size) is an exception, because that is a setup
    error and not a measurement.
    """
    if image is None:
        raise WitnessError("no frame to check the mount against")
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if gray.shape != reference.gray.shape:
        raise WitnessError("the capture is %dx%d but the witness frame is %dx%d: the capture "
                           "size changed, which invalidates the rig anyway"
                           % (gray.shape[1], gray.shape[0],
                              reference.gray.shape[1], reference.gray.shape[0]))

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
    forward, status, _error = cv2.calcOpticalFlowPyrLK(
        reference.gray, gray, reference.points, None, winSize=(21, 21), maxLevel=3,
        criteria=criteria)
    if forward is None:
        raise WitnessError("optical flow failed")
    back, status_back, _error = cv2.calcOpticalFlowPyrLK(
        gray, reference.gray, forward, None, winSize=(21, 21), maxLevel=3, criteria=criteria)
    if back is None:
        raise WitnessError("optical flow failed on the way back")
    forward = forward.reshape(-1, 2)
    back = back.reshape(-1, 2)

    keep = ((status.reshape(-1) == 1) & (status_back.reshape(-1) == 1)
            & (np.linalg.norm(back - reference.points, axis=1) < 0.6))
    source = reference.points[keep]
    target = forward[keep]
    if len(source) < 8:
        raise WitnessError("only %d of %d witness patches are still visible: the view changed "
                           "enough that the mount cannot be verified"
                           % (len(source), len(reference.points)))

    homography, inliers = cv2.findHomography(source.reshape(-1, 1, 2),
                                             target.reshape(-1, 1, 2), cv2.RANSAC, 2.0)
    if homography is None or inliers is None or int(inliers.sum()) < 8:
        raise WitnessError("the background does not match the witness frame: either the camera "
                           "moved a long way, or it is pointed somewhere else")

    inlier_mask = inliers.reshape(-1).astype(bool)
    predicted = cv2.perspectiveTransform(
        source[inlier_mask].reshape(-1, 1, 2), homography).reshape(-1, 2)
    # The homography's own residual is small for *any* consistent motion, so it
    # says nothing about drift. What matters is where the background went:
    # |H p - p|, the displacement of each patch from where it used to be.
    fit_error = np.linalg.norm(predicted - target[inlier_mask], axis=1)
    displacement = np.linalg.norm(predicted - source[inlier_mask], axis=1)
    drift = float(np.median(displacement))
    scale = float(np.sqrt(abs(np.linalg.det(homography[:2, :2]))))

    reasons = []
    if drift > max_drift_px:
        reasons.append("%.1f px of drift (limit %.1f)" % (drift, max_drift_px))
    if abs(scale - 1.0) > 0.05:
        reasons.append("the field of view changed by %.0f%% (focus or zoom)"
                       % (100.0 * (scale - 1.0)))
    return {
        "ok": not reasons,
        "drift_px": round(drift, 2),
        "max_px": round(float(np.max(displacement)), 2),
        "fit_px": round(float(np.median(fit_error)), 2),
        "scale": round(scale, 4),
        "inliers": int(inlier_mask.sum()),
        "patches": int(len(source)),
        "reference": str(reference.path) if reference.path else None,
        "reason": "; ".join(reasons) or None,
    }


def millimetres_per_pixel(rig, K, point=(0.30, 0.0, 0.0)):
    """Rough pixel-to-millimetre at the work area, for the operator's message."""
    if rig is None:
        return None
    K = np.asarray(K, dtype=float)
    focal = float(np.mean([K[0, 0], K[1, 1]]))
    if focal <= 0:
        return None
    distance = float(np.linalg.norm(np.asarray(rig.T_base_cam, float)[:3, 3]
                                    - np.asarray(point, dtype=float)))
    return 1000.0 * distance / focal


def load_for(rig, base_dir):
    """The reference stored in a rig file, if it has one.

    No witness block is not an error -- the operator may have fitted without
    one. A witness block whose frame cannot be read *is* an error: the rig was
    fitted expecting the mount check, and running without it silently is exactly
    the failure this module exists to prevent.
    """
    if rig is None or not rig.witness:
        return None
    record = rig.witness
    image = record.get("image")
    if not image:
        raise WitnessError("the rig records a witness with no image")
    path = Path(image)
    if not path.is_absolute():
        path = Path(base_dir) / path
    return load(path, record.get("points") or [], record.get("image_size"))
