#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Camera intrinsics shared by the calibration and the target locator.

A phone is not one camera: front and back have different optics, the browser may
hand you 1280x720 for the preview and 4032x3024 for a still, and digital zoom
changes the focal length without changing the declared resolution. A `K`
measured at 1920x1080 and applied to a 1280x720 frame is wrong by the width
ratio, so both scripts go through `load()` / `scale()` instead of reading the
JSON by hand.

Only the pinhole model is stored. The phone's distortion is small at the ranges
this pipeline works at (0.6-1.5 m) and is fitted anyway when a checkerboard is
used; `dist` is kept so `cv2.projectPoints` and `solvePnP` agree with the fit.
"""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


class CalibError(Exception):
    """Intrinsics are missing, malformed, or for a different aspect ratio."""


def scale_K(K: np.ndarray, src_size, dst_size) -> np.ndarray:
    """Rescale a camera matrix between two image sizes of the same sensor crop.

    Refuses a change of aspect ratio: the principal point of a cropped stream is
    not the scaled principal point, and guessing here quietly biases every point
    that comes out of the pipeline.
    """
    src_w, src_h = float(src_size[0]), float(src_size[1])
    dst_w, dst_h = float(dst_size[0]), float(dst_size[1])
    if src_w <= 0 or src_h <= 0 or dst_w <= 0 or dst_h <= 0:
        raise CalibError("image size must be positive, got %s -> %s" % (src_size, dst_size))
    sx, sy = dst_w / src_w, dst_h / src_h
    if abs(sx - sy) / max(sx, sy) > 0.01:
        raise CalibError(
            "aspect ratio changed (%sx%s -> %sx%s); recalibrate instead of rescaling"
            % (int(src_w), int(src_h), int(dst_w), int(dst_h))
        )
    scaled = np.array(K, dtype=float).reshape(3, 3).copy()
    scaled[0, 0] *= sx
    scaled[1, 1] *= sy
    scaled[0, 2] *= sx
    scaled[1, 2] *= sy
    return scaled


@dataclass
class Calib:
    """A fitted pinhole camera for one phone camera at one capture size."""

    path: Path
    K: np.ndarray
    dist: np.ndarray
    image_size: tuple
    rms: float = 0.0
    device: str = ""
    raw: dict = field(default_factory=dict)

    @property
    def fingerprint(self) -> str:
        """Short stable id of this fit, for the trajectory's provenance block."""
        blob = json.dumps(
            {
                "K": np.round(self.K, 3).tolist(),
                "dist": np.round(self.dist, 6).tolist(),
                "size": [int(v) for v in self.image_size],
            },
            sort_keys=True,
        )
        return hashlib.sha256(blob.encode()).hexdigest()[:12]

    def describe(self) -> str:
        w, h = self.image_size
        return (
            "%dx%d  fx %.1f  fy %.1f  cx %.1f  cy %.1f  rms %.3f px  [%s]"
            % (w, h, self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2],
               self.rms, self.fingerprint)
        )


def load(path, image_size=None) -> Calib:
    """Read a calibration JSON and rescale it to `image_size` if one is given."""
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CalibError("no calibration at %s (run calibrate_intrinsics.py)" % path) from exc
    except json.JSONDecodeError as exc:
        raise CalibError("%s is not JSON: %s" % (path, exc)) from exc

    try:
        K = np.array(data["K"], dtype=float).reshape(3, 3)
        dist = np.array(data.get("dist", [0.0] * 5), dtype=float).reshape(-1)
        size = (int(data["image_size"][0]), int(data["image_size"][1]))
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibError("%s: expected K, dist and image_size, got %s" % (path, exc)) from exc

    if image_size is not None and (int(image_size[0]), int(image_size[1])) != size:
        K = scale_K(K, size, image_size)
        size = (int(image_size[0]), int(image_size[1]))

    return Calib(
        path=path,
        K=K,
        dist=dist,
        image_size=size,
        rms=float(data.get("rms", 0.0)),
        device=str(data.get("device", "")),
        raw=data,
    )


def slug(text: str, fallback: str = "camera") -> str:
    """Filesystem-safe name for a per-device calibration file."""
    cleaned = re.sub(r"[^a-z0-9]+", "-", (text or "").lower()).strip("-")
    return (cleaned or fallback)[:64]
