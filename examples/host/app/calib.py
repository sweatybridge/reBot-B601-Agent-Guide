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


def same_shape(a, b) -> bool:
    """Do two image sizes describe the same rectangle (same aspect ratio)?

    A quarter turn exchanges the sides and fails this test; that case is
    `swapped_shape`'s question.
    """
    aw, ah = float(a[0]), float(a[1])
    bw, bh = float(b[0]), float(b[1])
    if aw <= 0 or ah <= 0 or bw <= 0 or bh <= 0:
        return False
    ratio = (aw / ah) / (bw / bh)
    return abs(ratio - 1.0) <= 0.01


def swapped_shape(a, b) -> bool:
    """Is `b` the quarter-turn of `a` (dimensions exchanged)?"""
    aw, ah = float(a[0]), float(a[1])
    bw, bh = float(b[0]), float(b[1])
    if aw <= 0 or ah <= 0 or bw <= 0 or bh <= 0:
        return False
    return same_shape((aw, ah), (bh, bw)) and abs(aw / ah - ah / aw) > 0.01


def turns_for_shapes(calib_size, frame_size) -> int:
    """Quarter turns (clockwise) the frame is rotated relative to the calibration.

    Either 0 (same shape, maybe another resolution) or an odd number -- but from
    dimensions alone a quarter turn has no sign: 90 and 270 both exchange the
    sides. Callers that know the device orientation pass it in; this returns 1 as
    the documented default and says so.
    """
    if swapped_shape(calib_size, frame_size):
        return 1
    if same_shape(calib_size, frame_size):
        return 0
    raise CalibError(
        "the frame is %sx%s but the calibration is %sx%s; that is neither the same "
        "aspect ratio nor a quarter turn of it"
        % (int(frame_size[0]), int(frame_size[1]),
           int(calib_size[0]), int(calib_size[1])))


def map_pixel(u, v, size, turns):
    """A pixel after `turns` clockwise quarter turns of an image of `size`.

    `size` is the size of the image the pixel is measured in. One turn maps
    (u, v) -> (h - v, u) and swaps the sides; the inverse is three turns, so a
    caller never needs a second formula:

        calibrated = map_pixel(u_frame, v_frame, frame_size, (4 - turns) % 4)
        frame      = map_pixel(u_cal, v_cal, calib_size, turns)
    """
    width, height = float(size[0]), float(size[1])
    for _ in range(int(turns) % 4):
        u, v = height - v, u
        width, height = height, width
    return u, v


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
    # Linear ratio of this grid to the on-disk fit. 1.0 is the native grid; a
    # WebRTC bandwidth drop delivers the same sensor crop at another size, and
    # `rescaled` tracks the ratio so pixel-denominated gates can follow it.
    scale: float = 1.0

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

    @property
    def source_fingerprint(self) -> str:
        """The on-disk fit's id, invariant under `rescaled` / `load(image_size=...)`.

        Provenance (rigs, targets, sag-map keys) names the fit, not the grid a
        particular frame happened to arrive on: a bandwidth drop must not
        re-key the rig under every learned map that references it.
        """
        if not self.raw.get("K") or not self.raw.get("image_size"):
            return self.fingerprint
        blob = json.dumps(
            {
                "K": np.round(np.array(self.raw["K"], dtype=float).reshape(3, 3), 3).tolist(),
                "dist": np.round(np.array(self.raw.get("dist", [0.0] * 5), dtype=float)
                                 .reshape(-1), 6).tolist(),
                "size": [int(v) for v in self.raw["image_size"]],
            },
            sort_keys=True,
        )
        return hashlib.sha256(blob.encode()).hexdigest()[:12]

    def rescaled(self, image_size) -> "Calib":
        """The same fit on another grid of the same sensor crop.

        A no-op at the current size. `scale_K` refuses an aspect change: a
        cropped or zoomed stream is a different camera, not a rescale.
        """
        size = (int(image_size[0]), int(image_size[1]))
        if size == (int(self.image_size[0]), int(self.image_size[1])):
            return self
        K = scale_K(self.K, self.image_size, size)
        return Calib(path=self.path, K=K, dist=self.dist, image_size=size,
                     rms=self.rms, device=self.device, raw=self.raw,
                     scale=self.scale * size[0] / float(self.image_size[0]))

    def describe(self) -> str:
        w, h = self.image_size
        return (
            "%dx%d  fx %.1f  fy %.1f  cx %.1f  cy %.1f  rms %.3f px  [%s]"
            % (w, h, self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2],
               self.rms, self.fingerprint)
        )


def load(path, image_size=None) -> Calib:
    """Read a calibration JSON and rescale it to `image_size` if one is given.

    `image_size` must be the same shape as the calibration, possibly at another
    resolution. A frame that is a quarter turn of it is not a rescale: load it
    without `image_size` and map the pixels with `map_pixel`, so `K` keeps
    describing the grid it was measured on.
    """
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

    calib = Calib(
        path=path,
        K=K,
        dist=dist,
        image_size=size,
        rms=float(data.get("rms", 0.0)),
        device=str(data.get("device", "")),
        raw=data,
    )
    if image_size is not None and (int(image_size[0]), int(image_size[1])) != size:
        if swapped_shape(size, image_size):
            raise CalibError(
                "the frame (%sx%s) is a quarter turn of the calibration (%sx%s): load "
                "without image_size and map the pixels instead of rescaling"
                % (int(image_size[0]), int(image_size[1]), int(size[0]), int(size[1])))
        calib = calib.rescaled(image_size)
    return calib


def slug(text: str, fallback: str = "camera") -> str:
    """Filesystem-safe name for a per-device calibration file."""
    cleaned = re.sub(r"[^a-z0-9]+", "-", (text or "").lower()).strip("-")
    return (cleaned or fallback)[:64]
