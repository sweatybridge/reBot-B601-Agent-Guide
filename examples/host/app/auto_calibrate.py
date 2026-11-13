#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive the fixed sweep, take the stills, find the tip, fit the rig.

This is the automatic version of the commissioning flow: one call moves the arm
through `sweep.POSES`, grabs a frame at each pose with the joint angles the arm
actually had, finds the tool tip in the picture, and fits `T_base<-cam`.

The tip is found without anyone clicking, and without a marker:

1. the pose is known, so the tool's approximate pixel is known -- `--camera-at`
   only has to be right to a few tens of centimetres for that;
2. the *background* for each pose is the median of the other seven frames, so
   whatever moved is the arm and nothing else;
3. the tip is the point of that blob furthest along the wrist->tool axis. The
   criterion is translation-invariant, which is why a rough camera position is
   enough, and it locks onto the same physical corner every time. That last part
   is what matters: the fit solves for the tip's offset in `gripper_end`, so a
   consistent point is as good as the true tip.

Two passes: detect with the bootstrap pose, fit, then re-detect against the
fitted pose (a much tighter projection) and fit again.

Nothing here decides that the result is *good*. `verify()` does, with the same
leave-one-out the manual flow uses, and the caller is expected to refuse a rig
that fails it.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

import extrinsics as ex
import mediamtx
import rebot_ik
import sweep
import sweep_video
import witness


class AutoError(Exception):
    """The sweep could not run, or a pose produced no usable tip."""


# A detection this close to where the camera pose says the tip should be is
# "clean". Above it, the detector found *something* -- and on a real bench that
# something is often a shadow or a highlight, so it is not evidence.
CLOSE_PX = 40.0

# The arm's lime covers in HSV, and how much of them has to be in frame before
# the colour detector is worth believing (a parked arm can be mostly hidden).
GREEN_LOW = (30, 70, 45)
GREEN_HIGH = (95, 255, 255)
MIN_GREEN_PX = 2000
GREEN_MIN_BLOB_PX = 600.0      # a link's cover, not a highlight
GREEN_NEAREST = 0.4            # the blob has to sit within this of the projection


# --------------------------------------------------------------------------- #
# Talking to the arm gateway
# --------------------------------------------------------------------------- #

class ArmGateway:
    """The host's only route to the arm: `/pose` and `/move` on the gateway."""

    def __init__(self, url, token="", timeout=600.0):
        self.url = str(url).rstrip("/")
        self.token = token or ""
        self.timeout = float(timeout)

    def _call(self, method, path, body=None, timeout=None):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        headers = {"Content-Type": "application/json"} if data else {}
        if self.token:
            headers["X-Rebot-Token"] = self.token
        request = urllib.request.Request(self.url + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as answer:
                return answer.status, json.loads(answer.read() or b"{}")
        except urllib.error.HTTPError as error:
            try:
                payload = json.loads(error.read() or b"{}")
            except (ValueError, OSError):
                payload = {"error": "the gateway answered %s" % error.code}
            return error.code, payload
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise AutoError("cannot reach the arm gateway at %s: %s" % (self.url, error)) from error

    def pose(self):
        status, payload = self._call("GET", "/pose", timeout=30)
        if status != 200:
            raise AutoError(payload.get("error", "the gateway refused /pose"))
        pose = payload.get("pose") or {}
        if not isinstance(pose.get("q"), list) or len(pose["q"]) < 6:
            raise AutoError("the gateway's pose has no joint angles")
        return pose

    def move(self, q6, seconds, hold=0.0):
        """Ask for one joint move and wait for it; `hold` keeps the pose at the end.

        The call blocks until the board has taken the arm out, held it and walked
        it back, so callers that need to photograph the pose run it in a thread.
        """
        status, payload = self._call(
            "POST", "/move", {"q": [float(v) for v in q6], "seconds": float(seconds),
                              "hold": float(hold)},
            timeout=max(120.0, 4.0 * (float(seconds) + float(hold))))
        if status != 200:
            raise AutoError(payload.get("error", "the gateway refused the move"))
        return payload

    def move_sequence(self, targets, seconds, hold=0.0):
        """Walk through `targets` in one trajectory and wait for the whole pass.

        The board returns the arm to the pose it started from when the file ends,
        so this is one round trip for the sweep: out through every pose with a hold
        at each, and back to park once.
        """
        body = {"targets": [[float(v) for v in target] for target in targets],
                "seconds": [float(value) for value in seconds],
                "hold": float(hold)}
        total = float(sum(body["seconds"])) + float(hold) * len(body["targets"])
        status, payload = self._call("POST", "/move", body, timeout=max(600.0, 4.0 * total))
        if status != 200:
            raise AutoError(payload.get("error", "the gateway refused the sweep"))
        return payload


# --------------------------------------------------------------------------- #
# Finding the tip
# --------------------------------------------------------------------------- #

@dataclass
class DetectOptions:
    threshold: int = 26            # grey-levels between the arm and its background
    roi_px: int = 340              # half-width of the search window
    profile_px: float = 10.0       # half-width of the perpendicular window on the axis
    min_area: float = 120.0        # px, smaller blobs are noise
    max_offset_px: float = 260.0   # how far the tip may be from the projection
    band_px: float = 6.0           # the distal cap: this deep in, and averaged
    walk: bool = False             # walk the axis instead of taking the contour
    # Template matching, when the tap came with its image: the gripper's patch,
    # searched around the projection. Off by default -- measured on the real
    # gripper it scores 0.5-0.7 as the wrist turns the tool out of plane, which is
    # not enough to converge, so the silhouette detector stays in charge until
    # this beats it on a recorded sweep.
    template: bool = False
    patch_px: int = 120            # the crop around the tip, both sides
    angles: tuple = (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)
    min_score: float = 0.45        # a match below this is not the gripper
    # The arm's own colour, which no bench object shares. Default on: it is the
    # only detector that does not depend on the gripper's appearance or on a
    # background, and it falls back to the silhouette where the colour is absent.
    green: bool = True


@dataclass
class Options:
    """Everything the automatic run needs beyond the camera's intrinsics."""

    plane_z: float = 0.0
    settle_s: float = 0.5
    still_rad: float = 0.008
    detect: DetectOptions = field(default_factory=DetectOptions)
    work_radius: float = 0.55
    # "median" differences each pose against the other seven. "auto" also takes
    # one frame at the parked pose before the sweep: if the camera cannot see the
    # parked arm, that is a clean background and the silhouette is exact. A path
    # to an arm-free image is the third option, and the best one.
    background: str = "median"
    # The one tap: {"q": [...], "uv": [u, v]}. This is the only way in -- the
    # sweep searches the family of poses that tap allows and lets its own
    # detections pick one.
    click: dict | None = None


@dataclass
class Detection:
    u: float
    v: float
    offset_px: float
    area: float
    pose: int
    # How well the gripper template matched, 0..1. Zero for the silhouette
    # detector, which has no such number.
    score: float = 0.0


@dataclass
class RunResult:
    observations: list
    captures: list
    detections: list
    bootstrap: np.ndarray
    fit: "ex.FitResult"
    seconds: float
    first_frame: np.ndarray | None = None
    first_pose: np.ndarray | None = None
    warnings: list = field(default_factory=list)


def _gray(frame):
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _tool_pixels(q, K, dist, T_base_cam, t_tip):
    """Where the wrist, the last joint and the tip should be, in the image."""
    tool = rebot_ik.forward(np.asarray(q, dtype=float))
    wrist = tool[:3, 3]
    tip = tool[:3, :3] @ np.asarray(t_tip, dtype=float) + wrist
    joint6 = rebot_ik.forward(np.asarray(q, dtype=float), with_frames=True)[1][-1][0]
    T_cam_base = ex.invert(np.asarray(T_base_cam, dtype=float))
    return (ex.pixel(wrist, K, dist, T_cam_base),
            ex.pixel(tip, K, dist, T_cam_base),
            ex.pixel(joint6, K, dist, T_cam_base))


def _window(centre, half, size):
    width, height = int(size[0]), int(size[1])
    u, v = int(round(float(centre[0]))), int(round(float(centre[1])))
    u0, u1 = max(0, u - int(half)), min(width, u + int(half))
    v0, v1 = max(0, v - int(half)), min(height, v + int(half))
    if u1 - u0 < 48 or v1 - v0 < 48:
        raise AutoError("the tool projects to (%d, %d), outside the frame: the camera position "
                        "is wrong, or the arm is not in view" % (u, v))
    return u0, v0, u1, v1


def _profile_tip(difference, wrist_local, direction, options: DetectOptions):
    """Walk the tool axis outward and stop where the arm stops.

    The point that comes back is on the projected axis by construction: no
    perpendicular bias from a blob's shape, which is what made a contour's
    extreme point drift by tens of millimetres as the link foreshortened. Only
    the *distance along the axis* is measured from the image, and that offset
    is a constant in the tool frame, so the fit absorbs it as `tip_offset`.

    Averaging across the perpendicular window keeps a single textured pixel
    from ending the walk early, and the run has to break for four samples
    before it counts as the end.
    """
    normal = np.array([-direction[1], direction[0]])
    offsets = np.arange(-options.profile_px, options.profile_px + 1.0, 3.0)
    distances = np.arange(6.0, float(options.roi_px), 2.0)
    height, width = difference.shape

    profile = []
    for distance in distances:
        centre = wrist_local + distance * direction
        points = centre + offsets[:, None] * normal
        columns = np.clip(points[:, 0].astype(int), 0, width - 1)
        rows = np.clip(points[:, 1].astype(int), 0, height - 1)
        profile.append(float(np.mean(difference[rows, columns])))
    profile = np.asarray(profile)

    end, missing = None, 0
    for index, value in enumerate(profile):
        if value > options.threshold:
            end, missing = index, 0
        elif end is not None:
            missing += 1
            if missing >= 4:
                break
    if end is None:
        return None
    return wrist_local + float(distances[end]) * direction


def detect_tip(gray, behind, q, K, dist, T_base_cam, t_tip, options: DetectOptions, size):
    """The tip of the arm in this frame, or raise with what was wrong.

    `behind` is the same view with the arm out of the way: either a frame taken
    with the arm parked clear, or the median of the *other* poses, which cannot
    contain this pose's tool. The difference is then the arm and nothing else.

    The tip is the blob's extreme point along the last link. That criterion does
    not care about a constant offset in the bootstrap pose -- only a constant
    *rotation* would matter, and the second pass fixes that.
    """
    wrist, tip, joint6 = _tool_pixels(q, K, dist, T_base_cam, t_tip)
    if tip is None or wrist is None:
        raise AutoError("the tool projects behind the camera; check --camera-at")

    # The last link's own direction. It is short in pixels -- 35 mm of tool is
    # ~20 px at a metre -- but it is the direction the distal band is defined
    # against. Base-to-tip was tried as a more stable substitute and is worse:
    # when the elbow is bent the last link is at an angle to it, so the band
    # slices across the link and the centroid slides.
    direction = np.asarray(tip, dtype=float) - np.asarray(joint6 or wrist, dtype=float)
    length = float(np.linalg.norm(direction))
    if length < 1.0:
        direction = np.asarray(tip, dtype=float) - np.asarray(wrist, dtype=float)
        length = float(np.linalg.norm(direction))
    if length < 1.0:
        raise AutoError("the tool projects to a single pixel; check --camera-at")
    direction = direction / length

    u0, v0, u1, v1 = _window(tip, options.roi_px, size)
    region = gray[v0:v1, u0:u1]
    difference = cv2.absdiff(region, behind[v0:v1, u0:u1])
    difference = cv2.GaussianBlur(difference, (5, 5), 0)
    target = np.asarray(tip, dtype=float) - np.array([u0, v0], dtype=float)
    wrist_local = np.asarray(wrist, dtype=float) - np.array([u0, v0], dtype=float)

    # Preferred: measure one distance, along the axis, so the point cannot slide
    # sideways; a silhouette point slides by the tool's half-width as the link
    # foreshortens. The contour method below is the fallback for a pose where
    # the walk finds an arm that never ends (a reflection, or a background that
    # happens to move).
    walk = _profile_tip(difference, wrist_local, direction, options) if options.walk else None
    if walk is not None:
        offset = float(np.linalg.norm(walk - target))
        if offset <= options.max_offset_px:
            return Detection(u=float(walk[0] + u0), v=float(walk[1] + v0),
                             offset_px=offset, area=0.0, pose=0)

    _level, mask = cv2.threshold(difference, int(options.threshold), 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    best = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < options.min_area:
            continue
        points = contour.reshape(-1, 2).astype(float)
        along = (points - wrist_local) @ direction
        top = float(np.max(along))
        # The silhouette's extreme point is the *outer* edge of the tool, not a
        # point on the axis, and a single extremum lands on a corner. The
        # centroid of the distal band is the axis end. It sits one tool radius
        # beyond `gripper_end` in 3D, which is a constant and therefore becomes
        # part of the fitted `tip_offset` -- it does not bias the camera.
        band = points[along >= top - options.band_px]
        far = band.mean(axis=0)
        offset = float(np.linalg.norm(far - target))
        if offset > options.max_offset_px:
            continue
        # The arm is the *big* blob in the window; a speck of a highlight 90 px
        # away is not, however far along the axis it happens to sit. Picking by
        # "furthest along" let those specks win (a 536 px blob beat the arm on a
        # real sweep), so size decides and the projection only breaks ties.
        key = (area, -offset)
        if best is None or key > (best[0], -best[3]):
            best = (area, far, area, offset)

    if best is None:
        raise AutoError(
            "no arm-like blob near the projected tool tip (window %dx%d at %d, %d): the "
            "threshold may be too high, the background too busy, or the camera position off "
            "by more than %.0f cm" % (u1 - u0, v1 - v0, u0, v0, options.max_offset_px / 10.0))
    _along, far, area, offset = best
    return Detection(u=float(far[0] + u0), v=float(far[1] + v0), offset_px=offset,
                     area=area, pose=0)


def _project_tip(q, K, dist, T_base_cam, t_tip):
    """The tool's pixel in this pose, or None if it is behind the camera."""
    _wrist, tip, _sixth = _tool_pixels(q, K, dist, T_base_cam, t_tip)
    return tip


def _save_debug(frames, poses, uvs, T_base_cam, t_tip, K, dist, options, size, directory,
                indices):
    """Write the search window, the projection and the point kept, on the frame.

    A pixel count cannot say whether the blob was the gripper, a shadow or the
    tissue box. The picture can, and it survives the run.
    """
    names = []
    for index in indices:
        image = frames[index].copy()
        seen = _project_tip(poses[index], K, dist, T_base_cam, t_tip)
        if seen is not None:
            u0, v0, u1, v1 = _window(seen, options.roi_px, size)
            cv2.rectangle(image, (u0, v0), (u1, v1), (0, 220, 0), 2)
            cv2.drawMarker(image, (int(seen[0]), int(seen[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 30, 2)
        u, v = uvs[index]
        cv2.drawMarker(image, (int(u), int(v)), (255, 0, 0), cv2.MARKER_CROSS, 30, 2)
        path = Path(directory) / ("debug-pose-%d.jpg" % (index + 1))
        cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        names.append(path.name)
    return names


def tip_templates(image, uv, options: DetectOptions):
    """The gripper around the tap, at a few in-plane rotations.

    The tap is the only place the gripper's appearance is known, so the crop
    around it *is* the model: the tip sits at the patch's centre by construction.
    Between poses the wrist turns, so the same object can arrive rotated; rotating
    the patch covers the in-plane part of that for a few ms a frame.
    """
    if isinstance(image, (str, Path)):
        loaded = cv2.imread(str(image), cv2.IMREAD_COLOR)
        if loaded is None:
            raise AutoError("cannot read the tap image at %s" % image)
        image = loaded
    gray = _gray(image)
    half = max(16, int(options.patch_px) // 2)
    height, width = gray.shape
    u, v = int(round(float(uv[0]))), int(round(float(uv[1])))
    u0 = min(max(0, u - half), max(0, width - 2 * half))
    v0 = min(max(0, v - half), max(0, height - 2 * half))
    patch = gray[v0:v0 + 2 * half, u0:u0 + 2 * half]
    if patch.shape[0] < 8 or patch.shape[1] < 8:
        raise AutoError("the tap is too close to the frame's edge to cut a template from")
    centre = (float(u - u0), float(v - v0))
    templates = []
    for angle in options.angles:
        if abs(float(angle)) < 1e-6:
            templates.append(patch)
            continue
        matrix = cv2.getRotationMatrix2D(centre, float(angle), 1.0)
        templates.append(cv2.warpAffine(patch, matrix, (patch.shape[1], patch.shape[0])))
    return templates, centre


def match_tip(gray, templates, centre, expected, options: DetectOptions, size):
    """The best match of any template near `expected`, or None.

    Returns (u, v, score) where (u, v) is the *tip*: the template's centre maps
    there, because the crop was centred on the tap.
    """
    reach = float(options.max_offset_px)
    height, width = gray.shape
    patch_h, patch_w = templates[0].shape[:2]
    if patch_h >= height or patch_w >= width:
        return None
    half_u, half_v = int(round(centre[0])), int(round(centre[1]))
    u0 = int(max(0, expected[0] - reach - half_u))
    v0 = int(max(0, expected[1] - reach - half_v))
    u1 = int(min(width, expected[0] + reach + half_u + 1))
    v1 = int(min(height, expected[1] + reach + half_v + 1))
    if u1 - u0 < patch_w or v1 - v0 < patch_h:
        return None
    region = gray[v0:v1, u0:u1]

    best = None
    for patch in templates:
        result = cv2.matchTemplate(region, patch, cv2.TM_CCOEFF_NORMED)
        _low, high, _low_at, high_at = cv2.minMaxLoc(result)
        if best is None or high > best[0]:
            best = (float(high), high_at)
    score, (x, y) = best
    if score < float(options.min_score):
        return None
    # The template's centre is the tip, because the crop was centred on the tap.
    return (float(u0 + x + centre[0]), float(v0 + y + centre[1]), score)


def detect_tip_template(gray, templates, centre, q, K, dist, T_base_cam, t_tip,
                        options: DetectOptions, size):
    """The tip by template matching around where the camera says it should be."""
    _wrist, tip, _sixth = _tool_pixels(q, K, dist, T_base_cam, t_tip)
    if tip is None:
        raise AutoError("the tool projects behind the camera; check the plane and the rig")
    found = match_tip(gray, templates, centre, tip, options, size)
    if found is None:
        raise AutoError(
            "the gripper template does not match anywhere near the projected tip (%.0f px): "
            "the seed is off, the wrist is turned out of plane, or the camera moved"
            % float(options.max_offset_px))
    u, v, score = found
    return Detection(u=u, v=v, offset_px=float(np.hypot(u - tip[0], v - tip[1])),
                     area=0.0, pose=0, score=score)


def detect_all_template(frames, poses, K, dist, T_base_cam, t_tip, templates, centre,
                        options: DetectOptions, size):
    results = []
    for index, (frame, q) in enumerate(zip(frames, poses)):
        found = detect_tip_template(_gray(frame), templates, centre, q, K, dist, T_base_cam,
                                    t_tip, options, size)
        found.pose = index + 1
        results.append(found)
    return results


def green_mask(frame):
    """The arm's lime covers, and nothing else in this room.

    The B601 dresses its links in a bright green that no bench object shares, so
    the mask is a segmentation of the arm rather than a difference against a
    background. That is what makes it survive a cluttered scene and a moving
    gripper: it does not compare against anything, it looks for the colour.
    """
    if frame.ndim == 3:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    else:
        hsv = cv2.cvtColor(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(GREEN_LOW), np.array(GREEN_HIGH))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return mask


def green_tip(mask, expected, direction, options: DetectOptions, size):
    """The tip from the green blob nearest `expected`, along `direction`.

    Returns (u, v, area, nearest_px) or None. The *nearest* green component is
    the one the projection points at; its far end along the tool axis is the
    tip, and the black jaws beyond it are a constant offset that the fit absorbs.
    """
    u0, v0, u1, v1 = _window(expected, float(options.max_offset_px), size)
    window = mask[v0:v1, u0:u1]
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(window, 8)
    best = None
    for index in range(1, count):
        area = float(stats[index, cv2.CC_STAT_AREA])
        if area < float(options.min_area):
            continue
        rows, columns = np.nonzero(labels == index)
        points = np.stack([columns + u0, rows + v0], axis=1).astype(float)
        distance = np.hypot(points[:, 0] - expected[0], points[:, 1] - expected[1])
        nearest = float(distance.min())
        if best is not None and nearest >= best[0]:
            continue
        along = (points - np.asarray(expected, dtype=float)) @ direction
        best = (nearest, points[int(along.argmax())], area)
    if best is None:
        return None
    return float(best[1][0]), float(best[1][1]), best[2], best[0]


def detect_tip_green(frame, q, K, dist, T_base_cam, t_tip, options: DetectOptions, size,
                     mask=None):
    """The tip by colour: the arm's green, at the end the tool axis points to."""
    if mask is None:
        mask = green_mask(frame)
    if int(mask.sum()) // 255 < MIN_GREEN_PX:
        raise AutoError("no green arm in this frame (%d green pixels): the colour detector "
                        "needs the arm's covers in view" % (int(mask.sum()) // 255))
    wrist, tip, joint6 = _tool_pixels(q, K, dist, T_base_cam, t_tip)
    if tip is None or wrist is None:
        raise AutoError("the tool projects behind the camera; check the plane and the rig")
    direction = np.asarray(tip, dtype=float) - np.asarray(joint6 or wrist, dtype=float)
    length = float(np.linalg.norm(direction))
    if length < 1.0:
        direction = np.asarray(tip, dtype=float) - np.asarray(wrist, dtype=float)
        length = float(np.linalg.norm(direction))
    if length < 1.0:
        raise AutoError("the tool projects to a single pixel; check the plane and the rig")
    direction = direction / length
    found = green_tip(mask, tip, direction, options, size)
    if found is None:
        raise AutoError("no green blob near the projected tip (%.0f px): the seed is off, or "
                        "the arm is out of frame" % float(options.max_offset_px))
    u, v, area, nearest = found
    # Accept only what looks like the arm: a sizeable blob sitting close to where
    # the camera says the tip is. A stray greenish patch in a busy scene satisfies
    # "some green exists" but not this, and falling back is better than a false tip.
    if area < max(float(options.min_area), GREEN_MIN_BLOB_PX) or \
            nearest > float(options.max_offset_px) * GREEN_NEAREST:
        raise AutoError("the green near the projected tip is not the arm (area %.0f px, %.0f px "
                        "away); falling back to the silhouette" % (area, nearest))
    return Detection(u=u, v=v, offset_px=float(np.hypot(u - tip[0], v - tip[1])),
                     area=area, pose=0, score=max(0.0, 1.0 - nearest / float(options.max_offset_px)))


def detect_all_green(frames, poses, K, dist, T_base_cam, t_tip, options: DetectOptions, size,
                     backgrounds=None, masks=None):
    """The green detector per pose, falling back to the silhouette where it fails."""
    if masks is None:
        masks = [green_mask(frame) for frame in frames]
    results = []
    for index, (frame, q) in enumerate(zip(frames, poses)):
        try:
            found = detect_tip_green(frame, q, K, dist, T_base_cam, t_tip, options, size,
                                     mask=masks[index])
        except AutoError:
            if backgrounds is None:
                backgrounds = backgrounds_for(frames)
            found = detect_tip(_gray(frame), backgrounds[index], q, K, dist, T_base_cam,
                               t_tip, options, size)
        found.pose = index + 1
        results.append(found)
    return results


def backgrounds_for(frames, background=None):
    """One background per pose: an explicit frame, or the median of the others."""
    grays = [_gray(frame) for frame in frames]
    if background is not None:
        still = _gray(background)
        return [still] * len(grays)
    return [np.median(np.stack([other for j, other in enumerate(grays) if j != index]),
                      axis=0).astype(np.uint8) for index in range(len(grays))]


def detect_all(frames, poses, K, dist, T_base_cam, t_tip, options: DetectOptions, size,
               background=None, backgrounds=None):
    grays = [_gray(frame) for frame in frames]
    if backgrounds is None:
        backgrounds = backgrounds_for(frames, background)
    results = []
    for index, (gray, q) in enumerate(zip(grays, poses)):
        found = detect_tip(gray, backgrounds[index], q, K, dist, T_base_cam, t_tip,
                           options, size)
        found.pose = index + 1
        results.append(found)
    return results


# --------------------------------------------------------------------------- #
# The run
# --------------------------------------------------------------------------- #

def _rotation_between(a, b):
    """Smallest rotation taking unit vector `a` onto `b`."""
    a = np.asarray(a, dtype=float) / np.linalg.norm(a)
    b = np.asarray(b, dtype=float) / np.linalg.norm(b)
    cross = np.cross(a, b)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(a @ b, -1.0, 1.0))
    if sine < 1e-9:
        return np.eye(3) if cosine > 0.0 else -np.eye(3)
    return ex.rodrigues(cross / sine * math.atan2(sine, cosine))


def click_seeds(click, K, dist):
    """Camera poses consistent with one tap on a known point.

    A tap is two constraints on six, so it leaves a family, not a pose: the
    camera can sit anywhere on a ray back from the tool, at any distance, with
    any roll about that ray. This enumerates a coarse grid over those four
    degrees of freedom -- the tap fixes the *pixel*, so every candidate points
    the search window at the right place, which is what makes a coarse grid
    enough.

    It is a seed generator, not a fit: the caller scores these by how consistent
    the whole sweep is and lets the normal two passes refine the winner.
    """
    q = np.asarray(click["q"], dtype=float).reshape(6)
    point = rebot_ik.forward(q)[:3, 3]
    u, v = (float(click["uv"][0]), float(click["uv"][1]))
    K = np.asarray(K, dtype=float).reshape(3, 3)
    x, y = ex.undistort((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], dist)
    ray = np.array([x, y, 1.0])
    ray /= float(np.linalg.norm(ray))

    seeds = []
    for azimuth in range(0, 360, 90):
        for elevation in (20.0, 45.0, 70.0):
            for distance in (1.0, 1.6):
                direction = np.array([
                    math.cos(math.radians(azimuth)) * math.cos(math.radians(elevation)),
                    math.sin(math.radians(azimuth)) * math.cos(math.radians(elevation)),
                    math.sin(math.radians(elevation))])
                position = point + distance * direction
                if position[2] < 0.25 or float(np.linalg.norm(position[:2])) > 3.0:
                    continue
                look = ex.look_at(position, point)[:3, :3]
                want = (point - position) / float(np.linalg.norm(point - position))
                turn = _rotation_between(look @ ray, want)
                for roll in (0.0, 90.0, 180.0, 270.0):
                    rotation = turn @ look
                    if roll:
                        rotation = ex.rodrigues(want * math.radians(roll)) @ rotation
                    seeds.append(ex.transform(rotation, position))
    return seeds


def bootstrap_from_click(frames, poses, click, K, dist, options: "Options", size, background=None):
    """Pick the seed whose sweep is most self-consistent.

    The score is how many poses the candidate finds the tip in *where it says the
    tip is*: a wrong camera puts the eight detections on the wrong parts of the
    arm, and a fit through them is either bad or, worse, tidy. Cost is one
    detection plus one small fit per candidate, with the backgrounds computed
    once.

    `background` is honoured here too: the seed search and the passes that follow
    must see the same silhouettes, or the seed is chosen for the wrong scene.
    """
    backgrounds = backgrounds_for(frames, background)
    templates = None
    centre = None
    if isinstance(click, dict) and click.get("image") is not None and options.detect.template:
        templates, centre = tip_templates(click["image"], click["uv"], options.detect)
    masks = None
    if templates is None and options.detect.green:
        masks = [green_mask(frame) for frame in frames]
    best = None
    tried = 0
    for guess in click_seeds(click, K, dist):
        try:
            if templates is not None:
                detections = detect_all_template(frames, poses, K, dist, guess, (0.0, 0.0, 0.0),
                                                 templates, centre, options.detect, size)
            elif masks is not None:
                detections = detect_all_green(frames, poses, K, dist, guess, (0.0, 0.0, 0.0),
                                              options.detect, size, masks=masks)
            else:
                detections = detect_all(frames, poses, K, dist, guess, (0.0, 0.0, 0.0),
                                        options.detect, size, background=background)
            fitted = ex.fit([ex.Observation(q=np.asarray(q, dtype=float), uv=(d.u, d.v))
                             for q, d in zip(poses, detections)],
                            K, dist, ex.FitOptions(plane_z=float(options.plane_z),
                                                   image_size=size, max_weak_mm=1e9))
        except (AutoError, ex.ExtrinsicsError):
            continue
        tried += 1
        # A tip hundreds of millimetres from the flange is not this tool's tip:
        # the candidate locked onto something else, and its camera is nonsense.
        if float(np.linalg.norm(np.asarray(fitted.rig.tip_offset, dtype=float))) > sweep.TIP_LIMIT_M:
            continue
        # The score is how many poses landed *where the candidate says they
        # should be*, not the fit's residual: a wrong camera can pick a shadow
        # consistently and fit it beautifully, which is how a 100 px rig once
        # scored best. Ties are broken by the median offset, then by the fit.
        offsets = [float(d.offset_px) for d in detections]
        close = sum(1 for value in offsets if value <= CLOSE_PX)
        score = (close, -float(np.median(offsets)), -float(fitted.rig.rms_px))
        if best is None or score > best[0]:
            best = (score, guess)
    if best is None:
        raise AutoError(
            "the tap did not lead to a usable start: no camera pose consistent with it could "
            "find the arm in the sweep. Check that the tap is on the tool tip and that the "
            "camera did not move between the tap and the sweep.")
    return best[1]


def bootstrap(options: "Options"):
    """The only seed: a tap. There is deliberately no other way in.

    A tap is two constraints on six, so it leaves a family rather than a pose:
    the camera can sit anywhere on a ray back from the tool, at any distance,
    with any roll about it. `run()` fills that family in and picks the member
    whose detections across the sweep agree with each other -- which is the
    measurement, not a guess about where the bracket was bolted.
    """
    if not options.click:
        raise AutoError(
            "a sweep needs one tap: take a still, tap the tool tip in it, and pass that "
            "capture and pixel. There is no position to type and no second path.")
    return options.click


def _capture_dir(root: Path):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return root / ("%s-%s" % (stamp, uuid.uuid4().hex[:6]))


def _tick_progress(note, seconds, stop):
    """Move the progress line along while one long file plays.

    The pass is a single trajectory now, so nothing in the request loop can say
    which pose the arm is at. The schedule can: each leg's duration and the hold
    that follows it put every pose on a known timeline.
    """
    started = time.time()
    at = 0.0
    for index, value in enumerate(seconds, start=1):
        if index > 1:
            at += value + sweep.HOLD_SECONDS
        while not stop.is_set():
            remaining = at - (time.time() - started)
            if remaining <= 0:
                break
            stop.wait(min(0.2, remaining))
        if stop.is_set():
            return
        note(index, "moving")


def run(arm, eye, calibration, options: Options, captures_dir, recorder, mount="fixed",
        progress=None) -> RunResult:
    """The whole sweep: record, move, pick the frames, detect, fit twice.

    `recorder` is the media server's recorder (`mediamtx.Recorder`): it is asked
    to start and stop, and to list what it wrote. Tests pass a stand-in.

    `progress(pose, phase)` is called for each step -- phase is "recording",
    "extracting", "moving" or "fitting", and `pose` is 1-based (0 outside the
    sweep). It exists for the API's status route: a three-minute sweep behind one
    POST needs to say where it is.
    """
    def note(pose, phase):
        if progress is None:
            return
        try:
            progress(int(pose), str(phase))
        except Exception:      # noqa: BLE001 - a broken status hook must not stop the arm
            pass

    started = time.time()
    captures_dir = Path(captures_dir)
    captures_dir.mkdir(parents=True, exist_ok=True)

    start = arm.pose()
    q_start = np.asarray(start["q"][:6], dtype=float)
    problems = sweep.check(start=q_start)
    if problems:
        raise AutoError("the sweep cannot start: " + "; ".join(problems))

    frames, poses, captures, warnings = [], [], [], []
    background = None
    # The board returns the arm to the pose a move started from, so every leg is
    # a round trip *from where the arm already is*. That pose is therefore the
    # sweep's park pose, and there is nothing to gain by moving to HOME first:
    # the move would be undone before the next command.
    park = q_start
    recording = _capture_dir(captures_dir)
    recording.mkdir(parents=True, exist_ok=True)
    log = sweep_video.PoseLog(arm, recording / "poses.jsonl")
    try:
        if options.background and options.background != "median":
            if options.background == "auto":
                time.sleep(options.settle_s)
                background = eye.read()
            else:
                background = cv2.imread(str(options.background), cv2.IMREAD_COLOR)
                if background is None:
                    raise AutoError("cannot read the background image %r" % options.background)
        # The media server records the whole pass and the poses are picked out of
        # it afterwards, so the moves do not have to be photographed inside a
        # window: they just have to hold long enough to be found.
        note(0, "recording")
        record_from = time.time()
        log.start()
        # One trajectory for the whole pass: the arm visits every pose and the
        # board walks it back to park once, at the end. That is one enable/disable
        # cycle instead of eight, and no return leg between poses.
        plan = sweep.legs(park)
        targets = [target for target, _ in plan]
        seconds = [value for _, value in plan]
        # Both the recording and the pose log must stop whatever happens, and the
        # first failure is the one worth reporting: a failed enable() must not be
        # replaced by the disable() that tidies up after it.
        failure = None
        ticker = None
        stop_ticker = threading.Event()
        try:
            recorder.enable()
            ticker = threading.Thread(target=_tick_progress,
                                      args=(note, seconds, stop_ticker), daemon=True)
            ticker.start()
            arm.move_sequence(targets, seconds, hold=sweep.HOLD_SECONDS)
        except Exception as error:      # noqa: BLE001 - carried past the cleanup
            failure = error
        finally:
            stop_ticker.set()
            if ticker is not None:
                ticker.join(timeout=2.0)
            for step in (recorder.disable, log.stop):
                try:
                    step()
                except Exception as error:      # noqa: BLE001
                    failure = failure or error
        if failure is not None:
            raise failure
    except Exception:
        # Nothing to park: the board is already walking the arm back to `park`,
        # which is where it started. Say so if it did not get there.
        try:
            back = arm.pose()
            if float(np.max(np.abs(np.asarray(back["q"][:6], dtype=float) - park))) > 0.05:
                warnings.append("the arm is not back at the pose it started from")
        except Exception:          # noqa: BLE001 - already failing; the pose is best effort
            warnings.append("could not confirm the arm returned to its starting pose")
        raise

    note(0, "extracting")
    try:
        segments = recorder.segments(since=record_from)
        picked = sweep_video.extract(segments, recording, sweep.POSES,
                                     min_seconds=sweep_video.MIN_HOLD_SECONDS)
    except (sweep_video.SweepVideoError, mediamtx.MediaMtxError) as error:
        raise AutoError(str(error)) from error
    frames = [item["image"] for item in picked]
    poses = [np.asarray(item["q"], dtype=float) for item in picked]
    captures = [item["path"] for item in picked]

    size = (frames[0].shape[1], frames[0].shape[0])
    if any((frame.shape[1], frame.shape[0]) != size for frame in frames):
        raise AutoError("the frames changed size during the sweep; is the stream renegotiating?")
    if (int(calibration.image_size[0]), int(calibration.image_size[1])) != size:
        raise AutoError(
            "the frames are %dx%d but the intrinsics are for %dx%d: load the calibration at the "
            "capture size (calib.load scales it when the aspect ratio matches)"
            % (size[0], size[1], calibration.image_size[0], calibration.image_size[1]))

    K, dist = calibration.K, calibration.dist
    bootstrap(options)
    T_guess = bootstrap_from_click(frames, poses, options.click, K, dist, options, size,
                                        background=background)
    note(0, "fitting")
    fit_options = ex.FitOptions(plane_z=float(options.plane_z), image_size=size,
                                work_radius=float(options.work_radius),
                                # The fit is allowed to be bad here: `verify` is
                                # what decides, and it can see the covariance. A
                                # weak sweep should produce a *refusal*, not an
                                # exception that hides the captures.
                                max_weak_mm=1e9)

    # With the tap's image in hand, the detector becomes template matching: the
    # gripper's own pixels, searched around where the camera says it should be.
    # That is a far better signal on a cluttered bench than a silhouette's distal
    # band, and it is the same code the seed search uses.
    templates = centre = None
    if (options.detect.template and options.click and isinstance(options.click, dict)
            and options.click.get("image") is not None):
        templates, centre = tip_templates(options.click["image"], options.click["uv"],
                                          options.detect)

    masks = [green_mask(frame) for frame in frames] if options.detect.green else None

    def detect_pass(T_base_cam, t_tip):
        if templates is not None:
            return detect_all_template(frames, poses, K, dist, T_base_cam, t_tip, templates,
                                       centre, options.detect, size)
        if masks is not None:
            return detect_all_green(frames, poses, K, dist, T_base_cam, t_tip, options.detect,
                                    size, masks=masks)
        return detect_all(frames, poses, K, dist, T_base_cam, t_tip, options.detect, size,
                          background=background)

    first = detect_pass(T_guess, (0.0, 0.0, 0.0))
    observations = [ex.Observation(q=q, uv=(d.u, d.v), capture=directory.name)
                    for q, d, directory in zip(poses, first, captures)]
    try:
        coarse = ex.fit(observations, K, dist, fit_options,
                        calib_fingerprint=calibration.fingerprint, device=calibration.device,
                        mount=mount)
    except ex.ExtrinsicsError as error:
        names = _save_debug(frames, poses, [d.uv for d in observations], T_guess, (0.0, 0.0, 0.0),
                            K, dist, options.detect, size, recording, range(len(frames)))
        raise AutoError(
            "the detected tips do not describe a rigid arm: %s. The frames with the search "
            "window (green), the projection (red) and the point kept (blue) are in %s (%s)."
            % (error, recording.name, ", ".join(names))) from error

    refined = first
    try:
        refined = detect_pass(coarse.rig.T_base_cam, coarse.rig.tip_offset)
    except AutoError as error:
        warnings.append("second detection pass failed (%s); keeping the first" % error)

    observations = [ex.Observation(q=q, uv=(d.u, d.v), capture=directory.name)
                    for q, d, directory in zip(poses, refined, captures)]
    try:
        final = ex.fit(observations, K, dist, fit_options,
                       calib_fingerprint=calibration.fingerprint, device=calibration.device,
                       mount=mount)
    except ex.ExtrinsicsError as error:
        names = _save_debug(frames, poses, [d.uv for d in observations],
                            coarse.rig.T_base_cam, coarse.rig.tip_offset, K, dist,
                            options.detect, size, recording, range(len(frames)))
        raise AutoError(
            "the second detection pass does not describe a rigid arm: %s. The frames with the "
            "search window (green), the projection (red) and the point kept (blue) are in %s "
            "(%s)." % (error, recording.name, ", ".join(names))) from error

    # A detection far from where the *fitted* rig projects the tip is usually not
    # the tip: a shadow, a highlight, the edge of the tissue box. Measured against
    # the final fit, not the coarse one -- the coarse pass is allowed to be off by
    # a hundred pixels, that is what the second pass is for. One outlier the
    # Huber loss absorbs; two or more mean the data does not describe a rigid arm,
    # and fitting through them produces a tidy-looking nonsense rig.
    far = []
    for number, observation in enumerate(observations, start=1):
        motion = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        point = motion[:3, :3] @ np.asarray(final.rig.tip_offset, dtype=float) + motion[:3, 3]
        seen = ex.pixel(point, K, dist, ex.invert(final.rig.T_base_cam))
        error = float("inf") if seen is None else float(
            math.hypot(seen[0] - observation.uv[0], seen[1] - observation.uv[1]))
        if error > CLOSE_PX:
            far.append((number, error))
    if len(far) >= 2:
        names = _save_debug(frames, poses, [item.uv for item in observations],
                            final.rig.T_base_cam, final.rig.tip_offset, K, dist,
                            options.detect, size, recording,
                            [number - 1 for number, _ in far])
        raise AutoError(
            "the tip detector is not sure on %s: those points are %.0f-%.0f px from where the "
            "fitted rig projects the tip, so they are probably not the tip. The frames with the "
            "search window (green), the projection (red) and the point it kept (blue) are in "
            "%s (%s). If it persists the camera seed is off, or the background does not match "
            "the scene." % (", ".join("pose %d" % pose for pose, _ in far),
                            min(value for _, value in far), max(value for _, value in far),
                            recording.name, ", ".join(names)))
    if far:
        warnings.append("pose %d: the tip is %.0f px from the final projection"
                        % (far[0][0], far[0][1]))

    return RunResult(observations=observations, captures=captures, detections=refined,
                     bootstrap=T_guess, fit=final, seconds=time.time() - started,
                     first_frame=frames[0], first_pose=poses[0], warnings=warnings)


def verify(rig, observations, calibration, options: Options, converged=True):
    """Is the fitted rig good enough to write? The caller decides what to do.

    A calibration rig is a systematic thing: if the tip detector locked onto a
    different feature in one pose, or the model does not match the arm, the fit
    still looks tidy. The held-out residual is what notices.
    """
    K, dist = calibration.K, calibration.dist
    fit_options = ex.FitOptions(plane_z=float(options.plane_z),
                                image_size=tuple(int(v) for v in calibration.image_size))
    loo = ex.leave_one_out(observations, K, dist, fit_options)
    predicted = rig.predicted or {}
    reasons = []
    if not converged:
        reasons.append("the fit did not converge")
    if float(rig.rms_px) > 2.5:
        reasons.append("the clicks scatter by %.1f px around the fit" % rig.rms_px)
    if predicted.get("corner_mm") is None:
        reasons.append("no predicted error could be computed")
    elif float(predicted["corner_mm"]) > 8.0:
        reasons.append("the predicted error at the corner is %.1f mm" % predicted["corner_mm"])
    # Leaving a pose out of a thin design can leave a direction free; that is
    # what `weakest` is for, so an unavailable hold-out is not a failure. A
    # large one, when it can be computed, still is.
    if loo is not None and float(np.max(loo)) > 6.0:
        reasons.append("the worst held-out click is %.1f px" % float(np.max(loo)))
    return {
        "ok": not reasons,
        "reasons": reasons,
        "leave_one_out_px": None if loo is None else [round(float(v), 2) for v in loo],
        "rms_px": round(float(rig.rms_px), 2),
        "worst_px": None if loo is None else round(float(np.max(loo)), 2),
        "predicted_mm": predicted,
        "weakest": rig.weakest,
    }


def report(result: RunResult, verdict):
    """What the API returns: enough to trust or reject the run without a log."""
    return {
        "ok": bool(verdict["ok"]),
        "poses": len(result.observations),
        "seconds": round(result.seconds, 1),
        "camera": result.fit.rig.describe().splitlines()[0],
        "tip_mm": [round(1000.0 * float(v), 1) for v in result.fit.rig.tip_offset],
        "rig": result.fit.rig.provenance(),
        "verification": verdict,
        "detections": [{"pose": d.pose, "u": round(d.u, 1), "v": round(d.v, 1),
                        "offset_px": round(d.offset_px, 1), "area": round(d.area, 0)}
                       for d in result.detections],
        "captures": [str(path.name) for path in result.captures],
        "warnings": result.warnings,
    }


def attach_witness(rig, result: "RunResult", calibration, rig_path):
    """Build and store the mount witness from the first frame of the sweep."""
    if result.first_frame is None or result.first_pose is None:
        return None
    reference = witness.build(result.first_frame, rig=rig, q=result.first_pose,
                              K=calibration.K, dist=calibration.dist)
    name = "witness-%s.jpg" % rig.mount
    witness.save(reference, Path(rig_path).parent / name)
    rig.witness = dict(reference.to_dict(), image=name)
    return reference


def commit(result: "RunResult", calibration, options: Options, rig_path,
           calib_path="", mount="fixed"):
    """Verify, then write the rig only if it passes. No rig, no side effects."""
    rig = result.fit.rig
    verdict = verify(rig, result.observations, calibration, options, result.fit.converged)
    if not verdict["ok"]:
        return rig, verdict, None

    rig_path = Path(rig_path)
    rig_path.parent.mkdir(parents=True, exist_ok=True)
    attach_witness(rig, result, calibration, rig_path)
    ex.save_rig(rig, rig_path)
    return rig, verdict, rig_path
