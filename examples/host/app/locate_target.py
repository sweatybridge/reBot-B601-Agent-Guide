#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Turn one phone capture into a point in the arm's base frame.

The phone gives a JPEG, a tap coordinate and nothing else -- no depth, no pose.
There are two ways to supply the missing pose, and they are the same geometry
with a different source for `T_base<-cam`:

* **Fixed camera** (`--extrinsics`): the pose is a file, fitted once from the
  arm's tool tip. Nothing else has to be visible; the tap ray meets the table
  plane and that is the target. A witness frame checks the mount has not moved.
* **Handheld phone** (the tag): `solvePnP` on an ArUco tag standing on the base at
  a known offset from it gives `T_cam<-tag`; the placement gives `T_base<-tag`;
  the two compose into `T_base<-cam`. That is why the tag has to be in the same
  frame as the target.

    python locate_target.py --capture captures/<id> --calib calib/<device>.json \
        --extrinsics calib/extrinsics-fixed.json
    python locate_target.py --capture captures/<id> --calib calib/<device>.json \
        --marker-at 0.35,-0.20,0.0,15 --marker-size 0.080

`locate_capture()` is the same code the capture server calls when someone taps
the video, so a tap and a command line cannot drift apart.

The printed tag size gives PnP its metric scale, so the camera pose it supplies
is in metres; the table plane then turns the tap ray into the target point.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import calib as calib_mod  # noqa: E402
import base_marker
import extrinsics  # noqa: E402
import witness as witness_mod  # noqa: E402

ARUCO_DICTS = {
    "4x4_50": "DICT_4X4_50",
    "4x4_100": "DICT_4X4_100",
    "5x5_50": "DICT_5X5_50",
    "6x6_50": "DICT_6X6_50",
}


class LocateError(Exception):
    """No target from this capture; the message is meant for the operator."""


@dataclass
class Options:
    tag_size: float = 0.080
    tag_x: float = 0.0
    tag_y: float = 0.0
    tag_z: float = 0.0
    tag_yaw: float = 0.0
    plane_z: float = 0.0
    tap: tuple | None = None
    max_reproj: float = 1.5
    max_grazing: float = 80.0
    max_reach: float = 0.75
    dictionary: str = "4x4_50"
    tag_id: int | None = None
    annotate: str = ""
    extrinsics: object | None = None      # extrinsics.Rig; a fixed camera, no tag in frame
    witness: object | None = None         # witness.Reference; proves the mount has not moved
    witness_max_px: float = 2.0
    turn: int | None = None               # quarter turns to apply to the frame, 0..3


def tag_object_points(tag_size):
    """Corners in the tag frame: x right, y up, z out of the printed face.

    Same order as `detectMarkers` returns them (top-left, top-right, bottom-right,
    bottom-left as printed), so a face-up tag has +z pointing at the phone, which
    is also +z of the arm base.
    """
    half = float(tag_size) / 2.0
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def detect_tag(gray, dictionary, tag_id=None):
    """Find the tag, returning (id, 4x2 corners) or raising with what was seen."""
    params = cv2.aruco.DetectorParameters()
    refine = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", None)
    if refine is not None:
        params.cornerRefinementMethod = refine
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        raise LocateError("no ArUco marker in the frame -- is the tag inside the view and lit?")

    found = [int(v) for v in ids.reshape(-1)]
    if tag_id is None:
        index = 0
        if len(found) > 1:
            # Several tags: the largest one is the one the phone is pointed at.
            areas = [abs(cv2.contourArea(c.reshape(-1, 1, 2))) for c in corners]
            index = int(np.argmax(areas))
    else:
        if tag_id not in found:
            raise LocateError("tag id %d not in the frame (found %s)" % (tag_id, found))
        index = found.index(tag_id)
    return found[index], np.asarray(corners[index], dtype=np.float64).reshape(-1, 2)


def tag_pose(corners, tag_size, K, dist):
    """Pose of the tag in the camera frame, plus its reprojection error in px.

    A planar solve has a mirror twin; the real one sees the printed face (the
    tag's +z points back at the camera). Take the best front-facing candidate
    from three independent solvers, exactly as `base_marker.solve` does.
    Rotating one solution by Rx(pi) to flip its normal is NOT the mirror twin:
    it breaks the corner correspondence (~80 px reprojection on an exact
    solution) while looking like a fix.
    """
    obj = tag_object_points(tag_size)
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)

    def rms_of(rvec, tvec):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        errors = np.linalg.norm(projected.reshape(-1, 2) - corners, axis=1)
        return float(np.sqrt(np.mean(errors ** 2)))

    candidates = []
    solvers = [cv2.SOLVEPNP_IPPE_SQUARE,
               getattr(cv2, "SOLVEPNP_SQPNP", None),
               cv2.SOLVEPNP_ITERATIVE]
    for flags in [solver for solver in solvers if solver is not None]:
        try:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flags)
        except cv2.error:
            continue
        if not ok:
            continue
        rotation = cv2.Rodrigues(rvec)[0]
        translation = np.asarray(tvec, dtype=float).reshape(3)
        candidates.append({
            "rvec": rvec, "tvec": translation, "rotation": rotation,
            "rms": rms_of(rvec, tvec),
            "front": float(rotation[:, 2] @ (-translation)) > 0.0,
        })
    if not candidates:
        raise LocateError("solvePnP failed on the tag")
    front = [candidate for candidate in candidates if candidate["front"]]
    if not front:
        raise LocateError("the tag solved only from behind: is the printed face toward "
                          "the camera?")
    best = min(front, key=lambda candidate: candidate["rms"])
    rvec, tvec, rotation = best["rvec"], best["tvec"], best["rotation"]
    try:
        # LM writes through its inputs; copies keep the candidate intact if the
        # refinement turns out worse than what the solver already had.
        rvec2, tvec2 = cv2.solvePnPRefineLM(obj, img, K, dist,
                                            rvec.copy(), tvec.reshape(3, 1).copy())
        rotation2 = cv2.Rodrigues(rvec2)[0]
        tvec2 = np.asarray(tvec2, dtype=float).reshape(3)
        if (rms_of(rvec2, tvec2) <= best["rms"] * 1.05
                and float(rotation2[:, 2] @ (-tvec2)) > 0.0):
            rvec, tvec, rotation = rvec2, tvec2, rotation2
    except cv2.error:
        pass  # the solver's answer is already usable; refinement is a bonus

    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - corners, axis=1)))
    return rotation, np.asarray(tvec, dtype=float).reshape(3), reproj


def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tag_in_base(x, y, z, yaw_deg):
    """`T_base<-marker`: the tag stands on the side of the base.

    Its printed face points along the heading `yaw_deg` and its +y is the base's
    +z, so the image reads upright to a phone held in front of it. There is no
    flat-on-the-worktop variant any more: a tag lying down is seen at a glancing
    angle by a handheld phone, which is what made it fragile.
    """
    return base_marker.base_from_marker((x, y, z), math.radians(float(yaw_deg)),
                                        mode="vertical")


def base_from_camera(R_cam_tag, t_cam_tag, T_base_tag):
    """Compose the two transforms into `T_base<-cam`."""
    T_cam_tag = np.eye(4)
    T_cam_tag[:3, :3] = R_cam_tag
    T_cam_tag[:3, 3] = t_cam_tag
    return T_base_tag @ np.linalg.inv(T_cam_tag)


def ray_plane(K, T_base_cam, u, v, plane_z, dist=None):
    """Intersect the pixel's ray with the horizontal plane, in the base frame.

    The intersection itself lives in `extrinsics.py`, so the tag path and the
    fixed-camera path cannot disagree about it. It undoes the lens distortion
    first, which the old version skipped: `K` was fitted with `dist`, so the ray
    has to be too, or a click near the frame edge lands a few millimetres out.
    """
    try:
        return extrinsics.ray_plane(K, dist, T_base_cam, u, v, plane_z)
    except extrinsics.ExtrinsicsError as error:
        message = str(error)
        if "parallel" in message:
            raise LocateError("the tap ray is parallel to the table plane; pick a point further "
                              "away") from error
        if "away from" in message:
            raise LocateError("the tap ray points away from the table plane "
                              "(is the plane above the camera?)") from error
        raise LocateError(message) from error



def project(K, T_base_cam, point, dist=None):
    """Project a base-frame point back into the image (for the annotation and
    for placing the marker on the projected worktop plane)."""
    p_cam = T_base_cam[:3, :3].T @ (np.asarray(point, dtype=float) - T_base_cam[:3, 3])
    if p_cam[2] <= 1e-9:
        return None
    if dist is not None and np.asarray(dist).size:
        image_points, _ = cv2.projectPoints(
            np.asarray([p_cam], dtype=float), np.zeros(3), np.zeros(3),
            np.asarray(K, dtype=float), np.asarray(dist, dtype=float))
        return float(image_points[0, 0, 0]), float(image_points[0, 0, 1])
    q = np.asarray(K, dtype=float) @ p_cam
    return float(q[0] / q[2]), float(q[1] / q[2])


def load_capture(path):
    """Accept a capture directory or a bare image, and return image + meta."""
    path = Path(path)
    if path.is_dir():
        image_path = path / "frame.jpg"
        meta_path = path / "meta.json"
        if not image_path.is_file():
            raise LocateError("no frame.jpg in %s" % path)
        meta = {}
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        return cv2.imread(str(image_path), cv2.IMREAD_COLOR), meta, path, image_path
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise LocateError("cannot read %s" % path)
    return image, {}, path.parent, path


def tap_from_meta(meta, image_size):
    """Tap in image pixels, rescaled if the JPEG is not the size the page declared."""
    tap = meta.get("tap")
    if not tap:
        return None
    u, v = float(tap["u"]), float(tap["v"])
    declared = meta.get("image_size") or []
    if len(declared) == 2 and declared[0] and declared[1]:
        u *= image_size[0] / float(declared[0])
        v *= image_size[1] / float(declared[1])
    return u, v


def annotate(image, corners, tag_size, T_base_cam, K, dist, point, tap, out_path):
    """Draw what the solver believed: tag axes, the tap, the point projected back."""
    out = image.copy()
    if corners is not None:
        cv2.aruco.drawDetectedMarkers(out, [corners.reshape(-1, 1, 2).astype(np.float32)])
        ok, rvec, tvec = cv2.solvePnP(tag_object_points(tag_size), corners.reshape(-1, 1, 2),
                                      K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            cv2.drawFrameAxes(out, K, dist, rvec, tvec, tag_size / 2.0, 2)
    if tap is not None:
        cv2.circle(out, (int(round(tap[0])), int(round(tap[1]))), 9, (0, 220, 255), 2)
        cv2.drawMarker(out, (int(round(tap[0])), int(round(tap[1]))), (0, 220, 255),
                       cv2.MARKER_CROSS, 22, 2)
    pixel = project(K, T_base_cam, point, dist)
    if pixel is not None:
        cv2.drawMarker(out, (int(round(pixel[0])), int(round(pixel[1]))), (60, 220, 60),
                       cv2.MARKER_TRIANGLE_UP, 18, 2)
    cv2.putText(out, "%.3f, %.3f, %.3f m" % tuple(point), (12, out.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    Path(out_path).write_bytes(cv2.imencode(".jpg", out)[1].tobytes())


def locate(image, meta, calibration, options: Options):
    """Image + tap -> target payload. Pure enough to test, no printing."""
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    tag_id, corners, reproj = None, None, 0.0
    if options.extrinsics is not None:
        # A fixed camera: the pose is a file, so nothing has to be printed and
        # nothing has to be in frame except the thing being tapped.
        try:
            extrinsics.verify_pair(calibration, options.extrinsics)
        except extrinsics.ExtrinsicsError as error:
            raise LocateError(str(error)) from error
        T_base_cam = np.asarray(options.extrinsics.T_base_cam, dtype=float)
    else:
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_DICTS[options.dictionary]))
        tag_id, corners = detect_tag(gray, dictionary, options.tag_id)

        R_cam_tag, t_cam_tag, reproj = tag_pose(corners, options.tag_size, calibration.K,
                                                calibration.dist)
        if reproj > options.max_reproj:
            raise LocateError(
                "tag reprojection %.2f px is above %.2f: the phone may have moved during the "
                "exposure, or the printed size is not %.1f mm"
                % (reproj, options.max_reproj, options.tag_size * 1000))

        T_base_tag = tag_in_base(options.tag_x, options.tag_y, options.tag_z, options.tag_yaw)
        T_base_cam = base_from_camera(R_cam_tag, t_cam_tag, T_base_tag)

    report = None
    if options.witness is not None:
        try:
            report = witness_mod.check(options.witness, gray, options.witness_max_px)
        except witness_mod.WitnessError as error:
            raise LocateError("cannot verify the camera mount: %s" % error) from error
        if not report.get("ok"):
            raise LocateError(
                "the camera moved since the extrinsics were fitted (%s). Refit with "
                "calibrate_extrinsics.py --fit, or put the mount back." % report.get("reason"))


    tap = options.tap if options.tap is not None else tap_from_meta(meta, (width, height))
    if tap is None:
        raise LocateError("no tap in the capture; tap the target before capturing")
    point, _distance, grazing = ray_plane(calibration.K, T_base_cam, tap[0], tap[1],
                                          options.plane_z, calibration.dist)
    source = "tap+fixed" if options.extrinsics is not None else "tap+marker"
    # ray_plane's grazing is the ray's elevation ABOVE the plane (90 = straight
    # down, the best-conditioned tap). The gate is base_marker.solve's
    # convention: refuse the view that deviates too far from the ideal, i.e.
    # elevation below 90 - max_grazing. Comparing elevation > max_grazing
    # inverted it -- rejecting straight-down taps and accepting glancing ones.
    if (90.0 - grazing) > options.max_grazing:
        raise LocateError(
            "the tap ray meets the table at %.0f deg above the plane (%.0f deg from "
            "straight down, limit %.0f): a grazing view turns every pixel of the tap "
            "into centimetres -- aim further down"
            % (grazing, 90.0 - grazing, options.max_grazing))

    radius = float(np.linalg.norm(point))
    projected = project(calibration.K, T_base_cam, point, calibration.dist)
    payload = {
        "format": "rebot-target",
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
        "calib_fingerprint": calibration.fingerprint,
        "image_size": [int(width), int(height)],
        "tap": {"u": float(tap[0]), "v": float(tap[1])} if tap is not None else None,
        "plane_z": float(options.plane_z),
        "point": [float(v) for v in point],
        "pixel": (None if projected is None
                  else [float(projected[0]), float(projected[1])]),
        "radius": radius,
        "T_base_cam": [[float(v) for v in row] for row in T_base_cam],
        "_corners": corners,
        "_T_base_cam": T_base_cam,
    }
    if options.extrinsics is not None:
        payload["rig"] = options.extrinsics.provenance()
    else:
        payload["tag"] = {
            "id": int(tag_id),
            "size_m": float(options.tag_size),
            "placement": {"x": options.tag_x, "y": options.tag_y,
                          "z": options.tag_z, "yaw_deg": options.tag_yaw},
            "reproj_px": float(reproj),
        }
    if report is not None:
        payload["witness"] = report
    return payload


def locate_capture(directory, calib_path, options: Options):
    """The whole job for one capture directory: read, solve, annotate, write.

    A frame delivered in another orientation is turned back before anything is
    measured: `K` and `dist` describe the grid they were fitted on, and `cv2`
    solves the tag against exactly that grid. Turning the *frame* keeps both
    exact, and costs one rotate of a few megapixels.
    """
    image, meta, capture_dir, image_path = load_capture(directory)
    delivered = (image.shape[1], image.shape[0])
    calibration = calib_mod.load(calib_path)
    turn = turn_back(meta, delivered, calibration, options.turn)
    if turn:
        image, calibration, _turn = upright_frame(calib_path, image, turn)
    tap = options.tap if options.tap is not None else tap_from_meta(meta, delivered)
    if tap is not None and turn:
        # The tap is measured in the delivered frame; move it into the grid the
        # image was turned into before anything is solved.
        tap = calib_mod.map_pixel(tap[0], tap[1], delivered, turn)
    # The image is upright now, so the tap must be too; the meta's own tap would
    # be measured against the delivered frame.
    meta_upright = dict(meta)
    meta_upright.pop("tap", None)
    meta_upright["image_size"] = [int(image.shape[1]), int(image.shape[0])]
    upright = replace(options, tap=tap, turn=0)

    payload = locate(image, meta_upright, calibration, upright)
    payload["turns"] = turn               # what had to be undone, 0..3
    payload["delivered_size"] = [int(delivered[0]), int(delivered[1])]

    corners = payload.pop("_corners")
    T_base_cam = payload.pop("_T_base_cam")
    payload["capture"] = str(capture_dir)
    payload["frame"] = str(image_path)

    out_path = capture_dir / "target.json"
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if options.annotate:
        annotate(image, corners, options.tag_size, T_base_cam, calibration.K, calibration.dist,
                 np.array(payload["point"]),
                 None if payload["tap"] is None else (payload["tap"]["u"], payload["tap"]["v"]),
                 options.annotate)
    payload["_out"] = str(out_path)
    return payload


def rotate_image(image, turns):
    """Turn an image clockwise by `turns` quarter turns."""
    turns = int(turns) % 4
    if turns == 1:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if turns == 2:
        return cv2.rotate(image, cv2.ROTATE_180)
    if turns == 3:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return image


def turn_back(meta, frame_size, calibration, explicit=None):
    """Quarter turns to apply to the delivered frame to reach the calibration grid.

    One number, one meaning: *the correction*, never the measurement. A frame out
    of a landscape phone while the board was shot in portrait needs one clockwise
    turn to put it back, so this answers `1`; whoever wants "how did it arrive"
    can write `4 - turn_back(...)` at the point where that question is actually
    being asked.

    Sources, in order: the caller, the capture's own `turns`, and the two
    orientations (`orientation_deg` now and in the calibration). Dimensions alone
    cannot tell 90 from 270 -- only that it is *a* quarter turn -- so the shape
    fallback is a guess and the payload records what it used.
    """
    if explicit is not None:
        return int(explicit) % 4
    declared = meta.get("turns")
    if declared is not None:
        declared = int(declared) % 4
        # A declared 0 cannot be right when the shape says a quarter turn is
        # needed: that is a page asserting "no turn" while it does not know
        # (the undefined-as-0 bug). Fall through to the orientation/shape
        # sources instead of trusting it; any other declaration stands.
        if declared != 0 or not calib_mod.swapped_shape(calibration.image_size, frame_size):
            return declared
    frame_deg = meta.get("orientation_deg")
    calib_deg = (calibration.raw or {}).get("orientation_deg")
    if frame_deg is not None and calib_deg is not None:
        return int(round((float(frame_deg) - float(calib_deg)) / 90.0)) % 4
    # The shape says "a quarter turn" and nothing else: assume the frame arrived
    # clockwise, which three clockwise turns undo.
    return 0 if calib_mod.turns_for_shapes(calibration.image_size, frame_size) == 0 else 3


def upright_frame(calib_path, frame, turn=None):
    """A delivered frame turned back onto the calibration's grid.

    This is the one entry point for anything that measures a frame with `K`:
    the tap locator, the tag-seeded rig and the sweep all call it, so none of
    them can silently use intrinsics for a grid the image is not on.

    `turn` is the correction in quarter turns (`turn_back(...)`); `None` derives
    it. Returns `(frame, calibration, turn)`, and the frame, the calibration and
    any pixel moved by `map_pixel(..., turn)` are then all on the same grid.
    """
    calibration = calib_mod.load(calib_path)
    turn = turn_back({}, (frame.shape[1], frame.shape[0]), calibration, turn)
    if turn:
        frame = rotate_image(frame, turn)
        calibration = calib_mod.load(calib_path,
                                     image_size=(frame.shape[1], frame.shape[0]))
    return frame, calibration, turn


def main(argv=None):
    parser = argparse.ArgumentParser(description="Locate a tapped point in the arm base frame.")
    parser.add_argument("--capture", required=True, help="capture directory or image file")
    parser.add_argument("--calib", required=True, help="calib/<device>.json")
    parser.add_argument("--turns", type=int, default=None, choices=[0, 1, 2, 3],
                        help="quarter turns to apply to the frame to bring it onto the "
                             "calibration grid; default is the capture's own meta, then "
                             "the two orientations, then the frame shape")
    parser.add_argument("--out", default="", help="target.json (default: <capture>/target.json)")
    parser.add_argument("--dictionary", default="4x4_50", choices=sorted(ARUCO_DICTS))
    parser.add_argument("--tag-id", type=int, default=None, help="default: largest marker in view")
    parser.add_argument("--marker-at", default="",
                        help="the base marker's centre and heading: 'x,y,z,yaw' in metres and "
                             "degrees (yaw is the way its face points)")
    parser.add_argument("--marker-size", type=float, default=0.04,
                        help="the marker's printed side, metres")
    parser.add_argument("--extrinsics", default="",
                        help="extrinsics JSON for a fixed camera; default is "
                             "<calib-dir>/extrinsics-fixed.json when it exists")
    parser.add_argument("--witness-max-px", type=float, default=2.0,
                        help="refuse a capture whose background moved more than this, px")
    parser.add_argument("--plane-z", type=float, default=0.0, help="worktop height in base frame, m")
    parser.add_argument("--tap", nargs=2, type=float, metavar=("U", "V"),
                        help="override the tap stored in meta.json, image pixels")
    parser.add_argument("--max-reproj", type=float, default=1.5,
                        help="reject a tag pose above this reprojection error, px")
    parser.add_argument("--max-grazing", type=float, default=80.0,
                        help="reject taps whose ray is more than this many degrees from "
                             "straight down (default 80: refuse rays within 10 deg of the plane)")
    parser.add_argument("--max-reach", type=float, default=0.75, help="warn above this radius, m")
    parser.add_argument("--annotate", default="", help="write an annotated JPEG here")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    # The rig lives next to the intrinsics, and both the tag capture and the
    # sweep write the standard name there: a fitted camera needs no flag.
    if not args.extrinsics:
        candidate = Path(args.calib).resolve().parent / "extrinsics-fixed.json"
        if candidate.is_file():
            args.extrinsics = str(candidate)

    try:
        rig = extrinsics.load_rig(args.extrinsics) if args.extrinsics else None
        reference = (witness_mod.load_for(rig, Path(args.extrinsics).resolve().parent)
                     if rig is not None else None)
    except (extrinsics.ExtrinsicsError, witness_mod.WitnessError) as error:
        raise SystemExit(str(error)) from error

    marker = base_marker.parse_spec(args.marker_at, args.marker_size)
    if rig is None and marker is None:
        raise SystemExit("a capture needs either --extrinsics (a fixed camera) or --marker-at "
                         "(the base marker): without one the camera pose is unknown")
    options = Options(
        plane_z=args.plane_z,
        tap=tuple(args.tap) if args.tap else None,
        max_reproj=args.max_reproj, max_grazing=args.max_grazing, max_reach=args.max_reach,
        dictionary=args.dictionary, tag_id=args.tag_id, annotate=args.annotate,
        extrinsics=rig, witness=reference, witness_max_px=args.witness_max_px,
        turn=args.turns,
    )
    if marker is not None:
        options.tag_size = float(marker["size_m"])
        options.tag_x, options.tag_y, options.tag_z = marker["at"]
        options.tag_yaw = float(__import__("math").degrees(marker["yaw"]))
    try:
        payload = locate_capture(args.capture, args.calib, options)
    except LocateError as error:
        raise SystemExit(str(error)) from error

    out_path = payload.pop("_out")
    if args.out:
        out_path = args.out
        Path(out_path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    point = payload["point"]
    if payload["radius"] > args.max_reach:
        print("warning: target is %.3f m from the base axis, above --max-reach %.2f; IK will "
              "probably fail" % (payload["radius"], args.max_reach))

    if not args.quiet:
        camera = calib_mod.load(args.calib, image_size=tuple(payload["image_size"]))
        T = np.array(payload["T_base_cam"])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(T[2, 2])))))
        print("capture    : %s" % payload["capture"])
        print("calibration: %s" % camera.describe())
        if payload.get("rig"):
            rig_info = payload["rig"]
            print("rig        : %s, %d poses, %.1f mm rms, %.1f mm predicted, fitted %s"
                  % (rig_info.get("fingerprint"), rig_info.get("poses", 0),
                     rig_info.get("rms_mm") or 0.0, rig_info.get("predicted_mm") or 0.0,
                     rig_info.get("fitted", "")))
        if payload.get("tag"):
            print("tag        : id %d, %.0f mm, reprojection %.2f px"
                  % (payload["tag"]["id"], payload["tag"]["size_m"] * 1000,
                     payload["tag"]["reproj_px"]))
        if payload.get("witness"):
            print("mount      : %.2f px of drift since the fit (limit %.2f)"
                  % (payload["witness"]["drift_px"], args.witness_max_px))
        print("camera     : [%+.3f %+.3f %+.3f] m, optical axis %.1f deg from vertical"
              % (T[0, 3], T[1, 3], T[2, 3], tilt))
        if payload["tap"]:
            print("tap        : (%.1f, %.1f) px" % (payload["tap"]["u"], payload["tap"]["v"]))
        print("target     : [%+.4f %+.4f %+.4f] m  (%.3f m from the base axis)"
              % (point[0], point[1], point[2], payload["radius"]))
        print("wrote      : %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())

