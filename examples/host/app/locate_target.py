#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Turn one phone capture into a point in the arm's base frame.

The phone gives a JPEG, a tap coordinate and nothing else -- no depth, no pose.
The metric part comes from an ArUco tag lying flat on the worktop at a known
offset from the arm base: solvePnP on that tag gives `T_cam<-tag`, the placement
gives `T_base<-tag`, and the two compose into `T_base<-cam`. The tap is then a
ray, and the ray meets the table plane where the target is.

    python locate_target.py --capture captures/<id> --calib calib/<device>.json \
        --tag-size 0.080 --tag-x 0.35 --tag-y -0.20 --tag-yaw 15

`locate_capture()` is the same code the capture server calls when someone taps
the video, so a tap and a command line cannot drift apart.

The tag has to be visible **in the same frame as the target**, otherwise the
extrinsics are stale -- the phone is handheld, and a phone that moved 10 cm after
the pose was solved puts the target 10 cm off. This is the one rule the whole
design hangs on.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import calib as calib_mod  # noqa: E402

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
    point_mode: str = "tap"
    max_reproj: float = 1.5
    max_grazing: float = 80.0
    max_reach: float = 0.75
    dictionary: str = "4x4_50"
    tag_id: int | None = None
    annotate: str = ""


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
    """Pose of the tag in the camera frame, plus its reprojection error in px."""
    obj = tag_object_points(tag_size)
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)

    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise LocateError("solvePnP failed on the tag")
    try:
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    except cv2.error:
        pass  # IPPE's answer is already usable; refinement is a bonus

    R = cv2.Rodrigues(rvec)[0]
    # A planar solve has a mirror twin. The real one has +z of the tag pointing
    # back at the camera, i.e. opposite to t.
    if float(R[:, 2] @ (-tvec.reshape(3))) < 0.0:
        half_turn = cv2.Rodrigues(np.array([math.pi, 0.0, 0.0]))[0]
        R = R @ half_turn
        rvec = cv2.Rodrigues(R)[0]

    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - corners, axis=1)))
    return R, tvec.reshape(3).astype(float), reproj


def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tag_in_base(x, y, z, yaw_deg):
    """`T_base<-tag`: the printed face is up, its +x along the base x at yaw 0."""
    T = np.eye(4)
    T[:3, :3] = rotz(yaw_deg)
    T[:3, 3] = [float(x), float(y), float(z)]
    return T


def base_from_camera(R_cam_tag, t_cam_tag, T_base_tag):
    """Compose the two transforms into `T_base<-cam`."""
    T_cam_tag = np.eye(4)
    T_cam_tag[:3, :3] = R_cam_tag
    T_cam_tag[:3, 3] = t_cam_tag
    return T_base_tag @ np.linalg.inv(T_cam_tag)


def ray_plane(K, T_base_cam, u, v, plane_z):
    """Intersect the pixel's ray with the horizontal plane, in the base frame."""
    pixel = np.array([float(u), float(v), 1.0])
    direction_cam = np.linalg.solve(np.asarray(K, dtype=float), pixel)
    R = T_base_cam[:3, :3]
    origin = T_base_cam[:3, 3]
    direction = R @ direction_cam
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        raise LocateError("degenerate ray for pixel (%s, %s)" % (u, v))
    direction /= norm

    if abs(direction[2]) < 1e-6:
        raise LocateError("the tap ray is parallel to the table plane; pick a point further away")
    s = (float(plane_z) - origin[2]) / direction[2]
    if s <= 0.0:
        raise LocateError("the tap ray points away from the table plane "
                          "(is the plane above the camera?)")
    point = origin + s * direction
    grazing = math.degrees(math.asin(min(1.0, abs(direction[2]))))
    return point, s, grazing


def project(K, T_base_cam, point):
    """Project a base-frame point back into the image (for the annotation)."""
    p_cam = T_base_cam[:3, :3].T @ (np.asarray(point, dtype=float) - T_base_cam[:3, 3])
    if p_cam[2] <= 1e-9:
        return None
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
    cv2.aruco.drawDetectedMarkers(out, [corners.reshape(-1, 1, 2).astype(np.float32)])
    ok, rvec, tvec = cv2.solvePnP(tag_object_points(tag_size), corners.reshape(-1, 1, 2),
                                  K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if ok:
        cv2.drawFrameAxes(out, K, dist, rvec, tvec, tag_size / 2.0, 2)
    if tap is not None:
        cv2.circle(out, (int(round(tap[0])), int(round(tap[1]))), 9, (0, 220, 255), 2)
        cv2.drawMarker(out, (int(round(tap[0])), int(round(tap[1]))), (0, 220, 255),
                       cv2.MARKER_CROSS, 22, 2)
    pixel = project(K, T_base_cam, point)
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
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICTS[options.dictionary]))
    tag_id, corners = detect_tag(gray, dictionary, options.tag_id)

    R_cam_tag, t_cam_tag, reproj = tag_pose(corners, options.tag_size, calibration.K, calibration.dist)
    if reproj > options.max_reproj:
        raise LocateError(
            "tag reprojection %.2f px is above %.2f: the phone may have moved during the "
            "exposure, or the printed size is not %.1f mm"
            % (reproj, options.max_reproj, options.tag_size * 1000))

    T_base_tag = tag_in_base(options.tag_x, options.tag_y, options.tag_z, options.tag_yaw)
    T_base_cam = base_from_camera(R_cam_tag, t_cam_tag, T_base_tag)

    tap = options.tap if options.tap is not None else tap_from_meta(meta, (width, height))
    if options.point_mode == "tag":
        point = (T_base_tag @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]
        source = "tag-centre"
        tap = None
    else:
        if tap is None:
            raise LocateError("no tap in the capture; tap the target before capturing")
        point, _distance, grazing = ray_plane(calibration.K, T_base_cam, tap[0], tap[1], options.plane_z)
        source = "tap+plane"
        if grazing > options.max_grazing:
            raise LocateError(
                "the tap ray meets the table at %.0f deg (limit %.0f): a grazing view turns "
                "every pixel of the tap into centimetres -- aim further down"
                % (grazing, options.max_grazing))

    radius = float(np.linalg.norm(point))
    return {
        "format": "rebot-target",
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
        "calib_fingerprint": calibration.fingerprint,
        "image_size": [int(width), int(height)],
        "tag": {
            "id": int(tag_id),
            "size_m": float(options.tag_size),
            "placement": {"x": options.tag_x, "y": options.tag_y,
                          "z": options.tag_z, "yaw_deg": options.tag_yaw},
            "reproj_px": float(reproj),
        },
        "tap": {"u": float(tap[0]), "v": float(tap[1])} if tap is not None else None,
        "plane_z": float(options.plane_z),
        "point": [float(v) for v in point],
        "radius": radius,
        "T_base_cam": [[float(v) for v in row] for row in T_base_cam],
        "_corners": corners,
        "_T_base_cam": T_base_cam,
    }


def locate_capture(directory, calib_path, options: Options):
    """The whole job for one capture directory: read, solve, annotate, write."""
    image, meta, capture_dir, image_path = load_capture(directory)
    calibration = calib_mod.load(calib_path, image_size=(image.shape[1], image.shape[0]))
    payload = locate(image, meta, calibration, options)

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


def main(argv=None):
    parser = argparse.ArgumentParser(description="Locate a tapped point in the arm base frame.")
    parser.add_argument("--capture", required=True, help="capture directory or image file")
    parser.add_argument("--calib", required=True, help="calib/<device>.json")
    parser.add_argument("--out", default="", help="target.json (default: <capture>/target.json)")
    parser.add_argument("--dictionary", default="4x4_50", choices=sorted(ARUCO_DICTS))
    parser.add_argument("--tag-id", type=int, default=None, help="default: largest marker in view")
    parser.add_argument("--tag-size", type=float, default=0.080,
                        help="black square side including the border, metres")
    parser.add_argument("--tag-x", type=float, default=0.0, help="tag centre, base x, m")
    parser.add_argument("--tag-y", type=float, default=0.0, help="tag centre, base y, m")
    parser.add_argument("--tag-z", type=float, default=0.0, help="tag face height above base plane, m")
    parser.add_argument("--tag-yaw", type=float, default=0.0, help="tag rotation about base z, deg")
    parser.add_argument("--plane-z", type=float, default=0.0, help="worktop height in base frame, m")
    parser.add_argument("--tap", nargs=2, type=float, metavar=("U", "V"),
                        help="override the tap stored in meta.json, image pixels")
    parser.add_argument("--point-mode", choices=("tap", "tag"), default="tap",
                        help="'tag' uses the tag centre (self-test / sanity check)")
    parser.add_argument("--max-reproj", type=float, default=1.5,
                        help="reject a tag pose above this reprojection error, px")
    parser.add_argument("--max-grazing", type=float, default=80.0,
                        help="reject taps whose ray is flatter than this, deg from the plane")
    parser.add_argument("--max-reach", type=float, default=0.75, help="warn above this radius, m")
    parser.add_argument("--annotate", default="", help="write an annotated JPEG here")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    options = Options(
        tag_size=args.tag_size, tag_x=args.tag_x, tag_y=args.tag_y, tag_z=args.tag_z,
        tag_yaw=args.tag_yaw, plane_z=args.plane_z,
        tap=tuple(args.tap) if args.tap else None, point_mode=args.point_mode,
        max_reproj=args.max_reproj, max_grazing=args.max_grazing, max_reach=args.max_reach,
        dictionary=args.dictionary, tag_id=args.tag_id, annotate=args.annotate,
    )
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
        print("tag        : id %d, %.0f mm, reprojection %.2f px"
              % (payload["tag"]["id"], payload["tag"]["size_m"] * 1000, payload["tag"]["reproj_px"]))
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
