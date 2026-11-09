#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fit the phone camera's intrinsics from checkerboard shots taken in the browser.

The phone page has a "calibration" capture mode that writes `purpose: calib`
captures. Point it at a printed board in 15-20 different poses -- tilt it, put it
in the corners, fill the frame and then pull back -- and run this on the host:

    python examples/host/app/calibrate_intrinsics.py \
        --captures examples/host/state/captures --cols 9 --rows 6 --square 0.025

Output is `calib/<device>.json`, which `locate_target.py` reads. The RMS gate
matters more than the number of views: a 0.5 px fit on 12 good views beats
1.5 px on 30 sloppy ones, and a bad `K` shows up later as a target that is
"always 30 mm to the left".

Aspect-ratio changes are not rescaled here. If the page captured at 1920x1080
and later streams 1280x720, `calib.load(..., image_size)` does the scaling; it
refuses to cross aspect ratios, because a cropped frame's principal point is not
the scaled one.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import calib as calib_mod  # noqa: E402


def find_capture_dirs(root: Path, purpose: str, use_all: bool):
    """Capture directories under `root`, oldest first, filtered by meta.purpose."""
    found = []
    for entry in sorted(root.iterdir() if root.is_dir() else []):
        if not entry.is_dir():
            continue
        frame = entry / "frame.jpg"
        meta_path = entry / "meta.json"
        if not frame.is_file():
            continue
        meta = {}
        if meta_path.is_file():
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                meta = {}
        if not use_all and meta.get("purpose") != purpose:
            continue
        found.append((entry, meta))
    return found


def detect_corners(gray, cols, rows):
    """Find a chessboard, preferring the sector-based detector when available."""
    pattern = (cols, rows)
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = 0
        flags |= getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0)
        flags |= getattr(cv2, "CALIB_CB_ACCURACY", 0)
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        if ok:
            return True, corners.reshape(-1, 1, 2).astype(np.float32)

    flags = getattr(cv2, "CALIB_CB_ADAPTIVE_THRESH", 0) | getattr(cv2, "CALIB_CB_NORMALIZE_IMAGE", 0)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if not ok:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    return True, corners


def board_points(cols, rows, square):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * float(square)


def fit(views, size, objp, drops=0, max_view_error=2.0):
    """calibrateCamera over `views`, then drop the worst views until they settle.

    A single blurred or rolled board can drag the fit; the per-view reprojection
    error is the cheapest detector for it, and dropping it is worth more than
    adding five more views.
    """
    objpoints = [objp for _ in views]
    imgpoints = [corners for _, corners in views]
    dropped = []

    for _ in range(max(1, drops + 1)):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, size, None, None,
            flags=0,
            criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 60, 1e-9),
        )
        errors = []
        for corners, rvec, tvec in zip(imgpoints, rvecs, tvecs):
            projected, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
            errors.append(float(np.mean(np.linalg.norm(
                projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1))))
        if drops <= 0 or len(views) <= 6:
            break
        worst = int(np.argmax(errors))
        if errors[worst] <= max_view_error:
            break
        dropped.append((views[worst][0], errors[worst]))
        views = views[:worst] + views[worst + 1:]
        objpoints = [objp for _ in views]
        imgpoints = [corners for _, corners in views]
        drops -= 1

    return rms, K, dist, errors, views, dropped


def annotate(image, corners, cols, rows):
    out = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(out, (cols, rows), corners, True)
    return out


def main(argv=None):
    here = Path(__file__).resolve().parent
    state = here.parent / "state"
    parser = argparse.ArgumentParser(description="Fit phone-camera intrinsics from checkerboard captures.")
    parser.add_argument("--captures", default=str(state / "captures"),
                        help="capture root written by capture_server.py")
    parser.add_argument("--out", default=str(state / "calib"), help="where calibration JSONs go")
    parser.add_argument("--cols", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument("--square", type=float, default=0.025, help="square side, metres")
    parser.add_argument("--purpose", default="calib", help="only use captures with this purpose")
    parser.add_argument("--all", action="store_true", help="use every capture, ignore purpose")
    parser.add_argument("--device", default="", help="override the calibration file name")
    parser.add_argument("--min-views", type=int, default=8)
    parser.add_argument("--max-rms", type=float, default=1.0, help="reject the fit above this, px")
    parser.add_argument("--max-view-error", type=float, default=2.0,
                        help="drop views above this reprojection error, px")
    parser.add_argument("--drops", type=int, default=3, help="how many bad views to drop at most")
    parser.add_argument("--force", action="store_true", help="write the file even above --max-rms")
    parser.add_argument("--annotate", action="store_true", help="save detected corners next to the images")
    args = parser.parse_args(argv)

    root = Path(args.captures)
    if not root.is_dir():
        raise SystemExit("no capture directory at %s" % root)

    dirs = find_capture_dirs(root, args.purpose, args.all)
    if not dirs:
        raise SystemExit(
            "no %s captures under %s -- open the phone page, enable calibration mode, "
            "and take %d+ shots of the board" % (args.purpose, root, args.min_views))

    objp = board_points(args.cols, args.rows, args.square)
    views, rejected = [], []
    size = None
    for directory, meta in dirs:
        image = cv2.imread(str(directory / "frame.jpg"), cv2.IMREAD_COLOR)
        if image is None:
            rejected.append((directory.name, "unreadable"))
            continue
        if size is None:
            size = (image.shape[1], image.shape[0])
        elif (image.shape[1], image.shape[0]) != size:
            rejected.append((directory.name, "size %dx%d != %dx%d"
                             % (image.shape[1], image.shape[0], size[0], size[1])))
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, corners = detect_corners(gray, args.cols, args.rows)
        if not ok:
            rejected.append((directory.name, "no %dx%d board" % (args.cols, args.rows)))
            continue
        views.append((directory, corners))
        if args.annotate:
            (directory / "corners.jpg").write_bytes(
                cv2.imencode(".jpg", annotate(image, corners, args.cols, args.rows))[1].tobytes())

    print("checkerboard: %dx%d inner corners, %.1f mm squares" % (args.cols, args.rows, args.square * 1000))
    if size:
        print("captures    : %d usable, %d rejected, calibration at %dx%d"
              % (len(views), len(rejected), size[0], size[1]))
    else:
        print("captures    : none usable")
    for name, why in rejected:
        print("  skip %-28s %s" % (name, why))

    if len(views) < args.min_views:
        raise SystemExit("only %d usable views, need %d (take more, tilted and in the corners)"
                         % (len(views), args.min_views))

    rms, K, dist, errors, views, dropped = fit(
        views, size, objp, drops=args.drops, max_view_error=args.max_view_error)

    print("\n  view                          error")
    for (directory, _), error in sorted(zip(views, errors), key=lambda p: -p[1]):
        print("  %-28s %5.2f px" % (directory.name, error))
    for directory, error in dropped:
        print("  %-28s %5.2f px  (dropped)" % (directory.name, error))

    fx, fy = K[0, 0], K[1, 1]
    print("\n  views  : %d of %d" % (len(views), len(dirs)))
    print("  rms    : %.3f px (mean of the kept views %.3f px)" % (rms, float(np.mean(errors))))
    print("  fx/fy  : %.1f / %.1f px  (difference %.2f %%)" % (fx, fy, 100.0 * abs(fx - fy) / max(fx, fy)))
    print("  centre : (%.1f, %.1f) of %dx%d" % (K[0, 2], K[1, 2], size[0], size[1]))

    if abs(fx - fy) / max(fx, fy) > 0.03:
        print("  warning: fx and fy differ by more than 3 %%; the board may be bent or the shots too flat")
    if rms > args.max_rms and not args.force:
        raise SystemExit("  rms %.2f px is above --max-rms %.2f; retake the board shots or use --force"
                         % (rms, args.max_rms))

    # Name the file after the device the page reported, so a second phone or the
    # front camera does not overwrite the first calibration.
    device = args.device
    if not device:
        meta_path = views[0][0] / "meta.json"
        meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
        settings = meta.get("settings") or {}
        label = settings.get("deviceId") or (meta.get("device") or {}).get("ua", "")
        device = calib_mod.slug(str(label)[-40:], "camera")

    payload = {
        "format": "rebot-camera-calib",
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "device": device,
        "image_size": [int(size[0]), int(size[1])],
        "K": [[float(v) for v in row] for row in K],
        "dist": [float(v) for v in dist.reshape(-1)],
        "rms": float(rms),
        "board": {"cols": args.cols, "rows": args.rows, "square_m": float(args.square)},
        "views": len(views),
        "source_captures": [directory.name for directory, _ in views],
        "notes": "pinhole + radial/tangential; rescale with calib.scale_K for other sizes",
    }
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / ("%s.json" % device)
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print("\n  wrote  : %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
