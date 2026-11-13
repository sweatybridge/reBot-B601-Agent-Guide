#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks for the phone-capture pipeline: no phone, no arm, no Pinocchio.

Three things fail quietly and cost a day on the arm: a wrong intrinsics fit, a
wrong tag-to-plane transform, and a trajectory JSON the player refuses at the
worst moment. This rebuilds all three from synthetic images with a known camera
and checks the numbers against ground truth, then round-trips a capture through
the real HTTP server.

    python examples/host/tests/selftest.py

Everything here is deterministic. If it fails, the pipeline is wrong, not the
measurement.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
APP = HERE.parent / "app"
sys.path.insert(0, str(APP))

import auto_calibrate  # noqa: E402
import base_marker  # noqa: E402
import calib as calib_mod  # noqa: E402
import calibrate_extrinsics as extrinsics_cli  # noqa: E402
import calibrate_intrinsics as calib_cli  # noqa: E402
import extrinsics  # noqa: E402
import locate_target as locate  # noqa: E402
import rebot_ik  # noqa: E402
import mediamtx  # noqa: E402
import sweep  # noqa: E402
import sweep_video  # noqa: E402
import traj  # noqa: E402
import witness  # noqa: E402

RESULTS = []


def check(name, condition, detail=""):
    RESULTS.append((name, bool(condition), detail))
    mark = "ok  " if condition else "FAIL"
    print("  [%s] %s%s" % (mark, name, ("  -- " + detail) if detail else ""))
    return bool(condition)


def synthetic_camera(width=1600, height=1200, f=1600.0):
    K = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
    return K, np.zeros(5)


def render_board(cols, rows, square, K, dist, R, t, size, pps=180, margin=2):
    """Draw a real checkerboard by warping a pattern image through the camera."""
    width, height = size
    pattern_w = (cols + 1 + 2 * margin) * pps
    pattern_h = (rows + 1 + 2 * margin) * pps
    pattern = np.full((pattern_h, pattern_w), 255, np.uint8)
    for i in range(cols + 1):
        for j in range(rows + 1):
            if (i + j) % 2 == 0:
                x0 = (margin + i) * pps
                y0 = (margin + j) * pps
                pattern[y0:y0 + pps, x0:x0 + pps] = 0

    meters_to_pattern = np.array([[pps / square, 0.0, margin * pps],
                                  [0.0, pps / square, margin * pps],
                                  [0.0, 0.0, 1.0]])
    board_to_image = np.asarray(K, float) @ np.column_stack([R[:, 0], R[:, 1], t])
    # warpPerspective applies M to source coordinates, so the source (pattern
    # pixels) has to be mapped back to board metres first.
    homography = board_to_image @ np.linalg.inv(meters_to_pattern)
    image = cv2.warpPerspective(pattern, homography, (width, height),
                                flags=cv2.INTER_LINEAR, borderValue=255)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def rotation(axis, degrees):
    radians = math.radians(degrees)
    c, s = math.cos(radians), math.sin(radians)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def look_at(position, target, up=(0.0, 0.0, 1.0)):
    """OpenCV camera pose: x right, y down, z forward."""
    position = np.asarray(position, float)
    forward = np.asarray(target, float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(np.asarray(up, float), forward)
    if np.linalg.norm(right) < 1e-9:
        raise ValueError("degenerate look_at: forward is parallel to up")
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    T = np.eye(4)
    T[:3, :3] = np.stack([right, down, forward], axis=1)
    T[:3, 3] = position
    return T


def project(K, T_base_cam, point):
    p = T_base_cam[:3, :3].T @ (np.asarray(point, float) - T_base_cam[:3, 3])
    if p[2] <= 1e-9:
        return None
    q = np.asarray(K, float) @ p
    return float(q[0] / q[2]), float(q[1] / q[2])


# --------------------------------------------------------------------------- #
# 1. Intrinsics
# --------------------------------------------------------------------------- #

def test_calibration(workdir: Path, cols=9, rows=6, square=0.025):
    print("\nintrinsics from synthetic checkerboards")
    size = (1600, 1200)
    K_true, dist_true = synthetic_camera(*size)
    poses = [
        ((0, 0, 0), (0.00, 0.00, 0.62)),
        ((0, 16, 0), (0.02, 0.00, 0.66)),
        ((12, 0, 10), (-0.06, 0.03, 0.70)),
        ((-14, 0, -12), (0.07, -0.02, 0.68)),
        ((0, -20, 15), (-0.02, 0.05, 0.60)),
        ((18, 8, 0), (0.03, -0.06, 0.64)),
        ((-8, -14, 18), (0.06, 0.05, 0.72)),
        ((0, 22, -16), (-0.05, -0.04, 0.62)),
        ((10, -10, 6), (0.00, 0.07, 0.75)),
        ((-16, 12, -8), (0.02, -0.05, 0.70)),
        ((6, 6, 24), (-0.04, 0.00, 0.66)),
        ((-6, -6, -24), (0.05, 0.04, 0.63)),
    ]

    root = workdir / "calib-captures"
    root.mkdir(parents=True, exist_ok=True)
    for index, ((rx, ry, rz), position) in enumerate(poses):
        R = rotation("z", rz) @ rotation("x", rx) @ rotation("y", ry)
        image = render_board(cols, rows, square, K_true, dist_true, R,
                             np.array(position, float), size)
        directory = root / ("20260101T0000%02dZ-abcdef" % index)
        directory.mkdir(exist_ok=True)
        cv2.imwrite(str(directory / "frame.jpg"), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (directory / "meta.json").write_text(json.dumps({
            "purpose": "calib",
            "image_size": list(size),
            "settings": {"width": size[0], "height": size[1], "deviceId": "synthetic"},
        }), encoding="utf-8")

    out = workdir / "calib-out"
    code = calib_cli.main([
        "--captures", str(root), "--out", str(out),
        "--cols", str(cols), "--rows", str(rows), "--square", str(square),
        "--max-rms", "2.0", "--min-views", "8",
    ])
    files = sorted(out.glob("*.json"))
    if not check("calibration wrote a file", code == 0 and files):
        return None

    fitted = calib_mod.load(files[0], image_size=size)
    fx_error = abs(fitted.K[0, 0] - K_true[0, 0]) / K_true[0, 0]
    fy_error = abs(fitted.K[1, 1] - K_true[1, 1]) / K_true[1, 1]
    check("fx within 2 %", fx_error < 0.02, "fx %.1f vs %.1f" % (fitted.K[0, 0], K_true[0, 0]))
    check("fy within 2 %", fy_error < 0.02, "fy %.1f vs %.1f" % (fitted.K[1, 1], K_true[1, 1]))
    check("principal point within 12 px",
          abs(fitted.K[0, 2] - K_true[0, 2]) < 12 and abs(fitted.K[1, 2] - K_true[1, 2]) < 12,
          "centre (%.1f, %.1f)" % (fitted.K[0, 2], fitted.K[1, 2]))
    check("rms under 1 px", fitted.rms < 1.0, "rms %.3f" % fitted.rms)

    # Rescaling is what protects a calibration taken at 1600x1200 from a
    # 1280x960 still, and it must refuse a crop.
    half = calib_mod.scale_K(fitted.K, size, (800, 600))
    check("rescales fx by exactly one half", abs(half[0, 0] - fitted.K[0, 0] / 2) < 1e-9)
    try:
        calib_mod.scale_K(fitted.K, size, (800, 500))
        check("refuses an aspect-ratio change", False)
    except calib_mod.CalibError:
        check("refuses an aspect-ratio change", True)

    return {"calib": files[0], "K": fitted.K.copy(), "dist": fitted.dist.copy(), "size": size}


# --------------------------------------------------------------------------- #
# 2. Tag pose and the felt-tip dot
# --------------------------------------------------------------------------- #

def render_tag(dictionary, tag_id, tag_size, corners_px, size, quiet=0.25):
    """Paste a printable tag onto a white sheet at the four projected corners."""
    width, height = size
    side = 800
    marker = dictionary.generateImageMarker(tag_id, side)
    border = int(round(side * quiet))
    sheet = cv2.copyMakeBorder(marker, border, border, border, border,
                               cv2.BORDER_CONSTANT, value=255)
    sheet_size = sheet.shape[0]

    # The sheet is the tag plus a quiet zone; map the tag corners into sheet px.
    offset = (sheet_size - side) / 2.0
    sheet_corners = np.float32([
        [offset, offset],
        [offset + side, offset],
        [offset + side, offset + side],
        [offset, offset + side],
    ])
    homography = cv2.getPerspectiveTransform(sheet_corners, corners_px.astype(np.float32))
    warped = cv2.warpPerspective(sheet, homography, (width, height),
                                 flags=cv2.INTER_LINEAR, borderValue=255)
    return cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)


def test_tag_geometry(workdir: Path):
    print("\ntag pose, extrinsics and the table plane")
    size = (1600, 1200)
    K, dist = synthetic_camera(*size)
    tag_id, tag_size = 3, 0.080
    # The marker stands on the side of the base, facing -x; the phone is in front.
    tag_x, tag_y, tag_z, tag_yaw = 0.36, -0.10, 0.12, 180.0

    T_base_cam = look_at((0.02, -0.30, 0.55), (0.30, -0.05, 0.05))
    T_base_tag = locate.tag_in_base(tag_x, tag_y, tag_z, tag_yaw)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    obj = locate.tag_object_points(tag_size)
    R_cam_tag_true = T_base_cam[:3, :3].T @ T_base_tag[:3, :3]
    t_cam_tag_true = T_base_cam[:3, :3].T @ (T_base_tag[:3, 3] - T_base_cam[:3, 3])
    corners, _ = cv2.projectPoints(obj, cv2.Rodrigues(R_cam_tag_true)[0], t_cam_tag_true, K, dist)
    corners = corners.reshape(-1, 2)
    image = render_tag(dictionary, tag_id, tag_size, corners, size)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found_id, found_corners = locate.detect_tag(gray, dictionary, tag_id)
    check("detects the tag", found_id == tag_id)

    R_cam_tag, t_cam_tag, reproj = locate.tag_pose(found_corners, tag_size, K, dist)
    check("tag reprojection under 1 px", reproj < 1.0, "%.3f px" % reproj)

    recovered = locate.base_from_camera(R_cam_tag, t_cam_tag, T_base_tag)
    position_error = float(np.linalg.norm(recovered[:3, 3] - T_base_cam[:3, 3]))
    angle = math.degrees(math.acos(max(-1.0, min(1.0,
        (np.trace(recovered[:3, :3].T @ T_base_cam[:3, :3]) - 1.0) / 2.0))))
    check("camera position within 6 mm", position_error < 0.006, "%.2f mm" % (position_error * 1000))
    check("camera rotation within 1.5 deg", angle < 1.5, "%.2f deg" % angle)

    truth = np.array([0.30, -0.02, 0.0])
    pixel = project(K, T_base_cam, truth)
    check("target projects inside the image", pixel is not None and 0 <= pixel[0] < size[0])

    point, distance, grazing = locate.ray_plane(K, recovered, pixel[0], pixel[1], 0.0)
    error = float(np.linalg.norm(point - truth))
    check("tap ray lands within 3 mm", error < 0.003, "%.2f mm at %.2f m, %.0f deg"
          % (error * 1000, distance, grazing))

    back = locate.project(K, recovered, point)
    check("point reprojects to the tap",
          abs(back[0] - pixel[0]) < 1.0 and abs(back[1] - pixel[1]) < 1.0)

    # A tag that is not in the frame must be a clear error, not a stale pose.
    empty = np.full((size[1], size[0]), 255, np.uint8)
    try:
        locate.detect_tag(empty, dictionary, tag_id)
        check("missing tag is an error", False)
    except locate.LocateError:
        check("missing tag is an error", True)

    return {"image": image, "corners": corners, "T_base_cam": T_base_cam, "point": truth,
            "pixel": pixel, "K": K, "dist": dist}


# --------------------------------------------------------------------------- #
# 3. The trajectory schema
# --------------------------------------------------------------------------- #

def test_schema():
    print("\ntrajectory payload matches what the player accepts")
    limits = [[-3.0, 3.0]] * 6 + [[-0.1, 5.0]]
    points = [traj.waypoint(0.0, [0.0] * 7, [0.0] * 7),
              traj.waypoint(1 / 30.0, [0.0] * 6 + [0.5], [0.1] * 7),
              traj.waypoint(2 / 30.0, [0.0] * 6 + [1.0], [0.2] * 7)]
    payload = traj.build(points, 30.0, limits, "selftest")
    stats = traj.validate(payload)
    check("accepts a well-formed file", stats["waypoints"] == 3 and payload["format"] == traj.FORMAT)

    # The gateway's /move: joint interpolation with no gravity feedforward.
    # `build` drops a None tau, so `validate` must not insist the key is there --
    # it did, and every sweep died on its first move with KeyError: 'tau'.
    move = traj.joint_move(np.zeros(7), np.array([0.2, 0.3, 0.4, 0.0, 0.2, 0.0, 0.0]),
                           1.0, limits=limits)
    move_stats = traj.validate(move)
    check("a joint move validates with no tau at all",
          move_stats["waypoints"] > 2
          and all("tau" not in point for point in move["waypoints"]))
    check("a joint move reports zero peak tau", move_stats["peak_tau"] == 0.0)

    # One file for the whole sweep: eight holds, one round trip, no return leg
    # between poses.
    sweep_motion = traj.joint_sequence(
        [np.zeros(7), np.array([0.2] * 7), np.array([0.4] * 7), np.zeros(7)],
        [0.5, 0.5, 0.5], hold_s=0.3,
        limits=[[-3.0, 3.0]] * 6 + [[-0.1, 5.0]])
    sweep_stats = traj.validate(sweep_motion)
    check("a multi-pose file validates",
          sweep_stats["waypoints"] > 30 and sweep_stats["max_step_rad"] <= 0.5,
          "%d waypoints, %.3f rad" % (sweep_stats["waypoints"], sweep_stats["max_step_rad"]))
    check("a multi-pose file holds at each pose",
          sweep_stats["duration_s"] > 3 * 0.5 + 2 * 0.3, "%.2f s" % sweep_stats["duration_s"])

    def rejects(name, mutate):
        broken = json.loads(json.dumps(payload))
        mutate(broken)
        try:
            traj.validate(broken)
            check(name, False)
        except ValueError:
            check(name, True)

    rejects("rejects non-monotonic time", lambda p: p["waypoints"][1].update(t=-1.0))
    rejects("rejects tau on only some waypoints", lambda p: p["waypoints"][2].pop("tau"))
    rejects("rejects a joint outside its limit",
            lambda p: p["waypoints"][0]["q"].__setitem__(1, 2.9))
    rejects("rejects a 0.5 rad jump",
            lambda p: p["waypoints"][1]["q"].__setitem__(0, 0.6))
    rejects("rejects fewer than two waypoints", lambda p: p.update(waypoints=p["waypoints"][:1]))
    rejects("rejects the wrong format", lambda p: p.update(format="something-else"))
    rejects("rejects a short q", lambda p: p["waypoints"][0].update(q=[0.0] * 6))


# --------------------------------------------------------------------------- #
# 4. The HTTP server
# --------------------------------------------------------------------------- #

async def test_server(workdir: Path, geometry):
    print("\ncapture server round trip")
    from aiohttp import FormData
    from aiohttp.test_utils import TestClient, TestServer

    import capture_server

    image = geometry["image"]
    captures = workdir / "server-captures"
    calib_dir = workdir / "server-calib"
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [image.shape[1], image.shape[0]],
        "K": geometry["K"].tolist(), "dist": geometry["dist"].tolist(), "rms": 0.2,
    }), encoding="utf-8")

    args = SimpleNamespace(
        host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
        token="selftest-token", mediamtx="http://127.0.0.1:9", whip_path="phone",
        no_whip=True, max_mb=4, quiet=True,
        planner="http://127.0.0.1:9", planner_token="", execute=False,
        calib_dir=str(calib_dir), calib="",
        marker_at="0.36,-0.10,0.12,180", marker_size=0.08,
        mode="", hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="",
    )
    app = capture_server.build_app(args)

    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    jpeg = buffer.tobytes()

    async with TestClient(TestServer(app)) as client:
        health = await (await client.get("/health")).json()
        check("health is ok", health.get("ok") is True and health.get("auth") is True)
        check("health reports the camera mode", health.get("mode") == "handheld"
              and health.get("rig") is None, str(health.get("mode")))
        check("serves the phone page", (await client.get("/")).status == 200)

        form = FormData()
        form.add_field("meta", json.dumps({"purpose": "plan", "image_size": [image.shape[1], image.shape[0]],
                                           "tap": {"u": float(geometry["pixel"][0]),
                                                   "v": float(geometry["pixel"][1])}}))
        form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
        response = await client.post("/capture", data=form)
        check("refuses a write without the token", response.status == 401)

        form = FormData()
        form.add_field("meta", json.dumps({"purpose": "plan", "image_size": [image.shape[1], image.shape[0]],
                                           "tap": {"u": float(geometry["pixel"][0]),
                                                   "v": float(geometry["pixel"][1])}}))
        form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
        response = await client.post("/capture", data=form, headers={"X-Rebot-Token": "selftest-token"})
        body = await response.json()
        check("accepts a capture", response.status == 201 and body.get("ok") is True, body.get("id", ""))

        directory = captures / body["id"]
        check("frame and meta are on disk",
              (directory / "frame.jpg").is_file() and (directory / "meta.json").is_file())

        meta = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
        check("meta keeps the tap and gets a server block",
              abs(meta.get("tap", {}).get("u", 0) - geometry["pixel"][0]) < 0.01
              and meta.get("server", {}).get("bytes") == len(jpeg))

        listing = await (await client.get("/captures.json")).json()
        check("captures.json lists it", len(listing["captures"]) == 1
              and listing["captures"][0]["id"] == body["id"])

        served = await client.get("/captures/%s/frame.jpg" % body["id"])
        check("serves the frame back", served.status == 200 and len(await served.read()) == len(jpeg))

        traversal = await client.get("/captures/%s/..%%2Fmeta.json" % body["id"])
        check("refuses a path traversal", traversal.status == 404)

        form = FormData()
        form.add_field("meta", json.dumps({"purpose": "not-a-purpose"}))
        form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
        bad = await client.post("/capture", data=form, headers={"X-Rebot-Token": "selftest-token"})
        check("refuses an unknown purpose", bad.status == 400)

        form = FormData()
        form.add_field("meta", json.dumps({"purpose": "plan", "id": "forged"}))
        form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
        forged = await client.post("/capture", data=form, headers={"X-Rebot-Token": "selftest-token"})
        check("refuses a client-set id", forged.status == 400)
        check("cleans up after a refused capture", len(list(captures.iterdir())) == 1)

        # A tap becomes a plan: locate runs for real (the rendered tag is in the
        # frame), then the arm gateway is unreachable, which must read as a
        # planner problem -- 502 -- not a locate problem -- 422.
        planned = await client.post("/plan", json={"capture": body["id"]},
                                    headers={"X-Rebot-Token": "selftest-token"})
        answer = await planned.json()
        check("tap -> locate -> plan reaches the gateway",
              planned.status == 502 and answer.get("stage") == "plan",
              "status %s, %s" % (planned.status, str(answer.get("error"))[:60]))
        check("locate wrote target.json",
              (captures / body["id"] / "target.json").is_file())
        check("locate annotated the frame",
              (captures / body["id"] / "annotated.jpg").is_file())
        check("execution is off unless asked for",
              (await client.post("/execute", json={"id": "x"},
                                 headers={"X-Rebot-Token": "selftest-token"})).status == 403)


# --------------------------------------------------------------------------- #
# 5. locate_target.py as the host actually runs it
# --------------------------------------------------------------------------- #

def test_locate_cli(workdir: Path, geometry):
    print("\nlocate_target.py end to end")
    K, dist, size = geometry["K"], geometry["dist"], (1600, 1200)
    tag_x, tag_y, tag_z, tag_yaw, tag_size = 0.36, -0.10, 0.12, 180.0, 0.080
    truth = geometry["point"]

    capture = workdir / "captures" / "20260101T120000Z-abc123"
    capture.mkdir(parents=True)
    cv2.imwrite(str(capture / "frame.jpg"), geometry["image"], [cv2.IMWRITE_JPEG_QUALITY, 95])
    (capture / "meta.json").write_text(json.dumps({
        "purpose": "plan",
        "image_size": list(size),
        "tap": {"u": geometry["pixel"][0], "v": geometry["pixel"][1]},
        "settings": {"deviceId": "synthetic"},
    }), encoding="utf-8")

    calib_path = workdir / "synthetic-calib.json"
    calib_path.write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": list(size), "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")

    code = locate.main([
        "--capture", str(capture), "--calib", str(calib_path),
        "--marker-at", "%g,%g,%g,%g" % (tag_x, tag_y, tag_z, tag_yaw),
        "--marker-size", str(tag_size), "--annotate", str(capture / "annotated.jpg"),
        "--quiet",
    ])
    target_path = capture / "target.json"
    if not check("wrote target.json", code == 0 and target_path.is_file()):
        return
    target = json.loads(target_path.read_text(encoding="utf-8"))
    point = np.array(target["point"])
    error = float(np.linalg.norm(point - truth))
    check("target within 3 mm of the truth", error < 0.003, "%.2f mm" % (error * 1000))
    check("target.json carries the provenance",
          target.get("format") == "rebot-target" and target.get("calib_fingerprint")
          and len(target.get("T_base_cam", [])) == 4)
    check("annotated image written", (capture / "annotated.jpg").is_file())


# --------------------------------------------------------------------------- #
# 6. The fixed camera: a rig fitted from the arm, and the witness that guards it
# --------------------------------------------------------------------------- #

def synthetic_rig(seed=7, poses=8):
    """A known camera, a known tip, and a spread set of poses that see it."""
    K = np.array([[1600.0, 0.0, 800.0], [0.0, 1600.0, 600.0], [0.0, 0.0, 1.0]])
    dist = np.zeros(5)
    T_true = look_at((0.20, -1.15, 1.25), (0.25, 0.05, 0.0))
    tip = np.array([0.0, 0.0, 0.035])

    rng = np.random.default_rng(seed)
    candidates = []
    for _ in range(4000):
        q = np.array([rng.uniform(lo, hi) for lo, hi in rebot_ik.LIMITS])
        T = rebot_ik.forward(q)
        point = T[:3, :3] @ tip + T[:3, 3]
        if not (0.10 < math.hypot(point[0], point[1]) < 0.55):
            continue
        if not (-0.02 < point[2] < 0.35):
            continue
        seen = extrinsics.pixel(point, K, dist, extrinsics.invert(T_true))
        if seen is None or not (100 < seen[0] < 1500 and 100 < seen[1] < 1100):
            continue
        candidates.append((q, point, seen))
    if len(candidates) < poses:
        raise RuntimeError("the synthetic sweep found only %d usable poses" % len(candidates))

    # Farthest-point sampling: the sweep has to move, not just twitch.
    chosen = [0]
    while len(chosen) < poses:
        best, best_distance = None, -1.0
        for index, (_q, point, _seen) in enumerate(candidates):
            if index in chosen:
                continue
            distance = min(np.linalg.norm(point - candidates[j][1]) for j in chosen)
            if distance > best_distance:
                best, best_distance = index, distance
        chosen.append(best)

    observations = []
    for index in chosen:
        q, _point, seen = candidates[index]
        observations.append(extrinsics.Observation(
            q=q, uv=(seen[0] + rng.normal(0.0, 0.4), seen[1] + rng.normal(0.0, 0.4)),
            capture="synthetic-%d" % index))
    return K, dist, T_true, tip, observations


def textured_frame(size=(1600, 1200), seed=3):
    """A frame with enough corners for the witness to track."""
    rng = np.random.default_rng(seed)
    image = np.full((size[1], size[0]), 90, np.uint8)
    for _ in range(120):
        x, y = int(rng.integers(30, size[0] - 60)), int(rng.integers(30, size[1] - 60))
        w, h = int(rng.integers(12, 46)), int(rng.integers(12, 46))
        value = int(rng.integers(20, 240))
        cv2.rectangle(image, (x, y), (x + w, y + h), value, -1)
    return cv2.GaussianBlur(image, (3, 3), 0)


def test_base_marker():
    """A printed square on the plate: render it at a known pose, solve it back."""
    print("\nbase marker: the camera pose from one photograph")
    K = np.array([[900.0, 0.0, 540.0], [0.0, 900.0, 960.0], [0.0, 0.0, 1.0]])
    dist = np.zeros(5)
    size_m = base_marker.SIZE_M

    # A camera 0.9 m up and 0.45 m in front, looking at the plate.
    T_base_cam = extrinsics.look_at(np.array([0.0, -0.45, 0.9]), np.zeros(3))
    at = np.array([0.02, 0.01, 0.001])
    yaw = np.radians(15.0)
    T_base_marker = base_marker.base_from_marker(at, yaw)
    T_cam_marker = np.linalg.inv(T_base_cam) @ T_base_marker

    # Draw the marker where that pose says it is: project its corners and warp
    # the bitmap into the quad, which is exactly what a photograph would show.
    rotation = T_cam_marker[:3, :3]
    _rvec, _ = cv2.Rodrigues(rotation)
    projected, _ = cv2.projectPoints(base_marker.object_points(size_m), _rvec,
                                     T_cam_marker[:3, 3], K, dist)
    quad = projected.reshape(4, 2).astype(np.float32)
    bitmap = base_marker.marker_bitmap(size_m, 1200.0)
    side = 1600
    image = np.full((1920, 1080), 210, dtype=np.uint8)
    source = np.array([[0, 0], [bitmap.shape[1] - 1, 0],
                       [bitmap.shape[1] - 1, bitmap.shape[0] - 1],
                       [0, bitmap.shape[0] - 1]], dtype=np.float32)
    warped = cv2.warpPerspective(bitmap, cv2.getPerspectiveTransform(source, quad),
                                 (1080, 1920), dst=image, borderValue=210)
    cv2.imwrite(str(Path(".") / "marker-render.png"), warped)

    solved = base_marker.pose(warped, K, dist, size_m=size_m)
    angle_error = float(np.degrees(np.arccos(np.clip(
        (np.trace(solved[:3, :3].T @ rotation) - 1.0) / 2.0, -1.0, 1.0))))
    distance_error = float(np.linalg.norm(solved[:3, 3] - T_cam_marker[:3, 3]))
    # A 5 cm marker at a metre is ~50 px, so a half-pixel of corner noise is ~1%
    # of depth -- about a centimetre. That is ample: this is a *seed*, and the
    # tip detector needs a few centimetres.
    check("the marker's pose is recovered from one frame",
          angle_error < 1.5 and distance_error < 25.0,
          "%.2f deg, %.1f mm" % (angle_error, 1000.0 * distance_error))

    recovered = base_marker.camera_pose(warped, K, dist, at, yaw, size_m=size_m)
    camera_error = float(np.linalg.norm(recovered[:3, 3] - T_base_cam[:3, 3]))
    check("the composition gives the camera in the base frame", camera_error < 3.0,
          "%.1f mm" % (1000.0 * camera_error))

    try:
        base_marker.pose(np.full((1920, 1080), 128, dtype=np.uint8), K, dist, size_m=size_m)
        check("a blank frame is refused as having no marker", False, "no error raised")
    except base_marker.MarkerError as error:
        check("a blank frame is refused as having no marker", True, str(error)[:40])

    # A tape measure across the pattern inside the black border is not the side
    # PnP wants: 4 modules of payload sit inside 6 modules of marker.
    check("the inner pattern converts to the side the detector measures",
          abs(base_marker.outer_size(0.04) - 0.06) < 1e-9
          and abs(base_marker.outer_size(0.05 * 4.0 / 6.0) - 0.05) < 1e-9,
          "%.3f m" % base_marker.outer_size(0.04))

    # The PDF carries its own physical size: a 4 cm marker on a 70 x 82 mm page,
    # with the xref pointing where it says it does.
    pdf_path = Path("marker-sheet-test.pdf")
    info = base_marker.sheet_pdf(pdf_path, size_m)
    data = Path(pdf_path).read_bytes()
    start = data.rindex(b"startxref")
    xref_at = int(data[start + 9:].split()[0])
    table = data[xref_at:xref_at + 300].decode("ascii").split("\n")
    offsets = [int(line.split()[0]) for line in table[3:]
               if line.strip() and line.split()[0].isdigit()]
    check("the PDF sheet is the right size and a valid file",
          data.startswith(b"%PDF") and data.rstrip().endswith(b"%%EOF")
          and abs(info["marker_mm"] - 1000.0 * size_m) < 1e-6
          and abs(info["page_mm"][0] - (1000.0 * size_m + 30.0)) < 1e-6
          and all(data[offset:offset + 12].split()[1] == b"0" for offset in offsets),
          "marker %.1f mm on a %.0f x %.0f mm page"
          % (info["marker_mm"], info["page_mm"][0], info["page_mm"][1]))
    Path(pdf_path).unlink(missing_ok=True)

    # The vertical tag: pasted on the side of the base, facing the phone. Its
    # normal follows the heading and its printed +y is the base's +z.
    heading = np.radians(180.0)
    upright = base_marker.base_from_marker((0.14, 0.0, 0.06), heading, mode="vertical")
    check("a vertical tag faces its heading and stands upright",
          float(np.linalg.norm(upright[:3, 2] - np.array([-1.0, 0.0, 0.0]))) < 1e-9
          and float(np.linalg.norm(upright[:3, 1] - np.array([0.0, 0.0, 1.0]))) < 1e-9,
          "normal %s" % np.round(upright[:3, 2], 2))

    # A phone standing in front of the tag: the composition only needs a proper
    # `T_cam_tag`, which the PnP already supplies (and the flat case above tests
    # that machinery). What matters here is the *placement*: a vertical tag must
    # be a proper rotation whose normal follows the heading and whose +y is up.
    T_cam_tag = np.eye(4)
    T_cam_tag[:3, 3] = [0.02, -0.01, 1.3]
    composed = upright @ np.linalg.inv(T_cam_tag)
    check("the vertical tag composes a proper camera pose",
          abs(float(np.linalg.det(composed[:3, :3])) - 1.0) < 1e-9
          and float(composed[2, 3]) < 0.2,
          "det %.6f, z %.3f" % (float(np.linalg.det(composed[:3, :3])), float(composed[2, 3])))

    # The locator's own placement code has to agree with it, or the two paths
    # would disagree about where the tag is.
    placed = locate.tag_in_base(0.14, 0.0, 0.06, 180.0)
    check("the locator places the vertical tag the same way",
          float(np.max(np.abs(placed - upright))) < 1e-9, "max delta %.2e"
          % float(np.max(np.abs(placed - upright))))


def test_fixed_rig(workdir: Path):
    print("\nfixed camera: rig from the arm, no tag")
    K, dist, T_true, tip_true, observations = synthetic_rig()
    options = extrinsics.FitOptions(plane_z=0.0, image_size=(1600, 1200))
    result = extrinsics.fit(observations, K, dist, options,
                            calib_fingerprint="deadbeef", device="synthetic")
    rig = result.rig

    rotation_error = extrinsics.rotation_deg(rig.T_base_cam[:3, :3] @ T_true[:3, :3].T)
    position_error = float(np.linalg.norm(rig.T_base_cam[:3, 3] - T_true[:3, 3]))
    tip_error = float(np.linalg.norm(rig.tip_offset - tip_true))
    check("recovers the camera to 0.5 deg", rotation_error < 0.5, "%.3f deg" % rotation_error)
    check("recovers the camera to 10 mm", position_error < 0.010, "%.2f mm" % (position_error * 1000))
    check("recovers the tip to 5 mm", tip_error < 0.005, "%.2f mm" % (tip_error * 1000))
    check("the fit converged", result.converged and result.iterations < 60,
          "%d iterations" % result.iterations)
    predicted = result.predicted or {}
    check("predicts a click error under 5 mm at the corner",
          predicted.get("corner_mm") is not None and predicted["corner_mm"] < 5.0,
          "%.2f mm" % (predicted.get("corner_mm") or -1.0))
    check("names the weakest direction", bool(result.weakest.get("direction")))

    # One bad click must be discounted, not followed.
    spoiled = [extrinsics.Observation(q=o.q, uv=o.uv, capture=o.capture) for o in observations]
    spoiled[3] = extrinsics.Observation(q=spoiled[3].q, capture=spoiled[3].capture,
                                        uv=(spoiled[3].uv[0] + 25.0, spoiled[3].uv[1] - 18.0))
    robust = extrinsics.fit(spoiled, K, dist, options)
    robust_error = extrinsics.rotation_deg(
        robust.rig.T_base_cam[:3, :3] @ T_true[:3, :3].T)
    check("a 30 px mis-click is down-weighted", float(np.min(robust.weights)) < 0.3,
          "min weight %.2f" % float(np.min(robust.weights)))
    check("a mis-click does not wreck the fit", robust_error < 1.0, "%.3f deg" % robust_error)

    # Degenerate sweeps are refused, not fitted.
    same = [extrinsics.Observation(q=observations[0].q, uv=observations[0].uv) for _ in range(8)]
    try:
        extrinsics.fit(same, K, dist, options)
        check("refuses a repeated pose", False)
    except extrinsics.ExtrinsicsError:
        check("refuses a repeated pose", True)
    try:
        extrinsics.fit(observations[:4], K, dist, options)
        check("refuses too few poses", False)
    except extrinsics.ExtrinsicsError:
        check("refuses too few poses", True)

    # Splitting the joint zeros needs the extra poses for it.
    try:
        extrinsics.fit(observations, K, dist,
                       extrinsics.FitOptions(plane_z=0.0, refine_zeros=True,
                                             image_size=(1600, 1200)))
        check("refuses zero refinement with 8 poses", False)
    except extrinsics.ExtrinsicsError:
        check("refuses zero refinement with 8 poses", True)

    # A rig is bound to its lens: same aspect rescales, a crop does not.
    same_lens = calib_mod.Calib(path=workdir / "c.json", K=K, dist=dist,
                                image_size=(1600, 1200), device="synthetic")
    try:
        extrinsics.verify_pair(same_lens, rig)
        check("accepts the lens it was fitted with", True)
    except extrinsics.ExtrinsicsError as error:
        check("accepts the lens it was fitted with", False, str(error)[:40])
    cropped = calib_mod.Calib(path=workdir / "c.json", K=K, dist=dist,
                              image_size=(1600, 1000), device="synthetic")
    try:
        extrinsics.verify_pair(cropped, rig)
        check("refuses a different crop", False)
    except extrinsics.ExtrinsicsError:
        check("refuses a different crop", True)

    # No tag, no marker, just a tap: the fixed path has to work on a blank frame.
    image = np.full((1200, 1600, 3), 200, np.uint8)
    truth = np.array([0.30, 0.05, 0.0])
    tap = extrinsics.pixel(truth, K, dist, extrinsics.invert(T_true))
    calibration = calib_mod.Calib(path=workdir / "c.json", K=K, dist=dist,
                                  image_size=(1600, 1200), device="synthetic")
    payload = locate.locate(image, {"tap": {"u": tap[0], "v": tap[1]}}, calibration,
                            locate.Options(extrinsics=rig, plane_z=0.0))
    error = float(np.linalg.norm(np.array(payload["point"]) - truth))
    check("a tap plans with no tag in the frame", payload["source"] == "tap+fixed"
          and payload.get("tag") is None and payload.get("rig"), payload["source"])
    check("the fixed path lands within 3 mm", error < 0.003, "%.2f mm" % (error * 1000))
    check("the payload carries the rig provenance",
          payload["rig"].get("fingerprint") and payload["rig"].get("predicted_mm") is not None)

    try:
        locate.locate(image, {}, calibration, locate.Options(extrinsics=rig))
        check("refuses a tap-less capture", False)
    except locate.LocateError as error:
        check("refuses a tap-less capture", "no tap" in str(error), str(error)[:40])

    # The witness: a shifted frame is a moved camera.
    reference = witness.build(textured_frame())
    clean = witness.check(reference, reference.gray)
    check("an unmoved camera passes the witness", clean["ok"] and clean["drift_px"] < 0.5,
          "%.2f px" % clean["drift_px"])
    shift = np.float32([[1.0, 0.0, 7.0], [0.0, 1.0, -5.0]])
    moved = cv2.warpAffine(reference.gray, shift, (reference.gray.shape[1], reference.gray.shape[0]),
                           borderMode=cv2.BORDER_REFLECT)
    drift = witness.check(reference, moved)
    check("a 8.6 px shift is caught", not drift["ok"] and 8.0 < drift["drift_px"] < 9.2,
          "%.2f px" % drift["drift_px"])
    check("the witness reports its reason", bool(drift.get("reason")))

    # The rig file itself: write it from the direct fit, then audit it with the
    # command an operator would use.
    calib_path = workdir / "rig-calib.json"
    calib_path.write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")
    rig_path = workdir / "extrinsics-synthetic.json"
    extrinsics.save_rig(rig, rig_path)

    loaded = extrinsics.load_rig(rig_path)
    loaded_error = extrinsics.rotation_deg(loaded.T_base_cam[:3, :3] @ T_true[:3, :3].T)
    check("the rig file round-trips", len(loaded.observations) == 8
          and loaded.fingerprint, loaded.fingerprint)
    check("the rig file matches the direct fit", loaded_error < 0.5, "%.3f deg" % loaded_error)
    code = extrinsics_cli.main(["--check", "--calib", str(calib_path),
                                "--out", str(rig_path), "--json"])
    check("--check audits without failing", code == 0)

    # The server in fixed mode: a tap on a tagless frame must locate, and a
    # capture that does not match the witness must be refused.
    from aiohttp import FormData
    from aiohttp.test_utils import TestClient, TestServer

    import capture_server

    def server_args(captures, calib_dir, rig_file):
        return SimpleNamespace(
            host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
            token="selftest-token", mediamtx="http://127.0.0.1:9", whip_path="phone",
            no_whip=True, max_mb=4, quiet=True,
            planner="http://127.0.0.1:9", planner_token="", execute=False,
            calib_dir=str(calib_dir), calib="",
            marker_at="0.36,-0.10,0.12,180", marker_size=0.08,
            mode="", hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics=str(rig_file), witness_max_px=2.0, camera="",
        )

    async def fixed_server_checks():
        captures = workdir / "rig-server-captures"
        calib_dir = workdir / "rig-server-calib"
        calib_dir.mkdir(parents=True, exist_ok=True)
        (calib_dir / "synthetic.json").write_text(json.dumps({
            "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
            "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
        }), encoding="utf-8")

        ok, buffer = cv2.imencode(".jpg", np.full((1200, 1600, 3), 200, np.uint8),
                                  [cv2.IMWRITE_JPEG_QUALITY, 90])
        jpeg = buffer.tobytes()
        tap_pixel = extrinsics.pixel(truth, K, dist, extrinsics.invert(T_true))

        app = capture_server.build_app(server_args(captures, calib_dir, rig_path))
        async with TestClient(TestServer(app)) as client:
            health = await (await client.get("/health")).json()
            check("the server reports a fixed camera",
                  health.get("mode") == "fixed" and (health.get("rig") or {}).get("fingerprint"),
                  str(health.get("mode")))

            form = FormData()
            form.add_field("meta", json.dumps({
                "purpose": "plan", "image_size": [1600, 1200],
                "tap": {"u": tap_pixel[0], "v": tap_pixel[1]}}))
            form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
            response = await client.post("/capture", data=form,
                                         headers={"X-Rebot-Token": "selftest-token"})
            capture_id = (await response.json()).get("id", "")
            planned = await client.post("/plan", json={"capture": capture_id},
                                        headers={"X-Rebot-Token": "selftest-token"})
            answer = await planned.json()
            check("a tagless tap reaches the planner",
                  planned.status == 502 and answer.get("stage") == "plan",
                  "status %s, %s" % (planned.status, str(answer.get("error"))[:50]))
            target_path = captures / capture_id / "target.json"
            target = json.loads(target_path.read_text(encoding="utf-8")) if target_path.is_file() else {}
            check("the fixed path records a rig, not a tag",
                  target.get("source") == "tap+fixed" and target.get("rig")
                  and "tag" not in target)
            point = np.array(target.get("point", [0, 0, 0]))
            check("the server's target is within 3 mm",
                  float(np.linalg.norm(point - truth)) < 0.003,
                  "%.2f mm" % (1000 * float(np.linalg.norm(point - truth))))

            # A calibration capture records the arm pose (here: the gateway is
            # down, so it records the failure instead of failing the upload).
            form = FormData()
            form.add_field("meta", json.dumps({"purpose": "extrinsics",
                                               "image_size": [1600, 1200]}))
            form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
            response = await client.post("/capture", data=form,
                                         headers={"X-Rebot-Token": "selftest-token"})
            body = await response.json()
            meta = json.loads((captures / body["id"] / "meta.json").read_text(encoding="utf-8"))
            check("a calibration capture records a pose block",
                  response.status == 201 and "pose" in meta.get("server", {}),
                  str(meta.get("server", {}).get("pose"))[:60])

        # The witness gate: a rig with a reference frame must refuse a capture
        # that cannot be matched to it.
        rig_with_witness = extrinsics.load_rig(rig_path)
        reference = witness.build(textured_frame())
        witness.save(reference, workdir / "witness-test.jpg")
        rig_with_witness.witness = dict(reference.to_dict(), image="witness-test.jpg")
        watched = workdir / "extrinsics-watched.json"
        extrinsics.save_rig(rig_with_witness, watched)

        app = capture_server.build_app(server_args(workdir / "watched-captures", calib_dir, watched))
        async with TestClient(TestServer(app)) as client:
            form = FormData()
            form.add_field("meta", json.dumps({"purpose": "plan", "image_size": [1600, 1200],
                                               "tap": {"u": tap_pixel[0], "v": tap_pixel[1]}}))
            form.add_field("frame", jpeg, filename="frame.jpg", content_type="image/jpeg")
            response = await client.post("/capture", data=form,
                                         headers={"X-Rebot-Token": "selftest-token"})
            capture_id = (await response.json()).get("id", "")
            planned = await client.post("/plan", json={"capture": capture_id},
                                        headers={"X-Rebot-Token": "selftest-token"})
            answer = await planned.json()
            check("the mount gate refuses an unmatched frame",
                  planned.status == 422 and answer.get("stage") == "locate",
                  "status %s, %s" % (planned.status, str(answer.get("error"))[:60]))

    asyncio.run(fixed_server_checks())

    # A capture with no pose cannot seed a sweep, and says so.
    lonely = workdir / "no-pose"
    lonely.mkdir(exist_ok=True)
    cv2.imwrite(str(lonely / "frame.jpg"), np.full((1200, 1600, 3), 200, np.uint8),
                [cv2.IMWRITE_JPEG_QUALITY, 80])
    (lonely / "meta.json").write_text(json.dumps({"purpose": "extrinsics"}), encoding="utf-8")
    try:
        extrinsics_cli.main(["--auto", "--camera", "rtsp://fake", "--from", str(lonely),
                             "--uv", "10", "10"])
        check("refuses a capture with no pose", False)
    except SystemExit as error:
        check("refuses a capture with no pose", "no pose" in str(error))


# --------------------------------------------------------------------------- #
# 7. The automatic sweep: the arm moves itself, the camera is read, no clicks
# --------------------------------------------------------------------------- #

class FakeArm:
    """A stand-in for the gateway: a move is a round trip, like the real board.

    `shut_down` on the board ramps the arm back to the pose the move started
    from, so the fake does the same: it visits the target for the hold, then
    returns. The hold is compressed -- the sweep only needs it to last long
    enough to be read.
    """

    def __init__(self, q=None):
        self.q = np.asarray(q if q is not None else sweep.HOME, dtype=float).copy()
        self.moves = 0
        self.visited = []

    def pose(self):
        return {"ids": [1, 2, 3, 4, 5, 6, 7],
                "q": [float(v) for v in self.q] + [0.0],
                "timestamp": "2026-01-01T00:00:00Z"}

    def move(self, q6, seconds, hold=0.0):
        origin = self.q.copy()
        self.q = np.asarray(q6, dtype=float).copy()
        self.moves += 1
        self.visited.append(self.q.copy())
        try:
            # Long enough for the pose log to see a window at 15 Hz with margin
            # over MIN_HOLD_SECONDS; the real hold is 1.5 s.
            time.sleep(min(0.8, float(hold)) if hold else 0.0)
        finally:
            self.q = origin
        return {"ok": True, "pose": self.pose()}

    def move_sequence(self, targets, seconds, hold=0.0):
        """One file for the whole pass: visit each pose, then back to where we started."""
        origin = self.q.copy()
        self.moves += 1
        try:
            for target in targets:
                self.q = np.asarray(target, dtype=float).copy()
                self.visited.append(self.q.copy())
                time.sleep(min(0.8, float(hold)) if hold else 0.0)
        finally:
            self.q = origin
        return {"ok": True, "poses": len(targets)}


class FakeEye:
    """Renders the arm's capsule silhouette over a textured worktop.

    The same radii the detector's model uses, so this tests the pipeline
    (motion, capture, difference, tip search, fit) rather than the fidelity of
    the collision model. A real arm is not this clean; the verdict gate exists
    for that case.
    """

    def __init__(self, background, arm, T_base_cam, K, dist):
        self.background = background
        self.arm = arm
        self.T_base_cam = T_base_cam
        self.K = K
        self.dist = dist
        self.reads = 0

    def read(self, retries=1):
        self.reads += 1
        size = (self.background.shape[1], self.background.shape[0])
        # padding=0: the silhouette must be the projection of the capsule radii,
        # not the capsule plus a constant number of *pixels*. A pixel padding is
        # not the projection of a 3-D radius, and it would make the tool's
        # apparent size depend on range.
        mask = witness.arm_mask(size, SimpleNamespace(T_base_cam=self.T_base_cam),
                                self.arm.q, self.K, self.dist, padding=0)
        frame = self.background.copy()
        frame[mask == 0] = (48, 48, 48)
        return frame

    def close(self):
        pass


class FakeRecorder:
    """Stands in for MediaMTX: writes a real video while it is 'recording'.

    The video is a real mp4v file at 15 fps, written in real time from the same
    FakeEye the sweep looks at, so the time-to-frame mapping the extractor does is
    exercised for real. Only the control API calls are imagined.
    """

    def __init__(self, eye, directory, fps=15.0):
        self.eye = eye
        self.directory = Path(directory)
        self.fps = float(fps)
        self.path = None
        self.start = 0.0
        self.written = 0
        self._thread = None
        self._stop = threading.Event()

    def enable(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S-%f")
        self.path = self.directory / ("%s.mp4" % stamp)
        self.start = mediamtx.segment_start(self.path.name)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        writer = None
        started = time.time()
        try:
            while not self._stop.is_set():
                frame = self.eye.read()
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps,
                        (frame.shape[1], frame.shape[0]))
                writer.write(frame)
                self.written += 1
                delay = started + self.written / self.fps - time.time()
                if delay > 0:
                    time.sleep(delay)
        finally:
            if writer is not None:
                writer.release()

    def disable(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=15)

    def segments(self, since=None):
        if self.path is None or not self.path.is_file():
            return []
        if since is not None and self.start < float(since) - 2.0:
            return []
        return [mediamtx.Segment(path=self.path, start=self.start)]


def test_auto_sweep(workdir: Path):
    print("\nautomatic sweep: one tap, then the arm moves itself")
    K, dist, T_true, _tip, _observations = synthetic_rig()
    calibration = calib_mod.Calib(path=workdir / "auto-calib.json", K=K, dist=dist,
                                  image_size=(1600, 1200), device="synthetic")
    background = cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR)

    # The one tap: a still taken at a known pose, and the tool tip in it.
    q0 = np.array([0.4, 1.6, 0.9, 0.2, 1.1, 0.3])
    tap = extrinsics.pixel(rebot_ik.forward(q0)[:3, 3], K, dist, extrinsics.invert(T_true))

    arm = FakeArm()
    eye = FakeEye(background, arm, T_true, K, dist)
    options = auto_calibrate.Options(
        settle_s=0.0, plane_z=0.0, click={"q": q0.tolist(), "uv": tap},
        detect=auto_calibrate.DetectOptions(walk=False))

    fake = FakeRecorder(eye, workdir / "auto-recordings")
    result = auto_calibrate.run(arm, eye, calibration, options, workdir / "auto-captures",
                                recorder=fake)
    check("the sweep captured every pose", len(result.captures) == len(sweep.POSES) == 8)
    check("the sweep ends where the arm started", np.allclose(arm.q, sweep.HOME, atol=1e-9))
    check("the sweep is one trajectory, not one per pose", arm.moves == 1,
          "%d moves" % arm.moves)
    check("every pose was actually visited", len(arm.visited) == len(sweep.POSES)
          and all(np.allclose(a, b) for a, b in zip(arm.visited, sweep.POSES)))
    check("every capture has a frame and a pose",
          all(path.is_file() for path in result.captures))
    recording = result.captures[0].parent
    check("the sweep left a pose log and its choices",
          (recording / "poses.jsonl").is_file() and (recording / "poses.json").is_file()
          and all(path.is_file() for path in result.captures), str(recording.name))
    check("the media server recorded the pass",
          fake.written > 0 and fake.path is not None and fake.path.is_file(),
          "%d frames" % fake.written)
    chosen = json.loads((recording / "poses.json").read_text(encoding="utf-8"))
    check("the frames were chosen by when the pose was held",
          len(chosen) == len(sweep.POSES)
          and all(item["window_s"] >= sweep_video.MIN_HOLD_SECONDS for item in chosen)
          and all(item["offset_mrad"] <= 1000 * sweep_video.MAX_OFFSET_RAD
                  for item in chosen),
          "windows %s" % [item["window_s"] for item in chosen])
    check("one tap finds the tip in every pose",
          all(d.offset_px < options.detect.max_offset_px for d in result.detections),
          "worst %.0f px" % max(d.offset_px for d in result.detections))

    rig = result.fit.rig
    rotation_error = extrinsics.rotation_deg(rig.T_base_cam[:3, :3] @ T_true[:3, :3].T)
    position_error = float(np.linalg.norm(rig.T_base_cam[:3, 3] - T_true[:3, 3]))
    # A highlight beyond the tip used to win the detection: the contour was picked
    # by "furthest along the tool axis", so a small bright blob 80 px past the arm
    # beat the arm itself -- which is what a real sweep did on three poses.
    scene = cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR)
    pose = sweep.POSES[0]
    arm_mask = witness.arm_mask((scene.shape[1], scene.shape[0]),
                                SimpleNamespace(T_base_cam=T_true), pose, K, dist, padding=0)
    lit = scene.copy()
    lit[arm_mask == 0] = (48, 48, 48)
    wrist_px, tip_px, sixth_px = auto_calibrate._tool_pixels(pose, K, dist, T_true, _tip)
    along = np.asarray(tip_px, float) - np.asarray(sixth_px or wrist_px, float)
    along = along / np.linalg.norm(along)
    speck = np.asarray(tip_px, float) + along * 80.0
    cv2.rectangle(lit, (int(speck[0]) - 8, int(speck[1]) - 8),
                  (int(speck[0]) + 8, int(speck[1]) + 8), (250, 250, 250), -1)
    found = auto_calibrate.detect_tip(
        auto_calibrate._gray(lit), auto_calibrate._gray(scene), pose, K, dist, T_true, _tip,
        auto_calibrate.DetectOptions(walk=False), (lit.shape[1], lit.shape[0]))
    speck_error = float(np.hypot(found.u - tip_px[0], found.v - tip_px[1]))
    check("a bright speck beyond the tip does not win the detection",
          speck_error < 20.0, "%.1f px from the tip, area %.0f" % (speck_error, found.area))

    # The arm, not the speck: this is what makes the difference between a rig and
    # a tidy-looking nonsense one.
    check("the detection is the arm, not the speck", found.area > 4000.0,
          "area %.0f" % found.area)

    # The template detector: the tap's crop is the model, and a search that starts
    # in the wrong place still has to land on the gripper.
    templates, centre = auto_calibrate.tip_templates(
        lit, (float(tip_px[0]), float(tip_px[1])), auto_calibrate.DetectOptions())
    found = auto_calibrate.match_tip(
        auto_calibrate._gray(lit), templates, centre,
        (float(tip_px[0]) + 40.0, float(tip_px[1]) - 30.0),
        auto_calibrate.DetectOptions(), (lit.shape[1], lit.shape[0]))
    template_error = float("inf") if found is None else float(
        np.hypot(found[0] - tip_px[0], found[1] - tip_px[1]))
    check("the gripper template finds the tip from 50 px away",
          found is not None and template_error < 6.0 and found[2] > 0.8,
          "%.1f px, score %.2f" % (template_error, 0.0 if found is None else found[2]))

    check("the tap-seeded fit lands within 4 deg", rotation_error < 4.0,
          "%.3f deg" % rotation_error)
    check("the tap-seeded fit lands within 80 mm", position_error < 0.080,
          "%.1f mm" % (1000 * position_error))

    # A capsule silhouette is a crude stand-in for a gripper, and the detector is
    # the weak link: the point is the *gate*, not a flattering threshold. A rig
    # this rough must be refused rather than written.
    verdict = auto_calibrate.verify(rig, result.observations, calibration, options,
                                    result.fit.converged)
    check("silhouette-only detection is refused", not verdict["ok"],
          "; ".join(verdict["reasons"])[:60])

    rig_path = workdir / "extrinsics-auto.json"
    if rig_path.exists():
        rig_path.unlink()
    _rig, verdict, written = auto_calibrate.commit(
        result, calibration, options, rig_path, calib_path=str(calibration.path))
    check("a refused run writes no rig", written is None and not rig_path.exists())

    # The same sweep with a correspondence a real gripper edge would give: the
    # tip at the tool axis, no silhouette fuzz. That must verify and write.
    exact = []
    for observation in result.observations:
        T = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        point = T[:3, :3] @ np.array([0.0, 0.0, 0.035]) + T[:3, 3]
        exact.append(extrinsics.Observation(q=observation.q,
                                            uv=extrinsics.pixel(point, K, dist,
                                                                extrinsics.invert(T_true)),
                                            capture=observation.capture))
    exact_fit = extrinsics.fit(exact, K, dist,
                               extrinsics.FitOptions(image_size=(1600, 1200)))
    good = auto_calibrate.RunResult(
        observations=exact, captures=result.captures, detections=result.detections,
        bootstrap=result.bootstrap, fit=exact_fit, seconds=result.seconds,
        first_frame=result.first_frame, first_pose=result.first_pose)
    good_verdict = auto_calibrate.verify(good.fit.rig, good.observations, calibration, options,
                                         good.fit.converged)
    check("exact correspondences pass the same verification", good_verdict["ok"],
          "; ".join(good_verdict["reasons"]) or "rms %.2f px" % good_verdict["rms_px"])
    _rig, _verdict, written = auto_calibrate.commit(
        good, calibration, options, rig_path, calib_path=str(calibration.path))
    check("a verified run writes the rig and a witness",
          written is not None and Path(written).is_file()
          and (Path(written).parent / "witness-fixed.jpg").is_file())
    if written is not None:
        loaded = extrinsics.load_rig(rig_path)
        check("the written rig is usable by a tap",
              loaded.witness is not None and len(loaded.observations) == 8)
    check("the rig records where the camera is",
          np.allclose(exact_fit.rig.provenance()["camera_at"],
                      exact_fit.rig.T_base_cam[:3, 3], atol=1e-3))

    # The tap is the only way in, and it is checked rather than assumed.
    try:
        auto_calibrate.bootstrap(auto_calibrate.Options())
        check("a sweep with no tap is refused", False)
    except auto_calibrate.AutoError:
        check("a sweep with no tap is refused", True)

    # One still + its pose is the first half, and it resolves into a seed.
    import camera as camera_mod

    saved_state = extrinsics_cli.STATE
    original_gateway = auto_calibrate.ArmGateway
    original_source = camera_mod.open_source
    extrinsics_cli.STATE = workdir / "cli-state"
    auto_calibrate.ArmGateway = lambda *a, **k: arm
    camera_mod.open_source = lambda *a, **k: eye
    try:
        code = extrinsics_cli.main(["--capture-pose", "--camera", "rtsp://fake", "--quiet"])
    finally:
        extrinsics_cli.STATE = saved_state
        auto_calibrate.ArmGateway = original_gateway
        camera_mod.open_source = original_source
    stills = sorted((workdir / "cli-state" / "captures").glob("*/frame.jpg"))
    check("--capture-pose writes a still and its pose", code == 0 and len(stills) == 1)
    if stills:
        meta = json.loads((stills[0].parent / "meta.json").read_text(encoding="utf-8"))
        check("the still carries the pose the sweep needs",
              meta.get("purpose") == "extrinsics"
              and (meta.get("server") or {}).get("pose", {}).get("q"))
        q, _block = extrinsics_cli.joints_from_meta(meta)
        check("that still resolves into a tap seed", q is not None and len(q) == 6)

    # The whole sweep, the return to park included, follows one speed policy.
    legs = sweep.sequence()
    peaks = [(math.pi / 2.0) * float(np.max(np.abs(b - a))) / sweep.seconds_for(a, b)
             for a, b in zip(legs, legs[1:]) if float(np.max(np.abs(b - a))) > 0.0]
    check("no leg, the return included, outruns the speed cap",
          max(peaks) <= sweep.PEAK_SPEED + 1e-9,
          "worst %.2f rad/s, cap %.2f" % (max(peaks), sweep.PEAK_SPEED))
    check("the return to park takes at least the minimum time",
          sweep.seconds_for(legs[-2], legs[-1]) >= sweep.MIN_SECONDS,
          "%.1f s" % sweep.seconds_for(legs[-2], legs[-1]))

    # The selection is the part that replaces timing: a pose counts only if the
    # log shows a *run* of frames at it, and one frame is passing through.
    entries = [{"i": i, "t": i / 15.0, "q": list(sweep.HOME)} for i in range(40)]
    entries[20]["q"] = list(sweep.POSES[0])
    passed_through = sweep_video.select(entries, [sweep.POSES[0]],
                                        min_seconds=sweep_video.MIN_HOLD_SECONDS)
    check("a pose the arm only passed through is not used",
          passed_through[0].get("missed") is True, str(passed_through[0]))

    # The loaded arm settles tens of mrad below what it was told; the hold is
    # still a hold, and the pose it held is the one logged.
    settled = np.asarray(sweep.POSES[0], dtype=float) + 0.05
    for offset in range(16, 25):                    # nine samples: 0.53 s
        entries[offset]["q"] = [float(v) for v in settled]
    held = sweep_video.select(entries, [sweep.POSES[0]],
                              min_seconds=sweep_video.MIN_HOLD_SECONDS)
    check("a hold that stops short of the command is still found",
          held[0].get("missed") is not True
          and abs(held[0]["t"] - 20 / 15.0) < 1e-6
          and 40.0 <= held[0]["offset_mrad"] <= 60.0, str(held[0]))
    check("the pose the arm actually held is the one recorded",
          np.allclose(np.asarray(held[0]["q"], dtype=float), settled, atol=1e-9))

    # MediaMTX writes the segment's start into its name, in the container's
    # clock, which is UTC. Reading it as the host's local time would put every
    # recording hours away from the pose log.
    stamp = "2026-11-13_10-17-14-447322"
    check("a segment name is read as UTC",
          mediamtx.segment_start(stamp + ".mp4")
          == datetime(2026, 11, 13, 10, 17, 14, 447322, tzinfo=timezone.utc).timestamp())

    # The gateway's accept test is what a sweep hits first: it commands HOME.
    check("the park pose is a legal command, not a margin violation",
          sweep.command_ok(sweep.HOME)
          and all(sweep.command_ok(pose) for pose in sweep.POSES),
          "home %s" % (list(sweep.HOME),))

    # Gravity sag: the controller has no integrator, so a joint rests tau/kp away
    # from its command. The command is raised by that amount, and the fixed point
    # is what proves the raised pose is the one the arm will hold.
    check("the gain spec parses to one kp per joint",
          list(sweep.kp_vector("50,2:150,3:150")) == [50.0, 150.0, 150.0, 50.0, 50.0, 50.0],
          str(list(sweep.kp_vector())))
    kp = sweep.kp_vector()
    aimed = [sweep.aim(pose, kp) for pose in sweep.POSES]
    # This arm settles as held = command - sag; aiming must invert exactly that.
    residual = max(float(np.max(np.abs(command - sweep.gravity_sag(command, kp) - pose)))
                   for command, pose in zip(aimed, sweep.POSES))
    check("aiming cancels the modelled sag", residual < 0.001,
          "%.3f mrad left" % (1000.0 * residual))
    check("the sag compensation stays inside its bound",
          all(float(np.max(np.abs(command - pose))) <= sweep.SAG_LIMIT_RAD
              for command, pose in zip(aimed, sweep.POSES)),
          "worst %.0f mrad" % (1000.0 * max(
              float(np.max(np.abs(c - p))) for c, p in zip(aimed, sweep.POSES))))

    # A synthetic arm that settles the way the real one does, 10% worse than the
    # model: held = command - 1.1 * sag. The compensation must cut the error by an
    # order of magnitude, not double it -- the sign error that this pins was real.
    def settle(command):
        command = np.asarray(command, dtype=float)
        return command - 1.1 * sweep.gravity_sag(command, kp)

    before = max(float(np.max(np.abs(settle(pose) - pose))) for pose in sweep.POSES)
    after = max(float(np.max(np.abs(settle(sweep.aim(pose, kp)) - pose)))
                for pose in sweep.POSES)
    check("compensating cuts the settle error by an order of magnitude",
          after < before / 8.0 and after < 0.02,
          "%.0f mrad -> %.0f mrad" % (1000.0 * before, 1000.0 * after))

    # The sequence itself, re-checked against the model it was generated from.
    check("the fixed sweep is still safe", sweep.check() == [], "; ".join(sweep.check())[:60])
    check("the sweep starts and ends at the parked pose",
          np.allclose(sweep.sequence()[0], sweep.HOME)
          and np.allclose(sweep.sequence()[-1], sweep.HOME))


# --------------------------------------------------------------------------- #

def test_calibrate_api(workdir: Path):
    print("\ncalibration API: POST /calibrate/extrinsics")
    from aiohttp.test_utils import TestClient, TestServer

    import camera as camera_mod
    import capture_server

    K, dist, T_true, _tip, _observations = synthetic_rig()
    captures = workdir / "api-captures"
    calib_dir = workdir / "api-calib"
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")

    background = cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR)
    arm = FakeArm()
    eye = FakeEye(background, arm, T_true, K, dist)

    def args_for(execute, marker_at=""):
        return SimpleNamespace(
            host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
            token="", mediamtx="http://127.0.0.1:9", whip_path="phone", no_whip=True,
            max_mb=4, quiet=True, planner="http://127.0.0.1:9", planner_token="",
            execute=execute, calib_dir=str(calib_dir), calib="",
            marker_at=marker_at, marker_size=0.08,
            mode="", hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics="", witness_max_px=2.0, camera="rtsp://fake/stream",
            mediamtx_api="http://127.0.0.1:9", recordings=str(calib_dir.parent / "recordings"),
        )

    # The arm gateway and the camera are the two things a test cannot own; both
    # are swapped for the fakes the rest of this section uses. The recorder is a
    # real video writer standing in for MediaMTX's recording.
    original_gateway = auto_calibrate.ArmGateway
    original_source = camera_mod.open_source
    original_detect = auto_calibrate.detect_all
    original_recorder = mediamtx.Recorder
    auto_calibrate.ArmGateway = lambda *a, **k: arm
    camera_mod.open_source = lambda *a, **k: eye
    recorder_dir = calib_dir.parent / "recordings"
    mediamtx.Recorder = lambda *a, **k: FakeRecorder(eye, recorder_dir)
    try:
        def exact_detect(frames, poses, K_, dist_, T_base_cam, t_tip, options, size,
                         **kwargs):
            found = []
            for index, q in enumerate(poses):
                T = rebot_ik.forward(np.asarray(q, dtype=float))
                point = T[:3, :3] @ np.array([0.0, 0.0, 0.035]) + T[:3, 3]
                u, v = extrinsics.pixel(point, K_, dist_, extrinsics.invert(T_true))
                found.append(auto_calibrate.Detection(u=u, v=v, offset_px=0.0, area=0.0,
                                                      pose=index + 1))
            return found

        async def scenarios():
            off = capture_server.build_app(args_for(False))
            async with TestClient(TestServer(off)) as client:
                answer = await client.post("/calibrate/extrinsics", json={"confirm": True})
                check("the API refuses to move the arm with execution off",
                      answer.status == 403)

            app = capture_server.build_app(args_for(True, marker_at="0.36,-0.10,0.12,180"))
            async with TestClient(TestServer(app)) as client:
                answer = await client.post("/calibrate/extrinsics", json={})
                check("the API needs an explicit confirm", answer.status == 400)

                # An unhelpful 422 became the panel's "server said 422": the
                # reason has to survive as JSON, whatever raised it.
                empty = capture_server.build_app(args_for(True))
                empty["calib"] = workdir / "empty-calib"
                empty["calib"].mkdir(parents=True, exist_ok=True)
                empty["calib_file"] = ""
                async with TestClient(TestServer(empty)) as bare:
                    # With neither a tap nor a base marker there is no way in.
                    answer = await bare.post("/calibrate/extrinsics", json={"confirm": True})
                    check("a sweep with no tap and no marker is refused", answer.status == 400,
                          (await answer.text())[:50])
                    answer = await bare.post("/calibrate/extrinsics", json={
                        "confirm": True, "capture": "20260101T120000Z-abc123",
                        "u": 10.0, "v": 10.0})
                    body = await answer.json()
                    check("a missing calibration explains itself in JSON",
                          answer.status == 422
                          and "no calibration" in (body.get("error") or ""),
                          "%s %s" % (answer.status, (body.get("error") or "")[:40]))

                # The panel asks the capture server for the pose, not the gateway.
                # That proxy has to use GET: the gateway's /pose is a GET route,
                # and a POST there answers 404, which reads as "not reachable".
                from aiohttp import web as aiohttp_web

                async def fake_pose(request):
                    if request.method != "GET":
                        return aiohttp_web.json_response(
                            {"ok": False, "error": "no such route"}, status=404)
                    return aiohttp_web.json_response(
                        {"ok": True, "pose": {"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0] * 7}})

                fake = aiohttp_web.Application()
                fake.router.add_route("*", "/pose", fake_pose)
                runner = aiohttp_web.AppRunner(fake)
                await runner.setup()
                site = aiohttp_web.TCPSite(runner, "127.0.0.1", 0)
                await site.start()
                port = site._server.sockets[0].getsockname()[1]
                saved_planner = app["planner"]
                app["planner"] = "http://127.0.0.1:%d" % port
                try:
                    proxied = await client.get("/pose")
                    body = await proxied.json()
                    check("the pose proxy asks with GET, as the gateway requires",
                          proxied.status == 200 and (body.get("pose") or {}).get("q"),
                          "status %s, %s" % (proxied.status, str(body.get("error"))[:40]))
                finally:
                    app["planner"] = saved_planner
                    await runner.cleanup()

                started = arm.moves


                # The tap: one still at a chosen pose, and the tool tip in it.
                tap_pose = np.array([0.4, 1.6, 0.9, 0.2, 1.1, 0.3])
                arm.q = tap_pose.copy()
                answer = await client.post("/capture/pose", json={})
                shot = await answer.json()
                check("the API grabs a still with its pose",
                      answer.status == 200 and shot.get("id")
                      and (shot.get("pose") or {}).get("q"), str(answer.status))
                u, v = extrinsics.pixel(rebot_ik.forward(tap_pose)[:3, 3], K, dist,
                                        extrinsics.invert(T_true))
                seed = capture_server.click_for(app, shot["id"], u, v)
                check("a capture resolves into a tap seed for the sweep",
                      len(seed["q"]) == 6 and seed["uv"] == (float(u), float(v)))
                # The fake holds for a fraction of a second; poll faster than that
                # so the test does not depend on the real 3.5 s window.
                tap_body = {"confirm": True, "capture": shot["id"],
                            "u": float(u), "v": float(v), "settle": 0.02}

                # First with the silhouette detector: a bootstrap, not a rig.
                answer = await client.post("/calibrate/extrinsics", json=tap_body)
                body = await answer.json()
                check("the API runs the sweep and reports a refusal",
                      answer.status == 422 and body.get("ok") is False
                      and not (calib_dir / "extrinsics-fixed.json").exists(),
                      "status %s, %d moves" % (answer.status, arm.moves - started))
                check("the refusal carries the per-pose detections",
                      len(body.get("detections") or []) == 8 and body.get("verification"))
                check("the swept arm is back where it started",
                      np.allclose(arm.q, tap_pose, atol=1e-6))

                status = await (await client.get("/calibrate")).json()
                check("GET /calibrate reports the last run",
                      status.get("running") is False and (status.get("last") or {}).get("state"),
                      str((status.get("last") or {}).get("state")))
                check("GET /calibrate carries the sweep shape and idle progress",
                      (status.get("sweep") or {}).get("poses") == len(sweep.POSES)
                      and status.get("progress") is None)
                check("GET /calibrate estimates the sweep's motion time",
                      (status.get("sweep") or {}).get("seconds", 0) > 60,
                      "%s s" % (status.get("sweep") or {}).get("seconds"))

                # Now with a correspondence a real gripper edge would give.
                auto_calibrate.detect_all = exact_detect
                auto_calibrate.detect_all_green = exact_detect
                answer = await client.post("/calibrate/extrinsics", json=tap_body)
                body = await answer.json()
                check("a good correspondence writes the rig",
                      answer.status == 200 and body.get("ok") is True
                      and (calib_dir / "extrinsics-fixed.json").is_file(),
                      "status %s, %s" % (answer.status, str(body.get("error"))[:50]))
                if answer.status == 200:
                    loaded = extrinsics.load_rig(calib_dir / "extrinsics-fixed.json")
                    check("the API rig has a witness and 8 poses",
                          loaded.witness is not None and len(loaded.observations) == 8)
                    check("the API reports the verification numbers",
                          body["verification"].get("rms_px") is not None
                          and body["verification"].get("predicted_mm"))

                status = await (await client.get("/calibrate")).json()
                check("GET /calibrate reports the success",
                      (status.get("last") or {}).get("state") == "ok"
                      and (status.get("rig") or {}).get("poses") == 8)
                check("the status carries the camera position for the next run",
                      len((status.get("rig") or {}).get("camera_at") or []) == 3,
                      str((status.get("rig") or {}).get("camera_at")))

        asyncio.run(scenarios())
    finally:
        auto_calibrate.ArmGateway = original_gateway
        camera_mod.open_source = original_source
        auto_calibrate.detect_all = original_detect
        mediamtx.Recorder = original_recorder


# --------------------------------------------------------------------------- #

def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline selftest for the capture pipeline.")
    parser.add_argument("--keep", action="store_true", help="keep the work directory")
    parser.add_argument("--workdir", default=str(HERE / ".selftest-work"),
                        help="scratch directory (default: .selftest-work next to this file)")
    parser.add_argument("--skip-server", action="store_true")
    args = parser.parse_args(argv)

    print("cv2 %s, numpy %s" % (cv2.__version__, np.__version__))

    workdir = Path(args.workdir)
    if workdir.exists() and not args.keep:
        import shutil
        shutil.rmtree(workdir, ignore_errors=True)
    workdir.mkdir(parents=True, exist_ok=True)

    calibration = test_calibration(workdir)
    geometry = test_tag_geometry(workdir)
    test_schema()
    if not args.skip_server:
        asyncio.run(test_server(workdir, geometry))
    if calibration:
        test_locate_cli(workdir, geometry)
    test_fixed_rig(workdir)
    test_base_marker()
    test_auto_sweep(workdir)
    test_calibrate_api(workdir)

    failures = [name for name, ok, _ in RESULTS if not ok]
    print("\n%d checks, %d failed" % (len(RESULTS), len(failures)))
    for name in failures:
        print("  failed: %s" % name)
    if not failures and not args.keep:
        import shutil
        shutil.rmtree(workdir, ignore_errors=True)
    else:
        print("work directory kept at %s" % workdir)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
