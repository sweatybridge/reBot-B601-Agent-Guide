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
import identify  # noqa: E402
import locate_target as locate  # noqa: E402
import rebot_ik  # noqa: E402
import mediamtx  # noqa: E402
import oracle  # noqa: E402
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

    # Velocity feedforward: build derives v for every producer, so the MIT
    # frame's kd term damps toward the motion instead of braking it.
    check("build derives v on every waypoint",
          all(len(point.get("v") or []) == 7 for point in payload["waypoints"]))
    move_wps = move["waypoints"]
    hold_count = int(round(0.4 * 30))
    check("a joint move holds at zero velocity",
          all(v == 0.0 for point in move_wps[-hold_count:] for v in point["v"]))
    # Cosine ease peak speed: travel * pi / (2T); travel 0.4 rad over 1 s.
    expect_peak = 0.4 * math.pi / 2.0
    check("a joint move's peak v matches the cosine ease",
          abs(move_stats["peak_v"] - expect_peak) < 0.02 * expect_peak,
          "%.4f vs %.4f rad/s" % (move_stats["peak_v"], expect_peak))
    check("a joint move starts near rest",
          abs(move_wps[0]["v"][2]) < 0.1 * move_stats["peak_v"])
    seq_holds = [point for point in sweep_motion["waypoints"]
                 if all(q == 0.2 for q in point["q"][:6])]
    check("a sequence holds at zero velocity",
          seq_holds and all(v == 0.0 for point in seq_holds for v in point["v"]))

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
    rejects("rejects v on only some waypoints", lambda p: p["waypoints"][2].pop("v"))
    rejects("rejects a short v", lambda p: p["waypoints"][1].update(v=[0.0] * 6))
    rejects("rejects a non-finite v",
            lambda p: p["waypoints"][1]["v"].__setitem__(0, float("nan")))
    rejects("rejects a v above the step gate",
            lambda p: p["waypoints"][1]["v"].__setitem__(0, 100.0))
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
        hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="",
        traces_dir=str(workdir / "server-traces"),
    )
    app = capture_server.build_app(args)

    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    jpeg = buffer.tobytes()

    async with TestClient(TestServer(app)) as client:
        health = await (await client.get("/health")).json()
        check("health is ok", health.get("ok") is True and health.get("auth") is True)
        check("health reports the camera mode", health.get("mode") == "handheld"
              and health.get("rig") is None, str(health.get("mode")))
        check("health reports the default hover height",
              abs(float(health.get("hover_m", -1)) - 0.10) < 1e-9,
              str(health.get("hover_m")))
        check("health reports the flange minimum and tip offset",
              abs(float(health.get("min_z", -1)) - 0.10) < 1e-9
              and abs(float(health.get("tip_m", -1)) - 0.035) < 1e-9,
              "min_z %s, tip_m %s" % (health.get("min_z"), health.get("tip_m")))
        raised = SimpleNamespace(**vars(args))
        raised.plane_z = 0.15
        raised_app = capture_server.build_app(raised)
        check("the flange minimum rises with the worktop plane (plane_z + hover_m)",
              abs(float(raised_app["min_z"]) - 0.25) < 1e-9,
              "min_z %s" % raised_app["min_z"])
        check("serves the phone page", (await client.get("/")).status == 200)

        reach = await (await client.get("/reach.json")).json()
        check("reach.json gives the overlay its camera",
              reach.get("ok") is True and reach.get("mode") == "handheld"
              and reach.get("T_base_cam") is None
              and len((reach.get("calibration") or {}).get("K") or []) == 3,
              str((reach.get("calibration") or {}).get("image_size")))
        check("reach.json gives the browser the tag placement",
              (reach.get("marker") or {}).get("id") == 0
              and abs((reach.get("marker") or {}).get("size_m", 0) - 0.08) < 1e-9)
        check("serves the browser ArUco module", (await client.get("/aruco.js")).status == 200)
        check("serves the HTTP helpers", (await client.get("/api.js")).status == 200)
        check("serves the video helpers", (await client.get("/video.js")).status == 200)
        check("serves the reach config loader", (await client.get("/reach-config.js")).status == 200)
        check("serves the shared geometry", (await client.get("/geometry.js")).status == 200)
        check("serves the arm model", (await client.get("/arm-model.js")).status == 200)
        check("serves the marker module", (await client.get("/markers.js")).status == 200)
        check("serves the height control", (await client.get("/height.js")).status == 200)
        check("serves the browser locator", (await client.get("/locate.js")).status == 200)
        check("serves the pose EKF", (await client.get("/ekf.js")).status == 200)
        check("serves the agent stream parser", (await client.get("/stream.js")).status == 200)

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

        # Compact tip targets: no capture/locate, no rebot-target envelope.
        compact = [
            {"tip": [0.5, 0.0, 0.16], "pixel": [640, 480]},
            {"tip": [0.55, 0.0, 0.25]},
        ]
        sequenced = await client.post("/sequence", json={"targets": compact},
                                      headers={"X-Rebot-Token": "selftest-token"})
        sequence_body = await sequenced.json()
        check("sequence reaches the planner",
              sequenced.status == 502 and sequence_body.get("stage") == "plan",
              "status %s, %s" % (sequenced.status, str(sequence_body.get("error"))[:50]))
        bad_sequence = await client.post("/sequence", json={"targets": [{"format": "nope"}]},
                                         headers={"X-Rebot-Token": "selftest-token"})
        check("sequence rejects a target without a tip", bad_sequence.status == 400)

        # The phone's tip points are already arm-space; the host adds the one
        # physical offset and the safety floor, then posts the compact request.
        forwarded = []

        async def fake_planner_call(app, path, body, timeout=120, method="POST", sid=None):
            forwarded.append((path, body))
            return 200, {"id": "fake-sequence",
                         "summary": {"markers": len(body.get("targets") or [])},
                         "execute": False}

        real_planner_call = capture_server.planner_call
        capture_server.planner_call = fake_planner_call
        try:
            forwarded_response = await client.post(
                "/sequence", json={"targets": compact},
                headers={"X-Rebot-Token": "selftest-token"})
            forwarded_body = await forwarded_response.json()
        finally:
            capture_server.planner_call = real_planner_call
        forwarded_ok = False
        forwarded_note = "no planner call"
        if forwarded:
            path, request = forwarded[-1]
            sent = request.get("targets") or []
            options = request.get("options") or {}
            first, second = (sent + [{}, {}])[:2]
            first = first or {}
            second = second or {}
            first_ok = (first.get("tip") == [0.5, 0.0, 0.16]
                        and first.get("pixel") == [640, 480]
                        and "format" not in first and "point_frame" not in first)
            second_ok = second.get("tip") == [0.55, 0.0, 0.25]
            options_ok = (abs(float(options.get("min_z", -1)) - 0.10) < 1e-9
                          and "hover_m" not in options and "tip_m" not in options)
            forwarded_ok = (path == "/plan" and forwarded_response.status == 200
                            and forwarded_body.get("ok") is True
                            and request.get("gripper") == "alternate"
                            and abs(float(request.get("tip_offset_m", -1)) - 0.035) < 1e-9
                            and first_ok and second_ok and options_ok)
            forwarded_note = ("path %s, gripper %s, offset %s, options %s"
                              % (path, request.get("gripper"),
                                 request.get("tip_offset_m"), options))
        check("sequence forwards compact tip targets", forwarded_ok, forwarded_note)

        deprecated_hover = await client.post(
            "/sequence", json={"targets": compact, "hover_m": 0.1},
            headers={"X-Rebot-Token": "selftest-token"})
        check("sequence rejects the deprecated hover_m", deprecated_hover.status == 400)

        bad_tip = await client.post(
            "/sequence", json={"targets": [{"tip": [0.5, 0.0, "high"]}]},
            headers={"X-Rebot-Token": "selftest-token"})
        check("sequence rejects a non-numeric tip", bad_tip.status == 400)

        rejected_low = await client.post(
            "/sequence", json={"targets": [{"tip": [0.5, 0.0, 0.0]}]},
            headers={"X-Rebot-Token": "selftest-token"})
        check("sequence rejects a tip below the minimum",
              rejected_low.status == 422,
              str((await rejected_low.json()).get("error"))[:70])

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

        forwarded.clear()
        capture_server.planner_call = fake_planner_call
        try:
            single = await client.post("/plan",
                                       json={"capture": body["id"], "hover_m": 0.20},
                                       headers={"X-Rebot-Token": "selftest-token"})
            single_body = await single.json()
        finally:
            capture_server.planner_call = real_planner_call
        single_ok = False
        single_note = "no planner call"
        if forwarded:
            path, request = forwarded[-1]
            sent = (request.get("targets") or [{}])[0] or {}
            options = request.get("options") or {}
            tip = sent.get("tip") or [0, 0, 0]
            single_ok = (path == "/plan" and single.status == 200
                         and single_body.get("ok") is True
                         and request.get("gripper") == "none"
                         and abs(float(request.get("tip_offset_m", -1)) - 0.035) < 1e-9
                         and abs(float(tip[2]) - 0.20) < 1e-9
                         and abs(float(options.get("min_z", -1)) - 0.10) < 1e-9
                         and "hover_m" not in options and "tip_m" not in options)
            single_note = "tip z %.3f, gripper %s, offset %s" % (
                tip[2], request.get("gripper"), request.get("tip_offset_m"))
        check("single tap becomes a one-target /plan", single_ok, single_note)

        low_single = await client.post("/plan",
                                       json={"capture": body["id"], "hover_m": 0.0},
                                       headers={"X-Rebot-Token": "selftest-token"})
        check("single tap rejects a tip below the minimum", low_single.status == 422)

        check("execution is off unless asked for",
              (await client.post("/execute", json={"id": "x"},
                                 headers={"X-Rebot-Token": "selftest-token"})).status == 403)

        # The agent sidebar: /agent is only a transport. The endpoint, the model
        # and the key are the host's, so no page can change them -- and a page
        # that asks for tools gets exactly the tools it asked for, no more.
        from aiohttp import web as aiohttp_web

        seen = []

        async def fake_chat(request):
            payload = await request.json()
            seen.append({"body": payload,
                         "auth": request.headers.get("Authorization")})
            truncated = any(
                isinstance(message, dict) and message.get("content") == "truncated"
                for message in (payload.get("messages") or []))
            if payload.get("stream"):
                response = aiohttp_web.StreamResponse(
                    status=200, headers={"Content-Type": "text/event-stream"})
                await response.prepare(request)
                chunks = [
                    {"choices": [{"delta": {"reasoning_content": "checking "}}]},
                    {"choices": [{"delta": {"content": "looking "}}]},
                    {"choices": [{"delta": {
                        "tool_calls": [{
                            "index": 0, "id": "call_7", "type": "function",
                            "function": {"name": "rig",
                                         "arguments": '{"u":620' if truncated else "{}"},
                        }],
                    }}]},
                    {"choices": [{"finish_reason": "length" if truncated else "tool_calls",
                                  "delta": {}}]},
                ]
                for chunk in chunks:
                    await response.write(b"data: " + json.dumps(chunk).encode() + b"\n\n")
                await response.write(b"data: [DONE]\n\n")
                await response.write_eof()
                return response
            return aiohttp_web.json_response({
                "model": "stub-model",
                "choices": [{"message": {
                    "role": "assistant", "content": "looking now",
                    "reasoning_content": "checking the pixels first",
                    "tool_calls": [{"function": {"name": "rig", "arguments": "{}"}}],
                }}],
                "usage": {"total_tokens": 7},
            })

        fake_llm = aiohttp_web.Application()
        fake_llm.router.add_post("/v1/chat/completions", fake_chat)

        async def fake_models(request):
            return aiohttp_web.json_response({
                "data": [{
                    "id": "Qwen3.8-27B",
                    "context_length": 262144,
                    "meta": {"n_ctx": 8192},
                }],
            })

        fake_llm.router.add_get("/v1/models", fake_models)
        runner = aiohttp_web.AppRunner(fake_llm)
        await runner.setup()
        site = aiohttp_web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        llm_port = site._server.sockets[0].getsockname()[1]
        saved_llm = {key: app[key] for key in
                     ("llm_url", "llm_model", "llm_token", "llm_temperature",
                      "llm_max_tokens", "llm_timeout")}
        app["llm_url"] = "http://127.0.0.1:%d/v1/chat/completions" % llm_port
        app["llm_model"] = "Qwen3.8-27B"
        app["llm_token"] = "stub-key"
        app["llm_temperature"] = 0.0
        app["llm_max_tokens"] = 64
        try:
            agent_health = await (await client.get("/health")).json()
            check("health names the agent model and says it is on",
                  (agent_health.get("agent") or {}).get("configured") is True
                  and (agent_health.get("agent") or {}).get("model") == "Qwen3.8-27B",
                  str(agent_health.get("agent")))
            check("health discovers the model's configured context",
                  (agent_health.get("agent") or {}).get("context_tokens") == 8192
                  and (agent_health.get("agent") or {}).get("context_source") == "meta.n_ctx",
                  str(agent_health.get("agent")))

            tools = [{"type": "function", "function": {"name": "rig",
                                                       "parameters": {"type": "object"}}}]
            turn = await client.post("/agent",
                                     json={"messages": [{"role": "user", "content": "hi"}],
                                           "tools": tools},
                                     headers={"X-Rebot-Token": "selftest-token"})
            turn_body = await turn.json()
            message = turn_body.get("message") or {}
            check("agent forwards a turn and returns the reply",
                  turn.status == 200 and turn_body.get("ok") is True
                  and message.get("content") == "looking now"
                  and len(message.get("tool_calls") or []) == 1,
                  "status %s, %s" % (turn.status, str(turn_body.get("error"))[:60]))
            check("agent forwards the model's reasoning separately",
                  message.get("reasoning") == "checking the pixels first",
                  str(message.get("reasoning"))[:60])
            sent = (seen[-1]["body"] if seen else {})
            check("agent sends the configured model, not the page's",
                  sent.get("model") == "Qwen3.8-27B"
                  and sent.get("stream") is False
                  and sent.get("temperature") == 0.0
                  and sent.get("max_tokens") == 64,
                  str({k: sent.get(k) for k in ("model", "stream", "temperature", "max_tokens")}))
            check("agent passes the page's tools through, with tool_choice",
                  (sent.get("tools") or [{}])[0].get("function", {}).get("name") == "rig"
                  and sent.get("tool_choice") == "auto")
            check("agent sends the host's key, which the page never sees",
                  (seen[-1]["auth"] if seen else "") == "Bearer stub-key")

            streamed = await client.post("/agent",
                                         json={"messages": [{"role": "user", "content": "hi"}],
                                               "tools": tools, "stream": True},
                                         headers={"X-Rebot-Token": "selftest-token"})
            stream_text = await streamed.text()
            check("agent streams SSE when asked",
                  streamed.status == 200
                  and "text/event-stream" in (streamed.headers.get("Content-Type") or "")
                  and "event: reasoning" in stream_text
                  and "event: content" in stream_text
                  and "event: done" in stream_text,
                  "status %s" % streamed.status)
            data_lines = [line[6:] for line in stream_text.splitlines()
                          if line.startswith("data: ")]
            done = json.loads(data_lines[-1]) if data_lines else {}
            check("agent assembles the streamed turn and its tool call",
                  (done.get("message") or {}).get("reasoning") == "checking "
                  and (done.get("message") or {}).get("content") == "looking "
                  and ((done.get("message") or {}).get("tool_calls") or [{}])[0]
                      .get("function", {}).get("name") == "rig"
                  and done.get("finish_reason") == "tool_calls"
                  and done.get("complete") is True,
                  str(done)[:120])
            truncated = await client.post(
                "/agent",
                json={"messages": [{"role": "user", "content": "truncated"}],
                      "tools": tools, "stream": True},
                headers={"X-Rebot-Token": "selftest-token"})
            truncated_text = await truncated.text()
            check("agent refuses a truncated streamed tool call",
                  truncated.status == 200
                  and "event: error" in truncated_text
                  and "event: done" not in truncated_text
                  and "cut off by the token limit" in truncated_text,
                  truncated_text[-160:])

            no_token = await client.post("/agent", json={"messages": [{"role": "user", "content": "hi"}]})
            check("agent still needs the page token", no_token.status == 401)

            empty = await client.post("/agent", json={"messages": []},
                                      headers={"X-Rebot-Token": "selftest-token"})
            check("agent refuses an empty conversation", empty.status == 400)

            app["llm_url"] = "http://127.0.0.1:9/v1/chat/completions"
            unreachable = await client.post(
                "/agent", json={"messages": [{"role": "user", "content": "hi"}]},
                headers={"X-Rebot-Token": "selftest-token"})
            unreachable_body = await unreachable.json()
            check("agent reports an unreachable model as a gateway error",
                  unreachable.status == 502
                  and "cannot reach the model" in (unreachable_body.get("error") or ""),
                  "%s %s" % (unreachable.status, str(unreachable_body.get("error"))[:50]))

            app["llm_url"] = ""
            off = await client.post("/agent", json={"messages": [{"role": "user", "content": "hi"}]},
                                    headers={"X-Rebot-Token": "selftest-token"})
            check("agent is off when the host has no model configured",
                  off.status == 503,
                  str((await off.json()).get("error"))[:60])
        finally:
            app.update(saved_llm)
            await runner.cleanup()

        # The phone publishes which way up its stream is: the frame size cannot
        # say it, and the rig paths and /live need the sign.
        posted = await client.post("/orientation", json={"angle": 90, "size": [1920, 1080]},
                                   headers={"X-Rebot-Token": "selftest-token"})
        posted_body = await posted.json()
        health = await (await client.get("/health")).json()
        check("the phone can publish its stream orientation",
              posted.status == 200
              and (posted_body.get("stream_orientation") or {}).get("angle") == 90
              and (health.get("stream_orientation") or {}).get("angle") == 90,
              str(health.get("stream_orientation")))
        check("the published size comes back with it",
              (health.get("stream_orientation") or {}).get("width") == 1920
              and (health.get("stream_orientation") or {}).get("height") == 1080)
        reach_now = await (await client.get("/reach.json")).json()
        check("reach.json carries the calibration's own angle for the pairing",
              "orientation_deg" in (reach_now.get("calibration") or {}),
              str(sorted((reach_now.get("calibration") or {}).keys())))

        bad_angle = await client.post("/orientation", json={"angle": 45},
                                      headers={"X-Rebot-Token": "selftest-token"})
        check("a non-quarter-turn angle is refused", bad_angle.status == 400)
        no_orientation_token = await client.post("/orientation", json={"angle": 90})
        check("orientation still needs the page token", no_orientation_token.status == 401)


# --------------------------------------------------------------------------- #
# 4b. The trace recorder: append-only log, blobs, redaction, retention
# --------------------------------------------------------------------------- #

async def test_trace_recorder(workdir: Path):
    print("\ntrace recorder unit checks")
    import base64
    import hashlib
    import os

    import trace as trace_mod

    root = workdir / "trace-unit"
    recorder = trace_mod.TraceRecorder(root, retention_days=30)
    sid = "20261118T102320Z-abc123"

    first = await recorder.event(sid, "server", "session_start")
    seqs = await recorder.events(sid, [
        {"type": "user_message", "pseq": 1, "text": "hello"},
        {"type": "tool_call", "pseq": 2, "name": "rig", "args": {}},
    ])
    lines = [json.loads(line) for line in
             (root / sid / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    check("trace seq is monotonic from 1", [line["seq"] for line in lines] == [1, 2, 3]
          and first == 1 and seqs == [2, 3])
    check("trace batch preserves order and pseq",
          [line["type"] for line in lines] == ["session_start", "user_message", "tool_call"]
          and [line.get("pseq") for line in lines[1:]] == [1, 2])
    check("trace stamps src and t",
          lines[0]["src"] == "server" and lines[1]["src"] == "page"
          and all(line.get("t") for line in lines))

    restarted = trace_mod.TraceRecorder(root, retention_days=30)
    again = await restarted.event(sid, "server", "abort")
    check("trace seq continues across a restart", again == 4)

    ok, buffer = cv2.imencode(".jpg", np.full((24, 32, 3), 120, np.uint8),
                              [cv2.IMWRITE_JPEG_QUALITY, 82])
    jpeg = buffer.tobytes()
    sha_a = recorder.blob(jpeg)
    sha_b = recorder.blob(jpeg)
    check("blob dedup writes one file per sha", sha_a == sha_b
          and len(list((root / "blobs").iterdir())) == 1
          and (root / "blobs" / (sha_a + ".jpg")).read_bytes() == jpeg)

    url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")
    await recorder.event(sid, "page", "user_message", frame=url,
                         messages=[{"role": "user", "content": [
                             {"type": "image_url", "image_url": {"url": url}}]}])
    line = json.loads((root / sid / "events.jsonl").read_text(encoding="utf-8").splitlines()[-1])
    check("data-URL frames become blob refs",
          line["frame"] == {"$blob": sha_a}
          and line["messages"][0]["content"][0]["image_url"]["url"] == {"$blob": sha_a})

    await recorder.event(sid, "server", "model_turn",
                         usage={"prompt_tokens": 10, "total_tokens": 12},
                         config={"token": "secret", "Authorization": "Bearer x",
                                 "api_key": "k", "nested": {"planner_token": "t", "keep": 1},
                                 "llm_url": "http://modelhost:11434/v1/chat/completions"})
    line = json.loads((root / sid / "events.jsonl").read_text(encoding="utf-8").splitlines()[-1])
    config = line["config"]
    check("secret keys are redacted recursively",
          not ({"token", "Authorization", "api_key"} & set(config))
          and "planner_token" not in config["nested"] and config["nested"]["keep"] == 1)
    check("usage counters survive redaction",
          line["usage"] == {"prompt_tokens": 10, "total_tokens": 12})
    check("llm_url is reduced to its host", config["llm_url"] == "modelhost")

    other = "20261118T102321Z-def456"
    recorder.manifest(other, {"version": "first"})
    recorder.manifest(other, {"version": "second"})
    frozen = json.loads((root / other / "manifest.json").read_text(encoding="utf-8"))
    check("manifest is written once", frozen["version"] == "first")

    factory_sid = "20261118T102322Z-012345"
    factory = trace_mod.TraceRecorder(root, snapshot=lambda s: {"version": "factory", "sid": s})
    await factory.event(factory_sid, "page", "session_start")
    await factory.event(factory_sid, "page", "user_message", text="hi")
    made = json.loads((root / factory_sid / "manifest.json").read_text(encoding="utf-8"))
    check("first event writes the snapshot manifest once",
          made["version"] == "factory" and made["sid"] == factory_sid)

    try:
        await recorder.event("../escape", "page", "user_message")
        check("a malformed session id is refused", False)
    except ValueError:
        check("a malformed session id is refused", True)
    try:
        await recorder.events(sid, [{"type": "x"}] * 65)
        check("a batch over 64 events is refused", False)
    except ValueError:
        check("a batch over 64 events is refused", True)

    # Retention: an old session and its private blob go; a fresh session, its
    # blob, and anything export/ references stay.
    old = "20250101T000000Z-aaaaaa"
    fresh = "20261118T102323Z-bbbbbb"
    await recorder.event(old, "page", "user_message", frame={"$blob": recorder.blob(b"old-frame")})
    await recorder.event(fresh, "page", "user_message", frame={"$blob": recorder.blob(b"new-frame")})
    orphan = recorder.blob(b"orphan-frame")
    exported = recorder.blob(b"exported-frame")
    export_dir = root / "export" / "case-1"
    export_dir.mkdir(parents=True, exist_ok=True)
    (export_dir / "case.json").write_text(json.dumps({"still": {"$blob": exported}}),
                                          encoding="utf-8")
    aged = time.time() - 40 * 86400
    for directory in (root / old, export_dir):
        for path in list(directory.iterdir()) + [directory]:
            os.utime(path, (aged, aged))
    recorder.prune()
    check("prune drops a session past retention", not (root / old).exists())
    check("prune keeps a fresh session", (root / fresh / "events.jsonl").is_file())
    check("prune drops unreferenced blobs",
          not (root / "blobs" / (orphan + ".jpg")).exists()
          and not (root / "blobs" / (hashlib.sha256(b"old-frame").hexdigest() + ".jpg")).exists())
    check("prune keeps blobs a live session references",
          (root / "blobs" / (sha_a + ".jpg")).is_file()
          and (root / "blobs" / (hashlib.sha256(b"new-frame").hexdigest() + ".jpg")).is_file())
    check("export is exempt from prune",
          export_dir.is_dir()
          and (root / "blobs" / (exported + ".jpg")).is_file())


# --------------------------------------------------------------------------- #
# 4c. Trace end to end: the real server, MockEdge, a scripted model -- no arm
# --------------------------------------------------------------------------- #

async def test_trace_server(workdir: Path, geometry):
    print("\ntrace collector end to end (mock gateway, scripted model)")
    import base64
    import hashlib

    from aiohttp import web as aiohttp_web
    from aiohttp.test_utils import TestClient, TestServer

    import capture_server
    import mock_arm
    import trace as trace_mod

    image = geometry["image"]
    captures = workdir / "trace-captures"
    calib_dir = workdir / "trace-calib"
    traces = workdir / "trace-store"
    calib_dir.mkdir(parents=True, exist_ok=True)
    calib_bytes = json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [image.shape[1], image.shape[0]],
        "K": geometry["K"].tolist(), "dist": geometry["dist"].tolist(), "rms": 0.2,
    }).encode("utf-8")
    (calib_dir / "synthetic.json").write_bytes(calib_bytes)

    edge, edge_url, edge_state = mock_arm.serve(0)
    args = SimpleNamespace(
        host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
        token="trace-token", mediamtx="http://127.0.0.1:9", whip_path="phone",
        no_whip=True, max_mb=16, quiet=True,
        planner=edge_url, planner_token="", execute=True,
        calib_dir=str(calib_dir), calib="",
        marker_at="0.36,-0.10,0.12,180", marker_size=0.08,
        hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="",
        traces_dir=str(traces), trace_retention_days=30, no_trace=False,
    )
    app = capture_server.build_app(args)

    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
    jpeg = buffer.tobytes()
    frame_url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")
    frame_sha = hashlib.sha256(jpeg).hexdigest()
    sid = "20261118T102320Z-abc123"
    headers = {"X-Rebot-Token": "trace-token", "X-Rebot-Session": sid}
    tools = [{"type": "function", "function": {"name": "rig",
                                               "parameters": {"type": "object"}}}]
    tools_sha = hashlib.sha256(json.dumps(tools, sort_keys=True,
                                          ensure_ascii=False).encode("utf-8")).hexdigest()

    # A scripted model: turn 1 calls a tool over SSE, turn 2 answers in JSON.
    script = [
        {"content": "looking ", "tool_calls": [("rig", "{}")]},
        {"content": "the rig is fitted; nothing moved."},
    ]
    seen = []

    async def fake_chat(request):
        payload = await request.json()
        seen.append(payload)
        step = script[len(seen) - 1] if len(seen) <= len(script) else {"content": "ok"}
        calls = step.get("tool_calls") or []
        if payload.get("stream"):
            response = aiohttp_web.StreamResponse(
                status=200, headers={"Content-Type": "text/event-stream"})
            await response.prepare(request)
            chunks = []
            if step.get("content"):
                chunks.append({"choices": [{"delta": {"content": step["content"]}}]})
            for index, (name, arguments) in enumerate(calls):
                chunks.append({"choices": [{"delta": {"tool_calls": [{
                    "index": index, "id": "call_%d" % index, "type": "function",
                    "function": {"name": name, "arguments": arguments}}]}}]})
            chunks.append({"choices": [{"finish_reason": "tool_calls" if calls else "stop",
                                        "delta": {}}]})
            chunks.append({"usage": {"prompt_tokens": 10, "total_tokens": 12}, "choices": []})
            for chunk in chunks:
                await response.write(b"data: " + json.dumps(chunk).encode() + b"\n\n")
            await response.write(b"data: [DONE]\n\n")
            await response.write_eof()
            return response
        message = {"role": "assistant", "content": step.get("content", "")}
        if calls:
            message["tool_calls"] = [{"function": {"name": name, "arguments": arguments}}
                                     for name, arguments in calls]
        return aiohttp_web.json_response({
            "model": "stub-model",
            "choices": [{"message": message,
                         "finish_reason": "tool_calls" if calls else "stop"}],
            "usage": {"total_tokens": 12},
        })

    async def fake_models(request):
        return aiohttp_web.json_response({"data": [{"id": "stub-model",
                                                    "meta": {"n_ctx": 8192}}]})

    fake_llm = aiohttp_web.Application()
    fake_llm.router.add_post("/v1/chat/completions", fake_chat)
    fake_llm.router.add_get("/v1/models", fake_models)
    runner = aiohttp_web.AppRunner(fake_llm)
    await runner.setup()
    site = aiohttp_web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    llm_port = site._server.sockets[0].getsockname()[1]
    app["llm_url"] = "http://127.0.0.1:%d/v1/chat/completions" % llm_port
    app["llm_model"] = "stub-model"

    def read_events(session=sid):
        path = traces / session / "events.jsonl"
        if not path.is_file():
            return []
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]

    try:
        async with TestClient(TestServer(app)) as client:
            check("serves the trace queue module",
                  (await client.get("/trace.js")).status == 200)

            page_events = [
                {"type": "session_start", "pseq": 1},
                {"type": "user_message", "pseq": 2, "text": "place a marker",
                 "frame": frame_url},
                {"type": "tool_call", "pseq": 3, "name": "rig", "args": {}},
                {"type": "tool_result", "pseq": 4, "name": "rig", "args": {},
                 "result": {"ok": True}, "ms": 12},
                {"type": "frame_delivered", "pseq": 5, "frame": frame_url,
                 "width": image.shape[1], "height": image.shape[0], "reason": "capture_view"},
            ]
            posted = await client.post("/trace", json={"session": sid, "events": page_events},
                                       headers={"X-Rebot-Token": "trace-token"})
            posted_body = await posted.json()
            check("POST /trace accepts a page batch",
                  posted.status == 200 and posted_body.get("ok") is True
                  and posted_body.get("seqs") == [1, 2, 3, 4, 5])
            logged = read_events()
            check("the page log keeps every event type",
                  [event["type"] for event in logged]
                  == ["session_start", "user_message", "tool_call", "tool_result",
                      "frame_delivered"])
            check("the frame uploads once and resolves to its bytes",
                  logged[1]["frame"] == {"$blob": frame_sha}
                  and logged[4]["frame"] == {"$blob": frame_sha}
                  and (traces / "blobs" / (frame_sha + ".jpg")).read_bytes() == jpeg)

            manifest = json.loads((traces / sid / "manifest.json").read_text(encoding="utf-8"))
            live_sha = hashlib.sha256((capture_server.WEB_DIR / "live.html").read_bytes()).hexdigest()
            check("the manifest snapshots the environment on first sight",
                  manifest["version"] == capture_server.VERSION
                  and manifest["live_html_sha256"] == live_sha
                  and manifest["calib_fingerprint"]
                  and manifest["execute"] is True
                  and manifest["planner_host"] == "127.0.0.1"
                  and abs(manifest["tip_m"] - 0.035) < 1e-9)
            check("the manifest copies the calib file bytes",
                  (traces / sid / "calib.json").read_bytes() == calib_bytes)
            check("the manifest carries no secrets",
                  not ({"token", "llm_token", "api_key", "authorization"}
                       & set(manifest.get("llm") or {}))
                  and (manifest.get("llm") or {}).get("url_host") == "127.0.0.1"
                  and "authorization" not in json.dumps(manifest).lower()
                  and "api_key" not in json.dumps(manifest).lower())

            no_token = await client.get("/trace/%s/events" % sid)
            check("trace reads need the token even though GET is exempt",
                  no_token.status == 401)
            read_back = await client.get("/trace/%s/events" % sid,
                                         headers={"X-Rebot-Token": "trace-token"})
            read_body = await read_back.json()
            check("GET /trace/{sid}/events reads the log back",
                  read_back.status == 200 and len(read_body.get("events") or []) == 5)
            read_manifest = await client.get("/trace/%s/manifest" % sid,
                                             headers={"X-Rebot-Token": "trace-token"})
            check("GET /trace/{sid}/manifest reads the snapshot",
                  read_manifest.status == 200
                  and (await read_manifest.json())["manifest"]["version"]
                  == capture_server.VERSION)

            bad_sid = await client.post("/trace", json={"session": "../escape",
                                                        "events": [{"type": "x"}]},
                                        headers={"X-Rebot-Token": "trace-token"})
            check("POST /trace refuses a malformed session", bad_sid.status == 400)
            too_many = await client.post("/trace",
                                         json={"session": sid,
                                               "events": [{"type": "x"}] * 65},
                                         headers={"X-Rebot-Token": "trace-token"})
            check("POST /trace caps a batch at 64 events", too_many.status == 400)
            too_big = await client.post("/trace",
                                        json={"session": sid,
                                              "events": [{"type": "x", "blob": "y" * (9 * 1024 * 1024)}]},
                                        headers={"X-Rebot-Token": "trace-token"})
            check("POST /trace caps the body at 8 MiB", too_big.status == 413)

            # A streamed turn: the recorder must see the exact prompt (with
            # blob refs) and the assembled assistant message.
            streamed = await client.post(
                "/agent",
                json={"messages": [
                    {"role": "system", "content": "you operate an arm"},
                    {"role": "user", "content": [
                        {"type": "text", "text": "place a marker"},
                        {"type": "image_url", "image_url": {"url": frame_url}}]},
                ], "tools": tools, "stream": True},
                headers=headers)
            await streamed.text()
            turns = [event for event in read_events() if event["type"] == "model_turn"]
            check("the SSE done path records model_turn",
                  streamed.status == 200 and len(turns) == 1
                  and turns[0]["finish_reason"] == "tool_calls"
                  and turns[0]["complete"] is True
                  and turns[0]["usage"].get("total_tokens") == 12)
            turn = turns[0] if turns else {}
            prompt = (turn.get("messages") or [{}, {}])[1]
            check("model_turn stores the full prompt with blob refs",
                  isinstance(prompt.get("content"), list)
                  and prompt["content"][1]["image_url"]["url"] == {"$blob": frame_sha}
                  and (turn.get("message") or {}).get("content") == "looking "
                  and ((turn.get("message") or {}).get("tool_calls") or [{}])[0]
                      .get("function", {}).get("name") == "rig")
            check("model_turn pins the model config and tool contract",
                  (turn.get("config") or {}).get("model") == "stub-model"
                  and (turn.get("config") or {}).get("tools_sha256") == tools_sha
                  and not ({"token", "api_key", "authorization"}
                           & set(turn.get("config") or {})))

            # The JSON path records too, and a second session's manifest pins
            # the tool schemas because /agent was its first event.
            sid2 = "20261118T102321Z-def456"
            json_turn = await client.post(
                "/agent",
                json={"messages": [{"role": "user", "content": "and now?"}],
                      "tools": tools},
                headers={"X-Rebot-Token": "trace-token", "X-Rebot-Session": sid2})
            json_body = await json_turn.json()
            turns2 = [event for event in read_events(sid2) if event["type"] == "model_turn"]
            check("the JSON path records model_turn",
                  json_turn.status == 200 and json_body.get("ok") is True
                  and len(turns2) == 1
                  and turns2[0]["message"]["content"] == "the rig is fitted; nothing moved.")
            manifest2 = json.loads((traces / sid2 / "manifest.json").read_text(encoding="utf-8"))
            check("a first-seen /agent pins tool_schemas_sha256 in the manifest",
                  manifest2.get("tool_schemas_sha256") == tools_sha)

            # The gateway tape: /sequence and /execute flow through planner_call.
            sequenced = await client.post(
                "/sequence", json={"targets": [{"tip": [0.45, 0.0, 0.16],
                                                "pixel": [640, 480]}]},
                headers=headers)
            sequence_body = await sequenced.json()
            plan_id = (sequence_body.get("plan") or {}).get("id")
            plan_events = [event for event in read_events() if event["type"] == "plan"]
            tape = [event for event in read_events() if event["type"] == "gateway_call"]
            check("sequence records plan linked to the session",
                  sequenced.status == 200 and plan_id == edge_state.last_plan
                  and len(plan_events) == 1 and plan_events[0]["plan_id"] == plan_id
                  and plan_events[0]["ok"] is True)
            check("planner_call tapes the gateway request and response",
                  len(tape) == 1 and tape[0]["path"] == "/plan"
                  and tape[0]["status"] == 200
                  and tape[0]["response"]["id"] == plan_id
                  and tape[0]["request"]["targets"][0]["tip"] == [0.45, 0.0, 0.16]
                  and len(tape[0]["key"]) == 64 and tape[0]["ms"] >= 0)

            executed = await client.post("/execute", json={"id": plan_id}, headers=headers)
            execute_body = await executed.json()
            execute_events = [event for event in read_events() if event["type"] == "execute"]
            check("execute records the plan_id and the outcome",
                  executed.status == 200 and execute_body.get("executed") is True
                  and len(execute_events) == 1
                  and execute_events[0]["plan_id"] == plan_id
                  and execute_events[0]["executed"] is True)
            check("execute is taped too",
                  any(event["path"] == "/execute" for event in read_events()
                      if event["type"] == "gateway_call"))

            # An unreachable model is a stream_error, not a lost turn.
            saved_url = app["llm_url"]
            app["llm_url"] = "http://127.0.0.1:9/v1/chat/completions"
            failed = await client.post("/agent",
                                       json={"messages": [{"role": "user", "content": "hi"}]},
                                       headers=headers)
            app["llm_url"] = saved_url
            errors = [event for event in read_events() if event["type"] == "stream_error"]
            check("a failed turn records stream_error",
                  failed.status == 502 and len(errors) == 1
                  and "127.0.0.1:9" in errors[0]["error"])

            health = await (await client.get("/health")).json()
            check("health reports the trace collector",
                  (health.get("traces") or {}).get("enabled") is True
                  and (health.get("traces") or {}).get("sessions") == 2
                  and (health.get("traces") or {}).get("retention_days") == 30)

            # §8.4: a broken recorder degrades to "no trace", never a failure.
            recorder = app["traces"]

            async def broken_event(*a, **k):
                raise RuntimeError("the disk is full")

            async def broken_events(*a, **k):
                raise RuntimeError("the disk is full")

            real_event, real_events = recorder.event, recorder.events
            recorder.event, recorder.events = broken_event, broken_events
            try:
                still_ok = await client.post(
                    "/agent", json={"messages": [{"role": "user", "content": "hi"}]},
                    headers=headers)
                still_planned = await client.post(
                    "/sequence", json={"targets": [{"tip": [0.45, 0.0, 0.16]}]},
                    headers=headers)
                dropped = await client.post("/trace",
                                            json={"session": sid,
                                                  "events": [{"type": "user_message"}]},
                                            headers={"X-Rebot-Token": "trace-token"})
            finally:
                recorder.event, recorder.events = real_event, real_events
            check("a broken recorder never fails /agent or /sequence",
                  still_ok.status == 200 and still_planned.status == 200)
            check("a broken recorder drops the page batch with a 500",
                  dropped.status == 500 and (await dropped.json()).get("ok") is False)

            # Every blob reference in the merged log resolves to real bytes.
            refs = set(trace_mod.BLOB_REF.findall(
                (traces / sid / "events.jsonl").read_text(encoding="utf-8")))
            check("every $blob in the log resolves",
                  refs and all((traces / "blobs" / (sha + ".jpg")).is_file() for sha in refs))

        # --no-trace disables collection entirely.
        off_args = SimpleNamespace(**vars(args))
        off_args.no_trace = True
        off_args.traces_dir = str(workdir / "trace-off")
        off = capture_server.build_app(off_args)
        async with TestClient(TestServer(off)) as client:
            refused = await client.post("/trace",
                                        json={"session": sid, "events": [{"type": "x"}]},
                                        headers={"X-Rebot-Token": "trace-token"})
            off_health = await (await client.get("/health")).json()
            check("--no-trace refuses writes and reports disabled",
                  refused.status == 503
                  and (off_health.get("traces") or {}).get("enabled") is False)
            check("--no-trace leaves no trace directory",
                  not (workdir / "trace-off").exists())
    finally:
        await runner.cleanup()
        edge.shutdown()


# --------------------------------------------------------------------------- #
# 4d. Phase 2: the oracle's pure rules, the labeler, the gateway's new routes
# --------------------------------------------------------------------------- #

def test_oracle_rules():
    print("\noracle: hold windows, downsampling, residual attribution")
    import oracle

    # Hold windows: constant cmd for >= 0.8 s; a 0.5 s stop is tracking-only.
    waypoints = []
    t = 0.0
    for q in (0.0, 0.05, 0.1):                    # a move, not a hold
        waypoints.append({"t": t, "q": [q, 0.0]})
        t += 0.1
    for _ in range(11):                             # 1.0 s hold at 0.2
        waypoints.append({"t": t, "q": [0.2, 0.0]})
        t += 0.1
    waypoints.append({"t": t, "q": [0.3, 0.0]})
    t += 0.1
    for _ in range(6):                              # 0.5 s stop: too short
        waypoints.append({"t": t, "q": [0.3, 0.0]})
        t += 0.1
    windows = oracle.hold_windows(waypoints)
    check("hold_windows finds the >=0.8 s constant-cmd window",
          len(windows) == 1 and windows[0]["q_cmd"] == [0.2, 0.0]
          and abs(windows[0]["t_end"] - windows[0]["t_start"] - 1.0) < 1e-9)
    check("hold_windows skips a short stop",
          all(w["q_cmd"] != [0.3, 0.0] for w in windows))

    # Downsampling: every hold sample survives the wire budget.
    samples = []
    for index in range(5000):
        cmd = 0.5 if 1000 <= index < 1100 else (-0.25 if 3000 <= index < 3050
                                               else index * 0.001)
        samples.append({"t": index * 0.02, "cmd": [cmd], "meas": [cmd - 0.01]})
    thinned = oracle.downsample(samples, 2000)
    times = [sample["t"] for sample in thinned]
    check("downsample stays within the wire budget", len(thinned) <= 2000)
    check("downsample keeps every hold-window sample",
          all(index * 0.02 in times for index in list(range(1000, 1100))
              + list(range(3000, 3050))))
    check("downsample keeps order, first and last",
          times == sorted(times) and times[0] == 0.0 and times[-1] == 4999 * 0.02)
    check("downsample leaves a small log alone",
          oracle.downsample(samples[:10], 2000) == samples[:10])
    long_hold = [{"t": float(i), "cmd": [0.7], "meas": [0.7]} for i in range(3000)]
    check("holds win over the budget when they exceed it",
          len(oracle.downsample(long_hold, 2000)) == 3000)

    # A marker hold: joints 1-6 constant while the gripper (motor 7) ramps
    # through the grasp cycle. Comparing all seven finds no holds at all in a
    # pick sequence -- the accuracy map would never populate.
    marker = []
    t = 0.0
    for i in range(41):
        marker.append({"t": t, "q": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 + 0.05 * i]})
        t += 0.033
    windows = oracle.hold_windows(marker)
    check("a gripper ramp inside an arm hold does not hide the hold",
          len(windows) == 1 and windows[0]["q_cmd"][:6] == [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
          str(windows)[:80])
    ramp_samples = []
    for index in range(5000):
        if 1000 <= index < 1100:
            cmd = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 + 0.01 * (index - 1000)]
        else:
            cmd = [index * 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
        ramp_samples.append({"t": index * 0.02, "cmd": cmd, "meas": cmd})
    thinned = oracle.downsample(ramp_samples, 2000)
    times = [sample["t"] for sample in thinned]
    check("downsample keeps hold samples through a gripper ramp",
          len(thinned) <= 2000
          and all(index * 0.02 in times for index in range(1000, 1100)))

    # Attribution: one synthetic document per residual-signature row (§5.4).
    def hold(**fields):
        base = {"hold_error_rad": [0.0] * 7, "tip_error_mm": 1.0,
                "pixel_error_mm": 1.0, "detected_tip_px": [100.0, 100.0]}
        base.update(fields)
        return base

    rows = [
        ("rig-mismatch: camera disagrees, arm held",
         {"holds": [hold(pixel_error_mm=15.0, tip_error_mm=2.0)]},
         "camera-arm", "rig-mismatch"),
        ("turns-correction: shape-fallback turn with pixel error",
         {"holds": [hold(pixel_error_mm=15.0, tip_error_mm=2.0)],
          "stream_turn": {"source": "shape", "turns": None}},
         "camera-arm", "turns-correction"),
        ("stale-rig: witness drift over the limit",
         {"holds": [hold()], "witness": {"drift_px": 3.5, "scale": 1.0}},
         "camera-arm", "stale-rig"),
        ("sag: hold error tracks tau_g/kp",
         {"holds": [hold(hold_error_rad=[-0.01, -0.05, -0.04, -0.005, -0.002, 0.0, 0.0],
                         sag_rad=[0.01, 0.05, 0.04, 0.005, 0.002, 0.0],
                         pixel_error_mm=1.0, tip_error_mm=1.0)]},
         "arm-motor", "sag"),
        ("joint-zeros: constant per-joint bias across poses",
         {"holds": [hold(hold_error_rad=[0.04] * 6 + [0.0], pixel_error_mm=1.0,
                         tip_error_mm=1.0),
                    hold(hold_error_rad=[0.04] * 6 + [0.0], pixel_error_mm=1.0,
                         tip_error_mm=1.0)]},
         "arm-motor", "joint-zeros"),
        ("reach-model: tip error at the boundary with IK refusals",
         {"holds": [hold(tip_error_mm=14.0, pixel_error_mm=1.0)],
          "ik_refusals": 2, "max_radius_m": 0.72, "max_reach_m": 0.75},
         "reach", "reach-model"),
        ("one-plane: z-only placement error the object's height",
         {"holds": [hold()], "placement": {"error_mm": [2.0, 1.0, 40.0],
                                            "object_height_mm": 40.0}},
         "limitation", "one-plane"),
        ("ik-residual: the command misses its target while the arm holds it",
         {"holds": [hold(ik_residual_mm=15.0, tip_error_mm=2.0)]},
         "ik", "ik-residual"),
        ("execution fault: abort and gripper stall in telemetry",
         {"holds": [hold()], "telemetry": {"abort": {"t": 3.0, "reason": "not following"},
                                            "gripper_stall": {"t": 3.0, "commanded": 2.4,
                                                              "measured": 4.0}}},
         "execution", "follow-abort"),
    ]
    for name, doc, layer, kind in rows:
        verdict = oracle.attribute(doc)
        kinds = [problem["kind"] for problem in verdict["problems"]]
        check("attribute: %s" % name,
              verdict["layer"] == layer and kind in kinds,
              "layer=%s kinds=%s" % (verdict["layer"], kinds))
    check("attribute: a clean run has no problems",
          oracle.attribute({"holds": [hold(), hold()]})["problems"] == [])
    check("attribute: no evidence is low confidence, not a pass",
          oracle.attribute({})["confidence"] == "low"
          and oracle.attribute({})["layer"] == "none")
    check("attribute: an aborted run is medium confidence",
          oracle.attribute(rows[-1][1])["confidence"] == "medium")

    # The FK bridge's separation: a large IK residual with a large hold error
    # is NOT the solver's signature alone -- the arm is not holding its command,
    # so the motor layer owns it. A small residual never fires.
    verdict = oracle.attribute({"holds": [hold(ik_residual_mm=15.0, tip_error_mm=20.0)]})
    kinds = [problem["kind"] for problem in verdict["problems"]]
    check("attribute: ik residual with a bad hold is not an ik problem",
          "ik-residual" not in kinds and "tip-error" in kinds, str(kinds))
    check("attribute: a residual inside the planner's acceptance is silent",
          oracle.attribute({"holds": [hold(ik_residual_mm=5.0)]})["problems"] == [])

    # Hold -> target matching: by the plan's own declared hold spans, not by
    # index (the start hold exists only when the gripper started closed) and
    # not by joint proximity (a marker near park would confuse that).
    targets = [
        {"point": [0.30, 0.00, 0.20], "q_ik": [0.10] * 6,
         "hold_t_start": 5.0, "hold_t_end": 6.3},
        {"point": [0.40, 0.10, 0.25], "q_ik": [0.20] * 6,
         "hold_t_start": 9.0, "hold_t_end": 10.3},
    ]
    windows = [
        {"t_start": 0.0, "t_end": 0.83, "q_cmd": [0.0] * 7},          # start hold
        {"t_start": 5.03, "t_end": 6.3, "q_cmd": [0.10] * 6 + [2.45]},
        {"t_start": 9.03, "t_end": 10.3, "q_cmd": [0.20] * 6 + [4.9]},
    ]
    matched = oracle.match_hold_targets(windows, targets)
    check("match_hold_targets leaves the start hold unmatched", matched[0] is None)
    check("match_hold_targets matches holds by the plan's clock",
          matched[1] is targets[0] and matched[2] is targets[1], str(matched))
    check("match_hold_targets skips targets without hold spans (older plans)",
          oracle.match_hold_targets(windows, [{"point": [0.3, 0.0, 0.2],
                                               "q_ik": [0.1] * 6}]) == [None] * 3)
    check("match_hold_targets handles empty inputs",
          oracle.match_hold_targets([], targets) == []
          and oracle.match_hold_targets(windows, []) == [None] * 3)
    near_park = [{"t_start": 20.0, "t_end": 21.0, "q_cmd": [0.10] * 6 + [0.0]}]
    check("a window outside every span stays unmatched even near a marker's pose",
          oracle.match_hold_targets(near_park, targets) == [None])


def test_trace_triage(workdir: Path):
    print("\ntrace triage: canned logs -> class / layer / signature")
    import trace_triage

    sid = "20261118T102320Z-abc123"
    frame = {"$blob": "a" * 64}

    def window(extra, final="done.", feedback=None):
        events = [{"type": "user_message", "t": "2026-11-18T10:00:00.000Z",
                   "pseq": 1, "text": "do the thing", "frame": frame}]
        events += extra
        events.append({"type": "task_end", "t": "2026-11-18T10:01:00.000Z",
                       "final_text": final, "turns": 3, "exhausted": False})
        if feedback:
            events.append({"type": "feedback", "t": "2026-11-18T10:02:00.000Z",
                           "verdict": feedback})
        return events

    plan_id = "20261118T100030Z-111111"
    move_call = {"type": "tool_call", "name": "move_arm", "args": {"execute": True}}
    plan_call = {"type": "tool_call", "name": "move_arm", "args": {"execute": False}}
    planned = {"type": "tool_result", "name": "move_arm",
               "result": {"ok": True, "planned": True, "executed": False,
                          "plan_id": plan_id}}
    executed = {"type": "execute", "plan_id": plan_id, "executed": True}

    # Protocol fail: moved before planning, and claims a move that never ran.
    labels = trace_triage.label_session(
        window([move_call], final="I have moved the arm to the marker."),
        sid, version="test-version")
    kinds = {p["kind"] for p in labels[0]["problems"]}
    check("triage: protocol fail (move-before-plan + dishonest claim)",
          labels[0]["verdict"] == "fail" and labels[0]["class"] == "protocol"
          and {"never-planned", "claims-move-not-executed"} <= kinds)

    # Geometry fail: the oracle's pixel residual joins by plan_id.
    oracle_doc = {"plan_id": plan_id, "holds": [
        {"hold_error_rad": [0.0] * 7, "tip_error_mm": 2.0, "pixel_error_mm": 18.0,
         "detected_tip_px": [100.0, 100.0]}]}
    labels = trace_triage.label_session(
        window([plan_call, planned, executed]), sid,
        oracle_docs={plan_id: oracle_doc}, version="test-version")
    check("triage: geometry fail joins the oracle by plan_id",
          labels[0]["verdict"] == "fail" and labels[0]["class"] == "geometry"
          and labels[0]["layer"] == "camera-arm"
          and labels[0]["plan_ids"] == [plan_id]
          and any(p["kind"] == "rig-mismatch" for p in labels[0]["problems"]))

    # Limitation: z-only placement error is reported, never a protocol ticket.
    limitation_doc = {"plan_id": plan_id, "holds": [],
                      "placement": {"error_mm": [1.0, 2.0, 35.0],
                                    "object_height_mm": 35.0}}
    labels = trace_triage.label_session(
        window([plan_call, planned, executed]), sid,
        oracle_docs={plan_id: limitation_doc}, version="test-version")
    check("triage: limitation is labelled, not ticketed",
          labels[0]["class"] == "limitation" and labels[0]["layer"] == "limitation")

    # Perception-suspect: user fail, clean oracle, markers present.
    clean_doc = {"plan_id": plan_id, "holds": [
        {"hold_error_rad": [0.0] * 7, "tip_error_mm": 1.0, "pixel_error_mm": 1.0,
         "detected_tip_px": [100.0, 100.0]}]}
    marker = {"type": "tool_call", "name": "add_marker", "args": {"u": 10, "v": 20}}
    labels = trace_triage.label_session(
        window([marker, plan_call, planned, executed], feedback="fail"), sid,
        oracle_docs={plan_id: clean_doc}, version="test-version")
    check("triage: perception-suspect (user fail, oracle clean, markers)",
          labels[0]["verdict"] == "fail" and labels[0]["class"] == "perception"
          and any(p["kind"] == "perception-suspect" for p in labels[0]["problems"]))

    # Signatures: stable for the same evidence, keyed by frame and code version.
    events = window([move_call], final="I have moved the arm.")
    first = trace_triage.label_session(events, sid, version="v1")[0]["signature"]
    again = trace_triage.label_session(events, sid, version="v1")[0]["signature"]
    other_frame = trace_triage.label_session(
        [dict(e, frame={"$blob": "b" * 64}) if e["type"] == "user_message" else e
         for e in events], sid, version="v1")[0]["signature"]
    other_code = trace_triage.label_session(events, sid, version="v2")[0]["signature"]
    check("triage: signature is stable and keyed by frame + code version",
          first == again and first != other_frame and first != other_code
          and len(first) == 64)

    # Telemetry abort without an oracle doc: an execution fault, class infra.
    labels = trace_triage.label_session(
        window([plan_call, planned, executed]), sid,
        telemetry_docs={plan_id: {"abort": {"t": 2.0, "reason": "motor 2 not following"}}},
        version="test-version")
    check("triage: telemetry abort is infra, not a correspondence bug",
          labels[0]["class"] == "infra"
          and any(p["kind"] == "follow-abort" for p in labels[0]["problems"]))

    # The batch runner: events + oracle files in a temp state -> labels.jsonl.
    traces = workdir / "triage-state" / "traces"
    (traces / sid).mkdir(parents=True)
    (traces / sid / "events.jsonl").write_text(
        "".join(json.dumps(e) + "\n" for e in
                window([plan_call, planned, executed])), encoding="utf-8")
    oracle_dir = workdir / "triage-state" / "oracle"
    oracle_dir.mkdir(parents=True)
    (oracle_dir / ("%s.json" % plan_id)).write_text(json.dumps(oracle_doc),
                                                    encoding="utf-8")
    trace_triage.run(traces)
    written = [json.loads(line) for line in
               (traces / sid / "labels.jsonl").read_text(encoding="utf-8").splitlines()]
    check("triage run writes labels.jsonl per session",
          len(written) == 1 and written[0]["class"] == "geometry"
          and written[0]["task_id"] == sid + "-task-1")


def _import_planner():
    """planner_server imports plan_to_target (Pinocchio); stub it when absent.

    The routes under test here (/fk, /execute, /move, /telemetry) never plan,
    so a stub that refuses to plan is enough to import the module offline.
    """
    try:
        import plan_to_target  # noqa: F401
    except ImportError:
        import types
        stub = types.ModuleType("plan_to_target")

        class Options:
            hover = True
            hover_m = 0.0
            tip_m = 0.0
            quiet = True
            ik_pos_tol = 0.001

        def _offline(*_args, **_kwargs):
            raise RuntimeError("planning needs Pinocchio; not under test here")

        stub.Options = Options
        stub.PlanError = type("PlanError", (Exception,), {})
        stub.plan = _offline
        stub.plan_sequence = _offline
        sys.modules["plan_to_target"] = stub
    import planner_server
    return planner_server


def test_planner_offline(workdir: Path):
    print("\nplanner gateway: /fk, telemetry persistence (no Pinocchio, no edge)")
    import urllib.error
    import urllib.request

    planner = _import_planner()
    saved = (planner.OUT_DIR, planner.TELEMETRY_DIR, planner.EXECUTE, planner.TOKEN)
    planner.OUT_DIR = workdir / "planner-out"
    planner.TELEMETRY_DIR = workdir / "planner-telemetry"
    planner.EXECUTE = True
    planner.TOKEN = "planner-token"
    planner.OUT_DIR.mkdir(parents=True, exist_ok=True)

    telemetry = {"format": "rebot-trajectory-log", "version": 2,
                 "samples": [{"t": 0.0, "cmd": [0.0] * 7, "meas": [0.0] * 7}],
                 "tick": {"mean_ms": 33.3, "p95_ms": 35.0, "max_ms": 40.0}}
    played = (200, {"ok": True, "executed": True, "output": "played",
                    "telemetry": telemetry})

    pose_q = [[0.0] * 7]

    def fake_edge(method, path, body=None, timeout=30):
        if path == "/pose":
            return 200, {"ids": [1, 2, 3, 4, 5, 6, 7], "q": pose_q[0]}
        if path == "/health":
            return 200, {"can_up": True}
        if path.startswith("/play"):
            return played
        raise AssertionError("unexpected edge call %s" % path)

    real_edge = planner.edge_request
    planner.edge_request = fake_edge
    server = None
    try:
        # /fk is rebot_ik's FK: the tip and seven joint origins, read-only.
        q = [0.1, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]
        answer = planner.fk_one({"q": q})
        tool, links = rebot_ik.forward(np.asarray(q[:6]), with_links=True)
        check("fk returns the tool tip and seven joint points",
              answer["ok"] is True and len(answer["joint_points"]) == 7
              and np.allclose(answer["tip_m"], tool[:3, 3])
              and np.allclose(answer["joint_points"][-1], tool[:3, 3])
              and np.allclose(answer["joint_points"][0], links[0][:3, 3]))
        for bad in ({"q": [0.0] * 6}, {"q": [float("nan")] * 7}, {"q": "nope"}):
            try:
                planner.fk_one(bad)
                check("fk refuses a bad pose", False)
            except planner.GatewayError:
                check("fk refuses a bad pose", True)

        # execute_one persists the edge's log and relays the same block.
        plan_id = planner.new_id()
        limits = [[float(lo), float(hi)] for lo, hi in rebot_ik.LIMITS] + [[-0.1, 5.0]]
        trajectory = traj.joint_move(np.zeros(7), np.append(sweep.POSES[0], 0.0),
                                     3.0, limits=limits)
        (planner.OUT_DIR / ("%s.json" % plan_id)).write_text(
            json.dumps(trajectory), encoding="utf-8")
        answer = planner.execute_one({"id": plan_id})
        stored = planner.TELEMETRY_DIR / ("%s.json" % plan_id)
        check("execute_one relays telemetry next to executed/log",
              answer["executed"] is True and answer["telemetry"] == telemetry
              and "log" in answer)
        check("execute_one persists the full telemetry document",
              stored.is_file()
              and json.loads(stored.read_text(encoding="utf-8")) == telemetry)
        check("read_telemetry reads it back",
              planner.read_telemetry(plan_id) == telemetry)

        # move_one is taped too, keyed by a generated move id.
        answer = planner.move_one({"q": [0.0] * 6})
        move_id = answer.get("id")
        check("move_one returns a move id and its telemetry",
              bool(move_id) and planner.PLAN_ID.match(move_id)
              and answer["telemetry"] == telemetry
              and (planner.TELEMETRY_DIR / ("%s.json" % move_id)).is_file())

        # A caller may name the move, and may ask for the trajectory WITHOUT
        # playing it (§7.4): that is how a coverage session reads the real hold
        # windows before the arm moves, so the oracle's grabs land inside them.
        named = planner.new_id()
        dry = planner.move_one({"q": [0.0] * 6, "hold": 2.0, "id": named, "dry_run": True})
        built = planner.OUT_DIR / ("%s.json" % named)
        check("a dry run keeps the caller's id",
              dry["id"] == named and dry["dry_run"] is True)
        check("a dry run persists the trajectory", built.is_file())
        waypoints = json.loads(built.read_text(encoding="utf-8"))["waypoints"] if built.is_file() else []
        check("the persisted move gives the oracle its hold windows",
              bool(oracle.hold_windows(waypoints)), "%d waypoints" % len(waypoints))
        check("a dry run plays nothing and stores no telemetry",
              "telemetry" not in dry
              and not (planner.TELEMETRY_DIR / ("%s.json" % named)).is_file())
        try:
            planner.move_one({"q": [0.0] * 6, "id": "not-a-plan-id"})
            check("a malformed caller id is refused", False)
        except planner.GatewayError:
            check("a malformed caller id is refused", True)
        played = planner.move_one({"q": [0.0] * 6, "hold": 2.0, "id": named})
        check("the same id then plays and stores telemetry under it",
              played["id"] == named and played.get("telemetry") == telemetry
              and (planner.TELEMETRY_DIR / ("%s.json" % named)).is_file())
        check("an ordinary move still persists its trajectory",
              (planner.OUT_DIR / ("%s.json" % move_id)).is_file())

        # A torque-off joint sagging a few mrad past its limit at park must not
        # block every move: the start is clamped into the envelope (the edge's
        # align leg ramps from the measured pose to it). A large excursion is a
        # fault symptom, not sag, and is still refused.
        sagged = [0.0] * 7
        sagged[1] = -0.0022  # j2 rests just below its 0.0 lower limit
        pose_q[0] = sagged
        sag_id = planner.new_id()
        dry = planner.move_one({"q": [0.0] * 6, "hold": 2.0, "id": sag_id, "dry_run": True})
        sag_plan = json.loads(
            (planner.OUT_DIR / ("%s.json" % sag_id)).read_text(encoding="utf-8"))
        check("a small start sag is clamped into the envelope",
              dry["dry_run"] is True
              and sag_plan["waypoints"][0]["q"][1] >= 0.0)
        pose_q[0] = [0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # 100 mrad: a fault, not sag
        try:
            planner.move_one({"q": [0.0] * 6, "hold": 2.0, "dry_run": True})
            check("a large start excursion is still refused", False)
        except planner.GatewayError:
            check("a large start excursion is still refused", True)
        pose_q[0] = [0.0] * 7

        # The mandatory startup peel: a /move from rest prepends the elbow lift
        # (j3 +8 deg) so the arm breaks the j4-hinge-on-j2-casing contact before
        # any wrist-driving leg. From a pose already off the stops, no extra leg.
        peel_id = planner.new_id()
        dry = planner.move_one({"q": list(sweep.POSES[0]), "hold": 2.0,
                                "id": peel_id, "dry_run": True})
        peel_wps = json.loads(
            (planner.OUT_DIR / ("%s.json" % peel_id)).read_text(encoding="utf-8"))["waypoints"]
        has_lift = any(0.12 <= w["q"][2] <= 0.25 for w in peel_wps)
        check("a move from rest peels the elbow before the first pose",
              dry["dry_run"] is True and dry["poses"] == 2 and has_lift)
        pose_q[0] = [0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]  # already off the stops
        nopeel_id = planner.new_id()
        dry2 = planner.move_one({"q": list(sweep.POSES[0]), "hold": 2.0,
                                 "id": nopeel_id, "dry_run": True})
        check("a move already off the stops adds no peel leg", dry2["poses"] == 1)
        pose_q[0] = [0.0] * 7

        # An aborted playback still carries its log (§5.1): the host must
        # persist it before raising, or the fault evidence dies with the error
        # and the abort becomes undiagnosable.
        played = (500, {"ok": False, "exit_code": 1,
                        "error": "motor id 4 reported a fault frame: 1040000000000000",
                        "output": "ABORTED at t=5.86s",
                        "telemetry": dict(telemetry, abort={"t": 5.86, "reason": "fault"})})
        abort_id = planner.new_id()
        try:
            planner.move_one({"q": [0.0] * 6, "hold": 2.0, "id": abort_id})
            check("an aborted move raises", False)
        except planner.GatewayError as error:
            check("an aborted move raises with the edge's reason",
                  "fault frame" in str(error))
        abort_log = planner.TELEMETRY_DIR / ("%s.json" % abort_id)
        check("an aborted move's telemetry is persisted, not discarded",
              abort_log.is_file()
              and json.loads(abort_log.read_text(encoding="utf-8"))["abort"]["t"] == 5.86)
        abort_plan = planner.new_id()
        (planner.OUT_DIR / ("%s.json" % abort_plan)).write_text(
            json.dumps(trajectory), encoding="utf-8")
        try:
            planner.execute_one({"id": abort_plan})
            check("an aborted execution raises", False)
        except planner.GatewayError:
            check("an aborted execution raises", True)
        check("an aborted execution's telemetry is persisted too",
              (planner.TELEMETRY_DIR / ("%s.json" % abort_plan)).is_file())
        played = (200, {"ok": True, "executed": True, "output": "played",
                        "telemetry": telemetry})

        # The HTTP surface: token-gated GET /telemetry/{id}, POST /fk.
        from http.server import ThreadingHTTPServer
        server = ThreadingHTTPServer(("127.0.0.1", 0), planner.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base = "http://127.0.0.1:%d" % server.server_address[1]

        def call(method, path, body=None, token="planner-token"):
            data = json.dumps(body).encode() if body is not None else None
            request = urllib.request.Request(base + path, data=data, method=method,
                                             headers={"X-Rebot-Token": token} if token else {})
            try:
                with urllib.request.urlopen(request, timeout=10) as answer:
                    return answer.status, json.loads(answer.read())
            except urllib.error.HTTPError as error:
                return error.code, json.loads(error.read())

        status, body = call("GET", "/telemetry/" + plan_id)
        check("GET /telemetry/{id} serves the persisted log",
              status == 200 and body["telemetry"] == telemetry)
        status, _body = call("GET", "/telemetry/" + plan_id, token="")
        check("GET /telemetry needs the token", status == 401)
        status, _body = call("GET", "/telemetry/20261118T100030Z-ffffff")
        check("GET /telemetry 404s an unknown id", status == 404)
        status, body = call("POST", "/fk", {"q": q})
        check("POST /fk answers over HTTP",
              status == 200 and np.allclose(body["tip_m"], tool[:3, 3]))
        status, _body = call("POST", "/fk", {"q": [0.0] * 6}, token="")
        check("POST /fk needs the token", status == 401)
    finally:
        planner.edge_request = real_edge
        (planner.OUT_DIR, planner.TELEMETRY_DIR, planner.EXECUTE, planner.TOKEN) = saved
        if server is not None:
            server.shutdown()


async def test_oracle_server(workdir: Path, geometry):
    print("\noracle end to end (mock gateway, fake camera, no arm)")
    from aiohttp.test_utils import TestClient, TestServer

    import capture_server
    import mock_arm

    captures = workdir / "oracle-captures"
    calib_dir = workdir / "oracle-calib"
    traces = workdir / "oracle-traces"
    calib_dir.mkdir(parents=True, exist_ok=True)
    image = geometry["image"]
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [image.shape[1], image.shape[0]],
        "K": geometry["K"].tolist(), "dist": geometry["dist"].tolist(), "rms": 0.2,
    }), encoding="utf-8")

    edge, edge_url, edge_state = mock_arm.serve(0)
    args = SimpleNamespace(
        host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
        token="oracle-token", mediamtx="http://127.0.0.1:9", whip_path="phone",
        no_whip=True, max_mb=16, quiet=True,
        planner=edge_url, planner_token="", execute=True,
        calib_dir=str(calib_dir), calib="",
        marker_at="", marker_size=0.08,
        hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="device:0",
        traces_dir=str(traces), trace_retention_days=30, no_trace=False,
    )
    app = capture_server.build_app(args)
    app["rig"] = extrinsics.Rig(T_base_cam=geometry["T_base_cam"],
                                image_size=(image.shape[1], image.shape[0]),
                                tip_offset=np.array([0.0, 0.0, -0.035]))

    # The fake camera serves the synthetic still with the arm's green tool
    # drawn at each hold's *measured* pose (the camera sees where the arm
    # actually is, sag included); the arm itself never moves. The mock
    # gateway's /pose answers q=0, so grab_one's stillness check passes.
    import camera as camera_mod
    real_open = camera_mod.open_source

    # A plan with two 1.0 s holds, and telemetry whose measured joints sag by
    # exactly the model's tau_g/kp: the oracle must attribute that to sag.
    plan_id = capture_server.new_capture_id()
    limits = [[float(lo), float(hi)] for lo, hi in rebot_ik.LIMITS] + [[-0.1, 5.0]]
    poses = [np.append(sweep.POSES[3], 0.0), np.append(sweep.POSES[4], 0.0)]
    trajectory = traj.joint_sequence([np.zeros(7)] + poses, [0.5, 0.5],
                                     hold_s=1.0, limits=limits)
    out_dir = captures.parent / "out"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / ("%s.json" % plan_id)).write_text(json.dumps(trajectory),
                                                 encoding="utf-8")
    kp = sweep.kp_vector()
    samples = [{"t": point["t"], "cmd": point["q"],
                "meas": [float(v) for v in
                         np.asarray(point["q"]) - np.append(sweep.gravity_sag(
                             np.asarray(point["q"][:6]), kp), 0.0)]}
               for point in trajectory["waypoints"]]
    edge_state.telemetry[plan_id] = {
        "format": "rebot-trajectory-log", "version": 2, "samples": samples,
        "kp": {str(m): float(kp[m - 1]) for m in range(1, 7)},
    }

    T_cam_base = extrinsics.invert(geometry["T_base_cam"])
    frame = image.copy()
    for pose in poses:
        q_meas = np.asarray(pose) - np.append(sweep.gravity_sag(
            np.asarray(pose[:6]), kp), 0.0)
        tool = rebot_ik.forward(q_meas[:6])
        wrist = tool[:3, 3]
        tip = wrist + tool[:3, :3] @ np.array([0.0, 0.0, -0.035])
        a = extrinsics.pixel(wrist, geometry["K"], geometry["dist"], T_cam_base)
        b = extrinsics.pixel(tip, geometry["K"], geometry["dist"], T_cam_base)
        if a and b:
            cv2.line(frame, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                     (0, 255, 0), 16)
    camera_mod.open_source = lambda url, **kw: SimpleNamespace(
        read=lambda: frame.copy(), close=lambda: None)

    sid = "20261118T102320Z-abc123"
    headers = {"X-Rebot-Token": "oracle-token", "X-Rebot-Session": sid}
    try:
        async with TestClient(TestServer(app)) as client:
            executed = await client.post("/execute", json={"id": plan_id},
                                         headers=headers)
            body = await executed.json()
            check("execute with the oracle still returns the gateway answer",
                  executed.status == 200 and body.get("executed") is True)
            pending = list(app["oracle_tasks"])
            check("the oracle verification runs as a background task", bool(pending))
            if pending:
                await asyncio.gather(*pending)

            doc_path = captures.parent / "oracle" / ("%s.json" % plan_id)
            check("the oracle doc is written", doc_path.is_file())
            doc = json.loads(doc_path.read_text(encoding="utf-8")) if doc_path.is_file() else {}
            holds = doc.get("holds") or []
            check("both holds were measured from telemetry",
                  len(holds) == 2 and all(h.get("q_meas") for h in holds)
                  and all(h.get("tip_fk_meas") for h in holds))
            check("hold error matches the injected sag",
                  holds and all(abs(h["hold_error_rad"][1]
                                      + sweep.gravity_sag(np.asarray(h["q_cmd"][:6]),
                                                          kp)[1]) < 1e-6
                                for h in holds))
            check("the verdict attributes the residual to sag",
                  (doc.get("verdict") or {}).get("layer") == "arm-motor"
                  and any(p["kind"] == "sag"
                          for p in (doc.get("verdict") or {}).get("problems", [])))
            check("hold frames and the pre/post stills are on disk",
                  (captures.parent / "oracle" / plan_id / "pre.jpg").is_file()
                  and (captures.parent / "oracle" / plan_id / "hold-1.jpg").is_file()
                  and (captures.parent / "oracle" / plan_id / "hold-2.jpg").is_file()
                  and (captures.parent / "oracle" / plan_id / "post.jpg").is_file())
            check("frames are content-addressed in the trace blobs",
                  doc.get("pre_frame")
                  and (traces / "blobs" / (doc["pre_frame"] + ".jpg")).is_file())
            events = [json.loads(line) for line in
                      (traces / sid / "events.jsonl").read_text(encoding="utf-8").splitlines()]
            check("an oracle trace event links the verdict to the session",
                  any(e["type"] == "oracle" and e["plan_id"] == plan_id
                      and (e.get("verdict") or {}).get("layer") == "arm-motor"
                      for e in events))
            check("the green tip detector finds the held tool",
                  all(h.get("detector") == "green" for h in holds)
                  and all(h.get("pixel_error_mm") is not None
                          and h["pixel_error_mm"] < 10.0 for h in holds),
                  str([(h.get("detector"), h.get("pixel_error_mm")) for h in holds]))

            # §8.4: no telemetry -> the doc still lands, /execute still passes.
            del edge_state.telemetry[plan_id]
            again = await client.post("/execute", json={"id": plan_id}, headers=headers)
            await asyncio.gather(*list(app["oracle_tasks"]))
            doc2 = json.loads(doc_path.read_text(encoding="utf-8"))
            check("missing telemetry degrades to a tracking-only doc",
                  again.status == 200 and doc2.get("telemetry") is None
                  and doc2.get("verdict") is not None)

            # A broken camera costs the evidence, never the execution.
            def broken_open(url, **kw):
                raise camera_mod.CameraError("no camera")
            camera_mod.open_source = broken_open
            third = await client.post("/execute", json={"id": plan_id}, headers=headers)
            check("a broken camera never fails /execute",
                  third.status == 200 and (await third.json()).get("executed") is True
                  and not app["oracle_tasks"])

            # A broken recorder: the oracle still writes its doc.
            camera_mod.open_source = lambda url, **kw: SimpleNamespace(
                read=lambda: frame.copy(), close=lambda: None)
            recorder = app["traces"]

            async def broken_event(*a, **k):
                raise RuntimeError("the disk is full")

            real_event = recorder.event
            recorder.event = broken_event
            try:
                fourth = await client.post("/execute", json={"id": plan_id},
                                           headers=headers)
                await asyncio.gather(*list(app["oracle_tasks"]))
            finally:
                recorder.event = real_event
            check("a broken recorder never fails /execute or the oracle doc",
                  fourth.status == 200 and doc_path.is_file())
    finally:
        camera_mod.open_source = real_open
        edge.shutdown()


# --------------------------------------------------------------------------- #
# 4e. Phase 3: the tape, the bundler, offline replay, export (§6, §9)
# --------------------------------------------------------------------------- #

def test_mock_tape(workdir: Path):
    print("\nmock arm tape mode: hit / miss / FIFO / fail_first precedence")
    import hashlib
    import urllib.error
    import urllib.request

    import mock_arm

    def call(base, method, path, body=None):
        data = json.dumps(body).encode() if body is not None else None
        request = urllib.request.Request(base + path, data=data, method=method)
        try:
            with urllib.request.urlopen(request, timeout=10) as answer:
                return answer.status, json.loads(answer.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    plan_body = {"targets": [{"tip": [0.45, 0.0, 0.16], "pixel": [640, 480]}],
                 "tip_offset_m": 0.035, "gripper": "alternate",
                 "options": {"tool_axis": "current", "yaw": 0.0, "min_z": 0.1}}
    plan_key = mock_arm.tape_key("POST", "/plan", plan_body)
    # The key must be capture_server.planner_call's computation, character for
    # character: same canonical JSON, same method|path|body framing.
    canonical = json.dumps(plan_body, sort_keys=True, ensure_ascii=False)
    expected = hashlib.sha256(("POST|/plan|%s" % canonical).encode("utf-8")).hexdigest()
    check("tape key matches planner_call's computation", plan_key == expected)

    pose_key = mock_arm.tape_key("GET", "/pose", {})
    tape = {"entries": [
        {"key": plan_key, "method": "POST", "path": "/plan", "request": plan_body,
         "status": 200, "response": {"ok": True, "id": "20261118T100030Z-aaaaaa",
                                     "summary": {"markers": 1, "waypoints": 660}}},
        {"key": pose_key, "method": "GET", "path": "/pose", "request": {},
         "status": 200, "response": {"ok": True, "pose": {"ids": [1, 2, 3, 4, 5, 6, 7],
                                                         "q": [0.1] * 7}}},
        {"key": pose_key, "method": "GET", "path": "/pose", "request": {},
         "status": 200, "response": {"ok": True, "pose": {"ids": [1, 2, 3, 4, 5, 6, 7],
                                                         "q": [0.2] * 7}}},
    ]}
    tape_path = workdir / "tape.json"
    tape_path.write_text(json.dumps(tape), encoding="utf-8")
    server, url, state = mock_arm.serve(0, tape=tape_path)
    try:
        status, body = call(url, "POST", "/plan", plan_body)
        check("tape hit returns the recorded status and body verbatim",
              status == 200 and body["id"] == "20261118T100030Z-aaaaaa"
              and body["summary"] == {"markers": 1, "waypoints": 660})
        first = call(url, "GET", "/pose")[1]
        second = call(url, "GET", "/pose")[1]
        check("same-key entries are a FIFO queue",
              first["pose"]["q"] == [0.1] * 7 and second["pose"]["q"] == [0.2] * 7)
        third = call(url, "GET", "/pose")[1]
        check("an exhausted queue falls back to synthetic and counts a miss",
              third["pose"]["q"] == [0.0] * 7 and state.tape_misses == 1)
        status, body = call(url, "POST", "/plan", {"targets": [{"tip": [0.5, 0.1, 0.2]}]})
        check("a key miss synthesizes and counts", status == 200 and body["ok"] is True
              and state.tape_misses == 2 and body["id"] != "20261118T100030Z-aaaaaa")
        snapshot = call(url, "GET", "/mock/state")[1]
        check("/mock/state exposes tape_misses", snapshot["tape_misses"] == 2
              and snapshot["tape_hits"] == 3)
        state.tape_reset()
        again = call(url, "GET", "/pose")[1]
        check("tape_reset rewinds the queue and counters",
              again["pose"]["q"] == [0.1] * 7 and state.tape_misses == 0)
    finally:
        server.shutdown()

    # fail_first keeps precedence: the injected refusal comes before the tape.
    server, url, state = mock_arm.serve(0, tape=tape, fail_first=1)
    try:
        status, body = call(url, "POST", "/plan", plan_body)
        check("fail_first refusal takes precedence over a tape hit",
              status == 422 and body["ok"] is False and state.tape_hits == 0)
        status, body = call(url, "POST", "/plan", plan_body)
        check("the tape answers once the injection is spent",
              status == 200 and body["id"] == "20261118T100030Z-aaaaaa")
    finally:
        server.shutdown()

    # No tape: today's synthetic behavior, unchanged, and no miss counting.
    server, url, state = mock_arm.serve(0)
    try:
        status, body = call(url, "POST", "/plan", plan_body)
        check("without a tape the mock is unchanged",
              status == 200 and body["ok"] is True and state.tape_misses == 0
              and state.tape_active is False)
    finally:
        server.shutdown()


def _phase3_still(workdir: Path, geometry):
    """The geometry fixture's frame as the JPEG a session would have uploaded."""
    ok, buffer = cv2.imencode(".jpg", geometry["image"], [cv2.IMWRITE_JPEG_QUALITY, 82])
    still = workdir / "phase3-still.jpg"
    still.write_bytes(buffer.tobytes())
    return still


def _phase3_rig(workdir: Path, geometry):
    """A rig file with the fixture's true pose -- no fit needed to replay it."""
    rig = extrinsics.Rig(T_base_cam=np.asarray(geometry["T_base_cam"], dtype=float),
                         calib_fingerprint="synthetic", image_size=(1600, 1200),
                         plane_z=0.0, tip_offset=np.array([0.0, 0.0, -0.035]),
                         device="synthetic", method="tip-sweep",
                         fitted="2026-11-18T10:00:00Z")
    path = workdir / "phase3-rig.json"
    extrinsics.save_rig(rig, path)
    return path, rig


def _phase3_calib(workdir: Path, geometry):
    path = workdir / "phase3-calib.json"
    path.write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": geometry["K"].tolist(),
        "dist": geometry["dist"].tolist(), "rms": 0.2,
    }), encoding="utf-8")
    return path


def _phase3_record(workdir, geometry, sid, extra_rig=False):
    """One recording pass: the harness's own tools against MockEdge, with the
    server taping every gateway call under the session id. Returns the merged
    page+server events, the manifest the server froze, and the plan id."""
    import base64

    import agent_eval
    import trace as trace_mod

    still = _phase3_still(workdir, geometry)
    rig_path, rig = _phase3_rig(workdir, geometry)
    calib_path = _phase3_calib(workdir, geometry)
    args = agent_eval.build_parser().parse_args([])
    args.calibration = str(calib_path)
    args.extrinsics = str(rig_path)
    args.plane_z, args.tip_mm, args.min_z, args.hover_m = 0.0, 35.0, 0.10, 0.10
    args.session = sid
    mock = agent_eval.MockEdge(args)
    # The server's manifest copies the calib it actually planned with.
    (mock.work / "calib").mkdir(parents=True, exist_ok=True)
    (mock.work / "calib" / "synthetic.json").write_bytes(calib_path.read_bytes())
    u, v = geometry["pixel"]
    sequence = [("rig", {}), ("pixel_to_arm", {"u": u, "v": v}),
                ("add_marker", {"u": u, "v": v}), ("move_arm", {"execute": False})]
    if extra_rig:
        sequence.insert(1, ("rig", {}))
    page = [{"type": "user_message", "text": "mark the target and plan the move",
             "frame": "data:image/jpeg;base64,"
                      + base64.b64encode(still.read_bytes()).decode("ascii")}]
    plan_id = ""
    try:
        session = agent_eval.Session(args, {"id": sid, "user": "mark and plan"}, still)
        for name, tool_args in sequence:
            result = session.call(name, tool_args)
            page.append({"type": "tool_call", "name": name, "args": tool_args})
            if name in ("pixel_to_arm", "add_marker"):
                # Page-only fields the harness cannot compute (web/arm-model.js):
                # the replay must list them as uncovered, never diff them.
                result = dict(result, reach_band_m=[0.15, 0.65], inside_band=True)
            if name == "move_arm":
                plan_id = result.get("plan_id") or ""
            page.append({"type": "tool_result", "name": name, "args": tool_args,
                         "result": result, "ms": 5})
        page.append({"type": "task_end", "final_text": "planned the move.",
                     "turns": 2, "exhausted": False})
        server_events = [json.loads(line) for line in
                         (mock.work / "traces" / sid / "events.jsonl")
                         .read_text(encoding="utf-8").splitlines()]
        manifest = json.loads((mock.work / "traces" / sid / "manifest.json")
                              .read_text(encoding="utf-8"))
        calib_bytes = (mock.work / "traces" / sid / "calib.json").read_bytes()
    finally:
        mock.close()
    # One merged log with explicit stamps: the window order is the truth, and
    # task_windows sorts by (t, src, pseq) exactly as the live merge does. The
    # gateway tape lands between the last tool result and task_end, where the
    # live server wrote it -- after task_end it would fall out of the window.
    merged = []
    pseq = 0
    ordered = page[:-1] + server_events + [page[-1]]
    for index, event in enumerate(ordered):
        if event.get("src") == "server" or "key" in event or event.get("type") == "plan":
            merged.append(dict(event, src="server",
                               t="2026-11-18T10:00:%02d.000Z" % index))
        else:
            pseq += 1
            merged.append(dict(event, src="page", pseq=pseq,
                               t="2026-11-18T10:00:%02d.000Z" % index))
    return {"merged": merged, "manifest": manifest, "plan_id": plan_id,
            "rig_path": rig_path, "rig": rig, "calib_bytes": calib_bytes,
            "still": still, "u": u, "v": v}


async def test_trace_bundle(workdir: Path, geometry):
    print("\ntrace bundle: freeze, completeness gate, export, deterministic replay")
    import hashlib

    import agent_eval
    import trace as trace_mod
    import trace_bundle
    import trace_triage

    sid = "20261118T102320Z-abc123"
    recorded = _phase3_record(workdir, geometry, sid)
    traces = workdir / "bundle-state" / "traces"
    state = traces.parent
    recorder = trace_mod.TraceRecorder(traces)
    await recorder.events(sid, recorded["merged"])
    labels = trace_triage.label_session(recorded["merged"], sid, version="test-version")
    (traces / sid / "labels.jsonl").write_text(
        "".join(json.dumps(label) + "\n" for label in labels), encoding="utf-8")
    # The frozen geometry beside the manifest, as trace_manifest writes it live.
    (traces / sid / "rig.json").write_bytes(recorded["rig_path"].read_bytes())
    (traces / sid / "calib.json").write_bytes(recorded["calib_bytes"])
    manifest = dict(recorded["manifest"], rig_file="rig.json",
                    rig={"fingerprint": recorded["rig"].fingerprint},
                    tool_schemas_sha256=hashlib.sha256(json.dumps(
                        agent_eval.TOOLS, sort_keys=True,
                        ensure_ascii=False).encode("utf-8")).hexdigest())
    (traces / sid / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (state / "out").mkdir(parents=True, exist_ok=True)
    (state / "out" / (recorded["plan_id"] + ".json")).write_text(
        json.dumps({"waypoints": []}), encoding="utf-8")

    result = trace_bundle.bundle_session(traces, sid)
    bundle = result["dir"]
    check("bundler freezes a complete bundle", result["complete"],
          "; ".join(result["reasons"]))
    check("bundle carries the window, tape, stills and frozen geometry",
          (bundle / "events.jsonl").is_file() and (bundle / "tape.json").is_file()
          and (bundle / "calib.json").is_file() and (bundle / "rig.json").is_file()
          and len(list((bundle / "stills").glob("*.jpg"))) == 1
          and len(json.loads((bundle / "tape.json").read_text())["entries"]) == 1
          and (bundle / "trajectories" / (recorded["plan_id"] + ".json")).is_file())

    # The gate: a missing blob or rig is an incomplete bundle, not an export.
    written = trace_bundle.read_events(traces / sid / "events.jsonl")
    sha = next(iter(set().union(*(trace_bundle.blob_refs(e) for e in written))))
    blob = traces / "blobs" / (sha + ".jpg")
    saved = blob.read_bytes()
    blob.unlink()
    broken = trace_bundle.bundle_session(traces, sid)
    check("a missing blob marks the bundle incomplete",
          not broken["complete"] and any("does not resolve" in r for r in broken["reasons"]))
    check("an incomplete bundle is not exported",
          trace_bundle.export_bundle(broken["dir"], workdir / "no-export") is None
          and not (workdir / "no-export").exists())
    blob.write_bytes(saved)
    rig_copy = traces / sid / "rig.json"
    rig_bytes = rig_copy.read_bytes()
    rig_copy.unlink()
    broken = trace_bundle.bundle_session(traces, sid)
    check("a missing rig marks the bundle incomplete",
          not broken["complete"] and any("rig.json" in r for r in broken["reasons"]))
    rig_copy.write_bytes(rig_bytes)
    trace_bundle.bundle_session(traces, sid)

    # Export -> load_case round-trip (§6.4).
    export_root = workdir / "plain-export"
    target = trace_bundle.export_bundle(bundle, export_root)
    case = agent_eval.load_case(target / "case.json")
    still = agent_eval.still_for(case)
    check("export round-trips through load_case/still_for",
          target is not None and still.is_file()
          and still.parent == (target / "stills").resolve()
          and case["needs_mock_edge"] is True and case["tape"] == "tape.json"
          and (target / "report.md").is_file() and (target / "trace.jsonl").is_file())

    # THE core regression (§9): unchanged code replays bit-exact ...
    clean_report = workdir / "replay-clean.json"
    code = agent_eval.main(["--trace", str(bundle), "--replay",
                            "--json", str(clean_report), "--quiet"])
    report = json.loads(clean_report.read_text(encoding="utf-8"))
    check("replay against unchanged code is bit-exact",
          code == 0 and report["passed"] is True and report["diffs"] == []
          and report["tape_misses"] == 0, "%d diffs" % len(report["diffs"]))
    check("page-only fields are uncovered, not diffed",
          "pixel_to_arm.reach_band_m" in report["uncovered"]
          and "add_marker.inside_band" in report["uncovered"])

    # ... and a perturbed geometry constant must FAIL with geometry-diff.
    original_init = agent_eval.Session.__init__

    def perturbed_init(self, *a, **k):
        original_init(self, *a, **k)
        self.plane_z += 0.02        # 2 cm of worktop: every tip z moves

    agent_eval.Session.__init__ = perturbed_init
    try:
        bad_report = workdir / "replay-perturbed.json"
        code = agent_eval.main(["--trace", str(bundle), "--replay",
                                "--json", str(bad_report), "--quiet"])
        report = json.loads(bad_report.read_text(encoding="utf-8"))
    finally:
        agent_eval.Session.__init__ = original_init
    classes = {diff["class"] for diff in report["diffs"]}
    check("a perturbed plane_z fails the replay with geometry-diff",
          code == 1 and report["passed"] is False and "geometry-diff" in classes,
          "classes: %s" % sorted(classes))
    check("the diff names the numbers that moved",
          any(diff["field"] == "plane_z" and diff["tool"] == "rig"
              for diff in report["diffs"])
          and any(diff["field"].startswith("tip_m") for diff in report["diffs"]))

    # A geometry failure: recorded oracle residual, frozen rig, full evidence.
    sid2 = "20261118T102321Z-def456"
    recorded2 = _phase3_record(workdir, geometry, sid2, extra_rig=True)
    traces2 = workdir / "geo-state" / "traces"
    state2 = traces2.parent
    recorder2 = trace_mod.TraceRecorder(traces2)
    await recorder2.events(sid2, recorded2["merged"])
    q = list(sweep.POSES[0]) + [0.0]
    tool = rebot_ik.forward(np.asarray(q[:6], dtype=float))
    tip = tool[:3, :3] @ np.array([0.0, 0.0, -0.035]) + tool[:3, 3]
    predicted = locate.project(geometry["K"], geometry["T_base_cam"], tip,
                               geometry["dist"])
    detected = [predicted[0] + 80.0, predicted[1]]
    point, _d, _g = extrinsics.ray_plane(geometry["K"], geometry["dist"],
                                         geometry["T_base_cam"], detected[0],
                                         detected[1], float(tip[2]))
    error_mm = 1000.0 * float(np.linalg.norm(np.asarray(point) - tip))
    plan_id2 = recorded2["plan_id"]
    oracle_doc = {"plan_id": plan_id2, "holds": [{
        "index": 1, "q_cmd": q, "q_meas": q, "hold_error_rad": [0.0] * 7,
        "tip_error_mm": 1.0, "pixel_error_px": 80.0, "pixel_error_mm": round(error_mm, 2),
        "predicted_tip_px": list(predicted), "detected_tip_px": detected}]}
    labels2 = trace_triage.label_session(recorded2["merged"], sid2,
                                         oracle_docs={plan_id2: oracle_doc},
                                         version="test-version")
    check("the canned geometry session labels as a camera-arm failure",
          labels2[0]["verdict"] == "fail" and labels2[0]["class"] == "geometry"
          and labels2[0]["layer"] == "camera-arm" and error_mm > 10.0,
          "%.1f mm" % error_mm)
    (traces2 / sid2 / "labels.jsonl").write_text(
        "".join(json.dumps(label) + "\n" for label in labels2), encoding="utf-8")
    (traces2 / sid2 / "rig.json").write_bytes(recorded2["rig_path"].read_bytes())
    (traces2 / sid2 / "calib.json").write_bytes(recorded2["calib_bytes"])
    manifest2 = dict(recorded2["manifest"], rig_file="rig.json",
                     rig={"fingerprint": recorded2["rig"].fingerprint},
                     tool_schemas_sha256=manifest["tool_schemas_sha256"])
    (traces2 / sid2 / "manifest.json").write_text(json.dumps(manifest2), encoding="utf-8")
    (state2 / "oracle").mkdir(parents=True, exist_ok=True)
    oracle_path = state2 / "oracle" / (plan_id2 + ".json")
    oracle_path.write_text(json.dumps(oracle_doc), encoding="utf-8")

    geo = trace_bundle.bundle_session(traces2, sid2)
    check("a geometry bundle with its oracle doc is complete", geo["complete"],
          "; ".join(geo["reasons"]))
    oracle_path.unlink()
    no_oracle = trace_bundle.bundle_session(traces2, sid2)
    check("a geometry failure without an oracle doc is incomplete",
          not no_oracle["complete"]
          and any("oracle" in r for r in no_oracle["reasons"]))
    oracle_path.write_text(json.dumps(oracle_doc), encoding="utf-8")
    geo = trace_bundle.bundle_session(traces2, sid2)
    geo_export = trace_bundle.export_bundle(geo["dir"], workdir / "geo-export")
    geo_case = json.loads((geo_export / "case.json").read_text(encoding="utf-8"))
    check("geometry export derives the oracle-recompute expect",
          geo_case["expect"].get("geometry_check") is True
          and (geo_export / "oracle.json").is_file()
          and (geo_export / "calib" / "rig.json").is_file())

    # Dedup (§6.5): same signature across sessions -> one canonical, N occurrences.
    groups = trace_bundle.dedup(traces2)
    check("dedup groups by signature with occurrences",
          len(groups) == 1 and next(iter(groups.values()))["occurrences"] == 1
          and (traces2 / "dedup.json").is_file())

    return {"geo_bundle": geo["dir"], "geo_export": geo_export, "sid2": sid2,
            "plan_id": plan_id2, "oracle_doc": oracle_doc,
            "u": recorded2["u"], "v": recorded2["v"],
            "rig_tip_offset": [0.0, 0.0, -0.035]}


def test_trace_live(workdir: Path, geometry, ctx):
    print("\ntrace live replay: expect derivation, classification, minimization, graduation")
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    import agent_eval
    import trace_bundle

    # expect{} derivation and classification are pure: drive them directly.
    protocol_label = {"class": "protocol", "problems": [
        {"kind": "repeated-tool-error"}, {"kind": "claims-move-not-executed"}]}
    expect = trace_bundle.derive_expect(protocol_label, [], {})
    check("protocol expect translates the retry and honesty rules",
          expect.get("plan_attempts_min") == 2 and expect.get("honest") is True
          and expect.get("plan_first") is True)
    perception = trace_bundle.derive_expect({"class": "perception", "problems": []}, [], {})
    check("perception expect is the scaffold-style zero-box case",
          perception.get("must_mark") == ["object"] and perception.get("plan_first") is True)
    check("classification: stable fail, flaky, pass",
          agent_eval.classify_runs([{"problems": ["x 1 mm"]}, {"problems": ["x 2 mm"]}])
          == "stable-fail"
          and agent_eval.classify_runs([{"problems": ["x"]}, {"problems": []}]) == "flaky"
          and agent_eval.classify_runs([{"problems": []}]) == "pass")

    # Minimization (§6.5): drop the redundant rig turn while the perturbed
    # replay still reproduces the same geometry-diff signature.
    bundle = ctx["geo_bundle"]
    original_init = agent_eval.Session.__init__

    def perturbed_init(self, *a, **k):
        original_init(self, *a, **k)
        self.plane_z += 0.02

    def replay_report(candidate):
        out = candidate / "min-replay.json"
        code = agent_eval.main(["--trace", str(candidate), "--replay",
                                "--json", str(out), "--quiet"])
        report = json.loads(out.read_text(encoding="utf-8"))
        return code, report

    def signature(report):
        # Which numbers moved, not how often: a redundant turn repeats a diff.
        return sorted({(d["tool"], d["field"], d["class"]) for d in report["diffs"]})

    agent_eval.Session.__init__ = perturbed_init
    try:
        _code, full = replay_report(bundle)
        wanted = signature(full)
        check("the perturbed geometry bundle reproduces before minimizing",
              _code == 1 and any(c == "geometry-diff" for _t, _f, c in wanted))

        def reproduces(candidate):
            code, report = replay_report(candidate)
            return code == 1 and signature(report) == wanted

        before = len(trace_bundle.read_events(bundle / "events.jsonl"))
        outcome = trace_bundle.minimize_bundle(bundle, reproduces)
        after = len(trace_bundle.read_events(bundle / "events.jsonl"))
        code, report = replay_report(bundle)
        check("delta-debug drops turns and keeps the failure signature",
              outcome["minimized"] and after < before and code == 1
              and signature(report) == wanted,
              "%d -> %d events" % (before, after))
    finally:
        agent_eval.Session.__init__ = original_init

    # A scripted OpenAI-compatible endpoint (the Phase 1 fake-model pattern):
    # turn 1 marks and plans, turn 2 finishes. No real model anywhere.
    class FakeModel(BaseHTTPRequestHandler):
        script = [
            {"content": "marking and planning.", "tool_calls": [
                ("add_marker", json.dumps({"u": ctx["u"], "v": ctx["v"]})),
                ("move_arm", json.dumps({"execute": False}))]},
            {"content": "done: the plan is ready."},
        ]
        seen = []

        def log_message(self, fmt, *args):  # noqa: A002 - stdlib signature
            pass

        def do_POST(self):  # noqa: N802 - stdlib signature
            length = int(self.headers.get("Content-Length") or 0)
            json.loads(self.rfile.read(length) or b"{}")
            FakeModel.seen.append(1)
            # The script repeats per conversation: every run is seeded identically.
            step = FakeModel.script[(len(FakeModel.seen) - 1) % len(FakeModel.script)]
            message = {"role": "assistant", "content": step.get("content", "")}
            calls = step.get("tool_calls") or []
            if calls:
                message["tool_calls"] = [
                    {"id": "call_%d" % i, "type": "function",
                     "function": {"name": name, "arguments": arguments}}
                    for i, (name, arguments) in enumerate(calls)]
            raw = json.dumps({"choices": [{"message": message,
                                           "finish_reason": "tool_calls" if calls else "stop"}]}
                             ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

    server = ThreadingHTTPServer(("127.0.0.1", 0), FakeModel)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    llm_url = "http://127.0.0.1:%d/v1/chat/completions" % server.server_address[1]
    try:
        # Live-model replay x2: the oracle residual reproduces every run, so
        # the classification is stable-fail and the case exports (§6.3b).
        live_report = workdir / "trace-live.json"
        code = agent_eval.main(["--trace", str(bundle), "--mock-edge", "--runs", "2",
                                "--llm-url", llm_url, "--model", "stub-model",
                                "--json", str(live_report), "--quiet"])
        report = json.loads(live_report.read_text(encoding="utf-8"))
        check("live-model replay classifies a stable fail and exports",
              code == 1 and report["classification"] == "stable-fail"
              and report["exported"] and Path(report["exported"]).is_dir()
              and len(FakeModel.seen) == 4,
              report["classification"])
        check("the live runs used the recorded prompt and bundle frames",
              all(problem and "geometry residual" in problem[0]
                  for problem in report["problems"]))

        # Graduation (§6.4): the exported case runs under plain --mock-edge.
        FakeModel.seen = []
        case = agent_eval.load_case(ctx["geo_export"] / "case.json")
        still = agent_eval.still_for(case)
        args = agent_eval.build_parser().parse_args(
            ["--mock-edge", "--llm-url", llm_url, "--model", "stub-model", "--runs", "1"])
        mock = agent_eval.MockEdge(args)
        args._mock = mock
        try:
            run = agent_eval.run_once(args, case, still)
            problems = agent_eval.judge(args, case, run)
        finally:
            mock.close()
        check("a graduated case runs under plain --mock-edge",
              run["turns"] >= 1 and any("geometry residual" in p for p in problems),
              "%d problem(s)" % len(problems))
        check("the exported report carries versions and repro commands",
              "git commit" in (ctx["geo_export"] / "report.md").read_text(encoding="utf-8")
              and "--replay" in (ctx["geo_export"] / "report.md").read_text(encoding="utf-8"))
    finally:
        server.shutdown()


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
# 5b. Landscape: the same intrinsics, turned back instead of recalibrated
# --------------------------------------------------------------------------- #

def test_rotation(workdir: Path, geometry):
    """A frame delivered a quarter turn round must still solve with one K."""
    print("\nlandscape: the same intrinsics, turned back")
    K, dist = geometry["K"], geometry["dist"]
    upright = geometry["image"]
    truth = geometry["point"]
    size = (upright.shape[1], upright.shape[0])          # 1600x1200
    turned_size = (size[1], size[0])                     # 1200x1600

    calib_path = workdir / "rotation-calib.json"
    calib_path.write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": list(size), "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
        "orientation_deg": 0,
    }), encoding="utf-8")

    rotated = cv2.rotate(upright, cv2.ROTATE_90_CLOCKWISE)
    tap = calib_mod.map_pixel(geometry["pixel"][0], geometry["pixel"][1], size, 1)
    capture = workdir / "captures" / "20260101T120001Z-rot123"
    capture.mkdir(parents=True)
    cv2.imwrite(str(capture / "frame.jpg"), rotated, [cv2.IMWRITE_JPEG_QUALITY, 95])
    (capture / "meta.json").write_text(json.dumps({
        "purpose": "plan",
        "image_size": list(turned_size),
        "tap": {"u": tap[0], "v": tap[1]},
        "settings": {"deviceId": "synthetic"},
    }), encoding="utf-8")

    options = locate.Options(tag_size=0.080, tag_x=0.36, tag_y=-0.10, tag_z=0.12,
                             tag_yaw=180.0, plane_z=0.0)
    target = locate.locate_capture(capture, calib_path, options)
    point = np.array(target["point"])
    error = float(np.linalg.norm(point - truth))
    check("a frame that arrived one turn clockwise needs three to go back",
          target.get("turns") == 3, "turns %s" % target.get("turns"))
    check("it is solved with the portrait K, not rescaled",
          list(target.get("image_size") or []) == list(size),
          str(target.get("image_size")))
    check("the turned tap lands within 3 mm of the truth", error < 0.003,
          "%.2f mm" % (error * 1000))

    # One number, one meaning: `turns` is the correction, and applying it to the
    # tap recovers the upright pixel. Its composition with 4 - turns is identity
    # for every turn, which is the algebra the whole pipeline rests on.
    back = calib_mod.map_pixel(tap[0], tap[1], turned_size, 3)
    check("turning the tap back recovers it",
          abs(back[0] - geometry["pixel"][0]) < 1e-9
          and abs(back[1] - geometry["pixel"][1]) < 1e-9,
          "(%.1f, %.1f) vs (%.1f, %.1f)"
          % (back[0], back[1], geometry["pixel"][0], geometry["pixel"][1]))
    check("the payload reports the correction, not the measurement",
          target.get("delivered_size") == list(turned_size)
          and target.get("turns") == 3 and "rotation" not in target,
          "%s, turns %s" % (target.get("delivered_size"), target.get("turns")))

    # t and 4 - t are inverses, in pixels and in pixels-on-disk.
    identity = True
    for turn in range(4):
        there = calib_mod.map_pixel(10.0, 20.0, (640, 480), turn)
        there_size = (480, 640) if turn % 2 else (640, 480)
        again = calib_mod.map_pixel(there[0], there[1], there_size, (4 - turn) % 4)
        identity = identity and abs(again[0] - 10.0) < 1e-9 and abs(again[1] - 20.0) < 1e-9
        back_image = locate.rotate_image(locate.rotate_image(rotated, turn), (4 - turn) % 4)
        identity = identity and np.array_equal(back_image, rotated)
    check("t and 4 - t are inverses for every turn", identity)

    # The Rig modal's one-pose path: the still is landscape, and the tag must
    # still seed the rig. Loading K at the delivered size is what used to fail.
    try:
        calib_mod.load(calib_path, image_size=turned_size)
        check("K is not rescaled onto a turned frame", False, "it scaled anyway")
    except calib_mod.CalibError as error:
        check("K is not rescaled onto a turned frame",
              "quarter turn" in str(error), str(error)[:60])

    upright, turned_calib, turn = locate.upright_frame(calib_path, rotated)
    # Pixel-exact: turning the wrong way round would give a size-correct frame
    # that is 180 degrees out, which the pose tests above cannot see with a
    # symmetric camera. This can.
    check("the still is turned back to the calibration's grid",
          turn == 3 and turned_calib.image_size == size
          and np.array_equal(upright, geometry["image"]),
          "turns %s, size %s, identical %s"
          % (turn, turned_calib.image_size, np.array_equal(upright, geometry["image"])))
    marker = base_marker.parse_spec("%g,%g,%g,%g" % (0.36, -0.10, 0.12, 180.0), 0.080)
    marker["id"] = 3            # the id the synthetic tag was rendered with
    run = auto_calibrate.solve_marker(upright, turned_calib, marker, plane_z=0.0)
    position = float(np.linalg.norm(
        np.asarray(run.rig.T_base_cam)[:3, 3] - geometry["T_base_cam"][:3, 3]))
    check("a landscape still still seeds the rig", position < 0.006,
          "%.2f mm" % (position * 1000))

    # The direction must not be guessed: the host measures it from the tag, and
    # this frame arrived one turn clockwise, so the correction is 3 -- hundreds
    # of pixels separate it from the shape's guess of 1.
    still = workdir / "rotation-still.jpg"
    cv2.imwrite(str(still), rotated, [cv2.IMWRITE_JPEG_QUALITY, 95])
    import capture_server  # imported here: the server tests import it on demand too
    app = {"camera": str(still), "marker": marker, "rig": run.rig}
    measured = capture_server.measure_stream_turn(app, turned_calib, frame=rotated)
    check("the host measures the turn from the tag",
          measured is not None and measured["turns"] == 3
          and measured["error_px"] < 5.0,
          None if measured is None else
          "turns %s, tag error %.1f px" % (measured["turns"], measured["error_px"]))

    # A frame of another aspect ratio that is not a quarter turn is still refused.
    awkward = workdir / "captures" / "20260101T120002Z-bad001"
    awkward.mkdir(parents=True)
    cv2.imwrite(str(awkward / "frame.jpg"), cv2.resize(upright, (800, 900)))
    (awkward / "meta.json").write_text(json.dumps({
        "purpose": "plan", "image_size": [800, 900], "tap": {"u": 10, "v": 10},
    }), encoding="utf-8")
    try:
        locate.locate_capture(awkward, calib_path, options)
        check("an unrelated aspect ratio is refused", False, "it solved anyway")
    except calib_mod.CalibError as error:
        check("an unrelated aspect ratio is refused", "quarter turn" in str(error),
              str(error)[:70])


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


def render_marker(image, T_base_cam, K, dist, at, yaw_deg, size_m, mode="vertical"):
    """Draw the tag where `T_base_cam` says it is: a photo without a camera."""
    T_base_marker = base_marker.base_from_marker(at, math.radians(yaw_deg), mode=mode)
    T_cam_marker = extrinsics.invert(np.asarray(T_base_cam, dtype=float)) @ T_base_marker
    rvec, _ = cv2.Rodrigues(T_cam_marker[:3, :3])
    projected, _ = cv2.projectPoints(base_marker.object_points(size_m), rvec,
                                     T_cam_marker[:3, 3], K, dist)
    quad = projected.reshape(4, 2).astype(np.float32)
    bitmap = base_marker.marker_bitmap(size_m, 1200.0)
    source = np.array([[0, 0], [bitmap.shape[1] - 1, 0],
                       [bitmap.shape[1] - 1, bitmap.shape[0] - 1],
                       [0, bitmap.shape[0] - 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, quad)
    height, width = image.shape[:2]
    warped = cv2.warpPerspective(cv2.cvtColor(bitmap, cv2.COLOR_GRAY2BGR), homography,
                                 (width, height))
    mask = cv2.warpPerspective(np.full(bitmap.shape, 255, np.uint8), homography,
                               (width, height))
    image[mask > 0] = warped[mask > 0]
    return image


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
    evidence = base_marker.solve(warped, K, dist, size_m=size_m)
    angle_error = float(np.degrees(np.arccos(np.clip(
        (np.trace(solved[:3, :3].T @ rotation) - 1.0) / 2.0, -1.0, 1.0))))
    distance_error = float(np.linalg.norm(solved[:3, 3] - T_cam_marker[:3, 3]))
    # A 5 cm marker at a metre is ~50 px, so a half-pixel of corner noise is ~1%
    # of depth -- about a centimetre. That is ample: this is a *seed*, and the
    # tip detector needs a few centimetres.
    check("the marker's pose is recovered from one frame",
          angle_error < 1.5 and distance_error < 0.025,
          "%.2f deg, %.1f mm" % (angle_error, 1000.0 * distance_error))
    check("the tag solve reports the evidence it was judged on",
          evidence["rms_px"] < 1.0 and evidence["side_px"] > 20.0
          and 0.0 <= evidence["grazing_deg"] < 80.0,
          "%.2f px rms, %.0f px side, %.0f deg"
          % (evidence["rms_px"], evidence["side_px"], evidence["grazing_deg"]))

    recovered = base_marker.camera_pose(warped, K, dist, at, yaw, size_m=size_m)
    camera_error = float(np.linalg.norm(recovered[:3, 3] - T_base_cam[:3, 3]))
    # The composition inherits the seed's error (16 mm of depth plus the ~1 deg
    # rotation's lever arm -> ~23 mm here). The bound is metres, like the
    # quantity: it catches a gross composition failure -- a mirror-branch solve
    # is metres off -- not the seed's pixel-quantization floor.
    check("the composition gives the camera in the base frame", camera_error < 0.030,
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
            hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics=str(rig_file), witness_max_px=2.0, camera="",
            traces_dir=str(workdir / "rig-server-traces"),
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
            check("a failed plan still returns the solved frame",
                  (answer.get("target") or {}).get("T_base_cam") is not None)

            reach = await (await client.get("/reach.json")).json()
            check("reach.json carries the fixed rig's pose",
                  reach.get("mode") == "fixed"
                  and np.asarray(reach.get("T_base_cam"), dtype=float).shape == (4, 4))
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
        self.poses = 0
        self.visited = []

    def pose(self):
        self.poses += 1
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


def test_marker_still_commit_flag(workdir: Path):
    print("\ntag solve: the oracle's pre-still is evidence, not a calibration")
    import capture_server as cs
    import auto_calibrate as ac
    app = {"calibrating": False, "marker": {"at": [0.0, 0.0, 0.0]}, "plane_z": 0.0,
           "calib": workdir, "captures": workdir / "caps"}
    frame = np.zeros((100, 100, 3), np.uint8)
    run = SimpleNamespace(
        rig=SimpleNamespace(mount="fixed", describe=lambda: "camera line",
                            provenance=lambda: {}),
        verdict={"ok": True}, evidence={}, seconds=0.0, warnings=[])
    committed = []
    saved = (cs.resolve_calibration, cs.stream_turn, cs.locator.upright_frame,
             cs.camera_mod.open_source, cs.calib_mod.load, ac.solve_marker, ac.commit_marker)
    cs.resolve_calibration = lambda app, requested=None: workdir / "calib.json"
    cs.stream_turn = lambda app, base: 0
    cs.calib_mod.load = lambda path, **k: SimpleNamespace(K=None, dist=None)
    cs.locator.upright_frame = lambda path, f, turn=None: (f, "CAL", 0)
    cs.camera_mod.open_source = lambda *a, **k: SimpleNamespace(
        read=lambda: frame, close=lambda: None)
    ac.solve_marker = lambda *a, **k: run
    ac.commit_marker = lambda *a, **k: (committed.append(1),
                                        (run.rig, run.verdict,
                                         workdir / "extrinsics-fixed.json"))[1]
    try:
        rep = cs.marker_from_still(app, frame, commit=False)
        check("a non-committing tag solve reports but does not write",
              rep["written"] is None and not committed
              and rep["marker"]["detected"] is True)
        payload = cs.grab_one(app, "rtsp://fake", commit=False)
        check("grab_one passes the non-committing flag through",
              payload.get("written") is None and not committed
              and payload.get("ok") is True)
        rep = cs.marker_from_still(app, frame)
        check("the default tag solve still commits",
              rep["written"] is not None and len(committed) == 1)
    finally:
        (cs.resolve_calibration, cs.stream_turn, cs.locator.upright_frame,
         cs.camera_mod.open_source, cs.calib_mod.load, ac.solve_marker, ac.commit_marker) = saved


def test_calib_dir_scanners(workdir: Path):
    print("\ncalib dir: the learned maps are not intrinsics calibrations")
    import agent_eval
    import capture_server
    import calibrate_extrinsics as extrinsics_cli
    state = workdir / "scan-state"
    folder = state / "calib"
    folder.mkdir(parents=True)
    device = folder / "4f68b7f34a702b1c043f44fcf805ac9a63e2a3d9.json"
    device.write_text("{}", encoding="utf-8")
    for name in ("extrinsics-fixed.json", "sag-5d9b446edf53.json", "accuracy-map.json"):
        (folder / name).write_text("{}", encoding="utf-8")
    app = {"calib": folder, "calib_file": ""}
    check("resolve_calibration skips the rig and the learned maps",
          capture_server.resolve_calibration(app) == device,
          str(capture_server.resolve_calibration(app)))
    check("the CLI's default calib skips them too",
          extrinsics_cli.default_calib(folder) == device)
    # The eval harness has its own scanner: it missed the sag-*/accuracy-map
    # exclusion once already, and picked a sag map as the intrinsics.
    saved_state = agent_eval.STATE
    agent_eval.STATE = state
    try:
        check("the eval harness's default calib skips them too",
              agent_eval.default_calibration() == device,
              str(agent_eval.default_calibration()))
    finally:
        agent_eval.STATE = saved_state


def test_hold_detection_gappy_log(workdir: Path):
    print("\nhold detection: a slow pose read must not shatter a hold")
    pose = np.asarray(sweep.POSES[0], dtype=float)

    def still_run(start_t, q, n=23):
        out = []
        t = start_t
        for _ in range(n):
            out.append({"i": len(out), "t": t, "q": list(q)})
            t += 1.0 / 15.0
        return out, t

    # A 3 s hold with a one-second /pose gap in the middle (the bus was busy
    # playing back); the arm drifted 20 mrad across the gap -- 0.02 rad/s,
    # well under the stillness speed. The hold must survive as one window.
    entries, t = still_run(1000.0, pose)
    drifted = pose + 0.02
    entries.append({"i": len(entries), "t": t + 1.0, "q": list(drifted)})
    more, _ = still_run(t + 1.0 + 1.0 / 15.0, drifted)
    entries.extend(more)
    chosen = sweep_video.select(entries, [pose])
    check("a hold survives a one-second pose-log gap",
          chosen[0].get("missed") is not True and chosen[0]["window_s"] >= 2.5,
          str(chosen[0]))

    # The same gap with real motion across it (0.3 rad/s) is not stillness:
    # the window must end before the gap, not bridge it.
    entries, t = still_run(1000.0, pose)
    entries.append({"i": len(entries), "t": t + 1.0, "q": list(pose + 0.3)})
    more, _ = still_run(t + 1.0 + 1.0 / 15.0, pose + 0.3)
    entries.extend(more)
    chosen = sweep_video.select(entries, [pose])
    check("motion across a gap does not extend the hold",
          chosen[0].get("missed") is not True and chosen[0]["window_s"] < 2.0,
          str(chosen[0]))

    # A gap longer than the cap hides too much: even with no drift at all it
    # cannot be claimed as evidence the arm stood still through it.
    entries, t = still_run(1000.0, pose)
    entries.append({"i": len(entries), "t": t + 3.0, "q": list(pose)})
    more, _ = still_run(t + 3.0 + 1.0 / 15.0, pose)
    entries.extend(more)
    chosen = sweep_video.select(entries, [pose])
    check("a gap over the cap is not evidence of stillness",
          chosen[0].get("missed") is not True and chosen[0]["window_s"] < 2.0,
          str(chosen[0]))


def test_unpark_peel(workdir: Path):
    print("\nstartup peel: lift the elbow off the rest stops before driving the wrist")
    rest = np.zeros(6)
    lifted = sweep.unpark_pose(rest)
    check("the peel lifts j3 by 8 deg from rest",
          abs(lifted[2] - math.radians(sweep.UNPARK_J3_DEG)) < 1e-9)
    check("the peel only changes j3",
          np.allclose(np.delete(lifted, 2), np.delete(rest, 2)))
    already = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0])
    check("a pose already off the stops is unchanged",
          np.allclose(sweep.unpark_pose(already), already))
    check("the peel is clamped to j3's upper limit",
          sweep.unpark_pose(rest)[2] <= rebot_ik.LIMITS[2, 1])

    # The sweep matches holds to poses by nearest, so the extra peel hold window
    # (j3 ~ 0.14, nowhere near a sweep pose) goes unclaimed and every pose still
    # extracts. Pin that against a future switch to index-based matching.
    pose = np.asarray(sweep.POSES[0], dtype=float)
    peel_q = sweep.unpark_pose(np.zeros(6))
    entries = []
    t = 0.0

    def hold(q, n=30):
        nonlocal t
        for _ in range(n):
            entries.append({"i": len(entries), "t": t, "q": list(q)})
            t += 1.0 / 15.0

    hold(peel_q)                                   # the peel hold comes first
    for k in range(1, 10):                         # transit to the pose
        entries.append({"i": len(entries), "t": t,
                        "q": list(peel_q + (pose - peel_q) * k / 10.0)})
        t += 1.0 / 15.0
    hold(pose)                                     # the measurement pose hold
    chosen = sweep_video.select(entries, [pose])
    check("the sweep ignores the peel hold and still finds the pose",
          chosen[0].get("missed") is not True
          and abs(chosen[0]["q"][2] - pose[2]) < 0.05, str(chosen[0]))


def test_auto_sweep_turned_stream(workdir: Path):
    print("\nautomatic sweep: a turned stream is turned back before the fit")
    K, dist, T_true, _tip, _observations = synthetic_rig()
    calibration = calib_mod.Calib(path=workdir / "turned-calib.json", K=K, dist=dist,
                                  image_size=(1600, 1200), device="synthetic")
    background = cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR)
    q0 = np.array([0.4, 1.6, 0.9, 0.2, 1.1, 0.3])
    tap = extrinsics.pixel(rebot_ik.forward(q0)[:3, 3], K, dist, extrinsics.invert(T_true))

    arm = FakeArm()
    upright = FakeEye(background, arm, T_true, K, dist)

    class TurnedEye:
        """The phone delivers the scene half a turn out; the recorder writes it
        as delivered, exactly like MediaMTX records the raw stream."""

        def read(self, retries=1):
            return cv2.rotate(upright.read(), cv2.ROTATE_180)

        def close(self):
            upright.close()

    eye = TurnedEye()
    options = auto_calibrate.Options(
        settle_s=0.0, plane_z=0.0, click={"q": q0.tolist(), "uv": tap},
        detect=auto_calibrate.DetectOptions(walk=False))
    fake = FakeRecorder(eye, workdir / "turned-recordings")
    result = auto_calibrate.run(arm, eye, calibration, options, workdir / "turned-captures",
                                recorder=fake, turn=2)
    check("the turned sweep captured every pose", len(result.captures) == len(sweep.POSES))
    check("the turned sweep still finds the tip in every pose",
          all(d.offset_px < options.detect.max_offset_px for d in result.detections),
          "worst %.0f px" % max(d.offset_px for d in result.detections))
    rig = result.fit.rig
    rotation_error = extrinsics.rotation_deg(rig.T_base_cam[:3, :3] @ T_true[:3, :3].T)
    position_error = float(np.linalg.norm(rig.T_base_cam[:3, 3] - T_true[:3, 3]))
    check("the turned-stream fit lands within 4 deg", rotation_error < 4.0,
          "%.3f deg" % rotation_error)
    check("the turned-stream fit lands within 80 mm", position_error < 0.080,
          "%.1f mm" % (1000 * position_error))

    # The stills kept as evidence must be on the calibration grid too, not the
    # delivered one: re-render the first pose and compare both orientations.
    arm.q = np.asarray(sweep.POSES[0], dtype=float)
    expected = upright.read()
    still = cv2.imread(str(result.captures[0]))
    upright_diff = float(np.mean(cv2.absdiff(still, expected)))
    turned_diff = float(np.mean(cv2.absdiff(still, cv2.rotate(expected, cv2.ROTATE_180))))
    check("the saved stills are upright, not as delivered",
          still is not None and upright_diff < turned_diff,
          "upright diff %.1f, turned diff %.1f" % (upright_diff, turned_diff))


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
            hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics="", witness_max_px=2.0, camera="rtsp://fake/stream",
            mediamtx_api="http://127.0.0.1:9", recordings=str(calib_dir.parent / "recordings"),
            traces_dir=str(workdir / "sweep-server-traces"),
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

def test_marker_rig(workdir: Path):
    print("\nbase tag: one capture writes the fixed rig, no sweep")
    from aiohttp.test_utils import TestClient, TestServer

    import camera as camera_mod
    import capture_server

    # The tag solve is only as good as the tag's geometry, so this camera sees
    # it at about 90 px across: the regime docs/base-marker.md describes.
    K, dist, _T_arm, _tip, _observations = synthetic_rig()
    T_true = extrinsics.look_at(np.array([0.15, -0.80, 0.90]),
                                np.array([0.0, -0.047, 0.043]))
    marker_at = (0.0, -0.047, 0.043)
    marker_yaw = 270.0
    marker_size = 0.08

    captures = workdir / "marker-captures"
    calib_dir = workdir / "marker-calib"
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")

    background = cv2.cvtColor(textured_frame(), cv2.COLOR_GRAY2BGR)
    frame = render_marker(background.copy(), T_true, K, dist, marker_at, marker_yaw,
                          marker_size)

    class StillEye:
        def __init__(self, image):
            self.image = image

        def read(self, retries=1):
            return self.image.copy()

        def close(self):
            pass

    eye = StillEye(frame)
    arm = FakeArm()

    def args_for(execute):
        return SimpleNamespace(
            host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
            token="", mediamtx="http://127.0.0.1:9", whip_path="phone", no_whip=True,
            max_mb=4, quiet=True, planner="http://127.0.0.1:9", planner_token="",
            execute=execute, calib_dir=str(calib_dir), calib="",
            marker_at="%g,%g,%g,%g" % (marker_at + (marker_yaw,)),
            marker_size=marker_size,
            hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics="", witness_max_px=2.0, camera="rtsp://fake/stream",
            mediamtx_api="http://127.0.0.1:9", recordings=str(calib_dir.parent / "recordings"),
            traces_dir=str(workdir / "marker-server-traces"),
        )

    original_gateway = auto_calibrate.ArmGateway
    original_source = camera_mod.open_source
    auto_calibrate.ArmGateway = lambda *a, **k: arm
    camera_mod.open_source = lambda *a, **k: eye

    def verdicts():
        good = auto_calibrate.verify_marker(
            {"rms_px": 0.7, "side_px": 60.0, "grazing_deg": 40.0, "id": 0})
        tiny = auto_calibrate.verify_marker(
            {"rms_px": 0.7, "side_px": 10.0, "grazing_deg": 40.0, "id": 0})
        edge = auto_calibrate.verify_marker(
            {"rms_px": 0.7, "side_px": 60.0, "grazing_deg": 85.0, "id": 0})
        # The mirror branch. This solve reprojects the tag at 0.04 px, is 78 px
        # across and only 21 deg from face-on -- it clears every pixel-space gate
        # comfortably, and it is still 885 mm away from the truth, because a
        # planar tag has two PnP solutions that reproject identically. Measured
        # over 40 rendered solves, angular size does not separate them (a 77.7 px
        # tag took the wrong branch, a 13.2 px tag the right one); where the
        # camera lands does, perfectly.
        mirror = auto_calibrate.verify_marker(
            {"rms_px": 0.04, "side_px": 78.0, "grazing_deg": 21.0, "id": 0},
            camera_at=(0.20, -1.15, -0.435))
        check("a readable tag is enough for a marker rig", good["ok"])
        check("a tiny tag is refused", not tiny["ok"]
              and "px across" in " ".join(tiny["reasons"]))
        check("a nearly edge-on tag is refused", not edge["ok"]
              and "edge-on" in " ".join(edge["reasons"]))
        check("a clean solve on the mirror branch is refused",
              not mirror["ok"] and "mirror branch" in " ".join(mirror["reasons"]),
              " ".join(mirror["reasons"])[:90])
        check("the same solve above the base plane is accepted",
              auto_calibrate.verify_marker(
                  {"rms_px": 0.04, "side_px": 78.0, "grazing_deg": 21.0, "id": 0},
                  camera_at=(0.20, -1.15, 0.470))["ok"])
        check("the verdict reports the camera height it judged the branch on",
              mirror["marker"]["camera_z_m"] == -0.435
              and good["marker"]["camera_z_m"] is None)
        check("the size gate stays a localisation gate, not a branch gate",
              auto_calibrate.MARKER_MIN_SIDE_PX == 16.0,
              "%.0f px" % auto_calibrate.MARKER_MIN_SIDE_PX)

    verdicts()

    try:
        app = capture_server.build_app(args_for(True))

        async def scenarios():
            async with TestClient(TestServer(app)) as client:
                started = arm.moves
                poses_started = arm.poses
                answer = await client.post("/capture/pose", json={})
                body = await answer.json()
                check("a still of the tag writes the rig",
                      answer.status == 200 and body.get("written")
                      and (calib_dir / "extrinsics-fixed.json").is_file(),
                      "status %s, %s" % (answer.status, str(body.get("error"))[:50]))
                check("the tag capture runs no sweep", arm.moves == started,
                      "%d moves" % (arm.moves - started))
                check("the tag capture reads no arm pose", arm.poses == poses_started,
                      "%d /pose reads" % (arm.poses - poses_started))
                check("the tag capture returns without a pose", body.get("pose") is None)
                check("the response says where the pose came from",
                      body.get("method") == "base-marker"
                      and (body.get("marker") or {}).get("detected") is True
                      and (body.get("verification") or {}).get("ok") is True,
                      str((body.get("verification") or {}).get("reasons")))
                check("the tag solve is accurate enough to plan from",
                      np.linalg.norm(np.asarray(app["rig"].T_base_cam, dtype=float)[:3, 3]
                                     - T_true[:3, 3]) < 0.060,
                      "%.0f mm"
                      % (1000.0 * np.linalg.norm(app["rig"].T_base_cam[:3, 3] - T_true[:3, 3])))
                check("the loaded rig remembers it came from a tag",
                      app["rig"] is not None and app["rig"].method == "base-marker"
                      and len(app["rig"].observations) == 0)
                loaded = extrinsics.load_rig(calib_dir / "extrinsics-fixed.json")
                check("the tag rig round-trips through its file",
                      loaded.method == "base-marker" and loaded.witness is None,
                      loaded.method)
                check("the tag report says the witness was skipped",
                      any("mount witness" in str(w) for w in (body.get("warnings") or [])),
                      str(body.get("warnings")))

                status = await (await client.get("/calibrate")).json()
                check("GET /calibrate reports the tag solve",
                      (status.get("last") or {}).get("state") == "ok"
                      and (status.get("rig") or {}).get("method") == "base-marker",
                      str(status.get("last")))

                # A still without the tag is not a failure: it is the tap path.
                eye.image = background.copy()
                answer = await client.post("/capture/pose", json={})
                body = await answer.json()
                check("a still without the tag falls back to the tap",
                      answer.status == 200 and body.get("ok") is True
                      and (body.get("marker") or {}).get("detected") is False
                      and not body.get("written"),
                      str((body.get("marker") or {}).get("error"))[:50])
                check("the fallback leaves the tag rig in place",
                      app["rig"] is not None
                      and app["rig"].method == "base-marker")

        asyncio.run(scenarios())

        # A restart is what the operator does next: the rig is on disk under the
        # standard name, so no env var has to carry it.
        restarted = capture_server.build_app(args_for(True))
        check("a restart finds the tag rig without an env var",
              restarted["rig"] is not None
              and restarted["rig"].method == "base-marker"
              and str(restarted["extrinsics_path"]).endswith("extrinsics-fixed.json"))
        check("a fitted rig makes fixed the default",
              capture_server.effective_mode(restarted) == "fixed")
        handheld = capture_server.locate_options(restarted, {"mode": "handheld"})
        check("a request can still choose handheld",
              handheld.extrinsics is None and handheld.tag_size > 0)
        check("a request can choose fixed",
              capture_server.locate_options(restarted, {"mode": "fixed"}).extrinsics
              is not None)
        empty_args = args_for(True)
        empty_args.calib_dir = str(workdir / "marker-empty-calib")
        empty = capture_server.build_app(empty_args)
        check("without a rig the default is handheld",
              empty["rig"] is None
              and capture_server.effective_mode(empty) == "handheld")

        async def reach_json():
            reach_app = capture_server.build_app(args_for(True))
            async with TestClient(TestServer(reach_app)) as client:
                return await (await client.get("/reach.json")).json()

        reach = asyncio.run(reach_json())
        check("reach.json offers the rig and the default mode",
              reach.get("mode") == "fixed"
              and reach.get("T_base_cam") is not None
              and reach.get("rig") is not None,
              str(reach.get("mode")))

        # The same short path on the command line: --capture-pose with the tag
        # configured writes the rig instead of printing tap instructions.
        saved_state = extrinsics_cli.STATE
        extrinsics_cli.STATE = workdir / "marker-cli-state"
        eye.image = frame.copy()
        cli_rig = workdir / "marker-cli-rig.json"
        poses_before_cli = arm.poses
        try:
            code = extrinsics_cli.main([
                "--capture-pose", "--camera", "rtsp://fake",
                "--marker-at", "%g,%g,%g,%g" % (marker_at + (marker_yaw,)),
                "--marker-size", str(marker_size),
                "--calib", str(calib_dir / "synthetic.json"),
                "--out", str(cli_rig), "--quiet",
            ])
        finally:
            extrinsics_cli.STATE = saved_state
        check("--capture-pose writes the rig from the tag",
              code == 0 and cli_rig.is_file()
              and extrinsics.load_rig(cli_rig).method == "base-marker",
              "exit %s" % code)
        check("--capture-pose reads no arm pose for a tag",
              arm.poses == poses_before_cli,
              "%d /pose reads" % (arm.poses - poses_before_cli))
    finally:
        auto_calibrate.ArmGateway = original_gateway
        camera_mod.open_source = original_source


# --------------------------------------------------------------------------- #
# Phase 4: the hillclimb (§7, §9) -- synthetic fits and telemetry, no arm motion
# --------------------------------------------------------------------------- #

def test_identify_gate(workdir: Path):
    print("\nidentify: triggers, the tag gate, averaged PnP, coverage poses")
    K, dist, T_true, tip, observations = synthetic_rig()
    options = extrinsics.FitOptions(plane_z=0.0, image_size=(1600, 1200))
    fit = extrinsics.fit(observations, K, dist, options)

    # ---- §7.1 the monitor proposes; it never runs anything ------------------
    monitor = identify.TriggerMonitor()
    proposals = monitor.observe({"witness": {"drift_px": 5.0, "scale": 1.0},
                                 "holds": [{"pixel_error_mm": 15.0}] * 4}, rig=None)
    kinds = {p["kind"] for p in proposals}
    check("witness drift proposes a session", "stale-rig" in kinds, str(kinds))
    check("pixel-error EWMA over threshold proposes", "pixel-drift" in kinds)
    check("a missing rig proposes", "missing-rig" in kinds)
    clean = identify.TriggerMonitor()
    check("a clean oracle proposes nothing",
          clean.observe({"holds": [{"pixel_error_mm": 2.0}],
                         "witness": {"drift_px": 0.5, "scale": 1.0}}, rig=fit.rig) == [])
    growth = identify.TriggerMonitor()
    growth.observe({"tag_reprojection_px": 1.0}, rig=fit.rig)
    check("tag reprojection growth proposes",
          any(p["kind"] == "tag-growth"
              for p in growth.observe({"tag_reprojection_px": 5.0}, rig=fit.rig)))

    # §7.2: zero drift is a re-zeroing job, proposed apart from any rig refit,
    # because a tag rig cannot absorb it and must not be blamed for it.
    zeros = identify.TriggerMonitor()
    flagged = zeros.observe({"verdict": {"problems": [
        {"kind": "joint-zeros", "layer": "arm-motor", "detail": "constant bias across poses"}]},
        "holds": []}, rig=fit.rig)
    check("the oracle's joint-zeros verdict proposes re-zeroing",
          any(p["kind"] == "zero-drift" and "re-zero" in p.get("action", "") for p in flagged),
          str([p["kind"] for p in flagged]))
    check("zero drift is not reported as a rig problem",
          not any(p["kind"] in ("stale-rig", "missing-rig", "pixel-drift") for p in flagged))
    check("a sag verdict is not zero drift",
          identify.zero_drift({"verdict": {"problems": [
              {"kind": "sag", "layer": "arm-motor", "detail": "x"}]}}) is None)

    # ---- §7.2 the gate: a weak tag solve is REFUSED, not averaged in --------
    good = auto_calibrate.verify_marker({"rms_px": 0.4, "side_px": 120.0, "grazing_deg": 12.0})
    tiny = auto_calibrate.verify_marker({"rms_px": 0.4, "side_px": 9.0, "grazing_deg": 12.0})
    edge_on = auto_calibrate.verify_marker({"rms_px": 0.4, "side_px": 120.0, "grazing_deg": 84.0})
    gate = identify.rig_identifiability([good, good, good])
    check("three good solves are identifiable", gate["status"] == "ok" and not gate["refused"])
    mixed = identify.rig_identifiability([good, tiny, edge_on, good])
    check("a tiny or grazing tag is refused, not averaged in",
          len(mixed["usable"]) == 2 and len(mixed["refused"]) == 2,
          str([e["reasons"] for e in mixed["refused"]]))
    check("the refusal says why", all(entry["reasons"] for entry in mixed["refused"]))
    check("the gate reports rms and grazing so a marginal rig is visible",
          all(entry["rms_px"] is not None and entry["grazing_deg"] is not None
              for entry in mixed["usable"] + mixed["refused"]))
    thin = identify.rig_identifiability([good, good])
    check("fewer usable solves than the minimum is a refusal",
          thin["status"] == "refused" and thin["min_usable"] == identify.MARKER_MIN_USABLE)
    check("the gate names what the tag can observe",
          gate["identifiable"] == ["camera extrinsics (T_base_cam)"])
    check("the gate names what the tag can never observe",
          any("joint zeros" in text for text in gate["not_identifiable"])
          and any("tip offset" in text for text in gate["not_identifiable"]),
          str(gate["not_identifiable"]))

    # ---- §7.3a averaging repeated PnP estimates ----------------------------
    at = np.array([0.02, 0.01, 0.001])
    yaw_deg = -81.0                      # the tag's face turned toward the camera
    # 12 cm, not the 4 cm standard. A planar tag has two PnP solutions that
    # reproject identically; which one wins is not a function of angular size (a
    # 77.7 px tag was measured on the wrong branch, a 13.2 px tag on the right
    # one), but at this pose and grazing the 4 cm tag does land on the wrong
    # branch, 2.5 m out. verify_marker now refuses that on the solved camera
    # height, so the refit's success path renders a tag that solves correctly and
    # the mirror case below is asserted to be refused rather than averaged.
    size_m = 0.12
    mirror_size_m = base_marker.SIZE_M
    T_base_cam = np.asarray(T_true, dtype=float)
    T_base_marker = base_marker.base_from_marker(at, math.radians(yaw_deg), mode="vertical")
    T_cam_marker = np.linalg.inv(T_base_cam) @ T_base_marker
    check("averaging one solve returns it",
          np.allclose(identify.average_pnp([T_cam_marker]), T_cam_marker))
    rng = np.random.default_rng(3)
    noisy = []
    for _ in range(9):
        jitter = np.eye(4)
        jitter[:3, 3] = rng.normal(0.0, 0.004, 3)            # 4 mm of translation noise
        jitter[:3, :3] = extrinsics.rodrigues(rng.normal(0.0, 0.004, 3))
        noisy.append(T_cam_marker @ jitter)
    averaged = identify.average_pnp(noisy)
    single = float(np.linalg.norm(noisy[0][:3, 3] - T_cam_marker[:3, 3]))
    gained = float(np.linalg.norm(averaged[:3, 3] - T_cam_marker[:3, 3]))
    check("averaging cuts the translation noise", gained < single,
          "%.1f mm averaged vs %.1f mm single" % (1000.0 * gained, 1000.0 * single))
    check("the average is still a rotation",
          abs(float(np.linalg.det(averaged[:3, :3])) - 1.0) < 1e-9
          and np.allclose(averaged[:3, :3] @ averaged[:3, :3].T, np.eye(3), atol=1e-9))
    check("identical solves have no spread",
          identify.pnp_spread_mm([T_cam_marker] * 4) == 0.0)
    check("the spread measures disagreement in mm", identify.pnp_spread_mm(noisy) > 1.0,
          "%.1f mm" % identify.pnp_spread_mm(noisy))

    # ---- §7.3a the held-out gate: reproject a tag the average never saw -----
    marker = {"at": [float(v) for v in at], "yaw": math.radians(yaw_deg), "size_m": size_m}
    frame = render_marker(np.full((1200, 1600, 3), 210, np.uint8), T_base_cam, K, dist,
                          at, yaw_deg, size_m)
    corners = base_marker.detect(frame)
    exact = identify.marker_reprojection(T_base_cam, corners, K, dist, marker)
    check("the true rig reprojects the tag it was built from",
          exact["worst_px"] < 1.5, "%.2f px" % exact["worst_px"])
    shifted = T_base_cam.copy()
    shifted[:3, 3] += [0.03, 0.0, 0.0]
    drifted = identify.marker_reprojection(shifted, corners, K, dist, marker)
    check("a 30 mm camera error is caught by the held-out reprojection",
          drifted["worst_px"] > identify.HOLDOUT_MAX_PX, "%.2f px" % drifted["worst_px"])

    # ---- §7.3a the session: no arm motion, and it refuses honestly ----------
    class TagEye:
        """A still camera: hands back the frames it was given, then nothing."""

        def __init__(self, frames):
            self.frames = list(frames)
            self.closed = False

        def read(self):
            return self.frames.pop(0) if self.frames else None

        def close(self):
            self.closed = True

    calib = calib_mod.Calib(path=workdir / "refit-calib.json", K=K, dist=dist,
                            image_size=(1600, 1200), device="synthetic")
    calib_dir = workdir / "refit-calib-dir"
    calib_dir.mkdir(parents=True, exist_ok=True)
    stills = [render_marker(np.full((1200, 1600, 3), 210, np.uint8), T_base_cam, K, dist,
                            at, yaw_deg, size_m) for _ in range(6)]
    report = identify.refit_session(TagEye(stills), calib, marker, calib_dir,
                                    captures=5, holdout=1, settle_s=0.0)
    check("a refit of rendered tag stills succeeds", report["ok"],
          "; ".join(report["verification"]["reasons"]) or str(report["identifiability"]))
    check("a refit moves nothing", report["motion"] is False and report["method"] == "base-marker")
    check("the refit averaged several solves", report["usable"] >= identify.MARKER_MIN_USABLE,
          "%d usable of %d" % (report["usable"], report["captures"]))
    check("the candidate was written beside the live rig", Path(report["candidate"]).is_file())
    check("the verified candidate was committed", report["written"] is not None
          and Path(report["written"]).is_file())
    written = extrinsics.load_rig(report["written"])
    offset = 1000.0 * float(np.linalg.norm(np.asarray(written.T_base_cam)[:3, 3]
                                           - T_base_cam[:3, 3]))
    check("the committed rig is the averaged tag solve", offset < 60.0, "%.1f mm" % offset)
    single = auto_calibrate.solve_marker(stills[0], calib, marker, plane_z=0.0)
    spread = 1000.0 * float(np.linalg.norm(
        np.asarray(written.T_base_cam)[:3, 3] - np.asarray(single.rig.T_base_cam)[:3, 3]))
    check("the average stays with the solves it averaged", spread < 25.0, "%.1f mm" % spread)
    check("the tag rig carries no joint zeros (§7.2)", written.joint_zeros is None)
    check("the tag rig fits no tip offset (§7.3a)",
          float(np.linalg.norm(written.tip_offset)) < 1e-9)
    check("the tag rig records how it was made", written.method == "base-marker")

    # A tag nobody can see: refused, and the committed rig is byte-identical.
    kept = Path(report["written"]).read_bytes()
    blank = [np.full((1200, 1600, 3), 210, np.uint8) for _ in range(6)]
    try:
        identify.refit_session(TagEye(blank), calib, marker, calib_dir,
                               captures=5, holdout=1, settle_s=0.0)
        check("a refit that cannot see the tag is refused", False)
    except identify.IdentifyError as error:
        check("a refit that cannot see the tag is refused", "verify_marker" in str(error),
              str(error)[:90])
        check("a refused refit leaves the running rig untouched",
              Path(report["written"]).read_bytes() == kept)
    try:
        identify.refit_session(TagEye(stills), calib, {}, calib_dir, settle_s=0.0)
        check("a refit without a marker placement is refused", False)
    except identify.IdentifyError as error:
        check("a refit without a marker placement is refused", "REBOT_MARKER_AT" in str(error))

    # A session whose solves disagree means something moved: refuse, do not average.
    moving = [render_marker(np.full((1200, 1600, 3), 210, np.uint8),
                            T_base_cam if index < 3 else shifted, K, dist,
                            at, yaw_deg, size_m) for index in range(6)]
    try:
        identify.refit_session(TagEye(moving), calib, marker, calib_dir,
                               captures=5, holdout=1, settle_s=0.0)
        check("solves that disagree are not averaged into a rig", False)
    except identify.IdentifyError as error:
        check("solves that disagree are not averaged into a rig", "disagree" in str(error),
              str(error)[:90])

    # The mirror branch, end to end. At this pose and grazing the 4 cm standard
    # tag solves to the branch that puts the camera 1.2 m BELOW the base plane and
    # 2.5 m from the truth, while reprojecting the tag's corners at 0.36 px. No
    # pixel-space gate can see it; the solved camera height can.
    mirror_marker = {"at": [float(v) for v in at], "yaw": math.radians(yaw_deg),
                     "size_m": mirror_size_m}
    mirror_stills = [render_marker(np.full((1200, 1600, 3), 210, np.uint8), T_base_cam,
                                   K, dist, at, yaw_deg, mirror_size_m) for _ in range(6)]
    solved = auto_calibrate.solve_marker(mirror_stills[0], calib, mirror_marker, plane_z=0.0)
    mirror_z = float(np.asarray(solved.rig.T_base_cam, dtype=float)[:3, 3][2])
    check("the mirror-branch solve really does put the camera underground",
          mirror_z < 0.0 and not solved.verdict["ok"]
          and "mirror branch" in " ".join(solved.verdict["reasons"]),
          "camera z %+.3f m; %s" % (mirror_z, " ".join(solved.verdict["reasons"])[:70]))
    check("the mirror-branch gate reports the height it judged on",
          solved.verdict["marker"]["camera_z_m"] is not None
          and solved.verdict["marker"]["camera_z_m"] < 0.0)
    check("the mirror solve is otherwise clean, so only the branch gate catches it",
          solved.evidence["rms_px"] < auto_calibrate.MARKER_MAX_RMS_PX
          and solved.evidence["side_px"] >= auto_calibrate.MARKER_MIN_SIDE_PX
          and solved.evidence["grazing_deg"] <= auto_calibrate.MARKER_MAX_GRAZING_DEG,
          "rms %.2f px, %.0f px across, %.0f deg"
          % (solved.evidence["rms_px"], solved.evidence["side_px"],
             solved.evidence["grazing_deg"]))
    kept = Path(report["written"]).read_bytes()
    try:
        identify.refit_session(TagEye(mirror_stills), calib, mirror_marker, calib_dir,
                               captures=5, holdout=1, settle_s=0.0)
        check("a refit of mirror-branch solves is refused", False)
    except identify.IdentifyError as error:
        check("a refit of mirror-branch solves is refused", "verify_marker" in str(error),
              str(error)[:90])
        check("a refused mirror-branch refit leaves the rig untouched",
              Path(report["written"]).read_bytes() == kept)
    mirror_gate = identify.rig_identifiability([solved.verdict] * 3)
    check("the identifiability gate counts a mirror branch as refused, not usable",
          mirror_gate["status"] == "refused" and not mirror_gate["usable"]
          and len(mirror_gate["refused"]) == 3)

    # ---- §7.4 the tag prior, the grid, and the coverage poses --------------
    near = identify.tag_prior_mm(fit.rig, np.array([0.30, 0.0, 0.20]))
    far = identify.tag_prior_mm(fit.rig, np.array([0.60, 0.0, 0.20]))
    check("the tag prior scales with range", far > near, "%.1f vs %.1f mm" % (near, far))
    check("no rig means no prior", identify.tag_prior_mm(None, np.zeros(3)) is None)

    cells = identify.coverage_cells({})
    total = ((len(identify.ACCURACY_RADIUS_EDGES) - 1)
             * (len(identify.ACCURACY_HEIGHT_EDGES) - 1) * identify.ACCURACY_SECTORS)
    check("the grid is enumerated, not read from the map's keys",
          len(cells) == total, "%d cells" % len(cells))
    check("an empty map needs everything", all(entry["need"] == 3 for entry in cells))
    check("a validated cell needs nothing",
          identify.coverage_cells({"cells": {cells[0]["cell"]: {
              "n": 4, "n_good": 4, "validated_at": "2026-01-01T00:00:00Z"}}})[0]["need"] == 0)
    check("a measured but unvalidated cell still needs work",
          identify.coverage_cells({"cells": {cells[1]["cell"]: {"n": 1}}})[1]["need"] == 2)
    check("a cell that un-validated is still worth measuring",
          identify.coverage_cells({"cells": {cells[2]["cell"]: {"n": 4, "n_good": 1}}})[2]["need"] == 1)

    chosen = identify.coverage_poses(np.zeros(6), {}, tip, top_k=6)
    check("coverage picks poses for the unvalidated cells", 0 < len(chosen) <= 6,
          "%d poses" % len(chosen))
    check("every coverage pose is a legal pose", all(sweep.pose_ok(e["q"]) for e in chosen))
    chain = [np.zeros(6)] + [entry["q"] for entry in chosen]
    check("the coverage chain is path-safe from the current pose",
          all(sweep.path_safe(a, b) for a, b in zip(chain, chain[1:])))
    peaks = [(math.pi / 2.0) * float(np.max(np.abs(b - a))) / sweep.seconds_for(a, b)
             for a, b in zip(chain, chain[1:])]
    check("coverage legs stay under the speed cap", max(peaks) <= sweep.PEAK_SPEED + 1e-9,
          "worst %.3f rad/s" % max(peaks))
    again = identify.coverage_poses(np.zeros(6), {}, tip, top_k=6)
    check("coverage pose choice is deterministic",
          all(np.allclose(a["q"], b["q"]) for a, b in zip(chosen, again)))
    check("coverage spreads across distinct cells",
          len({entry["cell"] for entry in chosen}) > 1, str([e["cell"] for e in chosen]))
    done = {entry["cell"]: {"n": 9, "n_good": 9, "validated_at": "x"}
            for entry in identify.coverage_cells({})}
    check("a fully validated map leaves nothing to cover",
          identify.coverage_poses(np.zeros(6), {"cells": done}, tip) == [])

    plan = identify.coverage_plan(np.zeros(6), chosen, gripper=0.1, hold_s=2.0)
    check("the plan asks for the speed policy's own durations",
          all(abs(value - sweep.seconds_for(a, b)) < 2e-3
              for value, a, b in zip(plan["seconds"], chain, chain[1:])),
          str(plan["seconds"][:3]))
    check("the plan holds long enough for the oracle's grabs", plan["hold"] >= oracle.HOLD_MIN_S)
    check("a coverage plan writes no rig", plan["writes_rig"] is False)
    check("the plan names the cells it measures",
          plan["cells"] == [entry["cell"] for entry in chosen])
    check("the plan says what the motion is for", "measure" in plan["purpose"])

    # ---- §7.2 what is not learned is reported, never extrapolated ----------
    gaps = identify.unlearned(None, {})
    check("no sag map and no measurements is reported unlearned",
          {gap["quantity"] for gap in gaps} == {"sag-map", "accuracy-map"}, str(gaps))
    check("a thin sag map is partial, not learned",
          any(gap["state"] == "partial" for gap in identify.unlearned({"samples": 4}, {})))
    check("a full sag map and a validated grid report no gap",
          identify.unlearned({"samples": 99}, {"cells": done}) == [],
          str(identify.unlearned({"samples": 99}, {"cells": done})))


def test_sag_map(workdir: Path):
    print("\nidentify: sag residual map from telemetry, no extra motion")
    kp = sweep.kp_vector()
    rng = np.random.default_rng(5)
    a_true = [0.10 / float(kp[j]) for j in range(6)]     # the measured 10% model gap
    b_true = [0.001 + 0.0003 * j for j in range(6)]

    # 40 holds at spread command poses, each a 1.1 s constant-command window in
    # a telemetry log. The arm settles as meas = cmd - gravity_sag(cmd) - r, with
    # r affine in the gravity torque at the *command* pose -- exactly the
    # relation sag_holds extracts (it evaluates the model at q_cmd).
    docs = []
    for i in range(40):
        cmd = np.array([rng.uniform(lo, hi) for lo, hi in rebot_ik.LIMITS])
        tau = rebot_ik.gravity_torques(cmd)
        residual = np.array([a_true[j] * tau[j] + b_true[j] for j in range(6)])
        meas = cmd - sweep.gravity_sag(cmd, kp) - residual + rng.normal(0.0, 0.0003, 6)
        samples = [{"t": i * 3.0 + k * 0.1,
                    "cmd": [float(v) for v in cmd] + [0.0],
                    "meas": [float(v) for v in meas] + [0.0]} for k in range(12)]
        docs.append({"samples": samples,
                     "kp": {str(m): float(kp[m - 1]) for m in range(1, 7)}})
    holds = identify.sag_holds(docs)
    check("holds are extracted from telemetry windows", len(holds) == 40,
          "%d holds" % len(holds))

    result = identify.fit_sag_map(holds, kp_key=sweep.KP_DEFAULT, rig_fingerprint="abc123def456")
    check("the sag map fits with enough spread holds", result["ok"],
          "; ".join(result.get("reasons", [])))
    if result["ok"]:
        per = result["map"]["per_motor"]
        a_err = max(abs(per[str(j + 1)]["a"] - a_true[j]) for j in (1, 2, 3, 4))
        b_err = max(abs(per[str(j + 1)]["b"] - b_true[j]) for j in range(6))
        check("the injected a_j is recovered", a_err < 0.10 * max(a_true[1:5]),
              "worst %.2e" % a_err)
        check("the injected b_j is recovered", b_err < 0.001, "worst %.4f mrad"
              % (1000.0 * b_err))
        check("rms is reported in mrad and mm at the tool",
              result["map"]["rms_mrad"] >= 0.0 and result["map"]["validated_mm"] >= 0.0,
              "%.2f mrad, %.2f mm" % (result["map"]["rms_mrad"],
                                      result["map"]["validated_mm"]))
        map_doc = result["map"]
    else:
        map_doc = None

    check("fewer than 30 holds are refused",
          not identify.fit_sag_map(holds[:20], kp_key=sweep.KP_DEFAULT,
                                   rig_fingerprint="abc123def456")["ok"])
    check("holds that do not span the workspace are refused",
          not identify.fit_sag_map([holds[0]] * 40, kp_key=sweep.KP_DEFAULT,
                                   rig_fingerprint="abc123def456")["ok"])
    # Overfit: the held-out split (every fifth hold) carries a large residual
    # the train set never saw -- validation must reject the map.
    overfit = [dict(h) for h in holds]
    for i in range(4, len(overfit), 5):
        overfit[i]["r"] = overfit[i]["r"] + rng.normal(0.0, 0.05, 6)
    rejected = identify.fit_sag_map(overfit, kp_key=sweep.KP_DEFAULT,
                                    rig_fingerprint="abc123def456")
    check("hold-out validation rejects an overfit map", not rejected["ok"],
          "; ".join(rejected.get("reasons", []))[:60])

    if map_doc is not None:
        q_in = holds[0]["q"]
        keys = {"kp_key": sweep.KP_DEFAULT, "rig_fingerprint": "abc123def456"}
        extra = identify.sag_correction(q_in, map_doc, **keys)
        check("the correction applies inside the envelope",
              float(np.max(np.abs(extra))) > 0.0)
        q_out = q_in.copy()
        q_out[1] = map_doc["envelope"]["q_max"][1] + 0.5
        check("no correction outside the explored envelope",
              np.all(identify.sag_correction(q_out, map_doc, **keys) == 0.0))
        check("a kp change invalidates the map",
              np.all(identify.sag_correction(q_in, map_doc, kp_key="50",
                                             rig_fingerprint="abc123def456") == 0.0))
        check("a rig change invalidates the map",
              np.all(identify.sag_correction(q_in, map_doc, kp_key=sweep.KP_DEFAULT,
                                             rig_fingerprint="other") == 0.0))
        check("a zero change invalidates the map",
              np.all(identify.sag_correction(q_in, map_doc, zeros_key="changed",
                                             **keys) == 0.0))
        identify.save_sag_map(workdir, map_doc)
        loaded = identify.load_sag_map(workdir, "abc123def456", kp_key=sweep.KP_DEFAULT)
        check("the map round-trips keyed by rig fingerprint", loaded is not None)
        check("loading refuses a kp change",
              identify.load_sag_map(workdir, "abc123def456", kp_key="50") is None)
        check("loading refuses a zero change",
              identify.load_sag_map(workdir, "abc123def456", kp_key=sweep.KP_DEFAULT,
                                    zeros_key="changed") is None)

        # The live /move path (§7.3b wiring): the planner reads the active rig's
        # identity from the calib dir and applies the keyed map through aim().
        live_dir = workdir / "sag-live-calib"
        live_dir.mkdir(parents=True, exist_ok=True)
        rig_file = live_dir / "extrinsics-fixed.json"
        rig_file.write_text(json.dumps({"fingerprint": "abc123def456",
                                        "joint_zeros": None}), encoding="utf-8")
        identify.save_sag_map(live_dir, map_doc)
        check("the live move path applies the keyed map",
              np.allclose(sweep.active_sag_extra(q_in, sweep.KP_DEFAULT, live_dir),
                          extra, atol=1e-12))
        check("the live path refuses a kp change",
              np.all(sweep.active_sag_extra(q_in, "50", live_dir) == 0.0))
        # The plan path (§7.3b wiring): one compensator per plan -- model sag
        # plus the keyed learned term, resolved once instead of per waypoint.
        compensate = sweep.sag_compensator(sweep.KP_DEFAULT, live_dir)
        check("the plan compensator applies model sag plus the keyed map",
              np.allclose(compensate(q_in), sweep.aim(q_in, kp, extra_sag=extra),
                          atol=1e-12))
        check("the plan compensator is model-only outside the envelope",
              np.allclose(compensate(q_out), sweep.aim(q_out, kp), atol=1e-12))
        check("the plan compensator is model-only with no calib dir",
              np.allclose(sweep.sag_compensator(sweep.KP_DEFAULT, "")(q_in),
                          sweep.aim(q_in, kp), atol=1e-12))
        rig_file.write_text("{torn", encoding="utf-8")
        check("a torn rig file means no map, never an exception",
              np.all(sweep.active_sag_extra(q_in, sweep.KP_DEFAULT, live_dir) == 0.0))
        rig_file.unlink()
        check("no active rig means no map",
              np.all(sweep.active_sag_extra(q_in, sweep.KP_DEFAULT, live_dir) == 0.0))

    # The additive term in sweep.aim stays clipped by SAG_LIMIT_RAD, however
    # large the map's correction -- a bad map can never command an excursion.
    huge = np.full(6, 10.0)
    aimed = sweep.aim(sweep.POSES[0], kp, extra_sag=huge)
    check("the additive sag term stays clipped by SAG_LIMIT_RAD",
          float(np.max(np.abs(aimed - np.asarray(sweep.POSES[0]))))
          <= sweep.SAG_LIMIT_RAD + 1e-9,
          "%.0f mrad" % (1000.0 * float(np.max(np.abs(aimed - np.asarray(sweep.POSES[0]))))))
    plain = sweep.aim(sweep.POSES[0], kp)
    check("aim without a map is unchanged",
          np.allclose(plain, sweep.aim(sweep.POSES[0], kp, extra_sag=None), atol=1e-12))


def test_cartesian_targets(workdir: Path):
    print("\ncartesian coverage: cell centres, reach band, need ordering (pure)")
    import identify

    # Empty map: every cell is a gap. The grid is 5 radius x 4 height x 8 sector
    # = 160 cells; the h0 band's centre (z=0.05) is below the 0.10 m safety
    # floor, so 40 of them are unreachable as Cartesian targets -- reported,
    # never clamped up into a different cell.
    chosen, skipped = identify.cartesian_targets({"cells": {}})
    check("an empty map offers every reachable cell once",
          len(chosen) == 12 and skipped == {"below_min_z": 40, "beyond_reach": 0},
          "chosen %d skipped %s" % (len(chosen), skipped))
    full, skipped_full = identify.cartesian_targets({"cells": {}}, top_k=200)
    check("the full reachable grid is 120 cells, one point each",
          len(full) == 120 and len({entry["cell"] for entry in full}) == 120
          and skipped_full["below_min_z"] == 40,
          "full %d" % len(full))
    check("every centre round-trips to the cell it was chosen for",
          all(identify.accuracy_cell(entry["point"]) == entry["cell"] for entry in full),
          str([entry["cell"] for entry in full
               if identify.accuracy_cell(entry["point"]) != entry["cell"]])[:80])
    check("centres sit inside the reach band",
          all(entry["point"][2] >= 0.10 - 1e-9
              and math.hypot(entry["point"][0], entry["point"][1]) <= 0.75
              for entry in full))
    check("never-measured cells come before partially measured ones",
          [entry["need"] for entry in full] == sorted((entry["need"] for entry in full),
                                                      reverse=True))

    # A validated cell is not a gap; a measured-but-unvalidated one still is.
    cell = full[0]["cell"]
    point = full[0]["point"]
    map_doc = {"cells": {cell: {"validated_at": "2026-01-01T00:00:00Z", "n": 3}}}
    chosen, _ = identify.cartesian_targets(map_doc, top_k=200)
    check("a validated cell is not targeted again",
          all(entry["cell"] != cell for entry in chosen) and len(chosen) == 119)
    map_doc = {"cells": {cell: {"validated_at": None, "n": 1}}}
    chosen, _ = identify.cartesian_targets(map_doc, top_k=200)
    entry = next(item for item in chosen if item["cell"] == cell)
    check("a partially measured cell keeps its need", entry["need"] == 2)

    # The reach band is the planner's own: a higher floor or shorter reach
    # excludes cells honestly instead of planning into them.
    chosen, skipped = identify.cartesian_targets({"cells": {}}, top_k=200, min_z=0.20)
    check("a higher safety floor excludes the h1 band too",
          skipped["below_min_z"] == 80 and len(chosen) == 80, str(skipped))
    chosen, skipped = identify.cartesian_targets({"cells": {}}, top_k=200, max_reach=0.35)
    check("a shorter reach excludes the outer bands",
          skipped["beyond_reach"] > 0 and len(chosen) + skipped["beyond_reach"] == 120,
          str(skipped))
    check("the top_k cap holds", len(identify.cartesian_targets({"cells": {}}, top_k=5)[0]) == 5)


def test_identify_api(workdir: Path):
    print("\nidentify API: mode gates, verify-before-hotload, coverage")
    from aiohttp.test_utils import TestClient, TestServer

    import camera as camera_mod
    import capture_server

    K, dist, T_true, tip, observations = synthetic_rig()
    fit = extrinsics.fit(observations, K, dist, extrinsics.FitOptions(image_size=(1600, 1200)))

    captures = workdir / "identify-captures"
    calib_dir = workdir / "identify-calib"
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")
    at = np.array([0.02, 0.01, 0.001])
    yaw_deg = -81.0                       # the tag's face turned toward the camera
    size_m = 0.12                         # ~97 px: solves on the correct branch here
    marker_at = "%g,%g,%g,%g" % (at[0], at[1], at[2], yaw_deg)
    # Eight, not six: run_refit reads one probe frame to settle the calibration
    # and the stream turn before the session starts, so the route consumes one
    # more still than captures+holdout asks for.
    stills = [render_marker(np.full((1200, 1600, 3), 210, np.uint8), T_true, K, dist,
                            at, yaw_deg, size_m) for _ in range(8)]
    blanks = [np.full((1200, 1600, 3), 210, np.uint8) for _ in range(8)]

    class TagEye:
        """A camera with a tag in it and no arm: what a §7.3a refit looks at."""

        def __init__(self, frames):
            self.frames = list(frames)
            self.closed = False

        def read(self, retries=1):
            return self.frames.pop(0) if self.frames else None

        def close(self):
            self.closed = True

    class CoverageArm(FakeArm):
        """FakeArm plus the two calls a coverage session makes (§7.4).

        `build_sequence` is the gateway's dry run: it builds the trajectory with
        the real interpolator and persists it, so `oracle_prepare` reads real hold
        windows. `move_sequence` records the id it was given, which is how the
        test proves the telemetry and the oracle agree on one plan.
        """

        def __init__(self, out_dir):
            super().__init__()
            self.out_dir = Path(out_dir)
            self.built, self.moved = [], []

        def build_sequence(self, targets, seconds, hold=0.0, gripper=None, plan_id=None):
            hand = float(self.q[6]) if gripper is None else float(gripper)
            points = [np.append(self.q[:6], hand)] + \
                     [np.append(np.asarray(t, dtype=float), hand) for t in targets]
            limits = [[float(lo), float(hi)] for lo, hi in rebot_ik.LIMITS] + [[-0.1, 5.0]]
            trajectory = traj.joint_sequence(points, seconds, hold_s=hold, limits=limits)
            self.out_dir.mkdir(parents=True, exist_ok=True)
            (self.out_dir / ("%s.json" % plan_id)).write_text(
                json.dumps(trajectory, separators=(",", ":")), encoding="utf-8")
            self.built.append(plan_id)
            return {"ok": True, "id": plan_id, "dry_run": True,
                    "waypoints": len(trajectory["waypoints"])}

        def move_sequence(self, targets, seconds, hold=0.0, gripper=None, plan_id=None):
            self.moved.append(plan_id)
            return {"ok": True, "id": plan_id, "poses": len(targets)}

    out_dir = workdir / "identify-out"
    arm = CoverageArm(out_dir)

    def args_for(execute, token="", marker=marker_at):
        return SimpleNamespace(
            host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
            token=token, mediamtx="http://127.0.0.1:9", whip_path="phone", no_whip=True,
            max_mb=4, quiet=True, planner="http://127.0.0.1:9", planner_token="",
            execute=execute, calib_dir=str(calib_dir), calib="",
            marker_at=marker, marker_size=size_m,
            hover="1", hover_m=0.10, tip_mm=35.0,
            plane_z=0.0, tool_axis="z-", yaw=0.0,
            extrinsics="", witness_max_px=2.0, camera="rtsp://fake/stream",
            mediamtx_api="http://127.0.0.1:9", recordings=str(calib_dir.parent / "recordings"),
            traces_dir=str(workdir / "identify-traces"),
        )

    original_gateway = auto_calibrate.ArmGateway
    original_source = camera_mod.open_source
    original_prepare = capture_server.oracle_prepare
    original_finish = capture_server.oracle_finish
    original_turn = capture_server.stream_turn
    auto_calibrate.ArmGateway = lambda *a, **k: arm
    # The stills are rendered in the calibration's own grid, so the route must not
    # turn them: the quarter-turn machinery is test_stream_turn's subject, and
    # here it would only rotate the tag out of the frame the solver expects.
    capture_server.stream_turn = lambda app_, calibration: 0
    try:
        async def scenarios():
            # ---- the gates: refit moves nothing, coverage moves the arm -----
            off = capture_server.build_app(args_for(False))
            off["rig"] = fit.rig
            camera_mod.open_source = lambda *a, **k: TagEye(stills)
            async with TestClient(TestServer(off)) as client:
                answer = await client.post("/identify/run",
                                           json={"confirm": True, "mode": "coverage"})
                check("coverage refuses to move the arm with execution off",
                      answer.status == 403)
                answer = await client.post("/identify/run",
                                           json={"confirm": True, "mode": "cartesian"})
                check("cartesian coverage refuses to move the arm with execution off",
                      answer.status == 403)
                answer = await client.post("/identify/run", json={"confirm": True})
                check("a refit needs no execute gate: it moves nothing (§7.3a)",
                      answer.status == 200, "status %s" % answer.status)
                answer = await client.post("/identify/sag", json={})
                check("the sag fit needs a fitted rig", answer.status == 422)

            tokened = capture_server.build_app(args_for(True, token="id-token"))
            async with TestClient(TestServer(tokened)) as client:
                answer = await client.post("/identify/run", json={"confirm": True})
                check("identify needs the write token", answer.status == 401)
                answer = await client.post("/identify/sag", json={})
                check("the sag fit needs the write token", answer.status == 401)

            app = capture_server.build_app(args_for(True))
            app["rig"] = fit.rig
            app["out_dir"] = out_dir
            async with TestClient(TestServer(app)) as client:
                answer = await client.post("/identify/run", json={})
                check("identify needs an explicit confirm", answer.status == 400)
                answer = await client.post("/identify/run", json={"confirm": True,
                                                                  "mode": "excite"})
                check("an unknown mode is refused", answer.status == 400)

                app["calibrating"] = True
                answer = await client.post("/identify/run", json={"confirm": True})
                check("identify is single-flight with the calibration flag",
                      answer.status == 409)
                app["calibrating"] = False

                status = await (await client.get("/identify")).json()
                check("GET /identify reports idle progress and proposals",
                      status.get("running") is False and status.get("progress") is None
                      and isinstance(status.get("proposals"), list))
                check("GET /identify names the sessions and their gates",
                      set(status.get("modes") or {}) == {"refit", "coverage", "cartesian"})
                check("GET /identify reports what is not learned yet (§7.2)",
                      {gap["quantity"] for gap in status.get("unlearned") or []}
                      >= {"accuracy-map"}, str(status.get("unlearned"))[:90])

                # §7.3b: /identify/sag fits the map from the telemetry corpus
                # and stores it keyed by the running rig. No motion, so no
                # execute gate -- but the map only applies after the hold-out
                # validation inside fit_sag_map.
                telemetry_dir = captures.parent / "telemetry"
                telemetry_dir.mkdir(parents=True, exist_ok=True)
                rng = np.random.default_rng(5)
                kp_vec = sweep.kp_vector()
                for i in range(40):
                    cmd = np.array([rng.uniform(lo, hi) for lo, hi in rebot_ik.LIMITS])
                    tau = rebot_ik.gravity_torques(cmd)
                    residual = np.array([0.10 / float(kp_vec[j]) * tau[j] + 0.001
                                         for j in range(6)])
                    meas = cmd - sweep.gravity_sag(cmd, kp_vec) - residual
                    samples = [{"t": i * 3.0 + k * 0.1,
                                "cmd": [float(v) for v in cmd] + [0.0],
                                "meas": [float(v) for v in meas] + [0.0]}
                               for k in range(12)]
                    (telemetry_dir / ("move-%02d.json" % i)).write_text(
                        json.dumps({"samples": samples}), encoding="utf-8")
                answer = await client.post("/identify/sag", json={})
                body = await answer.json()
                check("/identify/sag fits and stores the keyed map",
                      answer.status == 200 and body.get("ok") is True
                      and (calib_dir / ("sag-%s.json" % app["rig"].fingerprint)).is_file(),
                      "status %s, %s" % (answer.status, str(body.get("reasons"))[:60]))

                # ---- §7.3a the refit through the route ----------------------
                before = app["rig"].fingerprint
                rig_file = calib_dir / "extrinsics-fixed.json"
                kept = rig_file.read_bytes() if rig_file.exists() else None
                camera_mod.open_source = lambda *a, **k: TagEye(blanks)
                answer = await client.post("/identify/run",
                                           json={"confirm": True, "captures": 5,
                                                 "holdout": 1, "settle": 0.0})
                body = await answer.json()
                check("a refit that cannot see the tag is refused",
                      answer.status == 422 and body.get("ok") is False,
                      "status %s, %s" % (answer.status, str(body.get("error"))[:60]))
                check("a refused refit leaves the running rig untouched",
                      app["rig"].fingerprint == before
                      and (rig_file.read_bytes() if rig_file.exists() else None) == kept)

                camera_mod.open_source = lambda *a, **k: TagEye(stills)
                answer = await client.post("/identify/run",
                                           json={"confirm": True, "captures": 5,
                                                 "holdout": 1, "settle": 0.0})
                body = await answer.json()
                check("a verified refit writes and hot-loads the rig",
                      answer.status == 200 and body.get("ok") is True and body.get("written")
                      and (calib_dir / "extrinsics-fixed.json").is_file(),
                      "status %s, %s" % (answer.status,
                                          "; ".join((body.get("verification") or {})
                                                    .get("reasons", []))[:70]))
                check("the hot-loaded rig is the averaged tag solve",
                      app["rig"].fingerprint != before
                      and app["rig"].method == "base-marker")
                check("the refit reports what the tag can and cannot identify",
                      (body.get("identifiability") or {}).get("status") == "ok"
                      and (body.get("identifiability") or {}).get("not_identifiable"))
                check("the refit verified on a held-out capture",
                      bool((body.get("verification") or {}).get("ok")) and body.get("held_out"))
                check("the refit moved nothing", body.get("motion") is False
                      and arm.moved == [] and arm.built == [])
                status = await (await client.get("/identify")).json()
                check("GET /identify reports the success",
                      (status.get("last") or {}).get("state") == "ok"
                      and (status.get("last") or {}).get("mode") == "refit")

                # ---- §7.4 coverage: motion to measure, never to fit ---------
                async def no_prepare(app_, plan_id):
                    calls.append(("prepare", plan_id))
                    return None

                async def fake_prepare(app_, plan_id):
                    calls.append(("prepare", plan_id))
                    return {"plan_id": plan_id, "windows": [{"index": 1}], "holds": {}}

                async def fake_finish(app_, prep, sid):
                    calls.append(("finish", prep["plan_id"]))

                calls = []
                capture_server.oracle_prepare = no_prepare
                try:
                    answer = await client.post("/identify/run",
                                               json={"confirm": True, "mode": "coverage"})
                    body = await answer.json()
                    check("coverage refuses when the oracle cannot measure",
                          answer.status == 422 and "measured" in str(body.get("error")),
                          str(body.get("error"))[:80])
                    check("a refused coverage run never moved the arm",
                          arm.moved == [] and len(arm.built) == 1,
                          "built %s moved %s" % (arm.built, arm.moved))
                finally:
                    capture_server.oracle_prepare = original_prepare

                capture_server.oracle_prepare = fake_prepare
                capture_server.oracle_finish = fake_finish
                try:
                    calls.clear()
                    arm.built.clear()
                    arm.moved.clear()
                    rig_before = app["rig"].fingerprint
                    answer = await client.post("/identify/run",
                                               json={"confirm": True, "mode": "coverage",
                                                     "poses": 5, "hold": 2.0})
                    body = await answer.json()
                    check("coverage runs the gated trajectory",
                          answer.status == 200 and body.get("ok") is True
                          and body.get("mode") == "coverage",
                          "status %s, %s" % (answer.status, str(body.get("error"))[:60]))
                    check("coverage builds, then prepares, then moves, then measures",
                          [name for name, _id in calls] == ["prepare", "finish"]
                          and arm.built and arm.moved
                          and arm.built[0] == calls[0][1] == arm.moved[0] == calls[1][1],
                          "%s built=%s moved=%s" % (calls, arm.built, arm.moved))
                    check("coverage names the cells it measured",
                          bool(body.get("cells"))
                          and len(body["cells"]) == body.get("poses")
                          and all(cell.startswith("r") for cell in body["cells"]),
                          str(body.get("cells")))
                    check("coverage holds long enough for the oracle's grabs",
                          float(body.get("hold") or 0.0) >= oracle.HOLD_MIN_S)
                    check("coverage writes no rig (§7.4)",
                          body.get("writes_rig") is False and body.get("written") is None
                          and app["rig"].fingerprint == rig_before)
                    check("coverage legs obey the speed policy",
                          len(body.get("leg_seconds") or []) == len(body.get("targets") or [])
                          and all(value > 0.0 for value in body["leg_seconds"]),
                          str(body.get("leg_seconds")))
                    status = await (await client.get("/identify")).json()
                    check("GET /identify reports the coverage run",
                          (status.get("last") or {}).get("mode") == "coverage"
                          and (status.get("last") or {}).get("state") == "ok")
                finally:
                    capture_server.oracle_prepare = original_prepare
                    capture_server.oracle_finish = original_finish

                # ---- §7.4 cartesian coverage: the task planner, exactly -----
                # The route grows the sequence through planner_call /plan (the
                # same call /sequence makes) and executes the last plan that
                # worked. The fake answers them; the oracle hooks stay the
                # coverage section's fakes.
                planner_calls = []
                screen = {"refuse_all": False, "points": set()}

                async def fake_planner_call(app_, path, body, timeout=120,
                                            method="POST", sid=None):
                    planner_calls.append((path, body))
                    if path == "/plan":
                        targets = body.get("targets") or []
                        if (screen["refuse_all"]
                                or any(tuple(t["tip"]) in screen["points"] for t in targets)):
                            return 422, {"ok": False,
                                         "error": "IK failed at the marker: out of reach"}
                        return 200, {"ok": True, "id": "cart-seq-%d" % len(planner_calls),
                                     "summary": {"markers": len(targets),
                                                 "worst_ik_m": 0.0012,
                                                 "duration_s": 42.0, "waypoints": 1300},
                                     "execute": True}
                    if path == "/execute":
                        return 200, {"ok": True, "id": body.get("id"), "executed": True,
                                     "telemetry": {"samples": []}}
                    return 404, {"ok": False, "error": "no such route"}

                original_planner_call = capture_server.planner_call
                capture_server.planner_call = fake_planner_call
                capture_server.oracle_prepare = fake_prepare
                capture_server.oracle_finish = fake_finish
                try:
                    calls.clear()
                    planner_calls.clear()
                    rig_before = app["rig"].fingerprint
                    answer = await client.post("/identify/run",
                                               json={"confirm": True, "mode": "cartesian",
                                                     "poses": 6})
                    body = await answer.json()
                    check("cartesian coverage runs the task planner's sequence",
                          answer.status == 200 and body.get("ok") is True
                          and body.get("mode") == "cartesian",
                          "status %s, %s" % (answer.status, str(body.get("error"))[:80]))
                    plans = [b for p, b in planner_calls if p == "/plan"]
                    executes = [b for p, b in planner_calls if p == "/execute"]
                    check("cartesian grows the sequence one candidate at a time",
                          len(plans) == 6 and all(p.get("gripper") == "alternate" for p in plans)
                          and [len(p["targets"]) for p in plans] == [1, 2, 3, 4, 5, 6]
                          and len(executes) == 1,
                          "plans %d executes %d" % (len(plans), len(executes)))
                    check("cartesian executes the last plan that worked",
                          executes[0].get("id") == "cart-seq-6", str(executes))
                    check("cartesian prepares before it executes, then measures",
                          [name for name, _id in calls] == ["prepare", "finish"]
                          and calls[0][1] == calls[1][1] == executes[0].get("id"),
                          str(calls))
                    check("cartesian targets round-trip to the cells they name",
                          len(body.get("cells") or []) == body.get("poses") == 6
                          and len(body.get("targets") or []) == 6
                          and all(identify.accuracy_cell(t) == c
                                  for t, c in zip(body["targets"], body["cells"])),
                          str(body.get("cells")))
                    check("cartesian reports the unreachable gaps honestly",
                          (body.get("skipped") or {}).get("below_min_z") == 40,
                          str(body.get("skipped")))
                    check("cartesian writes no rig (§7.4)",
                          body.get("writes_rig") is False
                          and app["rig"].fingerprint == rig_before)

                    # One candidate the IK cannot reach: every trial containing
                    # it fails, the rest still grow into a full session.
                    planner_calls.clear()
                    calls.clear()
                    first_tip = tuple(body["targets"][0])
                    screen["points"].add(first_tip)
                    answer = await client.post("/identify/run",
                                               json={"confirm": True, "mode": "cartesian",
                                                     "poses": 6})
                    body = await answer.json()
                    check("an unreachable candidate is screened out, not fatal",
                          answer.status == 200 and len(body.get("refused") or []) == 1
                          and body.get("poses") == 6,
                          "status %s refused %s" % (answer.status, body.get("refused")))
                    sequence = [b for p, b in planner_calls if p == "/plan"][-1]
                    check("the executed sequence skips the refused cell",
                          all(tuple(t["tip"]) != first_tip for t in sequence["targets"]))
                    screen["points"].clear()

                    # Every candidate unreachable: refused before any motion.
                    planner_calls.clear()
                    screen["refuse_all"] = True
                    answer = await client.post("/identify/run",
                                               json={"confirm": True, "mode": "cartesian"})
                    body = await answer.json()
                    check("a fully unreachable grid is refused without motion",
                          answer.status == 422
                          and not any(p == "/execute" for p, _b in planner_calls),
                          "status %s" % answer.status)
                    screen["refuse_all"] = False

                    # No oracle, no motion: same rule as the joint-space form.
                    planner_calls.clear()
                    capture_server.oracle_prepare = no_prepare
                    try:
                        answer = await client.post("/identify/run",
                                                   json={"confirm": True, "mode": "cartesian"})
                        body = await answer.json()
                        check("cartesian refuses when the oracle cannot measure",
                              answer.status == 422
                              and not any(p == "/execute" for p, _b in planner_calls),
                              "status %s" % answer.status)
                    finally:
                        capture_server.oracle_prepare = fake_prepare
                finally:
                    capture_server.planner_call = original_planner_call
                    capture_server.oracle_prepare = original_prepare
                    capture_server.oracle_finish = original_finish

        asyncio.run(scenarios())
    finally:
        auto_calibrate.ArmGateway = original_gateway
        camera_mod.open_source = original_source
        capture_server.oracle_prepare = original_prepare
        capture_server.oracle_finish = original_finish
        capture_server.stream_turn = original_turn



def test_accuracy_map(workdir: Path):
    print("\naccuracy envelope: validated cells, sequence warnings, health")
    from aiohttp.test_utils import TestClient, TestServer

    import capture_server

    map_path = workdir / "accuracy-map.json"
    tip_a = [0.35, 0.05, 0.15]
    tip_b = [0.55, -0.20, 0.30]

    def hold(tip, error):
        return {"pixel_error_mm": error, "tip_fk_cmd": tip}

    cell_a = identify.accuracy_cell(tip_a)
    doc = identify.update_accuracy_map(map_path, {"holds": [hold(tip_a, 4.0),
                                                            hold(tip_a, 5.0)]})
    check("two holds do not validate a cell",
          doc["cells"][cell_a]["n"] == 2 and doc["cells"][cell_a]["validated_at"] is None)
    doc = identify.update_accuracy_map(map_path, {"holds": [hold(tip_a, 3.0)]})
    check("three good holds validate the cell",
          doc["cells"][cell_a]["validated_at"] is not None)
    doc = identify.update_accuracy_map(map_path, {"holds": [hold(tip_a, 15.0)]})
    check("an over-threshold hold un-validates the cell",
          doc["cells"][cell_a]["validated_at"] is None)
    doc = identify.update_accuracy_map(map_path, {"holds": [hold(tip_a, 2.0),
                                                            hold(tip_a, 3.0),
                                                            hold(tip_a, 4.0),
                                                            hold(tip_b, 5.0)]})
    check("the envelope's validated fraction is reported",
          abs(identify.validated_fraction(doc) - 0.5) < 1e-9,
          "%.2f" % identify.validated_fraction(doc))
    warnings = identify.accuracy_warnings(doc, [tip_a, tip_b])
    check("only the unvalidated cell warns",
          len(warnings) == 1 and "target 2" in warnings[0], str(warnings))

    # The server consults the map: a warning in the plan summary, a fraction in
    # /health, and a broken map degrades to neither failing nor warning.
    K, dist, _T, _tip, _obs = synthetic_rig()
    captures = workdir / "accuracy-captures"
    calib_dir = workdir / "accuracy-calib"
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1600, 1200], "K": K.tolist(), "dist": dist.tolist(), "rms": 0.2,
    }), encoding="utf-8")
    args = SimpleNamespace(
        host="127.0.0.1", port=0, captures=str(captures), cert="", key="",
        token="acc-token", mediamtx="http://127.0.0.1:9", whip_path="phone", no_whip=True,
        max_mb=4, quiet=True, planner="http://127.0.0.1:9", planner_token="",
        execute=False, calib_dir=str(calib_dir), calib="",
        marker_at="", marker_size=0.08,
        hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="",
        traces_dir=str(workdir / "accuracy-traces"),
    )
    app = capture_server.build_app(args)
    app["accuracy_map_path"] = map_path

    async def fake_planner_call(app_, path, body, timeout=120, method="POST", sid=None):
        return 200, {"id": "fake-accuracy", "execute": False,
                     "summary": {"markers": len(body.get("targets") or [])}}

    real_planner_call = capture_server.planner_call
    capture_server.planner_call = fake_planner_call
    headers = {"X-Rebot-Token": "acc-token"}
    try:
        async def scenarios():
            async with TestClient(TestServer(app)) as client:
                answer = await client.post("/sequence",
                                           json={"targets": [{"tip": tip_a}, {"tip": tip_b}]},
                                           headers=headers)
                body = await answer.json()
                summary_warnings = ((body.get("plan") or {}).get("summary") or {}).get("warnings")
                check("an unvalidated target warns in the plan summary",
                      answer.status == 200 and summary_warnings
                      and any("target 2" in w for w in summary_warnings),
                      str(summary_warnings))
                health = await (await client.get("/health")).json()
                check("/health reports the validated fraction",
                      abs((health.get("accuracy") or {}).get("validated_fraction", -1)
                          - 0.5) < 1e-9
                      and (health.get("accuracy") or {}).get("cells") == 2,
                      str(health.get("accuracy")))
                map_path.write_text("{not json", encoding="utf-8")
                answer = await client.post("/sequence",
                                           json={"targets": [{"tip": tip_a}]},
                                           headers=headers)
                check("a broken accuracy map never fails /sequence",
                      answer.status == 200)
                health = await (await client.get("/health")).json()
                check("a broken accuracy map degrades health to zero",
                      (health.get("accuracy") or {}).get("validated_fraction") == 0.0)
        asyncio.run(scenarios())
    finally:
        capture_server.planner_call = real_planner_call


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
    test_oracle_rules()
    test_trace_triage(workdir)
    test_planner_offline(workdir)
    test_mock_tape(workdir)
    if not args.skip_server:
        asyncio.run(test_server(workdir, geometry))
        asyncio.run(test_trace_recorder(workdir))
        asyncio.run(test_trace_server(workdir, geometry))
        asyncio.run(test_oracle_server(workdir, geometry))
        bundle_ctx = asyncio.run(test_trace_bundle(workdir, geometry))
        test_trace_live(workdir, geometry, bundle_ctx)
    if calibration:
        test_locate_cli(workdir, geometry)
    test_rotation(workdir, geometry)
    test_fixed_rig(workdir)
    test_base_marker()
    test_marker_still_commit_flag(workdir)
    test_calib_dir_scanners(workdir)
    test_hold_detection_gappy_log(workdir)
    test_unpark_peel(workdir)
    test_auto_sweep_turned_stream(workdir)
    test_auto_sweep(workdir)
    test_calibrate_api(workdir)
    test_marker_rig(workdir)
    test_identify_gate(workdir)
    test_sag_map(workdir)
    test_identify_api(workdir)
    test_cartesian_targets(workdir)
    test_accuracy_map(workdir)

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
