#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks for the phone-capture pipeline: no phone, no arm, no Pinocchio.

Three things fail quietly and cost a day on the arm: a wrong intrinsics fit, a
wrong tag-to-plane transform, and a trajectory JSON the player refuses at the
worst moment. This rebuilds all three from synthetic images with a known camera
and checks the numbers against ground truth, then round-trips a capture through
the real HTTP server.

    python examples/host/capture/selftest.py

Everything here is deterministic. If it fails, the pipeline is wrong, not the
measurement.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import calib as calib_mod  # noqa: E402
import calibrate_intrinsics as calib_cli  # noqa: E402
import locate_target as locate  # noqa: E402
import traj  # noqa: E402

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
    tag_x, tag_y, tag_yaw = 0.36, -0.10, 12.0

    T_base_cam = look_at((0.55, -0.28, 0.92), (0.32, -0.06, 0.0))
    T_base_tag = locate.tag_in_base(tag_x, tag_y, 0.0, tag_yaw)

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
        tag_size=0.080, tag_x=0.36, tag_y=-0.10, tag_yaw=12.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
    )
    app = capture_server.build_app(args)

    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    jpeg = buffer.tobytes()

    async with TestClient(TestServer(app)) as client:
        health = await (await client.get("/health")).json()
        check("health is ok", health.get("ok") is True and health.get("auth") is True)
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
    tag_x, tag_y, tag_yaw, tag_size = 0.36, -0.10, 12.0, 0.080
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
        "--tag-size", str(tag_size), "--tag-x", str(tag_x), "--tag-y", str(tag_y),
        "--tag-yaw", str(tag_yaw), "--annotate", str(capture / "annotated.jpg"),
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
