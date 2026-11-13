#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fit a fixed camera's pose from the arm, starting from one tap.

One process:

    1. take a still, with the pose the arm actually had
       python app/calibrate_extrinsics.py --capture-pose --camera rtsp://...

    2. look at it, read the tool tip's pixel, then let the arm sweep
       python app/calibrate_extrinsics.py --auto --camera rtsp://... \\
           --from state/captures/<id> --uv 812 455

    3. audit the result
       python app/calibrate_extrinsics.py --check

Step 2 moves the arm through the eight checked poses in `sweep.py`, takes a
still at each with the joint angles attached, and fits `T_base<-cam`.

The tap is what makes the sweep possible. It is two constraints, not a pose: it
fixes the ray back from a known 3D point, so the camera can sit anywhere on that
ray at any distance and any roll. `auto_calibrate.bootstrap_from_click()` fills
that family in coarsely and keeps the member whose eight detections agree with
each other -- the measurement decides, not a guess about where the bracket was
bolted. There is no camera position to type and no second workflow.

The rig is written only if the fit verifies: `--check` reprints the numbers it
was judged on, and every trajectory carries the rig's fingerprint.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
ROOT = HERE.parent
STATE = ROOT / "state"

import base_marker  # noqa: E402
import extrinsics as ex  # noqa: E402
import mediamtx  # noqa: E402
import rebot_ik  # noqa: E402
import witness  # noqa: E402


class CliError(Exception):
    """The command cannot do its job; the message is for the operator."""


def resolve(path):
    """A path from the command line, relative to the host directory."""
    path = Path(path)
    return path if path.is_absolute() else (ROOT / path)


def display(path):
    """A path relative to the host directory, for stable logs."""
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except (ValueError, OSError):
        return str(path)


def default_calib(folder):
    """The newest intrinsics file, never the rig itself."""
    candidates = sorted(Path(folder).glob("*.json"))
    candidates = [p for p in candidates if not p.name.startswith(("extrinsics-", "witness-"))]
    return candidates[-1] if candidates else None


def joints_from_meta(meta):
    """The arm pose a capture was taken at, whichever block the server wrote."""
    for block in (meta.get("server", {}).get("pose"), meta.get("pose"), meta.get("joints")):
        if not isinstance(block, dict):
            continue
        q = block.get("q")
        if not isinstance(q, (list, tuple)) or len(q) < 6:
            continue
        ids = block.get("ids")
        if isinstance(ids, (list, tuple)) and len(ids) >= 6 \
                and [int(v) for v in ids[:6]] != [1, 2, 3, 4, 5, 6]:
            raise CliError("the capture's pose is motors %s, not 1-6" % list(ids[:6]))
        return [float(v) for v in q[:6]], block
    return None, None


def _load_calibration(args, size=None):
    import calib as calib_mod

    path = args.calib or default_calib(STATE / "calib")
    if not path:
        raise CliError("no intrinsics found: run calibrate_intrinsics.py first, or pass --calib")
    return calib_mod.load(resolve(path), image_size=tuple(int(v) for v in size) if size else None)


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #

def command_capture_pose(args):
    """One still from the fixed camera, with the pose the arm actually had.

    This is the first half of the calibration: the still is what the operator
    taps. It deliberately does not touch the session or the rig.
    """
    import auto_calibrate as auto
    import camera as camera_mod

    url = args.camera or os.environ.get("REBOT_CAMERA", "")
    if not url:
        raise CliError("--capture-pose needs --camera (or REBOT_CAMERA)")
    eye = camera_mod.open_source(url)
    try:
        frame = eye.read()
    finally:
        eye.close()

    arm = auto.ArmGateway(args.planner or os.environ.get("REBOT_PLANNER",
                                                        "http://127.0.0.1:8000"),
                          args.planner_token or os.environ.get("REBOT_PLANNER_TOKEN", ""))
    first = arm.pose()
    time.sleep(0.2)
    second = arm.pose()
    moved = max(abs(float(a) - float(b)) for a, b in zip(first["q"], second["q"]))
    if moved > 0.01:
        raise CliError("the arm moved while the frame was taken (%.0f mrad): hold it still "
                       "and capture again" % (1000.0 * moved))

    capture_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
    directory = STATE / "captures" / capture_id
    directory.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(directory / "frame.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    (directory / "meta.json").write_text(json.dumps({
        "purpose": "extrinsics",
        "image_size": [int(frame.shape[1]), int(frame.shape[0])],
        "server": {
            "id": capture_id,
            "pose": first,
            "captured_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "source": "capture-pose",
        },
    }, indent=2) + "\n", encoding="utf-8")

    print("capture   : %s" % display(directory))
    print("pose      : %s" % " ".join("%+.3f" % float(v) for v in first["q"][:6]))
    print("next      : open %s, read the tool tip's pixel, then"
          % display(directory / "frame.jpg"))
    print("            %s --auto --from %s --uv U V"
          % ("calibrate_extrinsics.py", display(directory)))
    return 0


def command_auto(args):
    import auto_calibrate as auto
    import camera as camera_mod

    url = args.camera or os.environ.get("REBOT_CAMERA", "")
    if not url:
        raise CliError("--auto needs --camera (or REBOT_CAMERA)")
    if not args.from_capture or not args.uv:
        raise CliError("--auto needs --from <capture> --uv U V: one still with the tool tip "
                       "tapped in it (see --capture-pose)")

    directory = resolve(args.from_capture)
    meta_path = directory / "meta.json"
    if not meta_path.is_file():
        raise CliError("no capture at %s" % directory)
    q, _block = joints_from_meta(json.loads(meta_path.read_text(encoding="utf-8")))
    if q is None:
        raise CliError("that capture has no pose: take it with --capture-pose")

    out = resolve(args.out or (STATE / "calib" / "extrinsics-fixed.json"))
    marker = base_marker.parse_spec(getattr(args, 'marker_at', ''), getattr(args, 'marker_size', base_marker.SIZE_M))
    options = auto.Options(
        plane_z=float(args.plane_z if args.plane_z is not None else 0.0),
        settle_s=float(args.settle),
        work_radius=float(args.work_radius),
        click=None if marker else {"q": q, "uv": (float(args.uv[0]), float(args.uv[1])),
                                   "image": str(directory / "frame.jpg")},
        marker=marker,
    )

    # latency=0: the sweep records continuously and picks frames afterwards.
    eye = camera_mod.open_source(url, latency=0.0)
    try:
        # One frame first: the intrinsics have to be loaded at the size the
        # stream actually delivers, not the size the file was fitted at.
        probe = eye.read()
        size = (probe.shape[1], probe.shape[0])
        calibration = _load_calibration(args, size=size)
        arm = auto.ArmGateway(args.planner or os.environ.get("REBOT_PLANNER",
                                                            "http://127.0.0.1:8000"),
                              args.planner_token or os.environ.get("REBOT_PLANNER_TOKEN", ""))
        path = mediamtx.path_from_camera(url)
        if not path:
            raise CliError("the sweep is recorded by MediaMTX, so --camera must be its RTSP "
                           "path (rtsp://mediamtx:8554/phone); %r cannot be recorded" % url)
        recorder = mediamtx.Recorder(args.mediamtx_api, path=path,
                                     recordings=resolve(args.recordings or (STATE / "recordings")))
        print("sweep     : 8 poses, about 90 s of motion; the arm will move now", flush=True)
        result = auto.run(arm, eye, calibration, options, STATE / "captures",
                          recorder=recorder, mount="fixed")
    finally:
        eye.close()

    _rig, verdict, written = auto.commit(result, calibration, options, out,
                                         calib_path=args.calib, mount="fixed")
    payload = auto.report(result, verdict)
    payload["written"] = str(written) if written else None

    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(describe_auto(payload))
        print("wrote     : %s" % (display(written) if written else "nothing (verdict refused)"))
    return 0 if written else 1


def describe_auto(payload):
    verification = payload.get("verification") or {}
    predicted = verification.get("predicted_mm") or {}
    lines = [
        "sweep     : %d poses in %.0f s of motion" % (payload.get("poses", 0),
                                                      payload.get("seconds", 0.0)),
        "camera    : %s" % payload.get("camera", "?"),
        "tip       : %s mm in gripper_end" % payload.get("tip_mm"),
        "fit       : rms %.2f px, worst held-out %s px"
        % (verification.get("rms_px", 0.0),
           "n/a" if verification.get("worst_px") is None else "%.2f" % verification["worst_px"]),
        "predicted : %s mm at the centre, %s mm at the corner"
        % (predicted.get("centre_mm"), predicted.get("corner_mm")),
    ]
    for warning in payload.get("warnings") or []:
        lines.append("warning   : %s" % warning)
    if verification.get("ok"):
        lines.append("verdict   : ok")
    else:
        lines.append("verdict   : REFUSED -- %s" % "; ".join(verification.get("reasons") or []))
    return "\n".join(lines)


def command_check(args):
    """Audit a written rig: residuals, held-out, predicted error, age."""
    rig_path = resolve(args.out or (STATE / "calib" / "extrinsics-fixed.json"))
    rig = ex.load_rig(rig_path)
    observations = rig.observations
    if not observations:
        raise CliError("the rig at %s has no observations to audit" % display(rig_path))

    calibration = _load_calibration(args, size=rig.image_size)
    ex.verify_pair(calibration, rig)
    K, dist = calibration.K, calibration.dist
    T_cam_base = ex.invert(rig.T_base_cam)

    residual_px = []
    print("pose  capture                       residual")
    for index, observation in enumerate(observations, start=1):
        T = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        point = T[:3, :3] @ np.asarray(rig.tip_offset, dtype=float) + T[:3, 3]
        seen = ex.pixel(point, K, dist, T_cam_base)
        if seen is None:
            error = float("inf")
        else:
            error = math.hypot(seen[0] - observation.uv[0], seen[1] - observation.uv[1])
        residual_px.append(error)
        flag = "  <-- check this pose" if error > 3.0 else ""
        print("%4d  %-28s %7.2f px%s" % (index, observation.capture or "-", error, flag))

    residual_px = np.asarray(residual_px, dtype=float)
    print("\nrms       : %.2f px" % float(np.sqrt(np.mean(residual_px ** 2))))
    print("worst     : %.2f px" % float(np.max(residual_px)))

    loo = ex.leave_one_out(observations, K, dist,
                           ex.FitOptions(plane_z=rig.plane_z, image_size=rig.image_size))
    if loo is not None:
        print("leave-one-out: %.2f px mean, %.2f px worst"
              % (float(np.mean(loo)), float(np.max(loo))))

    predicted = rig.predicted or {}
    if predicted.get("centre_mm") is not None:
        print("predicted : %.1f mm at the centre, %.1f mm at the corner"
              % (predicted["centre_mm"], predicted["corner_mm"]))
    print("camera    : %s" % rig.describe().splitlines()[0])
    weak = rig.weakest or {}
    if weak:
        print("condition : weakest %s (%.2f mm)" % (weak.get("direction", "?"),
                                                    weak.get("sigma_mm", 0.0)))
    if rig.age_days is not None:
        print("age       : %.0f days" % rig.age_days)
    try:
        reference = witness.load_for(rig, rig_path.parent)
    except witness.WitnessError as error:
        reference = None
        print("witness   : UNUSABLE (%s)" % error)
    if reference is not None:
        print("witness   : %d patches from %s" % (len(reference.points), display(reference.path)))

    if args.json:
        print(json.dumps({"rig": rig.provenance(),
                          "residuals_px": [round(v, 2) for v in residual_px],
                          "rms_px": round(float(np.sqrt(np.mean(residual_px ** 2))), 2)},
                         indent=2))
    if float(np.max(residual_px)) > 6.0:
        print("\nwarning: at least one pose is 6 px off. Re-run the sweep, or give the "
              "detector a cleaner background (--background / REBOT_CAMERA).")
        return 1
    return 0


# --------------------------------------------------------------------------- #

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Fit a fixed camera's pose from the arm, starting from one tap.")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--capture-pose", action="store_true",
                        help="one still + pose, for the tap (the first half)")
    action.add_argument("--auto", action="store_true",
                        help="sweep the arm, fit, verify (needs --from and --uv)")
    action.add_argument("--check", action="store_true", help="audit a fitted rig")

    parser.add_argument("--from", dest="from_capture", default="",
                        help="the capture holding the tapped tool tip")
    parser.add_argument("--uv", nargs=2, type=float, metavar=("U", "V"),
                        help="the tool tip's pixel in that capture")
    parser.add_argument("--marker-at", default=os.environ.get("REBOT_MARKER_AT", ""),
                        help="'x,y,z' or 'x,y,z,yaw' of the base marker in the robot's frame "
                             "(metres, radians): with it, no tap is needed")
    parser.add_argument("--marker-size", type=float,
                        default=float(os.environ.get("REBOT_MARKER_SIZE", "0.04")),
                        help="the marker's printed side in metres (default 0.04)")
    parser.add_argument("--camera", default=os.environ.get("REBOT_CAMERA", ""),
                        help="rtsp://, http:// snapshot, or device:N")
    parser.add_argument("--mediamtx-api", default=os.environ.get("REBOT_MEDIAMTX_API",
                                                                 "http://127.0.0.1:9997"),
                        help="MediaMTX control API, used to record the sweep")
    parser.add_argument("--recordings", default=os.environ.get("REBOT_RECORDINGS", ""),
                        help="where MediaMTX writes its recordings")
    parser.add_argument("--calib", default="", help="intrinsics JSON (default: newest)")
    parser.add_argument("--out", default="", help="rig JSON (default: state/calib/extrinsics-fixed.json)")
    parser.add_argument("--plane-z", type=float, default=None, help="worktop height in base frame")
    parser.add_argument("--work-radius", type=float, default=0.55,
                        help="radius of the grid the predicted error is reported over, m")
    parser.add_argument("--settle", type=float, default=0.5,
                        help="seconds to wait at each pose before the still, s")
    parser.add_argument("--planner", default=os.environ.get("REBOT_PLANNER",
                                                           "http://127.0.0.1:8000"),
                        help="the arm gateway base URL")
    parser.add_argument("--planner-token", default=os.environ.get("REBOT_PLANNER_TOKEN", ""))
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        if args.capture_pose:
            return command_capture_pose(args)
        if args.auto:
            return command_auto(args)
        return command_check(args)
    except (CliError, ex.ExtrinsicsError) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    sys.exit(main())
