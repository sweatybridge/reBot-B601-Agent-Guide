#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTPS endpoint for the phone page: capture upload, WHIP/WHEP relay, index.

The phone needs a secure origin for `getUserMedia`, so this server is the one
TLS endpoint the phone talks to; it also relays the WebRTC signaling to MediaMTX
so the page is same-origin and MediaMTX can stay on plain loopback. Nothing here
touches the arm -- the output is a directory on disk that the planner reads.

    python examples/host/app/capture_server.py --cert tls/cert.pem --key tls/key.pem

Writes one directory per capture:

    captures/20261109T023512Z-a1b2c3/
        frame.jpg     the still, exactly as the phone encoded it
        meta.json     tap, device settings, timestamps, and this server's record

The server adds a `server` block and refuses to let the client set `id`,
`frame` or `server`, so a capture cannot be made to look like another one.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import hmac
import json
import math
import logging
import os
import re
import socket
import ssl
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from aiohttp import ClientSession, ClientTimeout, web

# The same locate code the CLI runs: a tap and a shell command cannot drift.
import auto_calibrate
import base_marker
import camera as camera_mod
import calib as calib_mod
import extrinsics
import locate_target as locator
import mediamtx
import sweep
import witness

VERSION = "0.2.0"
HERE = Path(__file__).resolve().parent      # app/
ROOT = HERE.parent                          # the host directory
WEB_DIR = ROOT / "web"
STATE = ROOT / "state"                       # captures, calib, tls, tags, out, work

CAPTURE_ID = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{6}$")
SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
PURPOSES = {"plan", "calib", "extrinsics", "debug"}
RESERVED_META = {"id", "frame", "server"}


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def new_capture_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]


def lan_ip() -> str:
    """Best-effort LAN address for the banner; no packet is sent."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def capture_dirs(root: Path):
    if not root.is_dir():
        return []
    return sorted((p for p in root.iterdir() if p.is_dir() and CAPTURE_ID.match(p.name)),
                  key=lambda p: p.name, reverse=True)


def read_meta(directory: Path) -> dict:
    path = directory / "meta.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {}


@web.middleware
async def json_errors(request: web.Request, handler):
    """Every error leaves as JSON, with its reason in `error`.

    aiohttp's `HTTPUnprocessableEntity(text=...)` answers `text/plain`, and the
    panel parses JSON -- so a 422 with a perfectly good explanation ("no
    calibration in /app/state/calib: run calibrate_intrinsics.py once") arrived
    as the useless "server said 422". JSON is the contract for this API.
    """
    try:
        return await handler(request)
    except web.HTTPException as error:
        if error.headers.get("Content-Type", "").startswith("application/json"):
            raise
        message = (error.text or "").strip() or error.reason or ("HTTP %d" % error.status)
        return web.json_response({"ok": False, "error": message}, status=error.status)


def token_middleware(token: str):
    """Reject writes without the shared token; reads stay open on the LAN."""
    @web.middleware
    async def middleware(request, handler):
        if token and request.method not in ("GET", "HEAD", "OPTIONS"):
            provided = request.headers.get("X-Rebot-Token", "")
            if not hmac.compare_digest(provided, token):
                return web.json_response({"ok": False, "error": "missing or wrong X-Rebot-Token"},
                                         status=401)
        return await handler(request)
    return middleware


async def handle_index(request: web.Request) -> web.StreamResponse:
    return await _page(request, "phone.html")


async def handle_live(request: web.Request) -> web.StreamResponse:
    return await _page(request, "live.html")


async def handle_viewer_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "viewer.js"
    if not script.is_file():
        return web.Response(text="viewer.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_reach_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "reach.js"
    if not script.is_file():
        return web.Response(text="reach.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_aruco_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "aruco.js"
    if not script.is_file():
        return web.Response(text="aruco.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_markers_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "markers.js"
    if not script.is_file():
        return web.Response(text="markers.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def _page(request: web.Request, name: str) -> web.StreamResponse:
    page = WEB_DIR / name
    if not page.is_file():
        return web.Response(text="%s is missing next to capture_server.py" % name, status=500)
    return web.Response(text=page.read_text(encoding="utf-8"), content_type="text/html",
                        headers={"Cache-Control": "no-store"})


async def handle_health(request: web.Request) -> web.Response:
    app = request.app
    captures = capture_dirs(app["captures"])
    return web.json_response({
        "ok": True,
        "version": VERSION,
        "server_time": utc_now(),
        "captures": len(captures),
        "latest": captures[0].name if captures else None,
        "whip": app["whip_url"],
        "whep": app["whep_url"],
        "tls": app["tls"],
        "auth": bool(app["token"]),
        "planner": app["planner"],
        "execute": app["execute"],
        "calib": sorted(p.name for p in app["calib"].glob("*.json")
                        if not p.name.startswith(("extrinsics-", "witness-"))),
        "mode": "fixed" if app["rig"] is not None else "handheld",
        "rig": (None if app["rig"] is None else
                dict(app["rig"].provenance(), path=str(app["extrinsics_path"]),
                     witness=bool(app["witness"]))),
        "tag": (None if app.get("marker") is None else
                {"size_m": app["marker"]["size_m"], "at": list(app["marker"]["at"]),
                 "yaw_deg": math.degrees(app["marker"]["yaw"])}),
        "plane_z": app["plane_z"],
    })


async def handle_reach(request: web.Request) -> web.Response:
    """The numbers the browser overlay needs, and nothing it can compute itself.

    `web/reach.js` owns the arm model, the reach search and the projection, so
    this route only hands it what lives on the host: the camera calibration and
    -- when a fixed rig was fitted -- `T_base_cam`. In handheld mode the camera
    pose is a per-frame measurement, so it is null here; the page anchors the
    overlay on the pose returned by the most recent plan instead.
    """
    app = request.app
    calibration = calib_mod.load(resolve_calibration(app, request.query.get("calib")))
    rig = app["rig"]
    marker = app.get("marker")
    return web.json_response({
        "ok": True,
        "mode": "fixed" if rig is not None else "handheld",
        "plane_z": app["plane_z"],
        "hover_m": app["hover_m"],
        "tip_m": app["tip_m"],
        # The browser locates the tag itself; it needs the tag's size, its
        # placement in the base frame, and its heading. Dictionary 4x4_50/id 0
        # is the only pattern the small client-side detector knows.
        "marker": None if marker is None else {
            "size_m": float(marker["size_m"]),
            "at": [float(v) for v in marker["at"]],
            "yaw_deg": math.degrees(float(marker.get("yaw", 0.0))),
            "id": 0 if marker.get("id") is None else int(marker["id"]),
            "dictionary": "4x4_50",
        },
        "calibration": {
            "K": np.asarray(calibration.K, dtype=float).tolist(),
            "dist": np.asarray(calibration.dist, dtype=float).reshape(-1).tolist(),
            "image_size": [int(v) for v in calibration.image_size],
            "fingerprint": calibration.fingerprint,
            "rms": calibration.rms,
        },
        "T_base_cam": (None if rig is None
                       else np.asarray(rig.T_base_cam, dtype=float).tolist()),
        "rig": None if rig is None else rig.provenance(),
    }, headers={"Cache-Control": "no-store"})


async def handle_captures(request: web.Request) -> web.Response:
    app = request.app
    try:
        limit = max(1, min(200, int(request.query.get("limit", "20"))))
    except ValueError:
        limit = 20
    items = []
    for directory in capture_dirs(app["captures"])[:limit]:
        meta = read_meta(directory)
        frame = directory / "frame.jpg"
        items.append({
            "id": directory.name,
            "created": meta.get("capturedAt") or meta.get("server", {}).get("received_at"),
            "purpose": meta.get("purpose"),
            "tap": meta.get("tap"),
            "image_size": meta.get("image_size"),
            "bytes": frame.stat().st_size if frame.is_file() else 0,
            "has_target": (directory / "target.json").is_file(),
        })
    return web.json_response({"ok": True, "captures": items})


async def handle_capture(request: web.Request) -> web.Response:
    app = request.app
    if not request.content_type.startswith("multipart/"):
        raise web.HTTPBadRequest(text="expected multipart/form-data")

    limit = app["max_bytes"]
    directory = app["captures"] / new_capture_id()
    directory.mkdir(parents=True, exist_ok=False)
    frame_path = directory / "frame.jpg"
    partial = directory / "frame.part"

    meta = None
    size = 0
    digest = hashlib.sha256()
    try:
        reader = await request.multipart()
        part = await reader.next()
        while part is not None:
            if part.name == "meta":
                body = await part.read()
                if len(body) > 65536:
                    raise web.HTTPBadRequest(text="meta is larger than 64 KiB")
                try:
                    meta = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise web.HTTPBadRequest(text="meta is not JSON: %s" % exc) from exc
            elif part.name == "frame":
                content_type = (part.headers.get("Content-Type") or "image/jpeg").lower()
                if not content_type.startswith("image/"):
                    raise web.HTTPBadRequest(text="frame must be an image, got %s" % content_type)
                with partial.open("wb") as handle:
                    while True:
                        chunk = await part.read_chunk(256 * 1024)
                        if not chunk:
                            break
                        size += len(chunk)
                        if size > limit:
                            raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=size)
                        digest.update(chunk)
                        handle.write(chunk)
            part = await reader.next()

        if meta is None:
            raise web.HTTPBadRequest(text="no 'meta' part")
        if size == 0:
            raise web.HTTPBadRequest(text="no 'frame' part")
        if not isinstance(meta, dict):
            raise web.HTTPBadRequest(text="meta must be a JSON object")

        reserved = RESERVED_META & set(meta)
        if reserved:
            raise web.HTTPBadRequest(text="meta must not set %s" % sorted(reserved))
        purpose = meta.get("purpose", "plan")
        if purpose not in PURPOSES:
            raise web.HTTPBadRequest(text="purpose must be one of %s" % sorted(PURPOSES))

        tap = meta.get("tap")
        if tap is not None:
            if (not isinstance(tap, dict) or "u" not in tap or "v" not in tap
                    or not all(isinstance(tap[k], (int, float)) for k in "uv")):
                raise web.HTTPBadRequest(text="tap must be {u: number, v: number}")

        partial.replace(frame_path)
        record = dict(meta)
        record["frame"] = "frame.jpg"
        record["server"] = {
            "id": directory.name,
            "received_at": utc_now(),
            "remote": request.remote,
            "bytes": size,
            "sha256": digest.hexdigest(),
            "content_type": "image/jpeg",
            "server_version": VERSION,
        }
        if purpose in ("extrinsics", "calib"):
            # The pose is taken here, not by the client: a calibration click is
            # only worth anything if the joints are the ones the arm had.
            record["server"]["pose"] = await capture_pose(app)
        (directory / "meta.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")

    except Exception:
        (directory / "meta.json").unlink(missing_ok=True)
        partial.unlink(missing_ok=True)
        frame_path.unlink(missing_ok=True)
        try:
            directory.rmdir()
        except OSError:
            pass
        raise

    logging.info("capture %s: %d bytes, purpose=%s, tap=%s", directory.name, size, purpose, tap)
    return web.json_response({
        "ok": True,
        "id": directory.name,
        "bytes": size,
        "sha256": digest.hexdigest(),
        "url": "/captures/%s/frame.jpg" % directory.name,
        "target_url": "/captures/%s/target.json" % directory.name,
    }, status=201)


async def handle_relay(request: web.Request) -> web.Response:
    """Forward WHIP/WHEP to MediaMTX so both pages stay same-origin.

    The phone POSTs an offer to /whip, the workstation asks for the stream at
    /whep. Only the signaling is proxied; the RTP goes phone -> MediaMTX on the
    LAN, which is why MediaMTX still has to bind an address the phone can reach.
    """
    app = request.app
    kind = request.path.strip("/").split("/")[0]
    upstream = "%s/%s/%s" % (app["mediamtx"], app["whip_path"], kind)
    session_id = request.match_info.get("session")
    url = upstream + ("/" + session_id if session_id else "")
    body = await request.read()
    headers = {"Content-Type": request.headers.get("Content-Type", "application/sdp")}
    try:
        async with app["client"].request(request.method, url, data=body, headers=headers,
                                         timeout=ClientTimeout(total=20)) as answer:
            payload = await answer.read()
            response = web.Response(status=answer.status, body=payload)
            for name in ("Location", "Content-Type"):
                if name in answer.headers:
                    response.headers[name] = answer.headers[name]
            if answer.status >= 400:
                logging.warning("relay %s %s -> %s: %s", kind, request.method, answer.status,
                                payload[:200].decode("utf-8", "replace"))
            return response
    except Exception as exc:  # noqa: BLE001 - surfaced to the browser as 502
        logging.warning("relay %s failed: %s", url, exc)
        return web.json_response({"ok": False, "error": "MediaMTX unreachable at %s" % upstream},
                                 status=502)


def resolve_calibration(app, requested=None) -> Path:
    """The calibration for this phone: an explicit file, else the newest one."""
    folder = app["calib"]
    if requested:
        candidate = Path(requested)
        if not candidate.is_absolute():
            candidate = folder / candidate
        if not candidate.is_file():
            raise web.HTTPUnprocessableEntity(text="no calibration %s" % requested)
        return candidate
    if app["calib_file"]:
        candidate = Path(app["calib_file"])
        if candidate.is_file():
            return candidate
    options = sorted(p for p in folder.glob("*.json")
                     if not p.name.startswith(("extrinsics-", "witness-"))) \
        if folder.is_dir() else []
    if not options:
        raise web.HTTPUnprocessableEntity(
            text="no calibration in %s: run calibrate_intrinsics.py once for this phone" % folder)
    return options[-1]


def locate_options(app, body):
    """The locator's options: a rig for a fixed camera, the base marker for a handheld one.

    Three ways in, in order of specificity: an explicit mode in the request, the
    server's configured default, and -- when neither is set -- whatever the profile
    can actually do (a rig if one was fitted, otherwise the marker).
    """
    explicit = str(body.get("mode") or "").strip().lower()
    mode = explicit or str(app.get("mode") or "").strip().lower()
    if not mode:
        mode = "fixed" if app.get("rig") is not None else "handheld"
    plane_z = float(body.get("plane_z", app["plane_z"]))
    if mode.startswith("hand"):
        marker = app.get("marker")
        if not marker:
            raise locator.LocateError(
                "handheld mode needs the base marker: print docs/base-marker.pdf at 100%, "
                "paste it upright on the side of the base, and set REBOT_MARKER_AT to its "
                "centre and heading")
        at = marker["at"]
        return locator.Options(
            tag_size=float(marker["size_m"]),
            tag_x=float(at[0]), tag_y=float(at[1]), tag_z=float(at[2]),
            tag_yaw=math.degrees(float(marker.get("yaw", 0.0))),
            tag_id=marker.get("id"),
            plane_z=plane_z,
            annotate="",
            extrinsics=None,
            witness=app["witness"],
            witness_max_px=app["witness_max_px"],
        )
    if app["rig"] is None:
        raise locator.LocateError(
            "fixed mode needs a rig: run the calibration sweep once, or switch to handheld "
            "with the base marker")
    return locator.Options(
        plane_z=plane_z,
        annotate="",
        extrinsics=app["rig"],
        witness=app["witness"],
        witness_max_px=app["witness_max_px"],
    )


async def capture_pose(app):
    """`GET /pose` twice, 120 ms apart, so a moving arm is caught at upload time.

    A calibration click is only useful if the arm was still when the shutter
    opened. The two reads cannot prove that, but they do catch the common case
    -- the arm still settling from the move that put it there.
    """
    status, payload = await planner_call(app, "/pose", {}, timeout=10, method="GET")
    if status != 200:
        return {"error": payload.get("error", "the arm gateway refused /pose")}
    first = payload.get("pose") or {}
    if not isinstance(first.get("q"), list):
        return {"error": "the arm gateway returned no joint angles"}
    await asyncio.sleep(0.12)
    second_status, second_payload = await planner_call(app, "/pose", {}, timeout=10, method="GET")
    second = (second_payload.get("pose") or {}) if second_status == 200 else {}
    record = dict(first)
    if isinstance(second.get("q"), list) and len(second["q"]) == len(first["q"]):
        moved = max(abs(float(a) - float(b)) for a, b in zip(first["q"], second["q"]))
        record["moved_rad"] = moved
        record["settled"] = moved < 0.005
    else:
        record["settled"] = None
    return record


async def planner_call(app, path, body, timeout=120, method="POST"):
    """Call the arm gateway; returns (status, payload).

    The method matters: `/pose` is a GET on the gateway and a POST gets a 404.
    The capture server used to POST everything, which made the panel report "the
    arm is not reachable" on a gateway that was answering fine.
    """
    headers = {"X-Rebot-Token": app["planner_token"]} if app["planner_token"] else {}
    url = app["planner"].rstrip("/") + path
    try:
        async with app["client"].request(method, url, json=body, headers=headers,
                                         timeout=ClientTimeout(total=timeout)) as answer:
            payload = await answer.json(content_type=None)
            return answer.status, payload
    except Exception as exc:  # noqa: BLE001 - surfaced to the viewer
        logging.warning("planner %s %s failed: %s", method, url, exc)
        return 502, {"ok": False,
                     "error": "cannot reach the arm gateway at %s (%s)" % (url, exc),
                     "unreachable": True}


async def handle_locate(request: web.Request) -> web.Response:
    """One capture -> one base-frame point, without asking the planner for a plan.

    The marker UI needs the point as soon as the user taps, so it can number the
    marker and show whether it is reachable. This is the same `locate_target.py`
    the plan route runs; only the planner call is missing.
    """
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc

    capture_id = str(body.get("capture") or "")
    if not CAPTURE_ID.match(capture_id):
        raise web.HTTPBadRequest(text="'capture' must be a capture id")
    directory = app["captures"] / capture_id
    if not (directory / "frame.jpg").is_file():
        raise web.HTTPNotFound(text="no frame for %s" % capture_id)

    calibration = resolve_calibration(app, body.get("calib"))
    try:
        options = locate_options(app, body)
    except locator.LocateError as error:
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": str(error)}, status=422)
    options.annotate = ""

    try:
        target = await asyncio.to_thread(locator.locate_capture, directory, calibration, options)
    except locator.LocateError as error:
        logging.info("locate %s: %s", capture_id, error)
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": str(error)}, status=422)
    except Exception as error:  # noqa: BLE001 - a bad calibration must not kill the server
        logging.exception("locate %s crashed", capture_id)
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": "%s: %s" % (type(error).__name__, error)}, status=500)
    target.pop("_out", None)
    return web.json_response({"ok": True, "capture": capture_id, "target": target})


async def handle_sequence(request: web.Request) -> web.Response:
    """Plan one hover pass through the markers the phone collected."""
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc

    targets = body.get("targets")
    if not isinstance(targets, list) or not targets:
        raise web.HTTPBadRequest(text="'targets' must be a non-empty list")
    if len(targets) > 20:
        raise web.HTTPBadRequest(text="a marker sequence is limited to 20 targets")
    for index, target in enumerate(targets, start=1):
        if not isinstance(target, dict) or target.get("format") != "rebot-target":
            raise web.HTTPBadRequest(text="target %d is not a rebot-target" % index)
        point = target.get("point")
        if not isinstance(point, list) or len(point) != 3:
            raise web.HTTPBadRequest(text="target %d has no [x, y, z] point" % index)

    plan_options = {
        "tool_axis": app["tool_axis"],
        "yaw": float(body.get("yaw", app["yaw"])),
        "hover": True,
        "hover_m": float(body.get("hover_m", app["hover_m"])),
        "tip_m": float(body.get("tip_m", app["tip_m"])),
    }
    plan_options.update(body.get("options") or {})

    status, payload = await planner_call(app, "/plan-sequence",
                                        {"targets": targets, "options": plan_options},
                                        timeout=300)
    if status != 200:
        return web.json_response({"ok": False, "stage": "plan",
                                  "error": payload.get("error", "sequence plan failed")},
                                 status=status)

    logging.info("sequence plan %s: %s markers", payload.get("id"),
                 (payload.get("summary") or {}).get("markers"))
    return web.json_response({
        "ok": True,
        "plan": payload,
        # Both gates must agree: the host's --execute and the gateway's.
        "execute": bool(app["execute"] and payload.get("execute", True)),
    })


async def handle_plan(request: web.Request) -> web.Response:
    """A tap became a capture; turn it into a target and a trajectory.

    The pixel work runs in `locate_target.py` (same image, same code the CLI
    uses); the kinematics run in the arm gateway. Neither is reimplemented here.
    """
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc

    capture_id = str(body.get("capture") or "")
    if not CAPTURE_ID.match(capture_id):
        raise web.HTTPBadRequest(text="'capture' must be a capture id")
    directory = app["captures"] / capture_id
    if not (directory / "frame.jpg").is_file():
        raise web.HTTPNotFound(text="no frame for %s" % capture_id)

    calibration = resolve_calibration(app, body.get("calib"))
    try:
        options = locate_options(app, body)
    except locator.LocateError as error:
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": str(error)}, status=422)
    options.annotate = str(directory / "annotated.jpg")

    # locate_target.py is CPU work (aruco + PnP): keep it off the event loop.
    try:
        target = await asyncio.to_thread(locator.locate_capture, directory, calibration, options)
    except locator.LocateError as error:
        logging.info("locate %s: %s", capture_id, error)
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": str(error),
                                  "preview": "/captures/%s/annotated.jpg" % capture_id},
                                 status=422)
    except Exception as error:  # noqa: BLE001 - a bad calibration must not kill the server
        logging.exception("locate %s crashed", capture_id)
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": "%s: %s" % (type(error).__name__, error)}, status=500)
    target.pop("_out", None)

    plan_options = {"tool_axis": app["tool_axis"], "yaw": float(body.get("yaw", app["yaw"])),
                    # A tap marks a place: the arm visits it at the hover height. The
                    # pick sequence is still available, but it is not what a tap means.
                    "hover": bool(body.get("hover", app["hover"])),
                    "hover_m": float(body.get("hover_m", app["hover_m"])),
                    # The planner aims the flange; the tip is below it. A fitted rig
                    # knows that distance exactly, a handheld profile has to be told.
                    "tip_m": float(body.get("tip_m", app["tip_m"]))}
    plan_options.update(body.get("options") or {})

    status, payload = await planner_call(app, "/plan", {"target": target, "options": plan_options})
    if status != 200:
        # Return the target as well: the camera pose in it is still valid, and
        # the phone's reach overlay anchors on it even when the arm cannot go
        # there. A tap that fails is still a solved frame.
        return web.json_response({"ok": False, "stage": "plan", "capture": capture_id,
                                  "error": payload.get("error", "plan failed"),
                                  "target": target}, status=status)

    logging.info("plan %s from tap at (%s, %s): %s", payload.get("id"),
                 (target.get("tap") or {}).get("u"), (target.get("tap") or {}).get("v"),
                 payload.get("summary", {}).get("target"))
    return web.json_response({
        "ok": True,
        "capture": capture_id,
        "target": target,
        "preview": "/captures/%s/annotated.jpg" % capture_id,
        "plan": payload,
        "execute": app["execute"],
    })


async def handle_execute(request: web.Request) -> web.Response:
    app = request.app
    if not app["execute"]:
        raise web.HTTPForbidden(text="execution is disabled on this host (REBOT_EXECUTE=1 to allow it)")
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    status, payload = await planner_call(app, "/execute", body, timeout=360)
    return web.json_response(payload, status=status)


async def handle_pose(request: web.Request) -> web.Response:
    status, payload = await planner_call(request.app, "/pose", {}, timeout=30, method="GET")
    return web.json_response(payload, status=status)


def click_for(app, capture_id, u, v):
    """The tap the sweep starts from: a capture id and a pixel on the tool tip."""
    capture_id = str(capture_id or "")
    if not CAPTURE_ID.match(capture_id):
        raise web.HTTPBadRequest(text="'capture' must be a capture id")
    directory = Path(app["captures"]) / capture_id
    meta_path = directory / "meta.json"
    if not meta_path.is_file():
        raise web.HTTPNotFound(text="no capture %s" % capture_id)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    pose = (meta.get("server") or {}).get("pose") or {}
    q = pose.get("q")
    if not isinstance(q, list) or len(q) < 6:
        raise web.HTTPUnprocessableEntity(
            text="capture %s has no pose: take it with /capture/pose" % capture_id)
    width, height = (meta.get("image_size") or [0, 0])[:2]
    u, v = float(u), float(v)
    if not (0.0 <= u <= float(width) and 0.0 <= v <= float(height)):
        raise web.HTTPUnprocessableEntity(
            text="the tap (%.0f, %.0f) is outside the %dx%d frame" % (u, v, width, height))
    return {"q": [float(value) for value in q[:6]], "uv": (u, v),
            # The tap's own pixels are the gripper's appearance, so the detector
            # has a model of the thing it is looking for, not just a ray.
            "image": str(directory / "frame.jpg")}


def run_sweep(app, camera_url, calibration_path, body):
    """Drive the arm, take the stills, fit, verify, maybe write -- off the loop.

    Runs in a worker thread: it is minutes of blocking HTTP to the gateway plus
    OpenCV work, and holding the event loop would freeze the phone page's
    WebRTC relay for the whole sweep.
    """
    def note(pose, phase):
        app["progress"] = {"pose": int(pose), "of": len(sweep.POSES), "phase": phase,
                           "started": app["calibration_started"]}

    click = None
    if body.get("capture") and body.get("u") is not None and body.get("v") is not None:
        click = click_for(app, body["capture"], body.get("u"), body.get("v"))
    marker = body.get("marker") or app.get("marker")
    if click is None and not marker:
        raise auto_calibrate.AutoError(
            "a sweep needs either a tap ('capture', 'u', 'v') or a base marker: set "
            "REBOT_MARKER_AT once the tag is pasted on the plate")

    # latency=0: the sweep records continuously and picks its frames afterwards,
    # so draining the stream would only throw away frames it could have used.
    eye = camera_mod.open_source(camera_url, latency=0.0)
    try:
        probe = eye.read()
        size = (probe.shape[1], probe.shape[0])
        calibration = calib_mod.load(calibration_path, image_size=size)
        options = auto_calibrate.Options(
            plane_z=float(body.get("plane_z", app["plane_z"])),
            settle_s=float(body.get("settle", 0.5)),
            background=str(body["background"]) if body.get("background") else "median",
            click=click,
            marker=marker,
        )
        arm = auto_calibrate.ArmGateway(app["planner"], app["planner_token"])
        # The sweep is recorded by MediaMTX, so the camera must be its stream:
        # a USB device or a snapshot URL has no path to record.
        path = mediamtx.path_from_camera(camera_url)
        if not path:
            raise auto_calibrate.AutoError(
                "the sweep is recorded by MediaMTX, so REBOT_CAMERA must be its RTSP path "
                "(rtsp://mediamtx:8554/phone); %r cannot be recorded" % camera_url)
        recorder = mediamtx.Recorder(app["mediamtx_api"], path=path,
                                     recordings=app["recordings"])
        result = auto_calibrate.run(arm, eye, calibration, options, app["captures"],
                                    recorder=recorder, mount="fixed", progress=note)
    finally:
        eye.close()

    rig_path = app["calib"] / ("extrinsics-%s.json" % result.fit.rig.mount)
    _rig, verdict, written = auto_calibrate.commit(
        result, calibration, options, rig_path,
        calib_path=str(calibration_path), mount="fixed")
    payload = auto_calibrate.report(result, verdict)
    payload["written"] = str(written) if written else None
    return payload


async def handle_calibrate(request: web.Request) -> web.Response:
    """`POST /calibrate/extrinsics` -- one call, eight poses, a rig or a refusal.

    This moves the arm without a human at the panel, so it needs the same two
    gates as an execution (the host's `--execute` and the edge's
    `--allow-execute`) plus an explicit `confirm`: a stray POST must not start a
    sweep.
    """
    app = request.app
    if not app["execute"]:
        raise web.HTTPForbidden(text="the calibration sweep moves the arm: start the capture "
                                    "server with REBOT_EXECUTE=1 and the edge with "
                                    "--allow-execute")
    if not app["camera"]:
        raise web.HTTPUnprocessableEntity(text="no camera: set REBOT_CAMERA or pass camera=")
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    if body.get("confirm") is not True:
        raise web.HTTPBadRequest(text='send {"confirm": true}: this moves the arm through %d poses'
                                      % len(sweep.POSES))
    marker = body.get("marker") or app.get("marker")
    if not marker:
        for field in ("capture", "u", "v"):
            if body.get(field) is None:
                raise web.HTTPBadRequest(
                    text="starting a sweep needs 'capture', 'u' and 'v' (one still with the tool "
                         "tip tapped in it), or a base marker: paste the tag and set "
                         "REBOT_MARKER_AT")
    if app["calibrating"]:
        return web.json_response({"ok": False, "error": "a calibration sweep is already running"},
                                 status=409)

    calibration_path = resolve_calibration(app, body.get("calib"))
    camera_url = body.get("camera") or app["camera"]

    app["calibrating"] = True
    app["progress"] = {"pose": 0, "of": len(sweep.POSES), "phase": "starting",
                       "started": time.time()}
    app["calibration_started"] = app["progress"]["started"]
    app["last_calibration"] = {"state": "running", "started": utc_now(), "camera": camera_url}
    try:
        payload = await asyncio.to_thread(run_sweep, app, camera_url, calibration_path, body)
    except (auto_calibrate.AutoError, camera_mod.CameraError,
            extrinsics.ExtrinsicsError) as error:
        logging.warning("calibration refused: %s", error)
        app["last_calibration"] = {"state": "failed", "error": str(error), "at": utc_now()}
        return web.json_response({"ok": False, "error": str(error)}, status=422)
    except Exception as error:      # noqa: BLE001 - a sweep must not kill the server
        logging.exception("calibration crashed")
        app["last_calibration"] = {"state": "failed", "at": utc_now(),
                                   "error": "%s: %s" % (type(error).__name__, error)}
        return web.json_response({"ok": False, "error": "%s: %s"
                                  % (type(error).__name__, error)}, status=500)
    finally:
        app["calibrating"] = False
        app["progress"] = None

    app["last_calibration"] = {"state": "ok" if payload["ok"] else "refused",
                               "at": utc_now(), "verification": payload.get("verification")}
    if payload["ok"] and payload.get("written"):
        # Use it now, not after a restart: the next tap should be on the new rig.
        try:
            app["rig"] = extrinsics.load_rig(payload["written"])
            app["extrinsics_path"] = payload["written"]
            app["witness"] = witness.load_for(app["rig"], Path(payload["written"]).parent)
        except (extrinsics.ExtrinsicsError, witness.WitnessError) as error:
            logging.warning("calibration wrote %s but it could not be loaded: %s",
                            payload["written"], error)
        logging.info("calibration wrote %s", payload.get("written"))
    return web.json_response(payload, status=200 if payload["ok"] else 422)


async def handle_calibrate_status(request: web.Request) -> web.Response:
    """`GET /calibrate` -- is a sweep running, and how did the last one end?"""
    app = request.app
    return web.json_response({
        "ok": True,
        "running": bool(app["calibrating"]),
        "progress": app["progress"],
        "camera": app["camera"],
        "execute": app["execute"],
        "sweep": {"poses": len(sweep.POSES), "home": list(sweep.HOME),
                  "seconds": round(sweep.sweep_seconds())},
        "last": app["last_calibration"],
        # The base marker, when one is configured: the panel can then start a
        # sweep without asking for a tap.
        "marker": app["marker"],
        "rig": (None if app["rig"] is None else app["rig"].provenance()),
    })


def grab_one(app, camera_url):
    """One still from the fixed camera, with the pose the arm actually had."""
    import cv2

    eye = camera_mod.open_source(camera_url)
    try:
        frame = eye.read()
    finally:
        eye.close()

    arm = auto_calibrate.ArmGateway(app["planner"], app["planner_token"])
    first = arm.pose()
    time.sleep(0.2)
    second = arm.pose()
    moved = max(abs(float(a) - float(b)) for a, b in zip(first["q"], second["q"]))
    if moved > 0.01:
        raise auto_calibrate.AutoError(
            "the arm moved while the frame was taken (%.0f mrad): hold it still and capture "
            "again" % (1000.0 * moved))

    capture_id = new_capture_id()
    directory = Path(app["captures"]) / capture_id
    directory.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(directory / "frame.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    (directory / "meta.json").write_text(json.dumps({
        "purpose": "extrinsics",
        "image_size": [int(frame.shape[1]), int(frame.shape[0])],
        "server": {"id": capture_id, "pose": first, "captured_at": utc_now(),
                   "source": "capture-pose"},
    }, indent=2) + "\n", encoding="utf-8")
    return {"ok": True, "id": capture_id,
            "image_url": "/captures/%s/frame.jpg" % capture_id,
            "image_size": [int(frame.shape[1]), int(frame.shape[0])],
            "pose": first}


async def handle_capture_pose(request: web.Request) -> web.Response:
    """`POST /capture/pose` -- one still + pose, for the click path."""
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError:
        body = {}
    camera_url = body.get("camera") or app["camera"]
    if not camera_url:
        raise web.HTTPUnprocessableEntity(text="no camera: set REBOT_CAMERA or pass camera=")
    try:
        payload = await asyncio.to_thread(grab_one, app, camera_url)
    except (auto_calibrate.AutoError, camera_mod.CameraError) as error:
        return web.json_response({"ok": False, "error": str(error)}, status=422)
    return web.json_response(payload)


async def handle_file(request: web.Request) -> web.StreamResponse:
    app = request.app
    capture_id = request.match_info["id"]
    name = request.match_info["name"]
    if not CAPTURE_ID.match(capture_id) or not SAFE_NAME.match(name):
        raise web.HTTPNotFound()
    path = (app["captures"] / capture_id / name).resolve()
    root = app["captures"].resolve()
    if root not in path.parents or not path.is_file():
        raise web.HTTPNotFound()
    return web.FileResponse(path, headers={"Cache-Control": "no-store"})


async def on_startup(app: web.Application):
    app["client"] = ClientSession()


async def on_cleanup(app: web.Application):
    await app["client"].close()


async def handle_favicon(request: web.Request) -> web.Response:
    return web.Response(status=204)


def build_app(args) -> web.Application:
    app = web.Application(client_max_size=args.max_mb * 1024 * 1024,
                          middlewares=[json_errors, token_middleware(args.token)])
    app["args"] = args
    app["captures"] = Path(args.captures).resolve()
    app["max_bytes"] = args.max_mb * 1024 * 1024
    app["token"] = args.token
    app["tls"] = bool(args.cert and args.key)
    app["mediamtx"] = args.mediamtx.rstrip("/")
    app["whip_path"] = args.whip_path.strip("/")
    app["whip_url"] = "" if args.no_whip else "%s/%s/whip" % (app["mediamtx"], app["whip_path"])
    app["whep_url"] = "" if args.no_whip else "%s/%s/whep" % (app["mediamtx"], app["whip_path"])
    app["planner"] = args.planner.rstrip("/")
    app["planner_token"] = args.planner_token
    app["execute"] = bool(args.execute)
    app["calib"] = Path(args.calib_dir).resolve()
    app["calib_file"] = args.calib
    app["extrinsics_path"] = args.extrinsics
    app["witness_max_px"] = float(args.witness_max_px)
    app["camera"] = args.camera
    app["mediamtx_api"] = (getattr(args, "mediamtx_api", "")
                           or os.environ.get("REBOT_MEDIAMTX_API", "http://127.0.0.1:9997")).rstrip("/")
    app["recordings"] = Path(getattr(args, "recordings", "")
                             or os.environ.get("REBOT_RECORDINGS", str(STATE / "recordings"))).resolve()
    # "x,y,z,yaw" in metres and radians: where the 5 cm tag sits on the base plate
    # in the robot's own frame. With this set, a sweep needs no tap at all.
    app["marker"] = base_marker.parse_spec(getattr(args, "marker_at", "")
                                or os.environ.get("REBOT_MARKER_AT", ""),
                                getattr(args, "marker_size", 0.0)
                                or float(os.environ.get("REBOT_MARKER_SIZE", "0.04")))
    # How a tap becomes a point: "fixed" projects through the rig a sweep fitted,
    # "handheld" solves the camera from the base marker on every tap. Empty means
    # "whatever this profile can do". The page may switch per tap either way.
    app["mode"] = (getattr(args, "mode", "")
                   or os.environ.get("REBOT_MODE", "")).strip().lower()
    app["hover"] = (getattr(args, "hover", "")
                    or os.environ.get("REBOT_HOVER", "1")) != "0"
    # The tool tip below the flange: fixed mode can take it from the rig, but a
    # handheld profile and the planner's fallback need a number.
    app["tip_m"] = float(getattr(args, "tip_mm", 0.0)
                         or os.environ.get("REBOT_TIP_MM", "35")) / 1000.0
    app["hover_m"] = float(getattr(args, "hover_m", 0.0)
                           or os.environ.get("REBOT_HOVER_M", "0.10"))
    app["calibrating"] = False
    app["last_calibration"] = None
    app["progress"] = None
    app["calibration_started"] = None
    app["rig"] = None
    app["witness"] = None
    if args.extrinsics:
        path = Path(args.extrinsics)
        try:
            app["rig"] = extrinsics.load_rig(path)
        except extrinsics.ExtrinsicsError as error:
            raise SystemExit(str(error)) from error
        try:
            app["witness"] = witness.load_for(app["rig"], path.resolve().parent)
        except witness.WitnessError as error:
            raise SystemExit("the rig's witness frame cannot be used: %s" % error) from error
    app["plane_z"] = args.plane_z
    app["tool_axis"] = args.tool_axis
    app["yaw"] = args.yaw
    app["captures"].mkdir(parents=True, exist_ok=True)
    app["calib"].mkdir(parents=True, exist_ok=True)

    app.router.add_get("/", handle_index)
    app.router.add_get("/live", handle_live)
    app.router.add_get("/health", handle_health)
    app.router.add_get("/reach.json", handle_reach)
    app.router.add_get("/captures.json", handle_captures)
    app.router.add_post("/capture", handle_capture)
    app.router.add_post("/locate", handle_locate)
    app.router.add_post("/plan", handle_plan)
    app.router.add_post("/sequence", handle_sequence)
    app.router.add_post("/execute", handle_execute)
    app.router.add_get("/pose", handle_pose)
    app.router.add_post("/calibrate/extrinsics", handle_calibrate)
    app.router.add_get("/calibrate", handle_calibrate_status)
    app.router.add_post("/capture/pose", handle_capture_pose)
    app.router.add_get("/viewer.js", handle_viewer_js)
    app.router.add_get("/reach.js", handle_reach_js)
    app.router.add_get("/aruco.js", handle_aruco_js)
    app.router.add_get("/markers.js", handle_markers_js)
    app.router.add_get("/captures/{id}/{name}", handle_file)
    app.router.add_get("/favicon.ico", handle_favicon)
    if not args.no_whip:
        for kind in ("whip", "whep"):
            app.router.add_post("/" + kind, handle_relay)
            app.router.add_patch("/" + kind + "/{session}", handle_relay)
            app.router.add_delete("/" + kind + "/{session}", handle_relay)

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description="Phone capture endpoint for the B601 host.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=int(os.environ.get("REBOT_PORT", "8443")))
    parser.add_argument("--captures", default=str(STATE / "captures"))
    parser.add_argument("--cert", default=os.environ.get("REBOT_CERT", ""),
                        help="PEM certificate chain (TLS)")
    parser.add_argument("--key", default=os.environ.get("REBOT_KEY", ""),
                        help="PEM private key (TLS)")
    parser.add_argument("--token", default=os.environ.get("REBOT_TOKEN", ""),
                        help="shared secret; required on every POST")
    parser.add_argument("--mediamtx", default=os.environ.get("REBOT_MEDIAMTX", "http://127.0.0.1:8889"),
                        help="MediaMTX base URL for the WHIP proxy")
    parser.add_argument("--whip-path", default="phone", help="MediaMTX path the phone publishes to")
    parser.add_argument("--no-whip", action="store_true", help="disable live preview publishing")
    parser.add_argument("--planner", default=os.environ.get("REBOT_PLANNER", "http://127.0.0.1:8000"),
                        help="arm gateway base URL")
    parser.add_argument("--planner-token", default=os.environ.get("REBOT_PLANNER_TOKEN", ""))
    parser.add_argument("--execute", action="store_true",
                        default=os.environ.get("REBOT_EXECUTE", "0") == "1",
                        help="allow /execute to move the arm (also needs the gateway's flag)")
    parser.add_argument("--calib-dir", default=os.environ.get("REBOT_CALIB_DIR", str(STATE / "calib")))
    parser.add_argument("--calib", default=os.environ.get("REBOT_CALIB", ""),
                        help="calibration JSON; default is the newest in --calib-dir")
    parser.add_argument("--marker-at", default=os.environ.get("REBOT_MARKER_AT", ""),
                        help="'x,y,z' or 'x,y,z,yaw' of the base marker in the robot's frame "
                             "(metres, radians): with it, a sweep needs no tap")
    parser.add_argument("--marker-size", type=float,
                        default=float(os.environ.get("REBOT_MARKER_SIZE", "0.04")),
                        help="the marker's printed side in metres (default 0.04)")
    parser.add_argument("--mode", default=os.environ.get("REBOT_MODE", ""),
                        choices=("", "fixed", "handheld"),
                        help="fixed: rig from a sweep; handheld: the base marker on every tap")
    parser.add_argument("--hover-m", type=float,
                        default=float(os.environ.get("REBOT_HOVER_M", "0.10")),
                        help="how far above the tapped point the arm ends, in metres")
    parser.add_argument("--extrinsics", default=os.environ.get("REBOT_EXTRINSICS", ""),
                        help="fixed-camera rig JSON; then taps need no tag")
    parser.add_argument("--camera", default=os.environ.get("REBOT_CAMERA", ""),
                        help="rtsp://, http:// snapshot, or device:N, for the automatic sweep")
    parser.add_argument("--mediamtx-api", default=os.environ.get("REBOT_MEDIAMTX_API",
                                                                 "http://127.0.0.1:9997"),
                        help="MediaMTX control API, used to record a sweep")
    parser.add_argument("--recordings", default=os.environ.get("REBOT_RECORDINGS",
                                                               str(STATE / "recordings")),
                        help="where MediaMTX writes its recordings")
    parser.add_argument("--witness-max-px", type=float,
                        default=float(os.environ.get("REBOT_WITNESS_MAX_PX", "2.0")),
                        help="refuse a capture whose background drifted more than this, px")
    parser.add_argument("--plane-z", type=float, default=float(os.environ.get("REBOT_PLANE_Z", "0.0")))
    parser.add_argument("--tool-axis", default=os.environ.get("REBOT_TOOL_AXIS", "current"))
    parser.add_argument("--yaw", type=float, default=float(os.environ.get("REBOT_YAW", "0")))
    parser.add_argument("--max-mb", type=int, default=25, help="largest accepted frame, MB")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    if bool(args.cert) != bool(args.key):
        raise SystemExit("--cert and --key must be given together")
    if args.cert and not (Path(args.cert).is_file() and Path(args.key).is_file()):
        # This is the state a fresh `compose up` lands in: the stack is up and
        # reachable, the camera is not. Say so once instead of crash-looping.
        print("warning: %s or %s is missing: serving plain http." % (args.cert, args.key))
        print("         The phone will refuse the camera until make_cert.py has run.")
        args.cert = args.key = ""
    if not args.cert:
        print("warning: no --cert/--key: the phone will refuse getUserMedia on plain http\n"
              "         (run make_cert.py and pass the files, or use mkcert)")

    logging.basicConfig(level=logging.WARNING if args.quiet else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")

    ssl_context = None
    if args.cert:
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(args.cert, args.key)

    app = build_app(args)
    scheme = "https" if ssl_context else "http"
    ip = lan_ip()
    print("capture server %s" % VERSION)
    print("  listen   : %s://%s:%d" % (scheme, args.host, args.port))
    print("  phone    : %s://%s:%d/%s" % (scheme, ip, args.port,
                                          "?token=" + args.token if args.token else ""))
    print("  captures : %s" % app["captures"])
    print("  whip     : %s" % (app["whip_url"] or "disabled"))
    print("  planner  : %s%s" % (app["planner"],
                                  "  (execute on)" if app["execute"] else "  (execute off)"))
    print("  calib    : %s" % app["calib"])
    if app["rig"] is None:
        marker = app["marker"]
        print("  marker   : %s" % ("%.0f mm at (%.3f, %.3f, %.3f), heading %.1f deg"
                                   % (marker["size_m"] * 1000, marker["at"][0], marker["at"][1],
                                      marker["at"][2], math.degrees(marker["yaw"]))
                                   if marker else "none: handheld mode will refuse, fixed needs a rig"))
    else:
        rig = app["rig"]
        print("  camera   : fixed, %s, rms %.2f px" % (rig.fingerprint, rig.rms_px))
        print("             %s" % rig.describe().splitlines()[0])
        print("             witness %s, limit %.1f px"
              % ("on" if app["witness"] is not None else "MISSING", app["witness_max_px"]))
    print("  mode     : %s, hover %.0f mm, tip %.0f mm below the flange"
          % (app["mode"] or "auto", app["hover_m"] * 1000, app["tip_m"] * 1000))
    if app["camera"]:
        print("  sweep    : %s (POST /calibrate/extrinsics)" % app["camera"])
    print("  auth     : %s" % ("token required on writes" if args.token else "open on the LAN"))
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context,
                print=None, shutdown_timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
