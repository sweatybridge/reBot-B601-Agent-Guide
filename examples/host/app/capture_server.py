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
import codecs
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
from urllib.parse import urlparse

import numpy as np
from aiohttp import ClientSession, ClientTimeout, web

# The same locate code the CLI runs: a tap and a shell command cannot drift.
import auto_calibrate
import base_marker
import camera as camera_mod
import calib as calib_mod
import extrinsics
import identify as identify_mod
import locate_target as locator
import mediamtx
import oracle as oracle_mod
import rebot_ik
import sweep
import trace as trace_mod
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
# A chat turn carries the frames the model has been shown, and every frame is a
# base64 JPEG. Past this many messages the page must drop old frames.
AGENT_MAX_MESSAGES = 80
# One page flush of trace events: a frame is ~100 KiB of base64, so 8 MiB is
# generous for 64 events and still refuses a runaway page in one request.
TRACE_MAX_BODY = 8 * 1024 * 1024


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


async def handle_api_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "api.js"
    if not script.is_file():
        return web.Response(text="api.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_video_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "video.js"
    if not script.is_file():
        return web.Response(text="video.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_reach_config_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "reach-config.js"
    if not script.is_file():
        return web.Response(text="reach-config.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_geometry_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "geometry.js"
    if not script.is_file():
        return web.Response(text="geometry.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_arm_model_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "arm-model.js"
    if not script.is_file():
        return web.Response(text="arm-model.js is missing next to capture_server.py", status=500)
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


async def handle_height_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "height.js"
    if not script.is_file():
        return web.Response(text="height.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_locate_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "locate.js"
    if not script.is_file():
        return web.Response(text="locate.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_ekf_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "ekf.js"
    if not script.is_file():
        return web.Response(text="ekf.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_stream_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "stream.js"
    if not script.is_file():
        return web.Response(text="stream.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def handle_trace_js(request: web.Request) -> web.StreamResponse:
    script = WEB_DIR / "trace.js"
    if not script.is_file():
        return web.Response(text="trace.js is missing next to capture_server.py", status=500)
    return web.Response(text=script.read_text(encoding="utf-8"),
                        content_type="text/javascript",
                        headers={"Cache-Control": "no-store"})


async def trace_event(app, sid, type, **fields):
    """Record one server event; §8.4: a broken collector never fails a request.

    Every recorder call in the chat and arm paths goes through here (or the
    equally wrapped handle_trace), so a full disk degrades to "no trace for
    this session", never to a failed /agent, /sequence or /execute.
    """
    recorder = app.get("traces")
    if recorder is None or not sid:
        return None
    try:
        return await recorder.event(sid, "server", type, **fields)
    except Exception as exc:  # noqa: BLE001 - tracing is strictly additive
        logging.warning("trace %s for %s failed: %s", type, sid, exc)
        return None


def trace_manifest(app, sid):
    """The environment snapshot frozen at first sight of a session (§4.2).

    The rig and calib *file bytes* are copied into the session dir, not just
    fingerprinted: the live files may be refitted later, and a replay must see
    the geometry this session actually ran with.
    """
    directory = app["traces"].session_dir(sid)
    directory.mkdir(parents=True, exist_ok=True)
    live = WEB_DIR / "live.html"
    measured = app.get("measured_turn")
    marker = app.get("marker")
    snapshot = {
        "version": VERSION,
        "created": utc_now(),
        "live_html_sha256": (hashlib.sha256(live.read_bytes()).hexdigest()
                             if live.is_file() else None),
        "rig": (None if app["rig"] is None else
                dict(app["rig"].provenance(), fingerprint=app["rig"].fingerprint)),
        "rig_file": None,
        "calib_fingerprint": None,
        "calib_file": None,
        "plane_z": app["plane_z"],
        "tip_m": app["tip_m"],
        "min_z": app["min_z"],
        "hover_m": app["hover_m"],
        "execute": app["execute"],
        "planner_host": urlparse(app["planner"]).hostname,
        "stream_turn": ({"turns": int(measured["turns"]), "source": "measured",
                         "error_px": round(float(measured["error_px"]), 2),
                         "size": measured["size"]}
                        if measured else {"turns": None, "source": "shape"}),
        "stream_orientation": app["stream_orientation"],
        "marker": (None if marker is None else
                   {"size_m": float(marker["size_m"]),
                    "at": [float(v) for v in marker["at"]],
                    "yaw_deg": math.degrees(float(marker.get("yaw", 0.0)))}),
        # The model config minus every secret: the recorder redacts keys too,
        # but the URL never enters the snapshot in full in the first place.
        "llm": {"model": app["llm_model"],
                "temperature": app["llm_temperature"],
                "max_tokens": app["llm_max_tokens"],
                "url_host": urlparse(app["llm_url"]).hostname,
                "context_tokens": app["llm_context"]["tokens"]},
        # Pinned by the first /agent turn that carries tools; null until then.
        "tool_schemas_sha256": app.get("tool_schemas_sha256"),
    }
    try:
        calib_path = resolve_calibration(app)
        snapshot["calib_fingerprint"] = calib_mod.load(calib_path).fingerprint
        (directory / "calib.json").write_bytes(calib_path.read_bytes())
        snapshot["calib_file"] = "calib.json"
    except Exception as exc:  # noqa: BLE001 - a session without calib still traces
        logging.warning("trace manifest for %s has no calibration: %s", sid, exc)
    if app["rig"] is not None and app["extrinsics_path"]:
        try:
            (directory / "rig.json").write_bytes(Path(app["extrinsics_path"]).read_bytes())
            snapshot["rig_file"] = "rig.json"
        except OSError as exc:
            logging.warning("trace manifest for %s has no rig copy: %s", sid, exc)
    return snapshot


async def handle_trace(request: web.Request) -> web.Response:
    """`POST /trace` -- one batch of page events for a sidebar session.

    Fire-and-forget from the page: the answer carries the assigned seqs, but a
    failure here only costs trace evidence, never a chat turn (§8.4).
    """
    app = request.app
    recorder = app.get("traces")
    if recorder is None:
        return web.json_response({"ok": False,
                                  "error": "tracing is disabled on this host (--no-trace)"},
                                 status=503)
    if (request.content_length or 0) > TRACE_MAX_BODY:
        raise web.HTTPRequestEntityTooLarge(max_size=TRACE_MAX_BODY,
                                            actual_size=request.content_length)
    body = await request.read()
    if len(body) > TRACE_MAX_BODY:
        raise web.HTTPRequestEntityTooLarge(max_size=TRACE_MAX_BODY, actual_size=len(body))
    try:
        data = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    sid = str(data.get("session") or "")
    if not CAPTURE_ID.match(sid):
        raise web.HTTPBadRequest(text="'session' must be a session id (stamp-hex6)")
    items = data.get("events")
    if not isinstance(items, list) or not items:
        raise web.HTTPBadRequest(text="'events' must be a non-empty list")
    if len(items) > trace_mod.MAX_BATCH:
        raise web.HTTPBadRequest(text="a trace batch is limited to %d events"
                                 % trace_mod.MAX_BATCH)
    try:
        seqs = await recorder.events(sid, items)
    except ValueError as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc
    except Exception as exc:  # noqa: BLE001 - the page drops the batch and moves on
        logging.warning("trace write for %s failed: %s", sid, exc)
        return web.json_response({"ok": False, "error": "the trace could not be written"},
                                 status=500)
    return web.json_response({"ok": True, "seqs": seqs})


async def handle_trace_read(request: web.Request) -> web.Response:
    """`GET /trace/{sid}/events|manifest` -- read a session back.

    token_middleware exempts GET so the LAN can read captures, but a trace
    carries prompts and kitchen frames: the token is checked explicitly here.
    """
    app = request.app
    token = app["token"]
    if token and not hmac.compare_digest(request.headers.get("X-Rebot-Token", ""), token):
        return web.json_response({"ok": False, "error": "missing or wrong X-Rebot-Token"},
                                 status=401)
    recorder = app.get("traces")
    if recorder is None:
        return web.json_response({"ok": False,
                                  "error": "tracing is disabled on this host (--no-trace)"},
                                 status=503)
    sid = request.match_info["sid"]
    if not CAPTURE_ID.match(sid):
        raise web.HTTPNotFound()
    kind = request.path.rsplit("/", 1)[-1]
    try:
        if kind == "manifest":
            manifest = await recorder.read_manifest(sid)
            return web.json_response({"ok": True, "session": sid, "manifest": manifest})
        events = await recorder.read_events(sid)
        return web.json_response({"ok": True, "session": sid, "events": events})
    except FileNotFoundError:
        raise web.HTTPNotFound(text="no trace %s for %s" % (kind, sid))


async def _page(request: web.Request, name: str) -> web.StreamResponse:
    page = WEB_DIR / name
    if not page.is_file():
        return web.Response(text="%s is missing next to capture_server.py" % name, status=500)
    return web.Response(text=page.read_text(encoding="utf-8"), content_type="text/html",
                        headers={"Cache-Control": "no-store"})


def models_url(llm_url):
    """The OpenAI-compatible model list for a chat-completions URL."""
    url = str(llm_url or "").rstrip("/")
    if url.endswith("/v1/chat/completions"):
        return url[:-len("/chat/completions")] + "/models"
    if "/v1/" in url:
        return url.split("/v1/", 1)[0] + "/v1/models"
    return url + "/v1/models"


async def ensure_llm_context(app):
    """Read the loaded model's configured context from /v1/models, once.

    llama.cpp exposes both `context_length` (the model's native maximum) and
    `meta.n_ctx` (the context this server allocated). The latter is the useful
    number: 262144 native does not mean 262144 is available. Other servers may
    expose neither, in which case `REBOT_LLM_CONTEXT_TOKENS` is the fallback.
    """
    if not app.get("llm_url"):
        return app["llm_context"]
    info = app["llm_context"]
    if info.get("tokens"):
        return info
    now = time.time()
    if info.get("checked_at") and now - info["checked_at"] < 60:
        return info
    lock = app.get("llm_context_lock")
    if lock is None:
        lock = asyncio.Lock()
        app["llm_context_lock"] = lock
    async with lock:
        info = app["llm_context"]
        if info.get("tokens"):
            return info
        now = time.time()
        if info.get("checked_at") and now - info["checked_at"] < 60:
            return info
        headers = {}
        if app.get("llm_token"):
            headers["Authorization"] = "Bearer " + app["llm_token"]
        info = {"tokens": 0, "source": "", "error": "", "checked_at": now}
        try:
            async with app["client"].get(models_url(app["llm_url"]), headers=headers,
                                         timeout=ClientTimeout(total=2)) as answer:
                if answer.status != 200:
                    info["error"] = "the model list answered %d" % answer.status
                else:
                    data = await answer.json()
                    entries = data.get("data") if isinstance(data, dict) else None
                    entry = next((item for item in (entries or [])
                                  if item.get("id") == app["llm_model"]), None)
                    if not isinstance(entry, dict):
                        info["error"] = "the model list has no entry for %s" % app["llm_model"]
                    else:
                        meta = entry.get("meta") or {}
                        configured = int(meta.get("n_ctx") or 0)
                        native = int(entry.get("max_model_len")
                                     or entry.get("context_length") or 0)
                        if configured:
                            info.update(tokens=configured, source="meta.n_ctx")
                        elif native:
                            info.update(tokens=native,
                                        source=("max_model_len"
                                                if entry.get("max_model_len")
                                                else "context_length"))
                        else:
                            info["error"] = "the model entry has no context length"
        except Exception as exc:  # noqa: BLE001 - a probe must not break health
            info["error"] = str(exc)
        app["llm_context"] = info
        if info["tokens"]:
            logging.info("agent context: %d tokens (%s)", info["tokens"], info["source"])
        else:
            logging.warning("agent context probe failed: %s", info["error"])
        return info


async def handle_health(request: web.Request) -> web.Response:
    app = request.app
    await ensure_llm_context(app)
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
                        if not p.name.startswith(("extrinsics-", "witness-", "sag-",
                                                  "accuracy-map"))),
        "mode": effective_mode(app),
        "rig": (None if app["rig"] is None else
                dict(app["rig"].provenance(), path=str(app["extrinsics_path"]),
                     witness=bool(app["witness"]))),
        "tag": (None if app.get("marker") is None else
                {"size_m": app["marker"]["size_m"], "at": list(app["marker"]["at"]),
                 "yaw_deg": math.degrees(app["marker"]["yaw"])}),
        "plane_z": app["plane_z"],
        "hover_m": app["hover_m"],
        "tip_m": app["tip_m"],
        "min_z": app["min_z"],
        # What the phone last published about its stream. A quarter turn has no
        # sign in the frame size, so this is the only way to know which way the
        # camera is turned, and every consumer of `K` reads it here.
        "stream_orientation": app["stream_orientation"],
        # ... and the correction the host last worked out, with who decided.
        # /reach.json carries the same pair for the page's own projections.
        "stream_turn": ({"turns": int(app["measured_turn"]["turns"]),
                         "source": "measured", "error_px":
                         round(float(app["measured_turn"]["error_px"]), 2),
                         "size": app["measured_turn"]["size"]}
                        if app.get("measured_turn") else None),
        # The chat sidebar is configured from the host's environment, never from
        # the page; all the page needs to know is whether it is there and what
        # it is talking to.
        "agent": {
            "configured": bool(app["llm_url"]),
            "model": app["llm_model"] if app["llm_url"] else "",
            "auth": bool(app["llm_token"]),
            "max_tokens": app["llm_max_tokens"],
            "context_tokens": app["llm_context"]["tokens"],
            "context_source": app["llm_context"]["source"],
            "context_error": app["llm_context"]["error"],
        },
        "traces": {
            "enabled": app.get("traces") is not None,
            "sessions": _trace_session_count(app),
            "retention_days": app.get("trace_retention_days", 30),
        },
        # The measured accuracy envelope (§7.4): what fraction of the explored
        # grid has been validated by oracle holds.
        "accuracy": _accuracy_health(app),
    })


def _accuracy_health(app):
    try:
        map_doc = identify_mod.load_accuracy_map(app["accuracy_map_path"])
    except Exception:  # noqa: BLE001 - health must survive a broken map file
        return {"cells": 0, "validated": 0, "validated_fraction": 0.0}
    cells = map_doc.get("cells") or {}
    validated = sum(1 for cell in cells.values() if cell.get("validated_at"))
    return {"cells": len(cells), "validated": validated,
            "validated_fraction": round(identify_mod.validated_fraction(map_doc), 4)}


def _trace_session_count(app) -> int:
    recorder = app.get("traces")
    if recorder is None:
        return 0
    try:
        return len(recorder.session_ids())
    except OSError:
        return 0  # health must survive an unreadable traces dir


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
    # Which way round the stream is. The tag is the ground truth -- project it
    # with the rig, see where it actually is -- so measure whenever the rig can
    # see it, and only fall back to the phone's published angle (or the shape)
    # when it cannot. The measurement is cached for 30 s, so this is at most one
    # still grab per half minute.
    measured = None
    if rig is not None and marker is not None:
        # Off the event loop: grabbing a still from RTSP blocks, and this server
        # is also relaying the phone's media.
        measured = await asyncio.get_running_loop().run_in_executor(
            None, measure_stream_turn, app, calibration)
    turn = stream_turn(app, calibration)
    if turn is None:
        turns, source = None, "shape"
    else:
        turns = int(turn)
        source = "measured" if app.get("measured_turn") else "phone"
    return web.json_response({
        "ok": True,
        "mode": effective_mode(app),
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
            # The screen angle the board was shot at, when the phone told us: the
            # page pairs it with the published stream angle to work out the turn.
            "orientation_deg": (calibration.raw or {}).get("orientation_deg"),
        },
        "stream_orientation": app["stream_orientation"],
        # The correction the page must apply to its own projections: `turns`
        # quarter turns clockwise, from whichever source could settle it. `0` is
        # an answer, `null` means nothing could and the shape is a guess.
        "stream_turn": {"turns": turns, "source": source},
        "T_base_cam": (None if rig is None
                       else np.asarray(rig.T_base_cam, dtype=float).tolist()),
        "rig": None if rig is None else rig.provenance(),
    }, headers={"Cache-Control": "no-store"})


async def handle_orientation(request: web.Request) -> web.Response:
    """`POST /orientation` -- the phone says which way up its stream is.

    The browser is the only thing that knows: a quarter turn has no sign in the
    frame size, so the publisher states it once when the camera starts and again
    when it restarts. The rig paths (`/capture/pose`, the sweep) and `/live` read
    it back from `/health`, so nothing has to guess a direction or be configured
    by hand.
    """
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    try:
        angle = int(round(float(body.get("angle")))) % 360
    except (TypeError, ValueError) as exc:
        raise web.HTTPBadRequest(text="'angle' must be a number of degrees") from exc
    if angle not in (0, 90, 180, 270):
        raise web.HTTPBadRequest(text="'angle' must be 0, 90, 180 or 270")
    width = height = None
    size = body.get("size")
    if isinstance(size, (list, tuple)) and len(size) == 2:
        try:
            width, height = int(size[0]), int(size[1])
        except (TypeError, ValueError) as exc:
            raise web.HTTPBadRequest(text="'size' must be [width, height]") from exc
    app["stream_orientation"] = {"angle": angle, "width": width, "height": height,
                                 "at": utc_now()}
    logging.info("stream orientation: %d deg (%sx%s)", angle, width, height)
    return web.json_response({"ok": True, "stream_orientation": app["stream_orientation"]})


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
                # Read with a running cap so an oversized part cannot buffer
                # unbounded memory before the size check rejects it.
                chunks = []
                meta_size = 0
                while True:
                    chunk = await part.read_chunk(64 * 1024)
                    if not chunk:
                        break
                    meta_size += len(chunk)
                    if meta_size > 65536:
                        raise web.HTTPBadRequest(text="meta is larger than 64 KiB")
                    chunks.append(chunk)
                body = b"".join(chunks)
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
    # The calib dir also holds the rig, the witness, the learned sag map and
    # the accuracy envelope: none of them is an intrinsics calibration, and
    # "the newest json" must not hand one of those to calib.load.
    options = sorted(p for p in folder.glob("*.json")
                     if not p.name.startswith(("extrinsics-", "witness-", "sag-",
                                               "accuracy-map"))) \
        if folder.is_dir() else []
    if not options:
        raise web.HTTPUnprocessableEntity(
            text="no calibration in %s: run calibrate_intrinsics.py once for this phone" % folder)
    return options[-1]


def effective_mode(app):
    """The mode a request that does not name one gets.

    There is no host-wide switch to keep in sync: a fitted rig makes fixed the
    default, and a client that wants the tag sends `"mode": "handheld"` (the
    phone's Camera chip does exactly that).
    """
    return "fixed" if app.get("rig") is not None else "handheld"


def check_camera_override(app, url):
    """Refuse a camera override that points away from the configured camera's host.

    The server fetches this URL itself, so with no write token an arbitrary
    value is an SSRF vector into the LAN. A local `device:` source is allowed;
    otherwise the override must stay on the configured camera's own host (for
    example another path on the same MediaMTX). The configured host is itself
    private (mediamtx), so a blanket private-IP ban would break normal use --
    pinning to that host is the proportionate boundary.
    """
    configured = app.get("camera") or ""
    if not url or url == configured or str(url).startswith("device:"):
        return
    target = urlparse(str(url))
    base = urlparse(str(configured))
    if target.scheme not in ("rtsp", "rtmps", "http", "https") or not base.hostname:
        raise web.HTTPUnprocessableEntity(
            text="camera override must use the configured camera's host (%s)"
                 % (base.hostname or "none configured"))
    if (target.hostname or "").lower() != base.hostname.lower():
        raise web.HTTPUnprocessableEntity(
            text="camera override may not point at %s (configured host: %s)"
                 % (target.hostname, base.hostname))


def locate_options(app, body):
    """The locator's options: a rig for a fixed camera, the base marker for a handheld one.

    Three ways in, in order of specificity: an explicit mode in the request, the
    server's configured default, and -- when neither is set -- whatever the profile
    can actually do (a rig if one was fitted, otherwise the marker).
    """
    explicit = str(body.get("mode") or "").strip().lower()
    mode = explicit or effective_mode(app)
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
            # The witness verifies a FIXED mount has not moved. A handheld
            # camera moves by design, so applying the fixed rig's witness here
            # would refuse every handheld tap once a rig exists.
            witness=None,
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


def compact_target(tip, source=None):
    """Build the small wire object the planner accepts: tip plus optional audit."""
    result = {"tip": [float(tip[0]), float(tip[1]), float(tip[2])]}
    if source:
        for key in ("pixel", "tap", "capture"):
            if source.get(key) is not None:
                result[key] = source[key]
        # Via points are validated tip triples by handle_sequence before this
        # runs; they ride to the planner on the marker they belong to.
        if source.get("via") is not None:
            result["via"] = [[float(point[0]), float(point[1]), float(point[2])]
                             for point in source["via"]]
    return result


def target_tip(target, index):
    """Validate one compact target and return its tip as floats."""
    tip = target.get("tip") if isinstance(target, dict) else None
    if not isinstance(tip, list) or len(tip) != 3:
        raise web.HTTPBadRequest(text="target %d has no [x, y, z] tip" % index)
    try:
        point = [float(value) for value in tip]
    except (TypeError, ValueError) as error:
        raise web.HTTPBadRequest(
            text="target %d has a non-numeric tip" % index) from error
    if not all(math.isfinite(value) for value in point):
        raise web.HTTPBadRequest(text="target %d has a non-finite tip" % index)
    return point


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


async def planner_call(app, path, body, timeout=120, method="POST", sid=None):
    """Call the arm gateway; returns (status, payload).

    The method matters: `/pose` is a GET on the gateway and a POST gets a 404.
    The capture server used to POST everything, which made the panel report "the
    arm is not reachable" on a gateway that was answering fine.

    With a session id this is also the response tape (§4.2): every gateway
    interaction flows through here, so one hook records them all. The key is
    the sha256 of method|path|canonical-JSON request, the replay lookup key.
    """
    headers = {"X-Rebot-Token": app["planner_token"]} if app["planner_token"] else {}
    url = app["planner"].rstrip("/") + path
    started = time.monotonic()
    try:
        async with app["client"].request(method, url, json=body, headers=headers,
                                         timeout=ClientTimeout(total=timeout)) as answer:
            payload = await answer.json(content_type=None)
            status = answer.status
    except Exception as exc:  # noqa: BLE001 - surfaced to the viewer
        logging.warning("planner %s %s failed: %s", method, url, exc)
        status = 502
        payload = {"ok": False,
                   "error": "cannot reach the arm gateway at %s (%s)" % (url, exc),
                   "unreachable": True}
    if sid:
        canonical = json.dumps(body, sort_keys=True, ensure_ascii=False)
        await trace_event(app, sid, "gateway_call", path=path, method=method,
                          request=body, response=payload, status=status,
                          ms=round((time.monotonic() - started) * 1000),
                          key=hashlib.sha256(("%s|%s|%s" % (method, path, canonical))
                                             .encode("utf-8")).hexdigest())
    return status, payload


async def handle_sequence(request: web.Request) -> web.Response:
    """Plan one pass through the compact tip targets the phone collected.

    Targets are `{"tip": [x, y, z], "pixel": [u, v]}` in the arm base frame.
    The host supplies the physical tip offset and the safety floor; the planner
    only sees tip points.
    """
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
    if body.get("hover_m") is not None:
        raise web.HTTPBadRequest(text="hover_m is deprecated; send tip points")

    tips = [target_tip(target, index) for index, target in enumerate(targets, start=1)]
    tap_heights = [tip[2] - float(app["plane_z"]) for tip in tips]
    for index, tip in enumerate(tips, start=1):
        if tip[2] < app["min_z"] - 1e-9:
            return web.json_response({
                "ok": False, "stage": "target",
                "error": "marker %d: tip z %.3f m is below the minimum %.3f m"
                         % (index, tip[2], app["min_z"]),
            }, status=422)
    # Via points: the same tip-space contract and the same height floor as the
    # markers they belong to. The planner re-checks reach and IK before motion.
    for index, target in enumerate(targets, start=1):
        via = target.get("via") if isinstance(target, dict) else None
        if via is None:
            continue
        if not isinstance(via, list) or len(via) > 5:
            raise web.HTTPBadRequest(
                text="marker %d: 'via' must be a list of at most 5 [x, y, z] tip points"
                     % index)
        for via_index, via_point in enumerate(via, start=1):
            if not isinstance(via_point, list) or len(via_point) != 3:
                raise web.HTTPBadRequest(
                    text="marker %d via %d has no [x, y, z] tip" % (index, via_index))
            try:
                point = [float(value) for value in via_point]
            except (TypeError, ValueError) as error:
                raise web.HTTPBadRequest(
                    text="marker %d via %d has a non-numeric tip"
                         % (index, via_index)) from error
            if not all(math.isfinite(value) for value in point):
                raise web.HTTPBadRequest(
                    text="marker %d via %d has a non-finite tip" % (index, via_index))
            if point[2] < app["min_z"] - 1e-9:
                return web.json_response({
                    "ok": False, "stage": "target",
                    "error": "marker %d via %d: tip z %.3f m is below the minimum %.3f m"
                             % (index, via_index, point[2], app["min_z"]),
                }, status=422)

    plan_options = {
        "tool_axis": app["tool_axis"],
        "yaw": float(body.get("yaw", app["yaw"])),
        "min_z": app["min_z"],
    }
    plan_options.update(body.get("options") or {})
    plan_options["min_z"] = app["min_z"]
    planner_targets = [compact_target(tip, target)
                       for tip, target in zip(tips, targets)]

    sid = request.headers.get("X-Rebot-Session", "")
    status, payload = await planner_call(app, "/plan", {
        "targets": planner_targets,
        "tip_offset_m": float(app["tip_m"]),
        "gripper": "alternate",
        "options": plan_options,
    }, timeout=300, sid=sid)
    # plan_id <-> session: the page's sequence_result and the server's tape
    # meet on this id, so a failed plan is replayable from either side.
    await trace_event(app, sid, "plan", plan_id=payload.get("id"),
                      targets=planner_targets, summary=payload.get("summary"),
                      execute=bool(app["execute"] and payload.get("execute", True)),
                      status=status, ok=status == 200,
                      error=payload.get("error") if status != 200 else None)
    if status != 200:
        return web.json_response({"ok": False, "stage": "plan",
                                  "error": payload.get("error", "sequence plan failed")},
                                 status=status)

    logging.info("sequence plan %s: %s markers at [%s] mm tap height",
                 payload.get("id"), (payload.get("summary") or {}).get("markers"),
                 ", ".join("%.0f" % (height * 1000) for height in tap_heights))
    # §7.4: targets in unvalidated accuracy cells get a WARNING in the plan
    # summary -- visible to the agent and the panel. The hard refusals (reach
    # band, min_z, IK) stay where they are; a missing or broken map degrades to
    # no warning, never to a failed plan (§8.4).
    try:
        map_doc = identify_mod.load_accuracy_map(app["accuracy_map_path"])
        warnings = identify_mod.accuracy_warnings(map_doc, tips)
    except Exception as exc:  # noqa: BLE001
        logging.warning("accuracy map read failed: %s", exc)
        warnings = []
    if warnings and isinstance(payload.get("summary"), dict):
        existing = payload["summary"].get("warnings")
        payload["summary"]["warnings"] = (list(existing) if isinstance(existing, list)
                                          else []) + warnings
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

    calibration_path = resolve_calibration(app, body.get("calib"))
    try:
        options = locate_options(app, body)
    except locator.LocateError as error:
        return web.json_response({"ok": False, "stage": "locate", "capture": capture_id,
                                  "error": str(error)}, status=422)
    options.annotate = str(directory / "annotated.jpg")

    # locate_target.py is CPU work (aruco + PnP): keep it off the event loop.
    try:
        target = await asyncio.to_thread(locator.locate_capture, directory, calibration_path,
                                         options)
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
    calibration = calib_mod.load(calibration_path,
                                 image_size=tuple(target.get("image_size") or (0, 0)))

    tap_height = float(body.get("hover_m", app["hover_m"]))
    if tap_height < 0:
        raise web.HTTPBadRequest(text="hover_m must be >= 0")
    tap = target.get("tap") or {}
    try:
        tip, _distance, _grazing = extrinsics.ray_plane(
            calibration.K, calibration.dist, np.asarray(target["T_base_cam"], dtype=float),
            tap["u"], tap["v"], float(app["plane_z"]) + tap_height)
    except (KeyError, TypeError, ValueError, extrinsics.ExtrinsicsError) as error:
        return web.json_response({
            "ok": False, "stage": "target", "capture": capture_id,
            "error": "cannot intersect the raised plane: %s" % error,
            "target": target,
        }, status=422)
    tip = [float(value) for value in tip]
    if tip[2] < app["min_z"] - 1e-9:
        return web.json_response({
            "ok": False, "stage": "target", "capture": capture_id,
            "error": "tip z %.3f m is below the minimum %.3f m"
                     % (tip[2], app["min_z"]),
            "target": target,
        }, status=422)

    plan_options = {
        "tool_axis": app["tool_axis"],
        "yaw": float(body.get("yaw", app["yaw"])),
        "min_z": app["min_z"],
    }
    plan_options.update(body.get("options") or {})
    plan_options["min_z"] = app["min_z"]

    status, payload = await planner_call(app, "/plan", {
        "targets": [compact_target(tip, {"pixel": target.get("pixel"),
                                         "tap": tap, "capture": capture_id})],
        "tip_offset_m": float(app["tip_m"]),
        "gripper": "none",
        "options": plan_options,
    })
    if status != 200:
        # Return the target as well: the camera pose in it is still valid, and
        # the phone's reach overlay anchors on it even when the arm cannot go
        # there. A tap that fails is still a solved frame.
        return web.json_response({"ok": False, "stage": "plan", "capture": capture_id,
                                  "error": payload.get("error", "plan failed"),
                                  "target": target}, status=status)

    logging.info("plan %s from tap at (%s, %s) at %.0f mm: tip [%.3f %.3f %.3f]",
                 payload.get("id"), tap.get("u"), tap.get("v"), tap_height * 1000,
                 tip[0], tip[1], tip[2])
    return web.json_response({
        "ok": True,
        "capture": capture_id,
        "target": target,
        "preview": "/captures/%s/annotated.jpg" % capture_id,
        "plan": payload,
        # Both gates must agree, exactly as /sequence: the host's --execute AND
        # the gateway's. Offering execute on the host flag alone advertises a
        # move the gateway will refuse.
        "execute": bool(app["execute"] and payload.get("execute", True)),
    })


def _agent_text(value):
    """A message field as text: some servers send content parts, not a string."""
    if isinstance(value, list):
        return "".join(part.get("text", "") for part in value if isinstance(part, dict))
    return str(value or "")


def _agent_reasoning(message):
    """The chain of thought under any of the names OpenAI-compatible servers use."""
    return _agent_text(message.get("reasoning_content") or message.get("reasoning")
                       or message.get("thinking") or "")


def _agent_tool_calls(raw):
    """Normalize tool calls to the shape the page already assembles."""
    calls = []
    for index, call in enumerate(raw or []):
        if not isinstance(call, dict):
            continue
        function = call.get("function") or {}
        arguments = function.get("arguments")
        if not isinstance(arguments, str):
            arguments = json.dumps(arguments or {}, ensure_ascii=False)
        calls.append({
            "id": call.get("id") or "",
            "type": call.get("type") or "function",
            "function": {
                "name": str(function.get("name") or ""),
                "arguments": arguments,
            },
        })
    return calls


def _agent_message(data):
    """The assistant message and usage from a non-streamed completion."""
    choices = data.get("choices") or [{}]
    message = (choices[0] or {}).get("message") or {}
    return {
        "role": "assistant",
        "content": _agent_text(message.get("content")),
        "tool_calls": _agent_tool_calls(message.get("tool_calls") or []),
        "reasoning": _agent_reasoning(message),
    }, data.get("usage") or {}


def _agent_tool_error(message, finish_reason=""):
    """Why this turn's tool calls must not be executed, or '' when they are sane.

    A cut-off tool call is worse than a failed one: the malformed assistant
    message cannot be sent back on the next turn (the model server rejects it),
    and a complete-looking but truncated argument object can move the arm to the
    wrong pixel. Refuse the turn instead.
    """
    calls = message.get("tool_calls") or []
    if not calls:
        return ""
    if str(finish_reason) == "length":
        return ("the model's tool call was cut off by the token limit; nothing was run. "
                "Raise REBOT_LLM_MAX_TOKENS or ask for one short tool call.")
    for call in calls:
        arguments = (call.get("function") or {}).get("arguments")
        if arguments is None or arguments == "":
            continue
        try:
            value = json.loads(arguments)
        except (TypeError, ValueError):
            return ("the model produced a tool call whose arguments are not valid JSON "
                    "(%s); nothing was run. Retry, or raise REBOT_LLM_MAX_TOKENS."
                    % str(arguments)[:80])
        if not isinstance(value, dict):
            return ("the model produced a tool call whose arguments are not a JSON object "
                    "(%s); nothing was run." % str(arguments)[:80])
    return ""


def _tools_sha256(tools):
    """Pin the tool contract: a replay against a changed page is flagged (§8.3)."""
    if not tools:
        return None
    return hashlib.sha256(json.dumps(tools, sort_keys=True, ensure_ascii=False)
                          .encode("utf-8")).hexdigest()


def _sse(event, data):
    """One Server-Sent Event frame, as bytes."""
    return ("event: %s\ndata: %s\n\n"
            % (event, json.dumps(data, ensure_ascii=False))).encode("utf-8")


def _merge_tool_delta(calls, parts):
    """Accumulate streamed tool-call fragments by index.

    OpenAI sends `function.arguments` in pieces; a tool must not run until the
    stream has finished and those pieces form valid JSON.
    """
    for part in parts or []:
        if not isinstance(part, dict):
            continue
        try:
            index = int(part.get("index", 0))
        except (TypeError, ValueError):
            index = 0
        slot = calls.setdefault(index, {
            "id": "", "type": "function",
            "function": {"name": "", "arguments": ""},
        })
        if part.get("id"):
            slot["id"] = str(part["id"])
        if part.get("type"):
            slot["type"] = str(part["type"])
        function = part.get("function") or {}
        name = function.get("name")
        if name and not slot["function"]["name"]:
            slot["function"]["name"] = str(name)
        arguments = function.get("arguments")
        if isinstance(arguments, dict):
            arguments = json.dumps(arguments, ensure_ascii=False)
        if arguments:
            slot["function"]["arguments"] += str(arguments)


def _sse_payload(frame):
    """The `data:` payload of one SSE frame, or None for comments/keepalives."""
    lines = []
    for line in frame.split("\n"):
        if line.startswith(":"):
            continue
        if line.startswith("data:"):
            lines.append(line[5:].lstrip(" "))
    return "\n".join(lines) if lines else None


async def _relay_sse(request, upstream, on_turn=None):
    """Relay one model turn as normalized SSE events.

    The browser sees `reasoning` and `content` deltas as they arrive, then one
    `done` with the complete assistant message. Tool-call fragments are assembled
    here so the page never parses a half-written argument string.

    `on_turn` (optional) receives the assembled turn -- or the error -- so the
    recorder sees exactly what the page saw. It is awaited after the event is
    sent and guarded: tracing must not change the stream (§8.4).
    """
    async def note(payload):
        if on_turn is None:
            return
        try:
            await on_turn(payload)
        except Exception as exc:  # noqa: BLE001 - the stream outranks the trace
            logging.warning("agent turn trace failed: %s", exc)

    response = web.StreamResponse(status=200, headers={
        "Content-Type": "text/event-stream; charset=utf-8",
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",
    })
    try:
        response.enable_compression(False)
    except AttributeError:
        pass
    await response.prepare(request)

    async def send(event, data):
        await response.write(_sse(event, data))

    content_parts = []
    reasoning_parts = []
    tool_calls = {}
    finish_reason = ""
    usage = {}
    finished = False
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    buffer = ""

    try:
        async for chunk in upstream.content.iter_any():
            buffer += decoder.decode(chunk)
            buffer = buffer.replace("\r\n", "\n")
            frames = buffer.split("\n\n")
            buffer = frames.pop()
            for frame in frames:
                payload = _sse_payload(frame)
                if payload is None:
                    continue
                if payload.strip() == "[DONE]":
                    finished = True
                    buffer = ""
                    break
                try:
                    data = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                if data.get("error"):
                    raise RuntimeError(str(data["error"]))
                if data.get("usage"):
                    usage = data["usage"]
                for choice in data.get("choices") or []:
                    if not isinstance(choice, dict):
                        continue
                    if choice.get("finish_reason"):
                        finish_reason = str(choice["finish_reason"])
                    delta = choice.get("delta") or choice.get("message") or {}
                    reasoning = _agent_reasoning(delta)
                    if reasoning:
                        reasoning_parts.append(reasoning)
                        await send("reasoning", {"text": reasoning})
                    content = _agent_text(delta.get("content"))
                    if content:
                        content_parts.append(content)
                        await send("content", {"text": content})
                    _merge_tool_delta(tool_calls, delta.get("tool_calls"))
            if finished:
                break
        # A stream that closes without its final blank line: parse what is left.
        if buffer.strip():
            payload = _sse_payload(buffer)
            if payload and payload.strip() != "[DONE]":
                try:
                    data = json.loads(payload)
                except json.JSONDecodeError:
                    data = None
                if data:
                    if data.get("usage"):
                        usage = data["usage"]
                    for choice in data.get("choices") or []:
                        if not isinstance(choice, dict):
                            continue
                        if choice.get("finish_reason"):
                            finish_reason = str(choice["finish_reason"])
                        delta = choice.get("delta") or choice.get("message") or {}
                        reasoning = _agent_reasoning(delta)
                        if reasoning:
                            reasoning_parts.append(reasoning)
                            await send("reasoning", {"text": reasoning})
                        content = _agent_text(delta.get("content"))
                        if content:
                            content_parts.append(content)
                            await send("content", {"text": content})
                        _merge_tool_delta(tool_calls, delta.get("tool_calls"))

        message = {
            "role": "assistant",
            "content": "".join(content_parts),
            "reasoning": "".join(reasoning_parts),
            "tool_calls": [tool_calls[index] for index in sorted(tool_calls)],
        }
        tool_error = _agent_tool_error(message, finish_reason)
        complete = bool(finished or finish_reason)
        if tool_error:
            await send("error", {"error": tool_error})
            await note({"error": tool_error})
        elif not message["content"] and not message["tool_calls"]:
            await send("error", {"error": "the model returned neither text nor tool calls"})
            await note({"error": "the model returned neither text nor tool calls"})
        else:
            logging.info("agent stream: %d chars, %d tool call(s), %d reasoning chars, "
                         "finish=%s complete=%s usage=%s",
                         len(message["content"]), len(message["tool_calls"]),
                         len(message["reasoning"]), finish_reason or "-", complete, usage)
            await send("done", {"message": message, "finish_reason": finish_reason,
                                "complete": complete, "usage": usage})
            await note({"message": message, "finish_reason": finish_reason,
                        "complete": complete, "usage": usage})
    except asyncio.CancelledError:
        raise
    except (ConnectionResetError, ConnectionAbortedError):
        # The page went away; the upstream is closed by the caller's context.
        pass
    except Exception as exc:  # noqa: BLE001 - a stream may fail after headers
        logging.warning("agent stream failed: %s", exc)
        try:
            await send("error", {"error": str(exc)})
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        await note({"error": str(exc)})
    finally:
        try:
            await response.write_eof()
        except (ConnectionResetError, ConnectionAbortedError):
            pass
    return response


async def handle_agent(request: web.Request) -> web.StreamResponse:
    """One model turn for the page's chat sidebar.

    The page owns the conversation and the tool loop, because every tool either
    touches the page (markers, a fresh frame) or a route this server already has
    (`/sequence`, `/execute`). What lives here is the transport and its
    configuration: the endpoint, the model name and any key stay on the host, so
    the browser never holds them and no page setting can change them.

    `stream: true` asks for SSE. The upstream deltas are normalized here into
    `reasoning`, `content`, `done` and `error` events, so the page never has to
    know which provider produced them or how its tool-call deltas are shaped.
    """
    app = request.app
    if not app["llm_url"]:
        return web.json_response({
            "ok": False,
            "error": "the agent is not configured: set REBOT_LLM_URL (and REBOT_LLM_MODEL)",
        }, status=503)
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc

    messages = body.get("messages")
    if not isinstance(messages, list) or not messages:
        raise web.HTTPBadRequest(text="'messages' must be a non-empty list")
    if len(messages) > AGENT_MAX_MESSAGES:
        raise web.HTTPBadRequest(
            text="a chat turn is limited to %d messages; drop the old frames"
                 % AGENT_MAX_MESSAGES)
    tools = body.get("tools")
    if tools is not None and not isinstance(tools, list):
        raise web.HTTPBadRequest(text="'tools' must be a list of tool schemas")

    # The sidebar names its session on every turn; without the header the turn
    # still runs, untraced. The full upstream messages array is stored per turn
    # so the log is immune to the page's context compaction (§4.2).
    sid = request.headers.get("X-Rebot-Session", "")
    tools_sha = _tools_sha256(tools)
    if tools and tools_sha:
        app["tool_schemas_sha256"] = tools_sha

    async def on_turn(payload):
        if "error" in payload:
            await trace_event(app, sid, "stream_error", error=payload["error"])
            return
        await trace_event(app, sid, "model_turn",
                          messages=messages,
                          message=payload.get("message"),
                          finish_reason=payload.get("finish_reason", ""),
                          complete=payload.get("complete"),
                          usage=payload.get("usage") or {},
                          stream=bool(body.get("stream")),
                          config={"model": app["llm_model"],
                                  "temperature": app["llm_temperature"],
                                  "max_tokens": app["llm_max_tokens"],
                                  "tools_sha256": tools_sha})

    stream = bool(body.get("stream"))
    payload = {
        "model": app["llm_model"],
        "messages": messages,
        "stream": stream,
        "temperature": app["llm_temperature"],
    }
    if app["llm_max_tokens"] > 0:
        payload["max_tokens"] = app["llm_max_tokens"]
    if tools:
        payload["tools"] = tools
        payload["tool_choice"] = "auto"
    headers = {"Content-Type": "application/json"}
    if stream:
        headers["Accept"] = "text/event-stream"
    if app["llm_token"]:
        headers["Authorization"] = "Bearer " + app["llm_token"]

    try:
        async with app["client"].post(app["llm_url"], json=payload, headers=headers,
                                      timeout=ClientTimeout(total=app["llm_timeout"])) as answer:
            if answer.status != 200:
                text = await answer.text()
                logging.warning("agent %s answered %d: %s", app["llm_url"],
                                answer.status, text[:200])
                error = "the model refused (%d): %s" % (answer.status, text[:300])
                await trace_event(app, sid, "stream_error", error=error,
                                  status=answer.status)
                return web.json_response({
                    "ok": False,
                    "error": error,
                }, status=502)
            content_type = answer.headers.get("Content-Type") or ""
            if stream and "text/event-stream" in content_type:
                return await _relay_sse(request, answer, on_turn=on_turn)
            # A server that ignores `stream`, or the ordinary non-streamed
            # request: read the one JSON body and answer in kind.
            text = await answer.text()
    except Exception as exc:  # noqa: BLE001 - surfaced to the chat
        logging.warning("agent %s failed: %s", app["llm_url"], exc)
        await trace_event(app, sid, "stream_error", error=str(exc))
        return web.json_response({
            "ok": False,
            "error": "cannot reach the model at %s (%s)" % (app["llm_url"], exc),
        }, status=502)

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        await trace_event(app, sid, "stream_error",
                          error="the model answered something that is not JSON")
        return web.json_response({
            "ok": False,
            "error": "the model answered something that is not JSON: %s" % text[:200],
        }, status=502)

    message, usage = _agent_message(data)
    finish_reason = (((data.get("choices") or [{}])[0] or {}).get("finish_reason") or "")
    tool_error = _agent_tool_error(message, finish_reason)
    if tool_error:
        await trace_event(app, sid, "stream_error", error=tool_error)
        return web.json_response({"ok": False, "error": tool_error}, status=502)
    if not message["content"] and not message["tool_calls"]:
        await trace_event(app, sid, "stream_error",
                          error="the model returned neither text nor tool calls")
        return web.json_response({
            "ok": False,
            "error": "the model returned neither text nor tool calls",
        }, status=502)
    complete = finish_reason != "length"
    logging.info("agent: %d chars, %d tool call(s), %d reasoning chars, finish=%s "
                 "complete=%s usage=%s",
                 len(message["content"]), len(message["tool_calls"]),
                 len(message["reasoning"]), finish_reason or "-", complete, usage)
    await on_turn({"message": message, "finish_reason": finish_reason,
                   "complete": complete, "usage": usage})
    return web.json_response({
        "ok": True,
        "model": data.get("model") or app["llm_model"],
        "message": message,
        "finish_reason": finish_reason,
        "complete": complete,
        "usage": usage,
    })


async def handle_execute(request: web.Request) -> web.Response:
    app = request.app
    if not app["execute"]:
        raise web.HTTPForbidden(text="execution is disabled on this host (REBOT_EXECUTE=1 to allow it)")
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    sid = request.headers.get("X-Rebot-Session", "")
    # The oracle piggybacks on the plan's own holds (§5.3): no extra arm motion,
    # no new path to /execute. Preparation is wrapped -- a broken camera or a
    # missing plan file costs the evidence, never the execution (§8.4).
    plan_id = str(body.get("id") or "")
    prep = None
    if CAPTURE_ID.match(plan_id):
        prep = await oracle_prepare(app, plan_id)
    status, payload = await planner_call(app, "/execute", body, timeout=360, sid=sid)
    await trace_event(app, sid, "execute", plan_id=body.get("id"), status=status,
                      ok=payload.get("ok"), executed=payload.get("executed"),
                      error=payload.get("error"))
    if prep is not None:
        if status == 200 and payload.get("ok") and payload.get("executed"):
            # Both gates agreed and the arm played: verify off the response
            # path so the page is not held up by the camera and the FK calls.
            task = asyncio.create_task(oracle_finish(app, prep, sid))
            app["oracle_tasks"].add(task)
            task.add_done_callback(app["oracle_tasks"].discard)
        else:
            prep["grabs"].cancel()
    return web.json_response(payload, status=status)


async def trace_blob(app, data):
    """Content-address one frame; §8.4: a bad blob write degrades to no ref."""
    recorder = app.get("traces")
    if recorder is None:
        return None
    try:
        return await asyncio.to_thread(recorder.blob, data)
    except Exception as exc:  # noqa: BLE001 - tracing is strictly additive
        logging.warning("trace blob failed: %s", exc)
        return None


def _oracle_grab_frame(app):
    """One raw still from the configured camera, or None.

    Deliberately not grab_one: mid-playback the oracle must not read the arm's
    pose (the edge is busy commanding it) and must not re-solve the rig from
    the tag -- a still during a hold is evidence, not a calibration.
    """
    try:
        eye = camera_mod.open_source(app["camera"])
        try:
            return eye.read()
        finally:
            eye.close()
    except Exception as exc:  # noqa: BLE001 - a missed frame is a missed sample
        logging.info("oracle grab failed: %s", exc)
        return None


async def oracle_prepare(app, plan_id):
    """Pre-execution evidence: one still, the plan's hold windows, a grab task.

    Returns None on any failure -- the execution proceeds unverified (§8.4).
    The pre-still goes through grab_one, the existing capture path, so its
    witness report and tag solve are recorded exactly as a manual capture's.
    """
    try:
        if not app["camera"]:
            return None
        path = app["out_dir"] / ("%s.json" % plan_id)
        if not path.is_file():
            return None
        trajectory = json.loads(path.read_text(encoding="utf-8"))
        windows = oracle_mod.hold_windows(trajectory.get("waypoints") or [])
        if not windows:
            return None
        # A marker sequence records each target's point and raw IK solution in
        # its provenance; the oracle matches holds to them to measure the IK
        # residual (§7.4). /move plans have no such list -- empty, not an error.
        targets = ((trajectory.get("capture") or {}).get("targets")) or []
        pre = await asyncio.to_thread(grab_one, app, app["camera"], False)
        frame_path = app["captures"] / pre["id"] / "frame.jpg"
        pre_bytes = await asyncio.to_thread(frame_path.read_bytes)
        directory = app["oracle_dir"] / plan_id
        await asyncio.to_thread(directory.mkdir, parents=True, exist_ok=True)
        await asyncio.to_thread((directory / "pre.jpg").write_bytes, pre_bytes)
        prep = {
            "plan_id": plan_id,
            "windows": windows,
            "targets": targets,
            "pre_capture": pre["id"],
            "pre_frame_path": str(frame_path),
            "pre_blob": await trace_blob(app, pre_bytes),
            # Wherever a capture path ran, its witness report is persisted.
            "witness": pre.get("verification"),
            "holds": {},
            "started": time.monotonic(),
        }
        prep["grabs"] = asyncio.create_task(_oracle_hold_grabs(app, prep))
        return prep
    except Exception as exc:  # noqa: BLE001 - the oracle never fails /execute
        logging.warning("oracle prep for %s failed: %s", plan_id, exc)
        return None


async def _oracle_hold_grabs(app, prep):
    """Two grabs 300 ms apart at each hold's midpoint; keep the later.

    The middle of the window is chosen so stream latency up to half the window
    cannot pull the frame out of the hold (the sweep_video.select argument).
    Windows under HOLD_MIN_S never reach here: tracking-only oracle for those.
    """
    import cv2

    try:
        for window in prep["windows"]:
            midpoint = 0.5 * (window["t_start"] + window["t_end"])
            delay = prep["started"] + midpoint - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            frame = None
            for attempt in range(2):
                if attempt:
                    await asyncio.sleep(0.3)
                shot = await asyncio.to_thread(_oracle_grab_frame, app)
                if shot is not None:
                    frame = shot
            if frame is None:
                continue
            number = window["index"]
            path = app["oracle_dir"] / prep["plan_id"] / ("hold-%d.jpg" % number)
            ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                continue
            data = buffer.tobytes()
            await asyncio.to_thread(path.write_bytes, data)
            prep["holds"][number] = {"path": str(path), "blob": await trace_blob(app, data)}
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001 - a missed hold is a missed sample
        logging.warning("oracle hold grabs for %s failed: %s", prep["plan_id"], exc)


def _oracle_detect(frame, q6, K, dist, T_base_cam, t_tip, scale=1.0):
    """The tip pixel: green, with a fragment-cleaned retry. None, never a guess.

    A failed detection is `detected: null` (§5.3): the oracle reports what it
    saw, and a missing measurement is not a zero error.

    There is no template fallback. Its template came from the PRE-execution
    still, where the arm is parked: at a hold pose that patch is background,
    and on the 20261120T064648Z coverage run it "matched" blank wall at 0.87
    and produced 16-221 mm pixel errors. A guess from background is worse than
    no measurement.

    Green itself can fail on a fragmented cover: the nearest component is a
    sliver below GREEN_MIN_BLOB_PX while the substantial cover sits a little
    farther out. The retry keeps only sizeable blobs -- still colour evidence
    from this frame, never background, never another frame.

    `scale` is the frame grid's ratio to the calibration grid (`Calib.scale`):
    a WebRTC bandwidth drop shrinks every blob, so the area gates follow s^2.
    """
    import cv2

    options = auto_calibrate.DetectOptions()
    area_scale = float(scale) * float(scale)
    size = (int(frame.shape[1]), int(frame.shape[0]))
    try:
        found = auto_calibrate.detect_tip_green(frame, q6, K, dist, T_base_cam, t_tip,
                                                options, size, scale=scale)
        return {"detector": "green", "uv": [found.u, found.v], "score": found.score}
    except auto_calibrate.AutoError:
        pass
    mask = auto_calibrate.green_mask(frame)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
    clean = np.zeros_like(mask)
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] >= auto_calibrate.GREEN_MIN_BLOB_PX * area_scale:
            clean[labels == index] = 255
    if int(clean.sum()) // 255 < auto_calibrate.MIN_GREEN_PX * area_scale:
        return None
    try:
        found = auto_calibrate.detect_tip_green(frame, q6, K, dist, T_base_cam, t_tip,
                                                options, size, mask=clean, scale=scale)
    except auto_calibrate.AutoError:
        return None
    return {"detector": "green", "uv": [found.u, found.v], "score": found.score}


def _oracle_tool_tip(app, rig):
    """The fitted flange-to-tip vector, or the configured offset along -z.

    plan_to_target aims `tip = gripper_end - [0, 0, tip_offset_m]`; the rig's
    fitted tip_offset is the measured version of the same vector.
    """
    if rig is not None and float(np.linalg.norm(rig.tip_offset)) > 1e-6:
        return np.asarray(rig.tip_offset, dtype=float)
    return np.array([0.0, 0.0, -float(app["tip_m"])])


async def oracle_finish(app, prep, sid):
    """Post-execution verification: telemetry + FK + pixels -> the oracle doc.

    Runs after /execute returned, as a background task. Every step is wrapped:
    the worst outcome is a doc with an `error` field, never a failed execution
    (§8.4). Nothing here moves the arm.
    """
    plan_id = prep["plan_id"]
    doc = {"plan_id": plan_id, "created": utc_now(), "session": sid or None,
           "pre_capture": prep["pre_capture"], "pre_frame": prep["pre_blob"],
           "witness": prep.get("witness"), "holds": [], "telemetry": None,
           "verdict": None}
    try:
        import cv2

        await prep["grabs"]
        status, payload = await planner_call(app, "/telemetry/" + plan_id, {},
                                             timeout=30, method="GET", sid=sid)
        telemetry = payload.get("telemetry") if status == 200 else None
        if telemetry:
            doc["telemetry"] = {key: telemetry.get(key) for key in
                                ("version", "abort", "gripper_stall", "tick",
                                 "follow_error_max", "kp", "kd")}
        samples = (telemetry or {}).get("samples") or []

        calibration = calib_mod.load(resolve_calibration(app))
        rig = app["rig"]
        K = np.asarray(calibration.K, dtype=float)
        dist = np.asarray(calibration.dist, dtype=float).reshape(-1)
        turn = stream_turn(app, calibration)
        measured = app.get("measured_turn")
        doc["stream_turn"] = ({"turns": int(measured["turns"]), "source": "measured",
                               "error_px": round(float(measured["error_px"]), 2)}
                              if measured else
                              {"turns": turn, "source": "phone" if turn is not None else "shape"})
        # The tag reprojection error measure_stream_turn already computed, and
        # a fresh witness check on the pre-still wherever the capture path ran.
        if measured:
            doc["tag_reprojection_px"] = round(float(measured["error_px"]), 2)
        pre_image = cv2.imread(prep["pre_frame_path"])
        if pre_image is not None:
            doc["landscape"] = bool(pre_image.shape[1] > pre_image.shape[0])
            # Templates and the witness live in the calibration grid: turn the
            # pre-still back the same way the hold frames are turned.
            pre_image, _pre_calib, _pre_turn = locator.upright_frame(
                resolve_calibration(app), pre_image, turn)
            doc["pre_image_size"] = [int(pre_image.shape[1]), int(pre_image.shape[0])]
            doc["pre_grid_scale"] = round(float(_pre_calib.scale), 4)
            if app["witness"] is not None and doc["witness"] is None:
                try:
                    doc["witness"] = witness.check(app["witness"], pre_image,
                                                   max_drift_px=app["witness_max_px"])
                except witness.WitnessError as exc:
                    doc["witness"] = {"error": str(exc)}
        t_tip = _oracle_tool_tip(app, rig)
        kp = sweep.kp_vector()
        if telemetry and isinstance(telemetry.get("kp"), dict):
            try:
                kp = np.asarray([float(telemetry["kp"][str(m)]) for m in range(1, 7)])
            except (KeyError, TypeError, ValueError):
                pass

        matched = oracle_mod.match_hold_targets(prep["windows"], prep.get("targets"))
        for position, window in enumerate(prep["windows"]):
            hold = {"index": window["index"], "t_start": window["t_start"],
                    "t_end": window["t_end"]}
            target = matched[position] if position < len(matched) else None
            inside = [s for s in samples
                      if window["t_start"] - 1e-6 <= float(s.get("t", -1))
                      <= window["t_end"] + 1e-6]
            tip_meas = None
            if inside:
                q_cmd = np.mean([s["cmd"] for s in inside], axis=0)
                q_meas = np.mean([s["meas"] for s in inside], axis=0)
                hold["q_cmd"] = [float(v) for v in q_cmd]
                hold["q_meas"] = [float(v) for v in q_meas]
                hold["hold_error_rad"] = [float(v) for v in (q_meas - q_cmd)]
                hold["hold_error_max_rad"] = float(np.max(np.abs(q_meas - q_cmd)))
                try:
                    hold["sag_rad"] = [float(v) for v in sweep.gravity_sag(q_cmd[:6], kp)]
                except ValueError:
                    pass
                fk_cmd = await planner_call(app, "/fk", {"q": hold["q_cmd"]},
                                            timeout=30, sid=sid)
                fk_meas = await planner_call(app, "/fk", {"q": hold["q_meas"]},
                                             timeout=30, sid=sid)
                tip_cmd = (fk_cmd[1].get("tip_m") if fk_cmd[0] == 200 else None)
                tip_meas = (fk_meas[1].get("tip_m") if fk_meas[0] == 200 else None)
                hold["tip_fk_cmd"] = tip_cmd
                hold["tip_fk_meas"] = tip_meas
                if tip_cmd and tip_meas:
                    hold["tip_error_mm"] = round(1000.0 * float(np.linalg.norm(
                        np.asarray(tip_meas) - np.asarray(tip_cmd))), 3)
                # The FK bridge (§7.4): with the marker's raw IK solution on
                # tape, the solver's residual and the net placement are measured
                # against the REQUESTED point, separately from the hold error.
                if target is not None:
                    hold["target_point"] = target.get("point")
                    flange_point = target.get("flange_point")
                    hold["flange_point"] = flange_point
                    q_ik = target.get("q_ik")
                    if isinstance(q_ik, (list, tuple)) and len(q_ik) >= 6:
                        hold["q_ik"] = [float(v) for v in q_ik[:6]]
                        fk_ik = await planner_call(app, "/fk",
                                                   {"q": hold["q_ik"] + [0.0]},
                                                   timeout=30, sid=sid)
                        tip_ik = (fk_ik[1].get("tip_m") if fk_ik[0] == 200 else None)
                        hold["tip_fk_ik"] = tip_ik
                        if tip_ik and flange_point:
                            hold["ik_residual_mm"] = round(1000.0 * float(np.linalg.norm(
                                np.asarray(tip_ik) - np.asarray(flange_point))), 3)
                        if tip_meas and flange_point:
                            hold["placement_mm"] = round(1000.0 * float(np.linalg.norm(
                                np.asarray(tip_meas) - np.asarray(flange_point))), 3)
            shot = prep["holds"].get(window["index"])
            if shot and rig is not None and tip_meas:
                frame = cv2.imread(shot["path"])
                if frame is not None:
                    frame, calibration, _turn = locator.upright_frame(
                        resolve_calibration(app), frame, turn)
                    K = np.asarray(calibration.K, dtype=float)
                    dist = np.asarray(calibration.dist, dtype=float).reshape(-1)
                    # The grid this hold was measured on: a bandwidth drop can
                    # rescale the stream between the pre-still and any hold.
                    hold["image_size"] = [int(frame.shape[1]), int(frame.shape[0])]
                    hold["grid_scale"] = round(float(calibration.scale), 4)
                    # The point the detector looks for: FK tip plus the tool
                    # offset, in the base frame (locate_target.project).
                    rotation = rebot_ik.forward(np.asarray(hold["q_meas"][:6]))[:3, :3]
                    point = np.asarray(tip_meas, dtype=float) + rotation @ t_tip
                    predicted = locator.project(K, rig.T_base_cam, point, dist)
                    hold["predicted_tip_px"] = (list(predicted) if predicted else None)
                    found = await asyncio.to_thread(
                        _oracle_detect, frame, np.asarray(hold["q_meas"][:6]),
                        K, dist, np.asarray(rig.T_base_cam, dtype=float), t_tip,
                        calibration.scale)
                    hold["detected_tip_px"] = found["uv"] if found else None
                    hold["detector"] = found["detector"] if found else None
                    hold["detector_score"] = found["score"] if found else None
                    hold["frame"] = shot["blob"]
                    if found and predicted:
                        error_px = float(np.hypot(found["uv"][0] - predicted[0],
                                                  found["uv"][1] - predicted[1]))
                        hold["pixel_error_px"] = round(error_px, 2)
                        mm_per_px = witness.millimetres_per_pixel(rig, K, point=point)
                        hold["pixel_error_mm"] = (round(error_px * mm_per_px, 2)
                                                  if mm_per_px else None)
            doc["holds"].append(hold)

        # Post-execution still: the arm is back at park (the board's one-shot
        # semantics), out of the way. The pre/post difference around each
        # destination is a secondary placement signal, not ground truth.
        post = await asyncio.to_thread(_oracle_grab_frame, app)
        if post is not None:
            ok, buffer = cv2.imencode(".jpg", post, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if ok:
                data = buffer.tobytes()
                await asyncio.to_thread(
                    (app["oracle_dir"] / plan_id / "post.jpg").write_bytes, data)
                doc["post_frame"] = await trace_blob(app, data)
            if pre_image is not None:
                post_upright, _calib, _turn = locator.upright_frame(
                    resolve_calibration(app), post, turn)
                pre_gray = cv2.cvtColor(pre_image, cv2.COLOR_BGR2GRAY)
                post_gray = cv2.cvtColor(post_upright, cv2.COLOR_BGR2GRAY)
                # A bandwidth drop between the two stills is a uniform rescale:
                # put the post on the pre's grid instead of dropping the diff.
                if pre_gray.shape != post_gray.shape and calib_mod.same_shape(
                        (pre_gray.shape[1], pre_gray.shape[0]),
                        (post_gray.shape[1], post_gray.shape[0])):
                    post_gray = cv2.resize(post_gray,
                                           (pre_gray.shape[1], pre_gray.shape[0]),
                                           interpolation=cv2.INTER_AREA)
                if pre_gray.shape == post_gray.shape:
                    for hold in doc["holds"]:
                        pixel = hold.get("predicted_tip_px")
                        if not pixel:
                            continue
                        # The prediction is on the hold's grid; the diff is on
                        # the pre-still's. They differ only by a uniform scale.
                        hold_size = hold.get("image_size")
                        factor = (pre_gray.shape[1] / float(hold_size[0])
                                  if hold_size else 1.0)
                        u, v = int(pixel[0] * factor), int(pixel[1] * factor)
                        half = 40
                        box_pre = pre_gray[max(0, v - half):v + half,
                                           max(0, u - half):u + half]
                        box_post = post_gray[max(0, v - half):v + half,
                                             max(0, u - half):u + half]
                        if box_pre.size and box_pre.shape == box_post.shape:
                            hold["pre_post_diff"] = round(float(np.mean(
                                cv2.absdiff(box_pre, box_post))), 2)
        radii = [float(np.hypot(h["tip_fk_cmd"][0], h["tip_fk_cmd"][1]))
                 for h in doc["holds"] if h.get("tip_fk_cmd")]
        if radii:
            doc["max_radius_m"] = round(max(radii), 4)
        doc["verdict"] = oracle_mod.attribute(doc)
        await asyncio.to_thread(
            (app["oracle_dir"] / ("%s.json" % plan_id)).write_text,
            json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=doc["verdict"],
                          holds=len(doc["holds"]),
                          telemetry=bool(telemetry), error=None)
        # §7.4/§7.1: the oracle's holds populate the accuracy envelope, and the
        # monitor watches them for identification proposals. Both are strictly
        # additive (§8.4): a failure here never touches the doc or the execution.
        try:
            identify_mod.update_accuracy_map(app["accuracy_map_path"], doc, rig=rig)
        except Exception as exc:  # noqa: BLE001
            logging.warning("accuracy map update for %s failed: %s", plan_id, exc)
        try:
            app["identify_monitor"].observe(doc, rig=app.get("rig"))
        except Exception as exc:  # noqa: BLE001
            logging.warning("identify monitor for %s failed: %s", plan_id, exc)
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001 - the oracle never fails /execute
        logging.warning("oracle for %s failed: %s", plan_id, exc)
        doc["error"] = "%s: %s" % (type(exc).__name__, exc)
        try:
            await asyncio.to_thread(
                (app["oracle_dir"] / ("%s.json" % plan_id)).write_text,
                json.dumps(doc, indent=2) + "\n", encoding="utf-8")
            await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=None,
                              holds=len(doc["holds"]), error=doc["error"])
        except Exception as inner:  # noqa: BLE001 - not even the report is fatal
            logging.warning("oracle report for %s failed: %s", plan_id, inner)


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
        base = calib_mod.load(calibration_path)
        turn = stream_turn(app, base)
        probe, calibration, turn = locator.upright_frame(
            calibration_path, probe, turn)
        if turn:
            # Every frame the sweep reads has to be in the same orientation as
            # the probe, or the tip pixels and the tag would be measured against
            # one grid and fitted with another.
            eye = UprightEye(eye, turn)
            logging.info("sweep: frames turned back by %d quarter turn(s)", turn)
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
                                    recorder=recorder, mount="fixed", progress=note,
                                    turn=turn)
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
    check_camera_override(app, camera_url)

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
            rig = extrinsics.load_rig(payload["written"])
        except extrinsics.ExtrinsicsError as error:
            logging.warning("calibration wrote %s but it could not be loaded: %s",
                            payload["written"], error)
        else:
            app["rig"] = rig
            app["extrinsics_path"] = payload["written"]
            try:
                app["witness"] = witness.load_for(rig, Path(payload["written"]).parent)
            except witness.WitnessError as error:
                logging.warning("the new rig has no usable witness: %s", error)
                app["witness"] = None
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
        # The base marker, when one is configured: a capture can then solve the
        # rig from the tag, and a sweep can start without asking for a tap.
        "marker": app["marker"],
        "rig": (None if app["rig"] is None else app["rig"].provenance()),
    })


def new_plan_id():
    """An id in the gateway's plan shape, so its telemetry store accepts it."""
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:6]


def run_refit(app, camera_url, calibration_path, body):
    """One tag-rig refit, off the loop (§7.3a). NO ARM MOTION, no recorder.

    Repeated stills of the base tag, each gated by `verify_marker`; the survivors
    are averaged into a candidate that a held-out capture must still reproject.
    The camera is the only thing this session touches, so it needs no execute
    gate and no MediaMTX recording -- the arm pose is read once, for the witness.
    """
    def note(index, phase):
        app["progress"] = {"pose": int(index), "of": app.get("identify_of", 0),
                           "phase": phase, "started": app["calibration_started"]}

    eye = camera_mod.open_source(camera_url, latency=0.0)
    try:
        probe = eye.read()
        base = calib_mod.load(calibration_path)
        turn = stream_turn(app, base)
        probe, calibration, turn = locator.upright_frame(calibration_path, probe, turn)
        if turn:
            eye = UprightEye(eye, turn)
        captures = int(body.get("captures", 5))
        holdout = int(body.get("holdout", 1))
        app["identify_of"] = captures + holdout
        q = None
        try:
            arm = auto_calibrate.ArmGateway(app["planner"], app["planner_token"])
            q = arm.pose().get("q")       # for the mount witness, not for motion
        except auto_calibrate.AutoError as error:
            logging.warning("the refit has no arm pose for its witness: %s", error)
        return identify_mod.refit_session(
            eye, calibration, app["marker"], app["calib"],
            plane_z=float(body.get("plane_z", app["plane_z"])), mount="fixed",
            captures=captures, holdout=holdout,
            settle_s=float(body.get("settle", 0.4)), progress=note, q=q)
    finally:
        eye.close()


async def run_coverage(app, body, sid):
    """One gated coverage excursion (§7.4): motion to MEASURE, never to fit.

    The oracle hooks are async and the motion is blocking, so the orchestration
    lives here rather than in identify.py (which stays pure policy). The order
    matters: a dry run makes the gateway build and persist the trajectory, so
    `oracle_prepare` reads the REAL hold windows and schedules each grab at a
    window's midpoint -- the same latency argument `sweep_video.select` makes.
    Only then does the arm move, through the gateway's existing /move check chain.

    A run whose oracle cannot prepare is refused: motion without measurement
    teaches nothing, and the point of coverage is the measurements.
    """
    rig = app["rig"]
    if rig is None:
        raise identify_mod.IdentifyError("coverage measures against a rig: capture the base "
                                         "marker first")
    arm = auto_calibrate.ArmGateway(app["planner"], app["planner_token"])
    map_doc = identify_mod.load_accuracy_map(app["accuracy_map_path"])
    tip = _oracle_tool_tip(app, rig)
    start = await asyncio.to_thread(arm.pose)
    q = np.asarray(start["q"], dtype=float).reshape(-1)
    current_q, gripper = q[:6], (float(q[6]) if q.size > 6 else 0.0)
    poses = identify_mod.coverage_poses(
        current_q, map_doc, tip, seed=int(body.get("seed", 11)),
        top_k=int(body.get("poses", identify_mod.COVERAGE_MAX_POSES)))
    if not poses:
        raise identify_mod.IdentifyError(
            "no safe pose from here reaches an unvalidated cell: the accuracy map is covered, "
            "or park the arm somewhere else first")
    plan = identify_mod.coverage_plan(
        current_q, poses, gripper=gripper,
        hold_s=float(body.get("hold", identify_mod.COVERAGE_HOLD_S)))
    app["identify_of"] = len(plan["targets"])

    plan_id = new_plan_id()
    await asyncio.to_thread(arm.build_sequence, plan["targets"], plan["seconds"],
                            plan["hold"], plan["gripper"], plan_id)
    prep = await oracle_prepare(app, plan_id)
    if prep is None:
        raise identify_mod.IdentifyError(
            "the oracle could not prepare for this coverage run (no hold windows, or no "
            "camera): the arm would move and nothing would be measured")
    started = time.time()
    try:
        answer = await asyncio.to_thread(arm.move_sequence, plan["targets"], plan["seconds"],
                                         plan["hold"], plan["gripper"], plan_id)
    finally:
        await oracle_finish(app, prep, sid)
    return dict(plan, ok=True, mode="coverage", motion=True,
                poses=len(plan["targets"]), plan_id=plan_id,
                move_id=(answer or {}).get("id"),
                leg_seconds=plan["seconds"],
                seconds=round(time.time() - started, 1),
                telemetry=bool((answer or {}).get("telemetry")))


async def run_cartesian_coverage(app, body, sid):
    """Cartesian coverage (§7.4): the task planner, run exactly as a tap sequence.

    The joint-space form (`run_coverage`) visits safe poses and FKs them into
    cells; this one aims AT the unvalidated cells' centres and lets the task
    planner's IK decide whether the arm can hold there. Same /plan -> /execute
    path as /sequence, so the peel, the capsule checks, the sag emission pass
    and the edge's aborts are the task path's own -- nothing reimplemented.

    The planner is also the screen: the sequence grows one candidate at a time
    and a candidate stays only while the whole trajectory plans, so one
    unreachable cell cannot kill the session and no proxy feasibility test can
    disagree with the real plan. The oracle prepares from the persisted plan's
    real hold windows, and the plan's per-marker q_ik lets it measure the IK
    residual separately from sag (the FK bridge).

    Writes no rig, ever. A run whose oracle cannot prepare is refused before
    any motion: coverage exists for the measurements.
    """
    rig = app["rig"]
    if rig is None:
        raise identify_mod.IdentifyError("coverage measures against a rig: capture the base "
                                         "marker first")
    map_doc = identify_mod.load_accuracy_map(app["accuracy_map_path"])
    wanted = int(body.get("poses", identify_mod.COVERAGE_MAX_POSES))
    # Screen the whole grid, keep the first `wanted` survivors. The need-then-
    # cell order puts the inner band first, and whole rings can be unreachable
    # with the tool down; the screen must walk past them to the cells that solve.
    candidates, skipped = identify_mod.cartesian_targets(
        map_doc, top_k=len(identify_mod.coverage_cells(map_doc)),
        min_z=float(app["min_z"]))
    if not candidates:
        raise identify_mod.IdentifyError(
            "no unvalidated cell is reachable as a Cartesian target: the map is covered, "
            "or the remaining gaps are below the safety floor / beyond reach "
            "(skipped: %s)" % skipped)

    # The planner IS the screen: grow the sequence one candidate at a time and
    # keep it only while the whole trajectory plans. No proxy -- a single-target
    # plan would test every cell as marker 1 (Cartesian approach from park),
    # but markers 2+ arrive by joint-space legs with a seed fallback, so a cell
    # that fails alone can still be reachable in sequence. Each trial is exactly
    # the trajectory shape that will run, peel and joint return included; the
    # last successful trial is the plan that gets executed.
    accepted, refused = [], []
    plan_payload = None
    for entry in candidates:
        if len(accepted) >= wanted:
            break
        trial = accepted + [entry]
        status, payload = await planner_call(app, "/plan", {
            "targets": [{"tip": item["point"]} for item in trial],
            "tip_offset_m": float(app["tip_m"]),
            "gripper": "alternate",
            "options": {"tool_axis": app["tool_axis"], "min_z": float(app["min_z"])},
        }, timeout=300, sid=sid)
        if status == 200 and payload.get("ok"):
            accepted, plan_payload = trial, payload
        else:
            refused.append({"cell": entry["cell"],
                            "error": payload.get("error", "the planner refused the target")})
    if not accepted:
        raise identify_mod.IdentifyError(
            "every candidate cell was refused by the planner: no Cartesian coverage "
            "is possible from this pose (%d refused)" % len(refused))

    plan_id = str(plan_payload["id"])
    summary = plan_payload.get("summary") or {}
    app["identify_of"] = len(accepted)
    await trace_event(app, sid, "plan", plan_id=plan_id, mode="cartesian-coverage",
                      status=200, ok=True, summary=summary,
                      screened=len(accepted) + len(refused), refused=len(refused))

    prep = await oracle_prepare(app, plan_id)
    if prep is None:
        raise identify_mod.IdentifyError(
            "the oracle could not prepare for this coverage run (no hold windows, or no "
            "camera): the arm would move and nothing would be measured")
    started = time.time()
    answer = {}
    try:
        status, answer = await planner_call(app, "/execute", {"id": plan_id},
                                            timeout=360, sid=sid)
        await trace_event(app, sid, "execute", plan_id=plan_id, mode="cartesian-coverage",
                          status=status, ok=answer.get("ok"),
                          executed=answer.get("executed"), error=answer.get("error"))
        if status != 200 or not answer.get("ok") or not answer.get("executed"):
            raise identify_mod.IdentifyError(
                answer.get("error", "the gateway refused the coverage execution"))
    finally:
        # Even a refused/aborted execution is measured: the edge persists the
        # abort log, and the oracle doc is exactly the evidence a fault needs.
        await oracle_finish(app, prep, sid)
    return {
        "ok": True, "mode": "cartesian", "motion": True,
        "plan_id": plan_id,
        "poses": len(accepted),
        "cells": [entry["cell"] for entry in accepted],
        "needs": [entry["need"] for entry in accepted],
        "targets": [entry["point"] for entry in accepted],
        "skipped": skipped,
        "refused": refused,
        "worst_ik_m": summary.get("worst_ik_m"),
        "duration_s": summary.get("duration_s"),
        "waypoints": summary.get("waypoints"),
        "seconds": round(time.time() - started, 1),
        "telemetry": bool(answer.get("telemetry")),
        "writes_rig": False,
        "purpose": "measure the unvalidated accuracy cells through the task planner; "
                   "the oracle validates them and attributes the IK residual",
    }


async def handle_identify_run(request: web.Request) -> web.Response:
    """`POST /identify/run` -- the gated identification sessions (§7.1).

    `mode: "refit"` (the default) is §7.3a: repeated stills of the base tag,
    averaged into a candidate rig, verified against a held-out capture before it
    hot-loads. It moves NOTHING, so it needs the confirm but not the execute gate
    -- it is strictly less authority than /capture/pose, which already writes a
    rig from one still.

    `mode: "coverage"` is §7.4: one trajectory through the accuracy cells nobody
    has validated, so the oracle can measure them. That moves the arm, so it
    needs the execute gate on top. It writes no rig, ever -- it is coverage of
    the grid, not a fit. `mode: "cartesian"` is the same job through the TASK
    planner: cell-centre targets, IK + peel, per-marker IK residual on tape.
    Same gates, same oracle, no rig.

    Both share the gates: an explicit confirm, the `calibrating` single-flight,
    and a camera. A trigger only PROPOSES; this route is the only way a session
    starts. Candidates are verified before the hot-load below.
    """
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError as exc:
        raise web.HTTPBadRequest(text="body is not JSON: %s" % exc) from exc
    mode = str(body.get("mode") or "refit").strip().lower()
    if mode not in ("refit", "coverage", "cartesian"):
        raise web.HTTPBadRequest(text='mode must be "refit" (tag stills, no motion), '
                                      '"coverage" (joint-space cell measurement) or '
                                      '"cartesian" (cell centres through the task planner)')
    if not app["camera"]:
        raise web.HTTPUnprocessableEntity(text="no camera: set REBOT_CAMERA or pass camera=")
    if body.get("confirm") is not True:
        raise web.HTTPBadRequest(
            text='send {"confirm": true}: this %s'
                 % ("moves the arm through the unvalidated accuracy cells"
                    if mode in ("coverage", "cartesian")
                    else "refits the camera rig from the base tag"))
    if mode in ("coverage", "cartesian"):
        if not app["execute"]:
            raise web.HTTPForbidden(text="coverage moves the arm: start the capture server with "
                                         "REBOT_EXECUTE=1 and the edge with --allow-execute")
        if app["rig"] is None:
            raise web.HTTPUnprocessableEntity(
                text="coverage measures against a rig: capture the base marker first")
    if app["calibrating"]:
        return web.json_response({"ok": False,
                                  "error": "a calibration or identification session is already "
                                           "running"}, status=409)

    calibration_path = resolve_calibration(app, body.get("calib"))
    camera_url = body.get("camera") or app["camera"]
    check_camera_override(app, camera_url)
    sid = request.headers.get("X-Rebot-Session", "")
    app["calibrating"] = True
    app["progress"] = {"pose": 0, "of": 0, "phase": "starting", "started": time.time()}
    app["calibration_started"] = app["progress"]["started"]
    app["identify_last"] = {"state": "running", "started": utc_now(), "camera": camera_url,
                            "mode": mode}
    try:
        if mode == "coverage":
            payload = await run_coverage(app, body, sid)
        elif mode == "cartesian":
            payload = await run_cartesian_coverage(app, body, sid)
        else:
            payload = await asyncio.to_thread(run_refit, app, camera_url, calibration_path, body)
    except (identify_mod.IdentifyError, auto_calibrate.AutoError, camera_mod.CameraError,
            base_marker.MarkerError, extrinsics.ExtrinsicsError) as error:
        logging.warning("identification (%s) refused: %s", mode, error)
        app["identify_last"] = {"state": "failed", "mode": mode,
                                "error": str(error), "at": utc_now()}
        return web.json_response({"ok": False, "mode": mode, "error": str(error)}, status=422)
    except Exception as error:      # noqa: BLE001 - a session must not kill the server
        logging.exception("identification (%s) crashed", mode)
        app["identify_last"] = {"state": "failed", "mode": mode, "at": utc_now(),
                                "error": "%s: %s" % (type(error).__name__, error)}
        return web.json_response({"ok": False, "mode": mode, "error": "%s: %s"
                                  % (type(error).__name__, error)}, status=500)
    finally:
        app["calibrating"] = False
        app["progress"] = None

    await trace_event(app, sid, "identify", mode=mode, ok=payload.get("ok"),
                      poses=payload.get("poses"), usable=payload.get("usable"),
                      cells=payload.get("cells"), written=payload.get("written"),
                      error=payload.get("error"))
    state = "ok" if payload["ok"] else "refused"
    app["identify_last"] = {"state": state, "mode": mode, "at": utc_now(),
                            "verification": payload.get("verification"),
                            "identifiability": payload.get("identifiability"),
                            "cells": payload.get("cells")}
    if payload["ok"] and payload.get("written"):
        # The candidate passed verify_marker and the held-out capture: hot-load it
        # exactly as /capture/pose's rig is hot-loaded, so the next tap uses it now.
        try:
            rig = extrinsics.load_rig(payload["written"])
        except extrinsics.ExtrinsicsError as error:
            logging.warning("identification wrote %s but it could not be loaded: %s",
                            payload["written"], error)
        else:
            app["rig"] = rig
            app["extrinsics_path"] = payload["written"]
            try:
                app["witness"] = witness.load_for(rig, Path(payload["written"]).parent)
            except witness.WitnessError as error:
                logging.warning("the identified rig has no usable witness: %s", error)
                app["witness"] = None
        logging.info("identification (%s) wrote %s", mode, payload.get("written"))
    return web.json_response(payload, status=200 if payload["ok"] else 422)


async def handle_identify_status(request: web.Request) -> web.Response:
    """`GET /identify` -- proposals, progress, the last session, the gaps (§7.1).

    Same shape as GET /calibrate. The proposals are the monitor's evidence;
    nothing here starts a session -- that takes POST /identify/run {confirm}.
    `unlearned` is §7.2's other half: what the sag and accuracy maps do not know
    yet, reported rather than extrapolated, so the panel can show the envelope's
    real boundary instead of implying coverage it does not have.
    """
    app = request.app
    monitor = app["identify_monitor"]
    proposals = list(monitor.proposals)
    for proposal in monitor.rig_proposals(app.get("rig")):
        if proposal["kind"] not in {p["kind"] for p in proposals}:
            proposals.append(proposal)
    try:
        gaps = identify_mod.unlearned(
            identify_mod.active_sag_map(app["calib"], sweep.KP_DEFAULT),
            identify_mod.load_accuracy_map(app["accuracy_map_path"]))
    except Exception as exc:  # noqa: BLE001 - a report never fails the route
        logging.warning("the unlearned report failed: %s", exc)
        gaps = []
    return web.json_response({
        "ok": True,
        "running": bool(app["calibrating"]),
        "progress": app["progress"],
        "camera": app["camera"],
        "execute": app["execute"],
        "last": app["identify_last"],
        "proposals": proposals,
        "modes": {"refit": "averaged tag stills; no arm motion, no execute gate (§7.3a)",
                  "coverage": "measure the unvalidated accuracy cells in joint space; "
                              "moves the arm (§7.4)",
                  "cartesian": "cell centres through the task planner (IK + peel), with the "
                               "IK residual attributed; moves the arm (§7.4)"},
        "unlearned": gaps,
        "rig": (None if app["rig"] is None else app["rig"].provenance()),
        "accuracy": _accuracy_health(app),
    })


async def handle_identify_sag(request: web.Request) -> web.Response:
    """`POST /identify/sag` -- fit the residual map from recorded telemetry (§7.3b).

    No arm motion: the holds come from executions that already happened, so
    this needs the write token but not the execute gate. The map is stored
    keyed by the active rig's fingerprint, kp and zeros; /move applies it only
    while those keys match and the pose stays inside the explored envelope,
    and fit_sag_map's hold-out validation is what lets it be stored at all.
    """
    app = request.app
    if app["rig"] is None:
        raise web.HTTPUnprocessableEntity(
            text="the sag map is keyed by a fitted rig: run /calibrate/extrinsics first")
    try:
        body = await request.json()
    except json.JSONDecodeError:
        body = {}
    if not isinstance(body, dict):
        body = {}
    kp_spec = str(body.get("kp") or sweep.KP_DEFAULT)
    docs = []
    for directory in (app["telemetry_dir"], app["oracle_dir"]):
        if directory.is_dir():
            for path in sorted(directory.glob("*.json")):
                try:
                    docs.append(json.loads(path.read_text(encoding="utf-8")))
                except (OSError, json.JSONDecodeError):
                    continue  # a torn log costs evidence, never the fit
    try:
        holds = identify_mod.sag_holds(docs, kp=sweep.kp_vector(kp_spec))
        result = identify_mod.fit_sag_map(
            holds, kp_key=kp_spec, rig_fingerprint=app["rig"].fingerprint,
            zeros_key=identify_mod.zeros_key(app["rig"]))
    except Exception as error:  # noqa: BLE001 - a fit must not kill the server
        logging.warning("sag fit failed: %s", error)
        return web.json_response({"ok": False, "error": str(error)}, status=500)
    if result["ok"]:
        identify_mod.save_sag_map(app["calib"], result["map"])
    return web.json_response(dict(result, holds=len(holds)),
                             status=200 if result["ok"] else 422)


def measure_stream_turn(app, calibration, frame=None):
    """Measure the turn against the base tag: the correction, in quarter turns.

    The frame size says "a quarter turn" but never which way, and a wrong guess
    leaves the reach band, every tap and the rig fit 180 degrees out -- which
    looks like "a rotated picture" rather than an error, so nothing complains.
    The base tag is a known point in the base frame: project it with the fitted
    rig into the calibration grid, then push that prediction into the delivered
    frame with the candidate corrections. Only the real one lands on the tag.

    `frame` is the delivered frame when the caller already has one (the sweep's
    probe); otherwise one is grabbed from the configured camera.

    Returns `{"turns", "error_px", "size"}` or `None` when there is no rig, no
    marker, no visible tag, or the frame is not turned at all.
    """
    marker = app.get("marker")
    rig = app.get("rig")
    if marker is None or rig is None:
        return None
    import cv2  # noqa: F401  (the camera sources need it, and so does the detector)

    if frame is None:
        cache = app.get("measured_turn")
        if cache and (time.time() - cache.get("at", 0.0)) < 30.0:
            return cache
        try:
            eye = camera_mod.open_source(app["camera"])
            try:
                frame = eye.read()
            finally:
                eye.close()
        except Exception as error:  # a hint must never break the caller
            logging.info("stream turn not measured: %s", error)
            return None
    size = (int(frame.shape[1]), int(frame.shape[0]))
    if calib_mod.turns_for_shapes(calibration.image_size, size) == 0:
        measured = {"turns": 0, "error_px": 0.0, "size": list(size), "at": time.time()}
        if size != (int(calibration.image_size[0]), int(calibration.image_size[1])):
            # Same crop, coarser grid: the turn is certain from the shape, but
            # the tag error is only real with K on the frame's grid. A missing
            # tag leaves the error at 0 -- the orientation is still known.
            try:
                scaled = calibration.rescaled(size)
                marker_id = marker.get("id")
                marker_id = base_marker.MARKER_ID if marker_id is None else int(marker_id)
                corners = base_marker.detect(frame, marker_id)
                centre = np.asarray(corners, dtype=float).reshape(-1, 2).mean(axis=0)
                predicted = np.asarray(locator.project(scaled.K, rig.T_base_cam,
                                                       np.asarray(marker["at"], dtype=float)),
                                       dtype=float)
                measured["error_px"] = float(np.linalg.norm(predicted - centre))
            except Exception as error:      # noqa: BLE001 - a hint must never break the caller
                logging.info("stream turn tag check skipped: %s", error)
        app["measured_turn"] = measured
        return measured
    try:
        # The id is optional in the spec and stored as None when it was not given.
        marker_id = marker.get("id")
        marker_id = base_marker.MARKER_ID if marker_id is None else int(marker_id)
        corners = base_marker.detect(frame, marker_id)
        centre = np.asarray(corners, dtype=float).reshape(-1, 2).mean(axis=0)
        # Both candidate turns exchange the sides, so the upright grid is the
        # delivered size swapped -- at the native resolution or a rescale of it.
        upright_size = (size[1], size[0])
        scaled = calibration.rescaled(upright_size)
        predicted = np.asarray(locator.project(scaled.K, rig.T_base_cam,
                                               np.asarray(marker["at"], dtype=float)),
                               dtype=float)
    except Exception as error:
        logging.info("stream turn not measured: %s", error)
        return None
    best = None
    for turn in (1, 3):
        # The prediction is in the upright grid; the detection is in the
        # delivered frame, so the forward direction is 4 - turn.
        placed = np.asarray(calib_mod.map_pixel(predicted[0], predicted[1],
                                                upright_size,
                                                (4 - turn) % 4),
                            dtype=float)
        error = float(np.linalg.norm(placed - centre))
        if best is None or error < best["error_px"]:
            best = {"turns": turn, "error_px": error}
    best["size"] = list(size)
    best["at"] = time.time()
    app["measured_turn"] = best
    logging.info("stream turn measured: %d quarter turn(s), tag error %.1f px",
                 best["turns"], best["error_px"])
    return best


def stream_turn(app, calibration):
    """Quarter turns to apply to the stream to reach the calibration grid.

    Order: what the tag measurement saw, then what the phone published, then
    `None` -- which lets the caller fall back to the frame shape, that knows "a
    quarter turn" but not which way.

    The phone branch is `stream_angle - calibration_angle`: `screen.orientation`
    grows as the device turns one way and the frame turns the other, and the two
    negations cancel, leaving the plain difference of the correction.
    """
    measured = app.get("measured_turn")
    if measured and measured.get("turns") is not None:
        return int(measured["turns"]) % 4
    orientation = app.get("stream_orientation") or {}
    angle = orientation.get("angle")
    calib_deg = (calibration.raw or {}).get("orientation_deg")
    if angle is None or calib_deg is None:
        return None
    return int(round((float(angle) - float(calib_deg)) / 90.0)) % 4


def marker_from_still(app, frame, q=None, commit=True):
    """Solve the fixed camera from the base tag in one still, and write the rig.

    This is the short path: the tag is a PnP, so a still that sees it needs no
    sweep and moves no arm. The sweep stays for a camera that cannot see the tag;
    a missing tag is a normal answer here, not an error on the capture.

    The frame is first turned back to the calibration's orientation: `K` belongs
    to the grid the checkerboard was shot on, and a landscape frame is a quarter
    turn of a portrait calibration. The direction comes from the orientation the
    phone published (`POST /orientation`); without it the shape decides.

    `commit=False` solves and reports without writing: the oracle's pre-still is
    evidence, not a calibration -- rewriting the rig there would change its
    fingerprint under every keyed sag map and accuracy cell (§7.3b).
    """
    if app.get("calibrating"):
        return {"marker": {"detected": None,
                           "error": "a calibration sweep is running; the tag was not read"}}
    try:
        calibration_path = resolve_calibration(app)
        base = calib_mod.load(calibration_path)
        turn = stream_turn(app, base)
        frame, calibration, turn = locator.upright_frame(
            calibration_path, frame, turn)
    except (calib_mod.CalibError, web.HTTPException) as error:
        return {"marker": {
            "detected": None,
            "error": "the tag needs intrinsics at this size: %s"
                     % (getattr(error, "text", None) or error)}}
    try:
        run = auto_calibrate.solve_marker(
            frame, calibration, app["marker"], plane_z=float(app["plane_z"]))
    except base_marker.MarkerError as error:
        return {"marker": {"detected": False, "error": str(error)}}
    except auto_calibrate.AutoError as error:
        return {"marker": {"detected": None, "error": str(error)}}

    if commit:
        rig_path = app["calib"] / ("extrinsics-%s.json" % run.rig.mount)
        _rig, _verdict, written = auto_calibrate.commit_marker(
            run, calibration, rig_path, frame=frame, q=q)
    else:
        written = None
    report = auto_calibrate.marker_report(run, written)
    report["turns"] = turn                    # what the solve had to undo
    if turn:
        report["note"] = ("the still was turned back by %d quarter turn(s) before "
                          "the solve" % turn)
    return report


class UprightEye:
    """A camera source whose frames are turned back to the calibration's grid.

    The sweep reads many frames and measures tip pixels and the tag in all of
    them; if the stream is a quarter turn from the calibration, they all have to
    be turned the same way, or the fit mixes two grids.
    """

    def __init__(self, eye, turns):
        self.eye = eye
        self.turns = int(turns) % 4

    def read(self, *args, **kwargs):
        frame = self.eye.read(*args, **kwargs)
        return locator.rotate_image(frame, self.turns) if self.turns else frame

    def close(self):
        self.eye.close()


def grab_one(app, camera_url, commit=True):
    """One still from the fixed camera, plus the pose the click path needs.

    The base tag is solved before the arm is touched: a PnP needs no motor read,
    so a CAN fault must not stop a tag calibration. The two `/pose` reads -- and
    the "arm held still" check they enable -- only happen when the still is
    actually going to seed a tap. `commit=False` keeps the tag solve as evidence
    without rewriting the rig (the oracle's pre-still).
    """
    import cv2

    eye = camera_mod.open_source(camera_url)
    try:
        frame = eye.read()
    finally:
        eye.close()
    size = [int(frame.shape[1]), int(frame.shape[0])]

    def save_capture(pose):
        capture_id = new_capture_id()
        directory = Path(app["captures"]) / capture_id
        directory.mkdir(parents=True, exist_ok=False)
        cv2.imwrite(str(directory / "frame.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        record = {"id": capture_id, "pose": pose, "captured_at": utc_now(),
                  "source": "capture-pose"}
        if pose is None:
            record["pose_note"] = "not read: the base tag solved the camera"
        (directory / "meta.json").write_text(json.dumps({
            "purpose": "extrinsics",
            "image_size": size,
            "server": record,
        }, indent=2) + "\n", encoding="utf-8")
        return capture_id

    marker_payload = None
    if app.get("marker"):
        # The tag path first: no arm pose, no motor read, no sweep.
        marker_payload = marker_from_still(app, frame, q=None, commit=commit)
        if marker_payload.get("written") or (
                not commit and (marker_payload.get("marker") or {}).get("detected")):
            capture_id = save_capture(None)
            payload = {"ok": True, "id": capture_id,
                       "image_url": "/captures/%s/frame.jpg" % capture_id,
                       "image_size": size, "pose": None}
            payload.update(marker_payload)
            payload["ok"] = True
            return payload

    # The tap path: record where the arm actually was, and that it held still.
    arm = auto_calibrate.ArmGateway(app["planner"], app["planner_token"])
    first = arm.pose()
    time.sleep(0.2)
    second = arm.pose()
    moved = max(abs(float(a) - float(b)) for a, b in zip(first["q"], second["q"]))
    if moved > 0.01:
        raise auto_calibrate.AutoError(
            "the arm moved while the frame was taken (%.0f mrad): hold it still and capture "
            "again" % (1000.0 * moved))

    capture_id = save_capture(first)
    payload = {"ok": True, "id": capture_id,
               "image_url": "/captures/%s/frame.jpg" % capture_id,
               "image_size": size, "pose": first}
    if marker_payload is not None:
        # The tag was configured but did not solve: say why, and keep the still
        # for the tap fallback.
        payload.update(marker_payload)
        payload["ok"] = True
    return payload


async def handle_capture_pose(request: web.Request) -> web.Response:
    """`POST /capture/pose` -- one still; the tag solve first, the pose if needed."""
    app = request.app
    try:
        body = await request.json()
    except json.JSONDecodeError:
        body = {}
    camera_url = body.get("camera") or app["camera"]
    check_camera_override(app, camera_url)
    if not camera_url:
        raise web.HTTPUnprocessableEntity(text="no camera: set REBOT_CAMERA or pass camera=")
    try:
        payload = await asyncio.to_thread(grab_one, app, camera_url)
    except (auto_calibrate.AutoError, camera_mod.CameraError) as error:
        return web.json_response({"ok": False, "error": str(error)}, status=422)

    written = payload.get("written")
    if written:
        # Use it now, not after a restart: the next tap should be on the new rig.
        try:
            rig = extrinsics.load_rig(written)
        except extrinsics.ExtrinsicsError as error:
            logging.warning("the tag wrote %s but it could not be loaded: %s", written, error)
        else:
            app["rig"] = rig
            app["extrinsics_path"] = written
            try:
                app["witness"] = witness.load_for(rig, Path(written).parent)
            except witness.WitnessError as error:
                logging.warning("the tag rig has no usable witness: %s", error)
                app["witness"] = None
        app["last_calibration"] = {"state": "ok", "at": utc_now(),
                                   "method": "base-marker",
                                   "verification": payload.get("verification")}
        logging.info("the base tag solved the rig: %s", written)
    elif (payload.get("marker") or {}).get("detected"):
        app["last_calibration"] = {"state": "refused", "at": utc_now(),
                                   "method": "base-marker",
                                   "verification": payload.get("verification")}
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


async def trace_prune_loop(app):
    """Retention sweep once a day (§3); off the loop, and never fatal."""
    while True:
        try:
            await asyncio.to_thread(app["traces"].prune)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - retention must not kill the server
            logging.warning("trace prune failed: %s", exc)
        await asyncio.sleep(86400)


async def on_startup(app: web.Application):
    app["client"] = ClientSession()
    if app.get("llm_url") and not app["llm_context"]["tokens"]:
        # Do not hold up the listener; health (and the page) uses the result as
        # soon as the probe returns.
        app["llm_context_task"] = asyncio.create_task(ensure_llm_context(app))
    if app.get("traces") is not None:
        app["trace_prune_task"] = asyncio.create_task(trace_prune_loop(app))


async def on_cleanup(app: web.Application):
    task = app.get("llm_context_task")
    if task is not None:
        task.cancel()
    task = app.get("trace_prune_task")
    if task is not None:
        task.cancel()
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
    app["extrinsics_path"] = ""
    app["witness_max_px"] = float(args.witness_max_px)
    # What the phone last said about its stream: the angle its screen had when
    # the camera started, and the frame size it published. The browser is the
    # only place that knows the sign of a quarter turn.
    app["stream_orientation"] = None
    app["camera"] = args.camera
    app["mediamtx_api"] = (getattr(args, "mediamtx_api", "")
                           or os.environ.get("REBOT_MEDIAMTX_API", "http://127.0.0.1:9997")).rstrip("/")
    app["recordings"] = Path(getattr(args, "recordings", "")
                             or os.environ.get("REBOT_RECORDINGS", str(STATE / "recordings"))).resolve()
    # "x,y,z,yaw" in metres and degrees: where the tag sits on the base, in the
    # robot's own frame. A capture that sees it writes the rig with no arm
    # motion; a sweep can also use it instead of a tap.
    app["marker"] = base_marker.parse_spec(getattr(args, "marker_at", "")
                                or os.environ.get("REBOT_MARKER_AT", ""),
                                getattr(args, "marker_size", 0.0)
                                or float(os.environ.get("REBOT_MARKER_SIZE", "0.04")))
    # The tool tip below the flange: fixed mode can take it from the rig, but a
    # handheld profile and the planner's fallback need a number.
    app["tip_m"] = float(getattr(args, "tip_mm", 0.0)
                         or os.environ.get("REBOT_TIP_MM", "35")) / 1000.0
    app["hover_m"] = float(getattr(args, "hover_m", 0.0)
                           or os.environ.get("REBOT_HOVER_M", str(sweep.DEFAULT_MIN_Z)))
    # The lowest tip z the planner will accept: the worktop plane plus the
    # hover height above it -- the same sum the page's fallback and
    # .env.example use -- so a tap can never plan closer to the table than the
    # hover height, whatever plane the worktop sits at.
    app["min_z"] = float(args.plane_z) + app["hover_m"]
    app["calibrating"] = False
    app["last_calibration"] = None
    app["progress"] = None
    app["calibration_started"] = None
    app["rig"] = None
    app["witness"] = None
    # A fitted rig is picked up from the standard path at startup. Both the tag
    # capture and the sweep write it there, so nothing has to be copied into the
    # environment. It is loaded even when the default mode is handheld: the two
    # are both available, and a request (or the page's Camera chip) picks one.
    # An explicit --extrinsics names a different file.
    rig_path = None
    if args.extrinsics:
        rig_path = Path(args.extrinsics)
    else:
        candidate = app["calib"] / "extrinsics-fixed.json"
        if candidate.is_file():
            rig_path = candidate
    if rig_path is not None:
        try:
            app["rig"] = extrinsics.load_rig(rig_path)
        except extrinsics.ExtrinsicsError as error:
            raise SystemExit(str(error)) from error
        try:
            app["witness"] = witness.load_for(app["rig"], rig_path.resolve().parent)
        except witness.WitnessError as error:
            raise SystemExit("the rig's witness frame cannot be used: %s" % error) from error
        app["extrinsics_path"] = str(rig_path)
    app["plane_z"] = args.plane_z
    app["tool_axis"] = args.tool_axis
    app["yaw"] = args.yaw
    # The chat sidebar's model: an OpenAI-compatible endpoint and the one model
    # that can see. Empty URL turns the sidebar off. These are deliberately not
    # settable from a page -- only from the host's environment or flags. Each
    # value falls back to the environment when the flag was not given, and 0 is
    # a meaningful temperature, so neither `or` is safe here.
    app["llm_url"] = str(getattr(args, "llm_url", "") or os.environ.get("REBOT_LLM_URL", "")).strip()
    app["llm_model"] = str(getattr(args, "llm_model", "")
                           or os.environ.get("REBOT_LLM_MODEL", "Qwen3.8-27B")).strip()
    app["llm_token"] = str(getattr(args, "llm_token", "")
                           or os.environ.get("REBOT_LLM_TOKEN", "")).strip()
    temperature = getattr(args, "llm_temperature", None)
    app["llm_temperature"] = float(temperature if temperature is not None
                                   else os.environ.get("REBOT_LLM_TEMPERATURE", "0.2"))
    max_tokens = getattr(args, "llm_max_tokens", None)
    app["llm_max_tokens"] = int(max_tokens if max_tokens is not None
                                else os.environ.get("REBOT_LLM_MAX_TOKENS", "1024"))
    context_tokens = int(getattr(args, "llm_context_tokens", 0)
                         or os.environ.get("REBOT_LLM_CONTEXT_TOKENS", "0") or 0)
    app["llm_context"] = {
        "tokens": context_tokens,
        "source": "env" if context_tokens else "",
        "error": "",
        "checked_at": time.time() if context_tokens else 0,
    }
    timeout = getattr(args, "llm_timeout", None)
    app["llm_timeout"] = float(timeout if timeout is not None
                               else os.environ.get("REBOT_LLM_TIMEOUT", "300"))
    # Session traces (§4). getattr defaults keep older callers (the selftest's
    # hand-built args) working: tracing is on unless --no-trace says otherwise.
    app["trace_retention_days"] = int(getattr(args, "trace_retention_days", 30) or 30)
    app["traces_dir"] = Path(getattr(args, "traces_dir", "") or STATE / "traces").resolve()
    app["tool_schemas_sha256"] = None
    if getattr(args, "no_trace", False):
        app["traces"] = None
    elif getattr(args, "pg_dsn", ""):
        # §8 cutover: traces go to the substrate (PG), not state/traces/. The
        # store seam lives in examples/host/substrate; psycopg must be installed.
        import sys
        substrate_dir = str(ROOT / "substrate")
        if substrate_dir not in sys.path:
            sys.path.insert(0, substrate_dir)
        from pg_trace import PgTraceRecorder
        from store import SubstrateStore
        app["traces"] = PgTraceRecorder(
            SubstrateStore(args.pg_dsn),
            workspace_id=getattr(args, "pg_workspace", "default"),
            snapshot=lambda sid: trace_manifest(app, sid))
    else:
        # The snapshot callable runs lazily at a session's first event, when
        # the app state (rig, calib, gates) is fully populated.
        app["traces"] = trace_mod.TraceRecorder(
            app["traces_dir"], retention_days=app["trace_retention_days"],
            snapshot=lambda sid: trace_manifest(app, sid))
    app["captures"].mkdir(parents=True, exist_ok=True)
    app["calib"].mkdir(parents=True, exist_ok=True)
    # The oracle's evidence (§5.3) lives beside the captures, under the same
    # state root the planner writes out/ to (a shared bind mount in production).
    # Deriving from --captures keeps a test workdir entirely self-contained.
    app["out_dir"] = app["captures"].parent / "out"
    app["oracle_dir"] = app["captures"].parent / "oracle"
    app["telemetry_dir"] = app["captures"].parent / "telemetry"
    app["oracle_tasks"] = set()
    # The hillclimb (§7): the monitor only proposes; the accuracy map is the
    # measured envelope /sequence warns from and /health reports.
    app["identify_monitor"] = identify_mod.TriggerMonitor()
    app["identify_last"] = None
    app["accuracy_map_path"] = app["calib"] / "accuracy-map.json"

    app.router.add_get("/", handle_index)
    app.router.add_get("/live", handle_live)
    app.router.add_get("/health", handle_health)
    app.router.add_get("/reach.json", handle_reach)
    app.router.add_get("/captures.json", handle_captures)
    app.router.add_post("/capture", handle_capture)
    app.router.add_post("/orientation", handle_orientation)
    app.router.add_post("/plan", handle_plan)
    app.router.add_post("/sequence", handle_sequence)
    app.router.add_post("/execute", handle_execute)
    app.router.add_post("/agent", handle_agent)
    app.router.add_post("/trace", handle_trace)
    app.router.add_get("/trace/{sid}/events", handle_trace_read)
    app.router.add_get("/trace/{sid}/manifest", handle_trace_read)
    app.router.add_get("/pose", handle_pose)
    app.router.add_post("/calibrate/extrinsics", handle_calibrate)
    app.router.add_get("/calibrate", handle_calibrate_status)
    app.router.add_post("/identify/run", handle_identify_run)
    app.router.add_post("/identify/sag", handle_identify_sag)
    app.router.add_get("/identify", handle_identify_status)
    app.router.add_post("/capture/pose", handle_capture_pose)
    app.router.add_get("/viewer.js", handle_viewer_js)
    app.router.add_get("/reach.js", handle_reach_js)
    app.router.add_get("/api.js", handle_api_js)
    app.router.add_get("/video.js", handle_video_js)
    app.router.add_get("/reach-config.js", handle_reach_config_js)
    app.router.add_get("/geometry.js", handle_geometry_js)
    app.router.add_get("/arm-model.js", handle_arm_model_js)
    app.router.add_get("/aruco.js", handle_aruco_js)
    app.router.add_get("/markers.js", handle_markers_js)
    app.router.add_get("/height.js", handle_height_js)
    app.router.add_get("/locate.js", handle_locate_js)
    app.router.add_get("/ekf.js", handle_ekf_js)
    app.router.add_get("/stream.js", handle_stream_js)
    app.router.add_get("/trace.js", handle_trace_js)
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
                             "(metres, degrees): a capture that sees it writes the rig; a "
                             "sweep needs no tap")
    parser.add_argument("--marker-size", type=float,
                        default=float(os.environ.get("REBOT_MARKER_SIZE", "0.04")),
                        help="the marker's printed side in metres (default 0.04)")
    parser.add_argument("--hover-m", type=float,
                        default=float(os.environ.get("REBOT_HOVER_M", str(sweep.DEFAULT_MIN_Z))),
                        help="height of the tap plane above the worktop, in metres; "
                             "also the lowest tip z the planner accepts")
    parser.add_argument("--extrinsics", default="",
                        help="a fitted rig JSON; default is "
                             "<calib-dir>/extrinsics-fixed.json when it exists")
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
    # The chat sidebar on /live. An OpenAI-compatible endpoint; set it empty to
    # disable the sidebar. The model has to be able to see, and this is the only
    # place it is named -- the page cannot change it. The default is the host's
    # own Ollama; compose overrides it with the container's view of the host.
    parser.add_argument("--llm-url",
                        default=os.environ.get("REBOT_LLM_URL",
                                               "http://localhost:11434/v1/chat/completions"),
                        help="OpenAI-compatible chat endpoint for the /live agent sidebar")
    parser.add_argument("--llm-model", default=os.environ.get("REBOT_LLM_MODEL", "Qwen3.8-27B"),
                        help="model name that endpoint expects; it must accept images")
    parser.add_argument("--llm-token", default=os.environ.get("REBOT_LLM_TOKEN", ""),
                        help="bearer token for that endpoint, if it needs one")
    parser.add_argument("--llm-temperature", type=float,
                        default=float(os.environ.get("REBOT_LLM_TEMPERATURE", "0.2")))
    parser.add_argument("--llm-max-tokens", type=int,
                        default=int(os.environ.get("REBOT_LLM_MAX_TOKENS", "1024")))
    parser.add_argument("--llm-context-tokens", type=int,
                        default=int(os.environ.get("REBOT_LLM_CONTEXT_TOKENS", "0") or 0),
                        help="context window override; 0 reads meta.n_ctx from /v1/models")
    parser.add_argument("--llm-timeout", type=float,
                        default=float(os.environ.get("REBOT_LLM_TIMEOUT", "300")),
                        help="seconds to wait for one model turn")
    parser.add_argument("--max-mb", type=int, default=25, help="largest accepted frame, MB")
    parser.add_argument("--traces-dir", default=os.environ.get("REBOT_TRACES_DIR",
                                                               str(STATE / "traces")),
                        help="where session traces and their blobs are written")
    parser.add_argument("--trace-retention-days", type=int,
                        default=int(os.environ.get("REBOT_TRACE_RETENTION_DAYS", "30")),
                        help="prune session dirs and unreferenced blobs older than this")
    parser.add_argument("--no-trace", action="store_true",
                        help="disable session trace collection entirely (sensitive deployments)")
    parser.add_argument("--pg-dsn", default=os.environ.get("REBOT_PG_DSN", ""),
                        help="substrate Postgres DSN; when set, traces go to PG (the new stack) "
                             "instead of state/traces/ (spec §8 cutover)")
    parser.add_argument("--pg-workspace", default=os.environ.get("REBOT_PG_WORKSPACE", "default"),
                        help="workspace id for trace sessions in the substrate")
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
    print("  phone    : %s://%s:%d/" % (scheme, ip, args.port))
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
    print("  mode     : default %s, hover %.0f mm, tip %.0f mm below the flange"
          % (effective_mode(app), app["hover_m"] * 1000, app["tip_m"] * 1000))
    if app["camera"]:
        print("  sweep    : %s (POST /calibrate/extrinsics)" % app["camera"])
    print("  agent    : %s" % ("%s at %s" % (app["llm_model"], app["llm_url"])
                                if app["llm_url"] else
                                "off (set REBOT_LLM_URL to enable the /live chat sidebar)"))
    print("  traces   : %s" % ("%s (retention %d days)" % (app["traces_dir"],
                                                            app["trace_retention_days"])
                                if app["traces"] is not None else "disabled (--no-trace)"))
    print("  auth     : %s" % ("token required on writes" if args.token else "open on the LAN"))
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context,
                print=None, shutdown_timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
