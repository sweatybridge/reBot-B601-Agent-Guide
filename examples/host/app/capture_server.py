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
import base64
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
import oracle_vision
import rebot_ik
import sweep
import trace as trace_mod
import witness

VERSION = "0.2.0"
HERE = Path(__file__).resolve().parent      # app/
ROOT = HERE.parent                          # the host directory
WEB_DIR = ROOT / "web"
# Flattened layout (§8): no state/. Authoritative data lives in the substrate
# (PG); work/ holds only disposable scratch — cv2 materializations, the MediaMTX
# spool, and file-mode artifacts when no substrate is configured (offline tests).
WORK = ROOT / "work"

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


def capture_frame_bytes(app, capture_id):
    """The capture's JPEG: the PG blob in PG mode, the file otherwise. None if absent."""
    store = app.get("pg_store")
    if store is not None:
        return store.read_capture_frame(capture_id)
    path = Path(app["captures"]) / capture_id / "frame.jpg"
    return path.read_bytes() if path.is_file() else None


def materialize_capture_frame(app, capture_id):
    """A file path for cv2. PG mode materializes the blob into the scratch cache
    (captures are content-addressed and immutable, so a cached copy is exact)."""
    directory = Path(app["captures"]) / capture_id
    path = directory / "frame.jpg"
    if path.is_file():
        return path
    data = capture_frame_bytes(app, capture_id)
    if data is None:
        return None
    directory.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


@web.middleware
async def json_errors(request: web.Request, handler):
    """Every error leaves as JSON, with its reason in `error`.

    aiohttp's `HTTPUnprocessableEntity(text=...)` answers `text/plain`, and the
    panel parses JSON -- so a 422 with a perfectly good explanation ("no
    calibration in /app/work/calib: run calibrate_intrinsics.py once") arrived
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


WEB_JS_ASSETS = frozenset({
    "viewer.js", "api.js", "video.js", "reach-config.js", "geometry.js",
    "arm-model.js", "reach.js", "aruco.js", "markers.js", "height.js",
    "locate.js", "ekf.js", "stream.js", "trace.js",
})


async def handle_web_js(request: web.Request) -> web.StreamResponse:
    name = request.path.lstrip("/")
    if name not in WEB_JS_ASSETS:
        raise web.HTTPNotFound()
    script = WEB_DIR / name
    if not script.is_file():
        return web.Response(text="%s is missing next to capture_server.py" % name, status=500)
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
    is_pg = bool(getattr(app["traces"], "is_pg", False))
    directory = None if is_pg else app["traces"].session_dir(sid)
    if directory is not None:
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
        if directory is not None:
            (directory / "calib.json").write_bytes(calib_path.read_bytes())
            snapshot["calib_file"] = "calib.json"
    except Exception as exc:  # noqa: BLE001 - a session without calib still traces
        logging.warning("trace manifest for %s has no calibration: %s", sid, exc)
    if directory is not None and app["rig"] is not None and app["extrinsics_path"]:
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
        if getattr(recorder, "is_pg", False):
            exists = await asyncio.to_thread(recorder.store.episode_exists, sid)
            if not exists:
                raise FileNotFoundError(sid)
        if kind == "manifest":
            manifest = await recorder.read_manifest(sid)
            if manifest is None:
                raise FileNotFoundError(sid)
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
        store = app.get("pg_store")
        if store is not None:
            try:
                await asyncio.to_thread(store.set_kv, "llm_context", info)
            except Exception as exc:  # noqa: BLE001
                logging.warning("llm_context persist failed: %s", exc)
        if info["tokens"]:
            logging.info("agent context: %d tokens (%s)", info["tokens"], info["source"])
        else:
            logging.warning("agent context probe failed: %s", info["error"])
        return info


async def handle_health(request: web.Request) -> web.Response:
    app = request.app
    await ensure_llm_context(app)
    store = app.get("pg_store")
    pg_summary = None
    if store is not None:
        try:
            pg_summary = await asyncio.to_thread(
                store.health_summary, app.get("pg_workspace", "default"),
                app.get("arm_id", "b601"))
        except Exception as exc:  # noqa: BLE001 - health must survive DB hiccup
            logging.warning("health_summary failed: %s", exc)
    if pg_summary is not None:
        cap_count = int(pg_summary.get("captures") or 0)
        latest_cap = pg_summary.get("latest")
        trace_sessions = int(pg_summary.get("trace_sessions") or 0) if app.get("traces") is not None else 0
        acc_map = pg_summary.get("accuracy_map") or {"version": 1, "cells": {}}
        cells = acc_map.get("cells") or {}
        validated = sum(1 for cell in cells.values() if cell.get("validated_at"))
        accuracy_info = {"cells": len(cells), "validated": validated,
                         "validated_fraction": round(identify_mod.validated_fraction(acc_map), 4)}
    else:
        captures = capture_dirs(app["captures"])
        cap_count = len(captures)
        latest_cap = captures[0].name if captures else None
        trace_sessions = _trace_session_count(app)
        accuracy_info = _accuracy_health(app)
    return web.json_response({
        "ok": True,
        "version": VERSION,
        "server_time": utc_now(),
        "captures": cap_count,
        "latest": latest_cap,
        "whip": app["whip_url"],
        "whep": app["whep_url"],
        "tls": app["tls"],
        "auth": bool(app["token"]),
        "planner": app["planner"],
        "workflow": app.get("workflow_url") or None,
        "substrate": store is not None,
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
        "vertical_hover_m": app["vertical_hover_m"],
        "tip_m": app["tip_m"],
        "min_z": app["min_z"],
        "vertical_min_z": app["vertical_min_z"],
        "stream_orientation": app["stream_orientation"],
        "stream_turn": ({"turns": int(app["measured_turn"]["turns"]),
                         "source": "measured", "error_px":
                         round(float(app["measured_turn"]["error_px"]), 2),
                         "size": app["measured_turn"]["size"]}
                        if app.get("measured_turn") else None),
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
            "sessions": trace_sessions,
            "retention_days": app.get("trace_retention_days", 30),
        },
        "accuracy": accuracy_info,
    })


def _accuracy_health(app):
    try:
        map_doc = _accuracy_map(app)
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
    except Exception:  # noqa: BLE001 - health must survive an unreadable store/dir
        return 0


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
    turn = stream_turn(app, calibration,
                       tuple(app["measured_turn"]["size"])
                       if app.get("measured_turn") and app["measured_turn"].get("size")
                       else None)
    if turn is None:
        turns, source = None, "shape"
    else:
        turns = int(turn)
        source = "measured" if app.get("measured_turn") else "phone"
    return web.json_response({
        "ok": True,
        "mode": effective_mode(app),
        "tool_axis": app["tool_axis"],
        "plane_z": app["plane_z"],
        "hover_m": app["hover_m"],
        "vertical_hover_m": app["vertical_hover_m"],
        "min_z": app["min_z"],
        "vertical_min_z": app["vertical_min_z"],
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
    store = app.get("pg_store")
    if store is not None:
        rows = await asyncio.to_thread(store.list_captures, limit)
        items = []
        for row in rows:
            snap = row["snapshot"] or {}
            items.append({
                "id": row["episode_id"],
                "created": snap.get("capturedAt") or (snap.get("server") or {}).get("received_at"),
                "purpose": snap.get("purpose"),
                "tap": snap.get("tap"),
                "image_size": snap.get("image_size"),
                "bytes": row["size_bytes"] or 0,
                "has_target": False,
            })
        return web.json_response({"ok": True, "captures": items})
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
        mirror = app.get("pg_mirror")
        if mirror is not None:
            # PG mode: the substrate is the only capture store. The streamed
            # partial is scratch — persist it, then remove it. A capture that
            # cannot be persisted is refused, not stranded on disk.
            kind = "calibration" if purpose in ("extrinsics", "calib") else "execution"
            try:
                frame_bytes = await asyncio.to_thread(partial.read_bytes)
                await asyncio.to_thread(
                    mirror.mirror_capture, directory.name, record, frame_bytes,
                    kind=kind, image_size=meta.get("image_size"))
            except Exception as exc:  # noqa: BLE001
                partial.unlink(missing_ok=True)
                try:
                    directory.rmdir()
                except OSError:
                    pass
                raise web.HTTPServiceUnavailable(text="capture store unavailable: %s" % exc)
            partial.unlink(missing_ok=True)
            try:
                directory.rmdir()
            except OSError:
                pass
        else:
            partial.replace(frame_path)
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
    """The calibration for this phone: explicit, the active rig's, else newest."""
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
    # Durable bench default: config/calib survives a wiped work/ directory and a
    # fresh checkout. Only for the default work/calib dir -- a test or caller
    # with its own --calib-dir must keep its own fixtures. An explicit --calib,
    # REBOT_CALIB, or the active rig's device file still wins above.
    bundled = ROOT / "config" / "calib"
    if bundled.is_dir() and folder.resolve() == (ROOT / "work" / "calib").resolve():
        options += [p for p in bundled.glob("*.json")
                    if not p.name.startswith(("extrinsics-", "witness-", "sag-",
                                              "accuracy-map"))]
    if not options:
        raise web.HTTPUnprocessableEntity(
            text="no calibration in %s: run calibrate_intrinsics.py once for this phone" % folder)
    # A synthetic fixture must never outrank a real device calibration just
    # because its name sorts later. Within the real ones, the newest file wins;
    # the active rig is the strongest tie to the intended camera.
    real = [p for p in options if p.stem.lower() != "synthetic"]
    pool = real or options
    rig = app.get("rig")
    if rig is not None:
        store = app.get("pg_store")
        if store is not None:
            try:
                with store._conn() as c:
                    row = c.execute(
                        "SELECT calib_doc->>'device' AS device FROM world.calibration "
                        "WHERE calib_fingerprint=%s", (rig.calib_fingerprint,)).fetchone()
                device = row["device"] if row else None
                if device:
                    candidate = folder / ("%s.json" % device)
                    if candidate.is_file():
                        return candidate
            except Exception:  # noqa: BLE001 - fall back to the file pool
                pass
    return max(pool, key=lambda p: p.stat().st_mtime)


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


def _is_vertical_request(app, body) -> bool:
    opts = body.get("options") if isinstance(body, dict) else None
    opts = opts if isinstance(opts, dict) else {}
    axis = str(opts.get("tool_axis") or app["tool_axis"])
    entry = str(opts.get("entry") or ("vertical" if axis == "x+" else "front"))
    return axis == "x+" or entry == "vertical"


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

    min_z = app["vertical_min_z"] if _is_vertical_request(app, body) else app["min_z"]
    tips = [target_tip(target, index) for index, target in enumerate(targets, start=1)]
    tap_heights = [tip[2] - float(app["plane_z"]) for tip in tips]
    for index, tip in enumerate(tips, start=1):
        if tip[2] < min_z - 1e-9:
            return web.json_response({
                "ok": False, "stage": "target",
                "error": "marker %d: tip z %.3f m is below the minimum %.3f m"
                         % (index, tip[2], min_z),
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
            if point[2] < min_z - 1e-9:
                return web.json_response({
                    "ok": False, "stage": "target",
                    "error": "marker %d via %d: tip z %.3f m is below the minimum %.3f m"
                             % (index, via_index, point[2], min_z),
                }, status=422)

    plan_options = {
        "tool_axis": app["tool_axis"],
        "yaw": float(body.get("yaw", app["yaw"])),
        "min_z": min_z,
    }
    plan_options.update(body.get("options") or {})
    plan_options["min_z"] = min_z
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
        map_doc = await asyncio.to_thread(_accuracy_map, app)
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
        if app.get("pg_store") is not None:
            def _materialize_for_plan():
                frame_path = materialize_capture_frame(app, capture_id)
                if frame_path is None:
                    return False
                meta_path = directory / "meta.json"
                if not meta_path.is_file():
                    meta_doc = app["pg_store"].read_capture_meta(capture_id) or {}
                    meta_path.write_text(json.dumps(meta_doc, indent=2) + "\n", encoding="utf-8")
                return True
            if not await asyncio.to_thread(_materialize_for_plan):
                raise web.HTTPNotFound(text="no frame for %s" % capture_id)
        else:
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

    is_vertical = _is_vertical_request(app, body)
    default_hover = app["vertical_hover_m"] if is_vertical else app["hover_m"]
    min_z = app["vertical_min_z"] if is_vertical else app["min_z"]
    tap_height = float(body.get("hover_m", default_hover))
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
    if tip[2] < min_z - 1e-9:
        return web.json_response({
            "ok": False, "stage": "target", "capture": capture_id,
            "error": "tip z %.3f m is below the minimum %.3f m"
                     % (tip[2], min_z),
            "target": target,
        }, status=422)

    plan_options = {
        "tool_axis": app["tool_axis"],
        "yaw": float(body.get("yaw", app["yaw"])),
        "min_z": min_z,
    }
    plan_options.update(body.get("options") or {})
    plan_options["min_z"] = min_z

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


def _parse_json_lenient(raw):
    """Parse a JSON object from a tool call, tolerating common quantized quirks.

    Handles double-encoded JSON strings ('"{\\"u\\": 400}"') and trailing commas
    before '}' or ']'. Returns (dict, canonical_json_str) or (None, None).
    """
    if raw is None or raw == "":
        return {}, "{}"
    if isinstance(raw, dict):
        return raw, json.dumps(raw, ensure_ascii=False)
    text = str(raw).strip()
    if not text:
        return {}, "{}"
    for candidate in (text, re.sub(r",\s*([}\]])", r"\1", text)):
        try:
            value = json.loads(candidate)
        except (TypeError, ValueError):
            continue
        if isinstance(value, str):
            inner = value.strip()
            for inner_cand in (inner, re.sub(r",\s*([}\]])", r"\1", inner)):
                try:
                    value = json.loads(inner_cand)
                    break
                except (TypeError, ValueError):
                    pass
        if isinstance(value, dict):
            return value, json.dumps(value, ensure_ascii=False)
    return None, None


_LEAKED_TOOL_TAG_RE = re.compile(r"<tool_call>\s*(.*?)\s*(?:</tool_call>|$)", re.DOTALL | re.IGNORECASE)
_FENCED_JSON_RE = re.compile(r"```(?:json)?\s*(\{.*?\})\s*```", re.DOTALL | re.IGNORECASE)


def _extract_leaked_tool_calls(content, allowed_names):
    """Recover tool calls dumped into plain content when llama.cpp fails to parse them.

    Quantized models occasionally emit slightly malformed <tool_call>{...} tags
    or fenced JSON blocks that bypass the server's parser. Only recovers objects
    whose `name` matches an explicitly declared tool in this request.
    """
    if not content or not allowed_names:
        return [], content
    found = []
    cleaned = content

    def _try_parse_call(blob):
        parsed, _ = _parse_json_lenient(blob)
        if not isinstance(parsed, dict):
            return None
        name = str(parsed.get("name") or "").strip()
        if name.startswith("functions."):
            name = name[len("functions."):]
        if name not in allowed_names:
            return None
        raw_args = parsed.get("arguments")
        if raw_args is None:
            raw_args = parsed.get("parameters") or {}
        args_dict, canonical = _parse_json_lenient(raw_args)
        if args_dict is None:
            return None
        return {
            "id": "recovered_%d" % len(found),
            "type": "function",
            "function": {"name": name, "arguments": canonical},
        }

    # 1. Explicit <tool_call>...</tool_call> (or unclosed <tool_call>{...})
    matches = list(_LEAKED_TOOL_TAG_RE.finditer(cleaned))
    if matches:
        kept = []
        last = 0
        for match in matches:
            call = _try_parse_call(match.group(1))
            if call is not None:
                found.append(call)
                kept.append(cleaned[last:match.start()])
                last = match.end()
        if found:
            kept.append(cleaned[last:])
            return found, "".join(kept).strip()

    # 2. Markdown-fenced ```json {"name": ..., "arguments": ...} ```
    fenced = list(_FENCED_JSON_RE.finditer(cleaned))
    if fenced:
        kept = []
        last = 0
        for match in fenced:
            call = _try_parse_call(match.group(1))
            if call is not None:
                found.append(call)
                kept.append(cleaned[last:match.start()])
                last = match.end()
        if found:
            kept.append(cleaned[last:])
            return found, "".join(kept).strip()

    # 3. Bare JSON object as the entire content (common with Q4 drift)
    stripped = cleaned.strip()
    if stripped.startswith("{") and stripped.endswith("}"):
        call = _try_parse_call(stripped)
        if call is not None:
            found.append(call)
            return found, ""

    return [], content


def _detect_repetition_loop(text):
    """Detect degenerate token/phrase repetition loops in streamed reasoning."""
    if len(text) < 600:
        return False
    tail = text[-900:]
    # Check 4+ consecutive repeats of a 20..160 char phrase at the tail.
    for span in range(20, 161):
        if len(tail) < span * 4:
            break
        unit = tail[-span:]
        if not unit.strip():
            continue
        if tail.endswith(unit * 4):
            return True
    # Also check repeated identical non-empty lines in the last 10 lines.
    lines = [line.strip() for line in tail.splitlines() if len(line.strip()) >= 16]
    if len(lines) >= 6:
        recent = lines[-10:]
        last_line = recent[-1]
        if recent.count(last_line) >= 5:
            return True
    return False


def _is_context_overflow_error(status, text):
    """Recognize llama.cpp / OpenAI context-window overflow responses."""
    if status not in (400, 413, 422):
        return False
    low = str(text or "").lower()
    return any(needle in low for needle in (
        "exceeds the available context",
        "context_length_exceeded",
        "context size",
        "maximum context length",
        "n_ctx",
        "prompt is too long",
    ))


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
        fn = call.get("function") or {}
        arguments = fn.get("arguments")
        if arguments is None or arguments == "":
            continue
        parsed, canonical = _parse_json_lenient(arguments)
        if parsed is None:
            # Distinguish invalid JSON vs valid non-object JSON.
            try:
                value = json.loads(arguments)
            except (TypeError, ValueError):
                return ("the model produced a tool call whose arguments are not valid JSON "
                        "(%s); nothing was run. Retry, or raise REBOT_LLM_MAX_TOKENS."
                        % str(arguments)[:80])
            if not isinstance(value, dict):
                return ("the model produced a tool call whose arguments are not a JSON object "
                        "(%s); nothing was run." % str(arguments)[:80])
        elif canonical != arguments:
            fn["arguments"] = canonical
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
    stream has finished and those pieces form valid JSON. Handles servers that
    omit `index` on subsequent tool calls by allocating a new slot on id change.
    """
    for part in parts or []:
        if not isinstance(part, dict):
            continue
        raw_index = part.get("index")
        if raw_index is None:
            part_id = str(part.get("id") or "")
            if part_id and calls:
                matched = next((idx for idx, s in calls.items() if s.get("id") == part_id), None)
                index = matched if matched is not None else (max(calls) + 1)
            else:
                index = max(calls) if calls else 0
        else:
            try:
                index = int(raw_index)
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


async def _relay_sse(request, upstream, on_turn=None, allowed_tools=None):
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
    reasoning_len = 0
    next_rep_check = 600
    loop_aborted = False
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
                        reasoning_len += len(reasoning)
                        await send("reasoning", {"text": reasoning})
                        if reasoning_len >= next_rep_check:
                            next_rep_check = reasoning_len + 200
                            if _detect_repetition_loop("".join(reasoning_parts)):
                                loop_aborted = True
                                break
                    content = _agent_text(delta.get("content"))
                    if content:
                        content_parts.append(content)
                        await send("content", {"text": content})
                    _merge_tool_delta(tool_calls, delta.get("tool_calls"))
                if loop_aborted:
                    break
            if finished or loop_aborted:
                break
        if loop_aborted:
            err = "the model entered a repetition loop while reasoning"
            logging.warning("agent stream aborted: %s (%d reasoning chars)", err, reasoning_len)
            await send("error", {"error": err, "retryable": True,
                                 "reason": "repetition_loop", "finish_reason": finish_reason})
            await note({"error": err})
            return response
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
        if not message["tool_calls"] and allowed_tools and finish_reason != "length":
            recovered, stripped = _extract_leaked_tool_calls(message["content"], allowed_tools)
            if recovered:
                logging.info("agent stream: recovered %d leaked tool call(s) from content",
                             len(recovered))
                message["tool_calls"] = recovered
                message["content"] = stripped
        tool_error = _agent_tool_error(message, finish_reason)
        complete = bool((finished or finish_reason) and finish_reason != "length")
        if tool_error:
            reason = "length_in_tool_call" if finish_reason == "length" else "malformed_tool_call"
            await send("error", {"error": tool_error, "retryable": True,
                                 "reason": reason, "finish_reason": finish_reason})
            await note({"error": tool_error})
        elif not message["content"].strip() and not message["tool_calls"]:
            if finish_reason == "length":
                err = ("the model hit its output token limit while reasoning before "
                       "producing a reply or tool call")
                reason = "length_in_reasoning"
            else:
                err = "the model returned neither text nor tool calls"
                reason = "empty_turn"
            await send("error", {"error": err, "retryable": True,
                                 "reason": reason, "finish_reason": finish_reason})
            await note({"error": err})
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
    allowed_tools = {
        str((t.get("function") or {}).get("name") or "").strip()
        for t in (tools or []) if isinstance(t, dict)
    } - {""}

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
                                  "max_tokens": effective_max_tokens,
                                  "tools_sha256": tools_sha})

    stream = bool(body.get("stream"))
    effective_max_tokens = app["llm_max_tokens"]
    if body.get("max_tokens_bump") and effective_max_tokens > 0:
        bumped = min(max(effective_max_tokens * 2, 2048), 4096)
        ctx_limit = int((app.get("llm_context") or {}).get("tokens") or 0)
        if ctx_limit > 1024:
            bumped = min(bumped, max(effective_max_tokens, ctx_limit // 4))
        effective_max_tokens = bumped
    payload = {
        "model": app["llm_model"],
        "messages": messages,
        "stream": stream,
        "temperature": app["llm_temperature"],
    }
    if effective_max_tokens > 0:
        payload["max_tokens"] = effective_max_tokens
    if tools:
        payload["tools"] = tools
        payload["tool_choice"] = "auto"
    thinking_pref = app.get("llm_thinking")
    if body.get("disable_thinking"):
        payload["chat_template_kwargs"] = {"enable_thinking": False}
    elif thinking_pref is not None:
        payload["chat_template_kwargs"] = {"enable_thinking": bool(thinking_pref)}
    headers = {"Content-Type": "application/json"}
    if stream:
        headers["Accept"] = "text/event-stream"
    if app["llm_token"]:
        headers["Authorization"] = "Bearer " + app["llm_token"]

    # Brief server-side backoff on 503 (llama-server loading / unwinding an
    # aborted slot right after the user presses Stop).
    backoffs = (0.6, 1.2)
    text = ""
    try:
        for attempt in range(len(backoffs) + 1):
            async with app["client"].post(app["llm_url"], json=payload, headers=headers,
                                          timeout=ClientTimeout(total=app["llm_timeout"])) as answer:
                if answer.status == 503 and attempt < len(backoffs):
                    logging.info("agent %s returned 503 (slot busy/loading); retrying in %.1f s",
                                 app["llm_url"], backoffs[attempt])
                    await asyncio.sleep(backoffs[attempt])
                    continue
                if answer.status != 200:
                    text = await answer.text()
                    logging.warning("agent %s answered %d: %s", app["llm_url"],
                                    answer.status, text[:200])
                    error = "the model refused (%d): %s" % (answer.status, text[:300])
                    await trace_event(app, sid, "stream_error", error=error,
                                      status=answer.status)
                    err_body = {"ok": False, "error": error}
                    if _is_context_overflow_error(answer.status, text):
                        err_body["retryable"] = True
                        err_body["reason"] = "context_overflow"
                    return web.json_response(err_body, status=502)
                content_type = answer.headers.get("Content-Type") or ""
                if stream and "text/event-stream" in content_type:
                    return await _relay_sse(request, answer, on_turn=on_turn,
                                            allowed_tools=allowed_tools)
                # A server that ignores `stream`, or the ordinary non-streamed
                # request: read the one JSON body and answer in kind.
                text = await answer.text()
                break
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
    if not message["tool_calls"] and allowed_tools and finish_reason != "length":
        recovered, stripped = _extract_leaked_tool_calls(message["content"], allowed_tools)
        if recovered:
            logging.info("agent: recovered %d leaked tool call(s) from content", len(recovered))
            message["tool_calls"] = recovered
            message["content"] = stripped
    tool_error = _agent_tool_error(message, finish_reason)
    if tool_error:
        reason = "length_in_tool_call" if finish_reason == "length" else "malformed_tool_call"
        await trace_event(app, sid, "stream_error", error=tool_error)
        return web.json_response({"ok": False, "error": tool_error, "retryable": True,
                                  "reason": reason, "finish_reason": finish_reason}, status=502)
    if not message["content"].strip() and not message["tool_calls"]:
        if finish_reason == "length":
            err = ("the model hit its output token limit while reasoning before "
                   "producing a reply or tool call")
            reason = "length_in_reasoning"
        else:
            err = "the model returned neither text nor tool calls"
            reason = "empty_turn"
        await trace_event(app, sid, "stream_error", error=err)
        return web.json_response({
            "ok": False,
            "error": err,
            "retryable": True,
            "reason": reason,
            "finish_reason": finish_reason,
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
    plan_id = str(body.get("id") or "")

    # Durable execute -> oracle (PG mode with an activity service configured).
    # The graph owns the whole chain: claim the play (at-most-once), run the
    # isolated execute/extract/detect activity, then fold the verdict. Capture
    # no longer schedules live hold grabs, so nothing here races playback.
    workflow_url = app.get("workflow_url")
    store = app.get("pg_store")
    if store is not None and workflow_url and CAPTURE_ID.match(plan_id):
        import sys
        substrate_dir = str(ROOT / "substrate")
        if substrate_dir not in sys.path:
            sys.path.insert(0, substrate_dir)
        import execute_workflow
        # The recording episode is per execution, not per chat session: the
        # fragments must be attachable to this execution regardless of which
        # sidebar conversation triggered it.
        episode = "exec-" + plan_id
        play_timeout_s = int(float(os.environ.get("REBOT_PLAY_TIMEOUT", "900")))
        try:
            receipt = await asyncio.to_thread(
                execute_workflow.start_execute_workflow,
                store, plan_id, plan_id, app.get("arm_id", "b601"), episode,
                workflow_url, app.get("pg_workspace", "default"),
                timeout_s=60,
                play_timeout_s=play_timeout_s,
                poll_s=2, await_s=max(600, play_timeout_s + 120),
                label="execute_oracle")
        except Exception as exc:  # noqa: BLE001 - the workflow is the execution path
            logging.exception("durable execute for %s failed to start", plan_id)
            await trace_event(app, sid, "execute", plan_id=plan_id, status=502,
                              ok=False, error=str(exc))
            return web.json_response({"ok": False, "id": plan_id, "executed": False,
                                      "error": "durable workflow failed to start: %s" % exc},
                                     status=502)
        out = receipt.get("receipt") or {}
        if out.get("pending"):
            return web.json_response({"id": plan_id, **out,
                                      "workflow_status": out.get("workflow_status")
                                      or receipt.get("status")}, status=202)
        executed = bool(out.get("ok"))
        verdict = out.get("verdict")
        error_text = out.get("error")
        await trace_event(app, sid, "execute", plan_id=plan_id, status=200 if executed else 502,
                          ok=executed, executed=bool(out.get("executed")),
                          workflow=out.get("workflow") or receipt.get("instance"),
                          error=None if executed else error_text)
        if verdict:
            await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=verdict,
                              holds=int(out.get("holds") or 0),
                              telemetry=bool((out.get("ledger") or {}).get("telemetry_sha")),
                              source="durable_offline_extract", error=None)
        return web.json_response({"id": plan_id, **out,
                                  "workflow_status": out.get("workflow_status")
                                  or receipt.get("status")},
                                 status=200 if executed else 502)

    # File mode / no activity service: the direct gateway path survives for
    # offline development. There is no live grab any more; the pre/post stills
    # are the only camera evidence, and the oracle doc reports the missing
    # holds rather than guessing them.
    prep = None
    if CAPTURE_ID.match(plan_id):
        prep = await oracle_prepare(app, plan_id)
    status, payload = await planner_call(app, "/execute", body, timeout=360, sid=sid)
    await trace_event(app, sid, "execute", plan_id=body.get("id"), status=status,
                      ok=payload.get("ok"), executed=payload.get("executed"),
                      error=payload.get("error"))
    if prep is not None:
        if status == 200 and payload.get("ok") and payload.get("executed"):
            task = asyncio.create_task(oracle_finish(app, prep, sid))
            app["oracle_tasks"].add(task)
            task.add_done_callback(app["oracle_tasks"].discard)
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
    """Pre-execution evidence: one still and the plan's hold windows.

    Returns None on any failure -- the execution proceeds unverified (§8.4).
    The pre-still goes through grab_one, the existing capture path, so its
    witness report and tag solve are recorded exactly as a manual capture's.
    Hold stills are NOT grabbed here any more: the durable workflow extracts
    them offline from the executed-trajectory recording.
    """
    try:
        if not app["camera"]:
            return None
        store = app.get("pg_store")
        if store is not None:
            raw = await asyncio.to_thread(store.read_plan_blob, plan_id)
            if raw is None:
                return None
            trajectory = json.loads(raw)
        else:
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
        pre_bytes = await asyncio.to_thread(capture_frame_bytes, app, pre["id"])
        if pre_bytes is None:
            return None
        # cv2 reads a path: the scratch materialization of the PG blob (PG mode)
        # or the capture file (file mode). The authoritative copy is the blob.
        frame_path = await asyncio.to_thread(materialize_capture_frame, app, pre["id"])
        if store is None:
            directory = app["oracle_dir"] / plan_id
            await asyncio.to_thread(directory.mkdir, parents=True, exist_ok=True)
            await asyncio.to_thread((directory / "pre.jpg").write_bytes, pre_bytes)
        prep = {
            "plan_id": plan_id,
            "windows": windows,
            "waypoints": trajectory.get("waypoints") or [],
            "targets": targets,
            "pre_capture": pre["id"],
            "pre_frame_path": str(frame_path),
            "pre_blob": await trace_blob(app, pre_bytes),
            # Wherever a capture path ran, its witness report is persisted.
            "witness": pre.get("verification"),
            "holds": {},
            "started": time.monotonic(),
        }
        # No live hold grabs: hold stills are extracted offline from the
        # executed-trajectory recording by the durable workflow. The pre-still
        # and the hold windows are still captured here for the coverage/legacy
        # paths that finish synchronously.
        return prep
    except Exception as exc:  # noqa: BLE001 - the oracle never fails /execute
        logging.warning("oracle prep for %s failed: %s", plan_id, exc)
        return None


# The detector lives in one module so the isolated workflow activity can run it
# without importing this aiohttp server (camera, routes, LLM client). Keep the
# old name for the selftest and the synchronous Python oracle path.
_oracle_detect = oracle_vision.detect_tip


def _oracle_tool_tip(app, rig):
    """The fitted flange-to-tip vector, or the configured offset along -z.

    plan_to_target aims `tip = gripper_end - [0, 0, tip_offset_m]`; the rig's
    fitted tip_offset is the measured version of the same vector.
    """
    if rig is not None and float(np.linalg.norm(rig.tip_offset)) > 1e-6:
        return np.asarray(rig.tip_offset, dtype=float)
    return np.array([0.0, 0.0, -float(app["tip_m"])])


def _read_robot_urdf(store):
    with store._conn() as c:
        row = c.execute("SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1").fetchone()
    return next(iter(row.values())) if row else None


async def _oracle_pg_compute(app, prep, samples, doc, K, dist, rig, t_tip, turn):
    """PG mode: detect each hold's tip (OpenCV, isolated here in the capture
    process), then let the substrate compute the residuals + verdict
    (oracle_compute). Returns True if it populated doc; False -> Python path."""
    mirror = app.get("pg_mirror")
    if mirror is None or rig is None:
        return False
    import cv2
    store = mirror.store
    urdf = await asyncio.to_thread(_read_robot_urdf, store)
    if not urdf:
        return False
    detections = {}
    for window in prep["windows"]:
        shot = prep["holds"].get(window["index"])
        inside = [s for s in samples
                  if window["t_start"] - 1e-6 <= float(s.get("t", -1)) <= window["t_end"] + 1e-6]
        if not shot or not inside:
            continue
        q_meas = np.mean([s["meas"] for s in inside], axis=0)[:6]
        frame = await asyncio.to_thread(cv2.imread, shot["path"])
        if frame is None:
            continue
        frame, calib_g, _turn = locator.upright_frame(resolve_calibration(app), frame, turn)
        found = await asyncio.to_thread(
            _oracle_detect, frame, q_meas, K, dist,
            np.asarray(rig.T_base_cam, dtype=float), t_tip, calib_g.scale)
        if found:
            detections[str(window["index"])] = dict(
                found, image_size=[int(frame.shape[1]), int(frame.shape[0])],
                grid_scale=round(float(calib_g.scale), 4), frame=shot["blob"])
    kp_spec = sweep.KP_DEFAULT
    tel = doc.get("telemetry") or {}
    if isinstance(tel.get("kp"), dict):
        kp_spec = ",".join("%d:%s" % (m, tel["kp"][str(m)])
                           for m in range(1, 7) if str(m) in tel["kp"])
    doc_extra = {k: doc[k] for k in (
        "telemetry", "stream_turn", "tag_reprojection_px", "witness", "landscape",
        "pre_image_size", "pre_grid_scale", "plan_id", "created", "session",
        "pre_capture", "pre_frame") if k in doc}
    K_flat = [float(v) for row in np.asarray(K, dtype=float).reshape(3, 3) for v in row]
    T_flat = [float(v) for v in np.asarray(rig.T_base_cam, dtype=float).reshape(-1)]
    dist_flat = [float(v) for v in np.asarray(dist, dtype=float).reshape(-1)]
    tip_flat = [float(v) for v in np.asarray(t_tip, dtype=float).reshape(-1)]

    def _call():
        return store.scalar(
            "SELECT oracle_compute(%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb,"
            "%s,%s,%s,%s,%s,%s::jsonb,%s)",
            (urdf, json.dumps(prep["waypoints"]), json.dumps(samples),
             json.dumps(prep.get("targets") or []), json.dumps(detections),
             K_flat, dist_flat, T_flat, tip_flat, kp_spec, json.dumps(doc_extra),
             app.get("arm_id", "b601")))
    res = await asyncio.to_thread(_call)
    doc["holds"] = res["holds"]
    doc["verdict"] = res["verdict"]
    if res.get("max_radius_m"):
        doc["max_radius_m"] = res["max_radius_m"]
    return True


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

        # Item 5: in PG mode read telemetry straight from the substrate (skip planner HTTP hop).
        telemetry = None
        if app.get("pg_store") is not None:
            telemetry = await asyncio.to_thread(app["pg_store"].read_telemetry_doc, plan_id)
        if telemetry is None:
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
        pre_image = cv2.imread(prep["pre_frame_path"])
        turn = stream_turn(app, calibration,
                           (pre_image.shape[1], pre_image.shape[0])
                           if pre_image is not None else None)
        measured = app.get("measured_turn")
        doc["stream_turn"] = ({"turns": int(measured["turns"]), "source": "measured",
                               "error_px": round(float(measured["error_px"]), 2)}
                              if measured else
                              {"turns": turn, "source": "phone" if turn is not None else "shape"})
        # The tag reprojection error measure_stream_turn already computed, and
        # a fresh witness check on the pre-still wherever the capture path ran.
        if measured:
            doc["tag_reprojection_px"] = round(float(measured["error_px"]), 2)
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

        pg_handled = await _oracle_pg_compute(app, prep, samples, doc, K, dist, rig, t_tip, turn)
        matched = (oracle_mod.match_hold_targets(prep["windows"], prep.get("targets"))
                   if not pg_handled else [])
        for position, window in enumerate(prep["windows"] if not pg_handled else []):
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
                        if tip_meas and tip_ik:
                            hold["z_error_mm"] = round(1000.0 * (float(tip_meas[2]) - float(tip_ik[2])), 3)
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
                mirror = app.get("pg_mirror")
                if mirror is not None:
                    try:
                        await asyncio.to_thread(
                            mirror.mirror_still, plan_id, data, "post_exec",
                            app.get("arm_id", "b601"))
                    except Exception as exc:  # noqa: BLE001
                        logging.warning("post still catalog for %s failed: %s", plan_id, exc)
                else:
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
        if not pg_handled:
            radii = [float(np.hypot(h["tip_fk_cmd"][0], h["tip_fk_cmd"][1]))
                     for h in doc["holds"] if h.get("tip_fk_cmd")]
            if radii:
                doc["max_radius_m"] = round(max(radii), 4)
            doc["verdict"] = oracle_mod.attribute(doc)
        mirror = app.get("pg_mirror")
        if mirror is not None:
            # PG mode: the verdict doc is persisted to the substrate only. The
            # plan and telemetry are already there (the planner persists them).
            def _mirror_oracle():
                arm_id = app.get("arm_id", "b601")
                mirror.store.ensure_arm(arm_id)
                rig_fp = rig.fingerprint if rig is not None else None
                if sid:
                    mirror.store.create_episode(sid, mirror.workspace_id, [arm_id], kind="execution")
                mirror.mirror_oracle(plan_id, arm_id, doc, rig_fingerprint=rig_fp)
            try:
                await asyncio.to_thread(_mirror_oracle)
            except Exception as exc:  # noqa: BLE001 - the oracle never breaks /execute (§8.4)
                logging.warning("oracle persist for %s failed: %s", plan_id, exc)
        else:
            await asyncio.to_thread(
                (app["oracle_dir"] / ("%s.json" % plan_id)).write_text,
                json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=doc["verdict"],
                          holds=len(doc["holds"]),
                          telemetry=bool(telemetry), error=None)
        try:
            if app.get("pg_mirror") is None:
                identify_mod.update_accuracy_map(app["accuracy_map_path"], doc, rig=rig)
        except Exception as exc:  # noqa: BLE001
            logging.warning("accuracy map update for %s failed: %s", plan_id, exc)
        try:
            # In PG mode mirror_oracle already updated learning.monitor_state via SQL monitor_observe;
            # keep the in-process TriggerMonitor mirrored for immediate local reads.
            app["identify_monitor"].observe(doc, rig=app.get("rig"))
        except Exception as exc:  # noqa: BLE001
            logging.warning("identify monitor for %s failed: %s", plan_id, exc)
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001 - the oracle never fails /execute
        logging.warning("oracle for %s failed: %s", plan_id, exc)
        doc["error"] = "%s: %s" % (type(exc).__name__, exc)
        try:
            mirror = app.get("pg_mirror")
            if mirror is not None:
                def _mirror_error():
                    arm_id = app.get("arm_id", "b601")
                    mirror.store.ensure_arm(arm_id)
                    if sid:
                        mirror.store.create_episode(sid, mirror.workspace_id, [arm_id], kind="execution")
                    mirror.mirror_oracle(plan_id, arm_id, doc)
                await asyncio.to_thread(_mirror_error)
            else:
                await asyncio.to_thread(
                    (app["oracle_dir"] / ("%s.json" % plan_id)).write_text,
                    json.dumps(doc, indent=2) + "\n", encoding="utf-8")
            await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=None,
                              holds=len(doc["holds"]), error=doc["error"])
        except Exception as inner:  # noqa: BLE001 - not even the report is fatal
            logging.warning("oracle report for %s failed: %s", plan_id, inner)


def _start_coverage_recording(app):
    """Record the coverage excursion so its holds are extractable offline.

    Coverage moves are not the durable /execute path (they are driven by the
    identify session), so this function owns the recording window. A missing
    MediaMTX is reported and the run proceeds: motion is gated by the planner,
    evidence is additive (§8.4).
    """
    path = mediamtx.path_from_camera(app["camera"])
    if not path:
        return None, None
    try:
        recorder = mediamtx.Recorder(app["mediamtx_api"], path=path,
                                     recordings=app["recordings"])
        recorder.enable()
        started = time.time()
        time.sleep(0.2)
        return recorder, started
    except Exception as exc:  # noqa: BLE001 - a missing recorder costs evidence only
        logging.warning("coverage recording unavailable: %s", exc)
        return None, None


def _finish_coverage_recording(app, recorder, record_wall, episode, play_wall):
    if recorder is None or record_wall is None:
        return None
    try:
        recorder.disable()
        time.sleep(0.5)
        # mediamtx.Recorder.segments() already subtracts 2.0 s internally.
        segments = recorder.segments(since=record_wall)
        if not segments:
            return None
        store = app["pg_store"]
        import cv2

        blobs, durations, paths = [], [], []
        first_start = None
        for segment in segments:
            p = Path(segment.path)
            blobs.append(p.read_bytes())
            cap = cv2.VideoCapture(str(p))
            try:
                fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
                frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
                duration = float(frames) / float(fps) if fps > 0 and frames > 0 else 10.0
            finally:
                cap.release()
            durations.append(duration)
            paths.append(p)
            if first_start is None:
                first_start = float(segment.start)
        store.ingest_episode_segments(
            episode, app.get("pg_workspace", "default"), app.get("arm_id", "b601"),
            blobs, durations, codec="mp4")
        for p in paths:
            try:
                p.unlink()
            except OSError:
                pass
        return float(play_wall - (first_start if first_start is not None else play_wall))
    except Exception as exc:  # noqa: BLE001 - evidence loss is reported, not fatal
        logging.warning("coverage recording ingest failed: %s", exc)
        return None


async def _recorded_coverage_oracle(app, plan_id, episode, t_offset, sid):
    """Adopt a coverage execution and run the SQL coverage graph offline.

    The graph is `coverage_workflow_start` (0022): adopt -> extract -> store
    stills -> detect -> finalize. Capture only starts/awaits it and records the
    trace event; the extract/detect decisions and the learning fold are SQL.
    """
    import sys
    substrate_dir = str(ROOT / "substrate")
    if substrate_dir not in sys.path:
        sys.path.insert(0, substrate_dir)
    import execute_workflow

    store = app["pg_store"]
    arm = app.get("arm_id", "b601")
    outcome = await asyncio.to_thread(
        execute_workflow.start_coverage_workflow,
        store, plan_id, plan_id, arm, episode, float(t_offset),
        app["workflow_url"], app.get("pg_workspace", "default"), 300,
        "coverage:" + plan_id)
    receipt = outcome.get("receipt") or {}
    doc = receipt.get("doc") or {}
    try:
        app["identify_monitor"].observe(doc, rig=app.get("rig"))
        mirror = app.get("pg_mirror")
        if mirror is not None:
            mirror.mirror_monitor(
                arm, ewma_pixel_error_mm=app["identify_monitor"].ewma,
                ewma_tag_rms_px=app["identify_monitor"].tag_baseline,
                last_doc={"proposals": app["identify_monitor"].proposals})
    except Exception as exc:  # noqa: BLE001
        logging.warning("coverage monitor for %s failed: %s", plan_id, exc)
    await trace_event(app, sid, "oracle", plan_id=plan_id, verdict=receipt.get("verdict"),
                      holds=int(receipt.get("holds") or 0),
                      source="durable_offline_extract", error=receipt.get("error"))
    return doc


async def handle_pose(request: web.Request) -> web.Response:
    status, payload = await planner_call(request.app, "/pose", {}, timeout=30, method="GET")
    return web.json_response(payload, status=status)


def click_for(app, capture_id, u, v):
    """The tap the sweep starts from: a capture id and a pixel on the tool tip."""
    capture_id = str(capture_id or "")
    if not CAPTURE_ID.match(capture_id):
        raise web.HTTPBadRequest(text="'capture' must be a capture id")
    directory = Path(app["captures"]) / capture_id
    store = app.get("pg_store")
    if store is not None:
        meta = store.read_capture_meta(capture_id)
        if meta is None:
            raise web.HTTPNotFound(text="no capture %s" % capture_id)
        # The detector reads a file path: materialize the PG blob into the cache.
        frame_path = directory / "frame.jpg"
        if not frame_path.is_file():
            data = store.read_capture_frame(capture_id)
            if data is None:
                raise web.HTTPNotFound(text="capture %s has no frame" % capture_id)
            directory.mkdir(parents=True, exist_ok=True)
            frame_path.write_bytes(data)
    else:
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
        turn = stream_turn(app, base, (probe.shape[1], probe.shape[0]))
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
            await asyncio.to_thread(_pg_mirror_rig, app, rig)
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
        turn = stream_turn(app, base, (probe.shape[1], probe.shape[0]))
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
    map_doc = await asyncio.to_thread(_accuracy_map, app)
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
    recorded = app.get("pg_store") is not None and bool(app.get("workflow_url"))
    recorder, record_wall = (await asyncio.to_thread(_start_coverage_recording, app)
                             if recorded else (None, None))
    episode = "coverage-" + plan_id
    started = time.time()
    play_wall = time.time()
    try:
        answer = await asyncio.to_thread(arm.move_sequence, plan["targets"], plan["seconds"],
                                         plan["hold"], plan["gripper"], plan_id)
    finally:
        t_offset = (await asyncio.to_thread(
            _finish_coverage_recording, app, recorder, record_wall, episode, play_wall)
            if recorded else None)
        try:
            if t_offset is not None:
                await _recorded_coverage_oracle(app, plan_id, episode, t_offset, sid)
            else:
                # File mode / no recording: keep the synchronous evidence path.
                await oracle_finish(app, prep, sid)
        except Exception as exc:  # noqa: BLE001 - evidence never fails the move report
            logging.warning("coverage oracle for %s failed: %s", plan_id, exc)
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
    map_doc = await asyncio.to_thread(_accuracy_map, app)
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
    recorded = app.get("pg_store") is not None and bool(app.get("workflow_url"))
    recorder, record_wall = (await asyncio.to_thread(_start_coverage_recording, app)
                             if recorded else (None, None))
    episode = "coverage-" + plan_id
    started = time.time()
    play_wall = time.time()
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
        t_offset = (await asyncio.to_thread(
            _finish_coverage_recording, app, recorder, record_wall, episode, play_wall)
            if recorded else None)
        try:
            if t_offset is not None:
                await _recorded_coverage_oracle(app, plan_id, episode, t_offset, sid)
            else:
                await oracle_finish(app, prep, sid)
        except Exception as exc:  # noqa: BLE001 - evidence never fails the move report
            logging.warning("coverage oracle for %s failed: %s", plan_id, exc)
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
            await asyncio.to_thread(_pg_mirror_rig, app, rig)
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
        def _status_gaps_and_health():
            store = app.get("pg_store")
            rig = app.get("rig")
            sag_map = (store.read_sag_map_doc(app.get("arm_id", "b601"),
                                              rig.fingerprint if rig is not None else None)
                       if store is not None
                       else identify_mod.active_sag_map(app["calib"], sweep.KP_DEFAULT))
            acc_map = _accuracy_map(app)
            gaps_list = identify_mod.unlearned(sag_map, acc_map)
            cells = acc_map.get("cells") or {}
            validated = sum(1 for cell in cells.values() if cell.get("validated_at"))
            health = {"cells": len(cells), "validated": validated,
                      "validated_fraction": round(identify_mod.validated_fraction(acc_map), 4)}
            return gaps_list, health
        gaps, acc_health = await asyncio.to_thread(_status_gaps_and_health)
    except Exception as exc:  # noqa: BLE001 - a report never fails the route
        logging.warning("the unlearned report failed: %s", exc)
        gaps = []
        acc_health = {"cells": 0, "validated": 0, "validated_fraction": 0.0}
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
        "accuracy": acc_health,
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
    store = app.get("pg_store")
    if store is not None:
        # Item 3 + C4 fix: aggregate holds from PG telemetry corpus + fit + persist in 1 SQL call.
        try:
            result = await asyncio.to_thread(
                store.fit_sag_map_from_corpus,
                app.get("arm_id", "b601"), app["rig"].fingerprint, kp_spec,
                identify_mod.zeros_key(app["rig"]))
        except Exception as error:  # noqa: BLE001
            logging.warning("sag fit (PG) failed: %s", error)
            return web.json_response({"ok": False, "error": str(error)}, status=500)
        if result.get("ok") and result.get("map"):
            await asyncio.to_thread(identify_mod.save_sag_map, app["calib"], result["map"])
        return web.json_response(result, status=200 if result.get("ok") else 422)
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


def stream_turn(app, calibration, frame_size=None):
    """Quarter turns to apply to the stream to reach the calibration grid.

    Order: what the tag measurement saw, then what the phone published, then
    `None` -- which lets the caller fall back to the frame shape, that knows "a
    quarter turn" but not which way.

    `frame_size` is optional because some callers only have the cached turn.
    When given, a turn whose parity disagrees with the shape is discarded: a
    phone that reports angle 0 for a landscape frame (or a stale cached turn)
    must not turn a 1920x1080 frame into the wrong grid and then be refused as
    an aspect change. The shape source then decides the quarter (sign included
    only as the documented clockwise default).
    """
    def consistent(turn):
        if frame_size is None:
            return True
        swapped = calib_mod.swapped_shape(calibration.image_size, frame_size)
        return swapped == (int(turn) % 2 == 1)

    measured = app.get("measured_turn")
    if measured and measured.get("turns") is not None:
        turn = int(measured["turns"]) % 4
        return turn if consistent(turn) else None
    orientation = app.get("stream_orientation") or {}
    angle = orientation.get("angle")
    calib_deg = (calibration.raw or {}).get("orientation_deg")
    if angle is None or calib_deg is None:
        return None
    turn = int(round((float(angle) - float(calib_deg)) / 90.0)) % 4
    return turn if consistent(turn) else None


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
        turn = stream_turn(app, base, (frame.shape[1], frame.shape[0]))
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


def _pg_mirror_rig(app, rig):
    mirror = app.get("pg_mirror")
    if mirror is not None and rig is not None:
        try:
            # The witness reference frame is evidence too: store it as a blob so
            # the scratch cache can be rematerialized after a wipe (§8 flatten).
            witness_sha = None
            record = getattr(rig, "witness", None) or {}
            image = record.get("image")
            base = Path(app.get("extrinsics_path") or ".").resolve().parent
            if image:
                wpath = Path(image)
                if not wpath.is_absolute():
                    wpath = base / wpath
                if wpath.is_file():
                    witness_sha = mirror.store.put_blob(wpath.read_bytes(), "still")
            mirror.mirror_rig(rig, witness_sha=witness_sha)
        except Exception as exc:  # noqa: BLE001 - mirror never breaks the rig path (§8.4)
            logging.warning("rig mirror failed: %s", exc)


def _pg_mirror_sag(app, map_doc):
    mirror = app.get("pg_mirror")
    if mirror is not None:
        try:
            mirror.mirror_sag(app.get("arm_id", "b601"), map_doc)
        except Exception as exc:  # noqa: BLE001
            logging.warning("sag mirror failed: %s", exc)


def _accuracy_map(app):
    """The accuracy envelope: PG-only in PG mode (the cells are the record);
    the file map only in file mode."""
    store = app.get("pg_store")
    rig = app.get("rig")
    if store is not None and rig is not None:
        try:
            return store.read_accuracy_map(app.get("arm_id", "b601"), rig.fingerprint)
        except Exception as exc:  # noqa: BLE001
            logging.warning("accuracy map PG read failed: %s", exc)
            return {}
    return identify_mod.load_accuracy_map(app["accuracy_map_path"])


def _load_active_rig(app):
    """Reconstruct the active rig from PG (lossless rig_doc); None if absent/error.

    Materializes the rig doc + witness frame into the scratch calib cache so the
    file-based consumers (witness.load_for, the trace manifest's rig copy) work
    after a cache wipe. PG remains the system of record."""
    store = app.get("pg_store")
    if store is None:
        return None
    try:
        row = store.read_active_rig_row(app.get("pg_workspace", "default"))
        if row is None or row["rig_doc"] is None:
            return None
        doc = row["rig_doc"]
        rig = extrinsics.Rig.from_dict(doc)
        calib = Path(app["calib"])
        calib.mkdir(parents=True, exist_ok=True)
        (calib / "extrinsics-fixed.json").write_text(json.dumps(doc), encoding="utf-8")
        record = getattr(rig, "witness", None) or {}
        image = record.get("image")
        if image and row["witness_sha"]:
            data = store.get_blob(row["witness_sha"])
            name = Path(image).name
            (calib / name).write_bytes(data)
            if Path(image).is_absolute():
                Path(image).parent.mkdir(parents=True, exist_ok=True)
                Path(image).write_bytes(data)
        return rig
    except Exception as exc:  # noqa: BLE001
        logging.warning("PG rig load failed: %s", exc)
        return None


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
        record = {"id": capture_id, "pose": pose, "captured_at": utc_now(),
                  "source": "capture-pose"}
        if pose is None:
            record["pose_note"] = "not read: the base tag solved the camera"
        meta = {"purpose": "extrinsics", "image_size": size, "server": record}
        mirror = app.get("pg_mirror")
        if mirror is not None:
            # PG mode: the substrate is the only capture store; cv2 consumers
            # materialize the blob on demand (materialize_capture_frame).
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise auto_calibrate.AutoError("cannot encode the capture frame")
            mirror.mirror_capture(capture_id, meta, buf.tobytes(),
                                  kind="calibration", image_size=size)
            return capture_id
        directory = Path(app["captures"]) / capture_id
        directory.mkdir(parents=True, exist_ok=False)
        cv2.imwrite(str(directory / "frame.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        (directory / "meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
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
            await asyncio.to_thread(_pg_mirror_rig, app, rig)
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
    store = app.get("pg_store")
    if store is not None:
        # PG mode: the catalog serves the frame and meta; no file fallback.
        if name == "frame.jpg":
            data = await asyncio.to_thread(store.read_capture_frame, capture_id)
            if data is None:
                raise web.HTTPNotFound()
            return web.Response(body=data, content_type="image/jpeg",
                                headers={"Cache-Control": "no-store"})
        if name == "meta.json":
            meta = await asyncio.to_thread(store.read_capture_meta, capture_id)
            if meta is None:
                raise web.HTTPNotFound()
            return web.json_response(meta, headers={"Cache-Control": "no-store"})
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


async def recordings_ingest_loop(app):
    """Sweep ONLY orphaned MediaMTX segments into PG; never race active recordings.

    Active executions and coverage sweeps own and unlink their own segments via
    workflow_service._ingest_recording / _finish_coverage_recording. This loop
    waits until a file's mtime is older than the maximum play budget before
    sweeping it as an orphan, eliminating the mid-trajectory deletion race (C6).
    """
    mirror = app.get("pg_mirror")
    rec_dir = app.get("recordings")
    orphan_age_s = max(float(os.environ.get("REBOT_PLAY_TIMEOUT", "900")) + 60.0, 180.0)
    seen = set()
    while True:
        try:
            if mirror is not None and rec_dir is not None and rec_dir.is_dir():
                now = time.time()
                for mp4 in sorted(rec_dir.rglob("*.mp4")):
                    key = str(mp4)
                    if key in seen:
                        continue
                    try:
                        st = mp4.stat()
                    except OSError:
                        continue
                    if (now - st.st_mtime) < orphan_age_s:
                        continue  # may belong to an in-flight execution or sweep
                    ep = "rec-" + mp4.stem
                    await asyncio.to_thread(
                        mirror.mirror_recording, ep, mp4, app.get("arm_id", "b601"))
                    try:
                        await asyncio.to_thread(mp4.unlink)
                        seen.add(key)
                    except OSError:
                        pass
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            logging.warning("recordings ingest failed: %s", exc)
        await asyncio.sleep(60)


async def on_startup(app: web.Application):
    app["client"] = ClientSession()
    if app.get("llm_url") and not app["llm_context"]["tokens"]:
        # Do not hold up the listener; health (and the page) uses the result as
        # soon as the probe returns.
        app["llm_context_task"] = asyncio.create_task(ensure_llm_context(app))
    if app.get("traces") is not None:
        app["trace_prune_task"] = asyncio.create_task(trace_prune_loop(app))
    if app.get("pg_mirror") is not None:
        app["recordings_ingest_task"] = asyncio.create_task(recordings_ingest_loop(app))


async def on_cleanup(app: web.Application):
    task = app.get("llm_context_task")
    if task is not None:
        task.cancel()
    task = app.get("trace_prune_task")
    if task is not None:
        task.cancel()
    task = app.get("recordings_ingest_task")
    if task is not None:
        task.cancel()
    await app["client"].close()
    store = app.get("pg_store")
    if store is not None:
        await asyncio.to_thread(store.close)


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
    app["arm_id"] = str(getattr(args, "pg_arm", "") or os.environ.get("REBOT_PG_ARM", "b601"))
    app["workflow_url"] = str(getattr(args, "workflow_url", "")
                              or os.environ.get("REBOT_WORKFLOW_URL", "")).rstrip("/")
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
                             or os.environ.get("REBOT_RECORDINGS", str(WORK / "recordings"))).resolve()
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
    app["calibrating"] = False
    app["last_calibration"] = None
    app["progress"] = None
    app["calibration_started"] = None
    app["rig"] = None
    app["witness"] = None
    app["captures"].mkdir(parents=True, exist_ok=True)
    app["calib"].mkdir(parents=True, exist_ok=True)
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
                                else os.environ.get("REBOT_LLM_MAX_TOKENS", "2048"))
    thinking_raw = str(getattr(args, "llm_thinking", "")
                       or os.environ.get("REBOT_LLM_THINKING", "")).strip().lower()
    if thinking_raw in ("0", "false", "no", "off"):
        app["llm_thinking"] = False
    elif thinking_raw in ("1", "true", "yes", "on"):
        app["llm_thinking"] = True
    else:
        app["llm_thinking"] = None
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
    app["traces_dir"] = Path(getattr(args, "traces_dir", "") or WORK / "traces").resolve()
    app["tool_schemas_sha256"] = None
    # The hillclimb (§7): the monitor only proposes; the accuracy map is the
    # measured envelope /sequence warns from and /health reports.
    app["identify_monitor"] = identify_mod.TriggerMonitor()
    app["pg_mirror"] = None
    pg_dsn = str(getattr(args, "pg_dsn", "") or "")
    if pg_dsn:
        # §8 cutover: PG is the system of record. Load arm_config, calib, rig,
        # witness, sag_map, monitor_state and llm_context in ONE round trip
        # (app_bootstrap), independent of --no-trace.
        substrate_dir = str(ROOT / "substrate")
        if substrate_dir not in sys.path:
            sys.path.insert(0, substrate_dir)
        from pg_trace import PgTraceRecorder
        from pg_evidence import PgEvidenceMirror
        from store import SubstrateStore
        pg_store = SubstrateStore(pg_dsn)
        pg_workspace = getattr(args, "pg_workspace", "default")
        app["pg_store"] = pg_store
        app["pg_workspace"] = pg_workspace
        app["pg_mirror"] = PgEvidenceMirror(pg_store, workspace_id=pg_workspace)
        boot = pg_store.app_bootstrap(pg_workspace, app["arm_id"]) or {}
        if boot.get("arm_config"):
            rebot_ik.apply_config(boot["arm_config"])
        try:
            crow = boot.get("calibration")
            if crow and crow.get("calib_doc"):
                cdoc = crow["calib_doc"]
                cname = str(cdoc.get("device") or crow["calib_fingerprint"])
                cpath = Path(app["calib"]) / ("%s.json" % cname)
                if not cpath.is_file():
                    cpath.parent.mkdir(parents=True, exist_ok=True)
                    cpath.write_text(json.dumps(cdoc, indent=2) + "\n", encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            logging.warning("intrinsics cache materialization failed: %s", exc)
        rig_boot = boot.get("rig")
        if rig_boot and rig_boot.get("rig_doc"):
            try:
                doc = rig_boot["rig_doc"]
                pg_rig = extrinsics.Rig.from_dict(doc)
                calib_dir = Path(app["calib"])
                (calib_dir / "extrinsics-fixed.json").write_text(json.dumps(doc), encoding="utf-8")
                record = getattr(pg_rig, "witness", None) or {}
                image = record.get("image")
                if image and rig_boot.get("witness_b64"):
                    wbytes = base64.b64decode(rig_boot["witness_b64"])
                    name = Path(image).name
                    (calib_dir / name).write_bytes(wbytes)
                    if Path(image).is_absolute():
                        Path(image).parent.mkdir(parents=True, exist_ok=True)
                        Path(image).write_bytes(wbytes)
                app["rig"] = pg_rig
                app["extrinsics_path"] = str(calib_dir / "extrinsics-fixed.json")
                try:
                    app["witness"] = witness.load_for(pg_rig, calib_dir)
                except witness.WitnessError as error:
                    logging.warning("the PG rig's witness cannot be used: %s", error)
                    app["witness"] = None
            except Exception as exc:  # noqa: BLE001
                logging.warning("PG rig bootstrap failed: %s", exc)
        elif app.get("rig") is not None:
            _pg_mirror_rig(app, app["rig"])
        try:
            smap = boot.get("sag_map")
            if smap and smap.get("rig_fingerprint"):
                (Path(app["calib"]) / ("sag-%s.json" % smap["rig_fingerprint"])).write_text(
                    json.dumps(smap), encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            logging.warning("sag cache materialization failed: %s", exc)
        try:
            marker = app.get("marker") or {}
            if marker and marker.get("at"):
                at = list(marker["at"])
                tag_id = str(marker["id"]) if marker.get("id") is not None else "base"
                app["pg_mirror"].mirror_tag(
                    tag_id, "base",
                    float(marker.get("size_m", 0.04)) * 1000.0,
                    [float(at[0]), float(at[1]), float(at[2]), float(marker.get("yaw", 0.0))])
            cached_ctx = boot.get("llm_context")
            if cached_ctx and not app["llm_context"]["tokens"]:
                app["llm_context"] = cached_ctx
            mon_state = boot.get("monitor")
            if mon_state:
                mon = app["identify_monitor"]
                mon.ewma = mon_state.get("ewma_pixel_error_mm")
                mon.tag_baseline = mon_state.get("ewma_tag_rms_px")
                mon.proposals = (mon_state.get("last_doc") or {}).get("proposals", [])
        except Exception as exc:  # noqa: BLE001
            logging.warning("tag/context/monitor restore failed: %s", exc)

    if getattr(args, "no_trace", False):
        app["traces"] = None
    elif app.get("pg_store") is not None:
        from pg_trace import PgTraceRecorder
        app["traces"] = PgTraceRecorder(
            app["pg_store"], workspace_id=app["pg_workspace"],
            snapshot=lambda sid: trace_manifest(app, sid))
    else:
        # The snapshot callable runs lazily at a session's first event, when
        # the app state (rig, calib, gates) is fully populated.
        app["traces"] = trace_mod.TraceRecorder(
            app["traces_dir"], retention_days=app["trace_retention_days"],
            snapshot=lambda sid: trace_manifest(app, sid))

    # Compute hover_m and min_z AFTER world.arm_config has been applied to
    # rebot_ik/sweep so custom min_z_m takes effect on startup.
    hover_arg = getattr(args, "hover_m", None)
    app["hover_m"] = float(hover_arg if hover_arg not in (None, 0, 0.0)
                           else os.environ.get("REBOT_HOVER_M", str(sweep.DEFAULT_MIN_Z)))
    vh_arg = getattr(args, "vertical_hover_m", None)
    app["vertical_hover_m"] = float(vh_arg if vh_arg is not None
                                    else os.environ.get("REBOT_VERTICAL_HOVER_M", "0.0"))
    # The lowest tip z the planner will accept: the worktop plane plus the
    # hover height above it -- the same sum the page's fallback and
    # .env.example use -- so a tap can never plan closer to the table than the
    # hover height, whatever plane the worktop sits at.
    app["min_z"] = float(args.plane_z) + app["hover_m"]
    app["vertical_min_z"] = float(args.plane_z) + app["vertical_hover_m"]
    # Scratch dirs for cv2 materializations and file-mode artifacts (§8 flatten):
    # in PG mode the authoritative records are in the substrate, these are cache.
    # Deriving from --captures keeps a test workdir entirely self-contained.
    app["out_dir"] = app["captures"].parent / "out"
    app["oracle_dir"] = app["captures"].parent / "oracle"
    app["telemetry_dir"] = app["captures"].parent / "telemetry"
    app["oracle_tasks"] = set()
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
    for _asset in WEB_JS_ASSETS:
        app.router.add_get("/" + _asset, handle_web_js)
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
    parser.add_argument("--captures", default=str(WORK / "captures"))
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
    parser.add_argument("--calib-dir", default=os.environ.get("REBOT_CALIB_DIR", str(WORK / "calib")))
    parser.add_argument("--calib", default=os.environ.get("REBOT_CALIB", ""),
                        help="calibration JSON; default is the newest in --calib-dir")
    parser.add_argument("--marker-at", default=os.environ.get("REBOT_MARKER_AT", ""),
                        help="'x,y,z' or 'x,y,z,yaw' of the base marker in the robot's frame "
                             "(metres, degrees): a capture that sees it writes the rig; a "
                             "sweep needs no tap")
    parser.add_argument("--marker-size", type=float,
                        default=float(os.environ.get("REBOT_MARKER_SIZE", "0.04")),
                        help="the marker's printed side in metres (default 0.04)")
    parser.add_argument("--hover-m", type=float, default=None,
                        help="height of the tap plane above the worktop, in metres; "
                             "also the lowest tip z the planner accepts")
    parser.add_argument("--vertical-hover-m", type=float, default=None,
                        help="default tap height and floor in top-down/vertical mode, in metres (default 0.0)")
    parser.add_argument("--extrinsics", default="",
                        help="a fitted rig JSON; default is "
                             "<calib-dir>/extrinsics-fixed.json when it exists")
    parser.add_argument("--camera", default=os.environ.get("REBOT_CAMERA", ""),
                        help="rtsp://, http:// snapshot, or device:N, for the automatic sweep")
    parser.add_argument("--mediamtx-api", default=os.environ.get("REBOT_MEDIAMTX_API",
                                                                 "http://127.0.0.1:9997"),
                        help="MediaMTX control API, used to record a sweep")
    parser.add_argument("--recordings", default=os.environ.get("REBOT_RECORDINGS",
                                                               str(WORK / "recordings")),
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
                        default=int(os.environ.get("REBOT_LLM_MAX_TOKENS", "2048")))
    parser.add_argument("--llm-thinking",
                        default=os.environ.get("REBOT_LLM_THINKING", ""),
                        help="override endpoint thinking mode: '0'/'false' to disable, "
                             "'1'/'true' to enable, empty to inherit endpoint default")
    parser.add_argument("--llm-context-tokens", type=int,
                        default=int(os.environ.get("REBOT_LLM_CONTEXT_TOKENS", "0") or 0),
                        help="context window override; 0 reads meta.n_ctx from /v1/models")
    parser.add_argument("--llm-timeout", type=float,
                        default=float(os.environ.get("REBOT_LLM_TIMEOUT", "300")),
                        help="seconds to wait for one model turn")
    parser.add_argument("--max-mb", type=int, default=25, help="largest accepted frame, MB")
    parser.add_argument("--traces-dir", default=os.environ.get("REBOT_TRACES_DIR",
                                                               str(WORK / "traces")),
                        help="where session traces and their blobs are written")
    parser.add_argument("--trace-retention-days", type=int,
                        default=int(os.environ.get("REBOT_TRACE_RETENTION_DAYS", "30")),
                        help="prune session dirs and unreferenced blobs older than this")
    parser.add_argument("--no-trace", action="store_true",
                        help="disable session trace collection entirely (sensitive deployments)")
    parser.add_argument("--pg-dsn", default=os.environ.get("REBOT_PG_DSN", ""),
                        help="substrate Postgres DSN; when set, traces go to PG (the new stack) "
                             "instead of the file log (spec §8 cutover)")
    parser.add_argument("--pg-workspace", default=os.environ.get("REBOT_PG_WORKSPACE", "default"),
                        help="workspace id for trace sessions in the substrate")
    parser.add_argument("--pg-arm", default=os.environ.get("REBOT_PG_ARM", "b601"),
                        help="arm id in the substrate (default: REBOT_PG_ARM or b601)")
    parser.add_argument("--workflow-url", default=os.environ.get("REBOT_WORKFLOW_URL", ""),
                        help="isolated execute/extract/detect activity URL; when set with a "
                             "PG DSN, /execute runs the durable execute->oracle workflow")
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
