#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive the phone page in headless Chrome: tap -> browser locate -> tip marker.

The page is hand-written JavaScript with a tap-to-pixel mapping, a browser
ray-plane solve, and a `getUserMedia` path that no unit test can reach. This runs
the real page in a real Chromium over the DevTools protocol, replaces the camera
with a canvas stream (`getUserMedia` returns a 1280x960 test pattern), stubs the
tag pose, clicks a known pixel, and checks the marker is a tip point in arm
space. It then drags up and down to check the height badge and that the drag is
not a marker.

The capture server needs a calibration in its `--calib-dir` and a marker
placement (`--marker-at`/`REBOT_MARKER_AT`), because the browser locator reads
`/reach.json` and this check stubs the tag solve.

    # terminal 1
    python examples/host/app/capture_server.py --token test-token
    # terminal 2
    python examples/host/tests/browser_check.py --url "https://127.0.0.1:8443/?token=test-token"

With `--serve` no external server is needed: the phone check's server is hosted
in-process with a synthetic calibration, and everything runs offline.

After the phone checks, the /live agent sidebar is driven in the same browser
against a second in-process server (fitted rig, MockEdge gateway, a scripted
local model -- no real endpoint, no hardware): a chat task must leave a complete
trace (user_message, tool_call/tool_result, frame_delivered, model_turn, plan,
gateway tape), a forced context compaction must be logged without losing prior
turns, and the feedback buttons must record a verdict.

If the shell cannot launch a browser itself (confined sandboxes block Chromium's
IPC), start one separately and attach:

    chrome --headless=new --remote-debugging-port=9333 --user-data-dir=/tmp/x about:blank
    python examples/host/tests/browser_check.py --connect 9333 --url "..."

Browser autodetect only probes the Windows Chrome/Edge install paths; anywhere
else pass `--chrome <path>`. When no browser is found the check skips with a
message instead of failing. It is a check of the page, not of the camera: the
lens, the focus and the intrinsics are still the phone's problem.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import shutil
import socket
import subprocess
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path
from types import SimpleNamespace

import aiohttp

HERE = Path(__file__).resolve().parent
APP = HERE.parent / "app"
WINDOWS_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]

STUB = """
(() => {
  const canvas = document.createElement('canvas');
  canvas.width = 1280; canvas.height = 960;
  const context = canvas.getContext('2d');
  context.fillStyle = '#efefef'; context.fillRect(0, 0, 1280, 960);
  context.fillStyle = '#111'; context.fillRect(120, 120, 260, 180);
  context.fillStyle = '#c33'; context.fillRect(900, 700, 220, 160);
  context.strokeStyle = '#2a7'; context.lineWidth = 12;
  context.beginPath(); context.arc(640, 480, 90, 0, Math.PI * 2); context.stroke();
  context.fillStyle = '#111'; context.font = '48px sans-serif';
  context.fillText('synthetic camera', 420, 120);
  const stream = canvas.captureStream(15);
  Object.defineProperty(navigator, 'mediaDevices', {
    configurable: true,
    value: {
      getUserMedia: async () => stream,
      enumerateDevices: async () => [],
      addEventListener: () => {},
    },
  });
  return [canvas.width, canvas.height];
})()
"""


class CdpError(Exception):
    pass


class Cdp:
    """Minimal DevTools-protocol client: one websocket, request/response by id."""

    def __init__(self, ws):
        self.ws = ws
        self.next_id = 0
        self.pending = {}

    async def call(self, method, params=None, session=None):
        self.next_id += 1
        message = {"id": self.next_id, "method": method, "params": params or {}}
        if session:
            message["sessionId"] = session
        await self.ws.send_str(json.dumps(message))
        while True:
            frame = await self.ws.receive(timeout=30)
            if frame.type != aiohttp.WSMsgType.TEXT:
                raise CdpError("websocket closed while waiting for %s" % method)
            reply = json.loads(frame.data)
            if reply.get("id") != self.next_id:
                continue  # an event, or a reply to something we do not wait for
            if "error" in reply:
                raise CdpError("%s: %s" % (method, reply["error"]))
            return reply.get("result", {})

    async def evaluate(self, expression, session=None):
        result = await self.call("Runtime.evaluate", {
            "expression": expression,
            "returnByValue": True,
            "awaitPromise": True,
        }, session)
        if result.get("exceptionDetails"):
            details = result["exceptionDetails"]
            raise CdpError("JS: %s" % (details.get("text") or details))
        return result.get("result", {}).get("value")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def server_args(workdir: Path, **overrides) -> SimpleNamespace:
    """The capture server's args, as the selftest builds them: all state local."""
    args = SimpleNamespace(
        host="127.0.0.1", port=0, captures=str(workdir / "captures"), cert="", key="",
        token="check-token", mediamtx="http://127.0.0.1:9", whip_path="phone",
        no_whip=True, max_mb=16, quiet=True,
        planner="http://127.0.0.1:9", planner_token="", execute=False,
        calib_dir=str(workdir / "calib"), calib="",
        marker_at="0.36,-0.10,0.12,180", marker_size=0.08,
        hover="1", hover_m=0.10, tip_mm=35.0,
        plane_z=0.0, tool_axis="z-", yaw=0.0,
        extrinsics="", witness_max_px=2.0, camera="",
        traces_dir=str(workdir / "traces"), trace_retention_days=30, no_trace=False,
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def write_synthetic_calib(calib_dir: Path) -> None:
    """The same 1280x960 grid the canvas camera stub publishes."""
    calib_dir.mkdir(parents=True, exist_ok=True)
    (calib_dir / "synthetic.json").write_text(json.dumps({
        "format": "rebot-camera-calib", "version": 1, "device": "synthetic",
        "image_size": [1280, 960],
        "K": [[1000.0, 0.0, 640.0], [0.0, 1000.0, 480.0], [0.0, 0.0, 1.0]],
        "dist": [0.0] * 5, "rms": 0.2,
    }), encoding="utf-8")


@asynccontextmanager
async def hosted_server(app):
    """The real capture server on a loopback port, in this process."""
    from aiohttp import web

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    port = site._server.sockets[0].getsockname()[1]
    try:
        yield "http://127.0.0.1:%d" % port
    finally:
        await runner.cleanup()


@asynccontextmanager
async def hosted_phone_server(workdir: Path):
    """A handheld host for the phone-page checks: synthetic calib, base marker."""
    sys.path.insert(0, str(APP))
    import capture_server

    write_synthetic_calib(workdir / "calib")
    app = capture_server.build_app(server_args(workdir))
    async with hosted_server(app) as url:
        yield url + "/?token=check-token"


def centimetres(label: str) -> float:
    """Read the height badge ("10.5 cm") back as a number."""
    return float(str(label).replace("cm", "").strip())


def find_browser(explicit: str):
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise SystemExit("no browser at %s" % explicit)
        return path
    for candidate in WINDOWS_CANDIDATES:
        if Path(candidate).is_file():
            return Path(candidate)
    return None


async def wait_for_devtools(port: int, timeout=20.0):
    deadline = time.time() + timeout
    async with aiohttp.ClientSession() as session:
        while time.time() < deadline:
            try:
                async with session.get("http://127.0.0.1:%d/json/version" % port) as response:
                    if response.status == 200 and "webSocketDebuggerUrl" in await response.json():
                        return
            except aiohttp.ClientError:
                pass
            await asyncio.sleep(0.2)
    raise SystemExit("Chrome did not open a DevTools port on %d" % port)


async def run(args) -> int:
    if args.connect:
        code = await drive(args, args.connect)
        if code == 0:
            code = await live_check(args, args.connect)
        return code

    browser = find_browser(args.chrome)
    if browser is None:
        print("skip: no Chrome or Edge found on this machine; the browser checks "
              "need one. Pass --chrome <path>, or start a browser with "
              "--remote-debugging-port and use --connect.")
        return 0
    port = free_port()
    profile = Path(args.profile)
    profile.mkdir(parents=True, exist_ok=True)
    stderr = open(profile / "browser.log", "wb")

    process = subprocess.Popen([
        str(browser),
        "--headless=new",
        "--remote-debugging-port=%d" % port,
        "--remote-allow-origins=*",
        "--user-data-dir=%s" % profile,
        "--no-first-run", "--no-default-browser-check",
        "--disable-gpu", "--hide-scrollbars", "--mute-audio",
        "--window-size=390,844",
        "about:blank",
    ], stdout=subprocess.DEVNULL, stderr=stderr)

    try:
        await wait_for_devtools(port)
        if args.serve:
            workdir = HERE / ".browser-check-work"
            shutil.rmtree(workdir, ignore_errors=True)
            workdir.mkdir(parents=True, exist_ok=True)
            async with hosted_phone_server(workdir / "phone") as url:
                args.url = url
                code = await drive(args, port)
        else:
            code = await drive(args, port)
        if code == 0:
            code = await live_check(args, port)
        return code
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        stderr.close()


async def drive(args, port: int) -> int:
    await wait_for_devtools(port)
    async with aiohttp.ClientSession() as session:
        async with session.get("http://127.0.0.1:%d/json/version" % port) as response:
            endpoint = (await response.json())["webSocketDebuggerUrl"]
        async with session.ws_connect(endpoint, max_msg_size=0) as ws:
            return await drive_page(cdp=Cdp(ws), args=args)


async def drive_page(cdp, args) -> int:
    target = await cdp.call("Target.createTarget", {"url": "about:blank"})
    attached = await cdp.call("Target.attachToTarget",
                              {"targetId": target["targetId"], "flatten": True})
    page = attached["sessionId"]
    await cdp.call("Page.enable", session=page)
    await cdp.call("Runtime.enable", session=page)
    await cdp.call("Page.navigate", {"url": args.url}, page)

    for _ in range(100):
        if await cdp.evaluate("document.readyState", page) == "complete":
            break
        await asyncio.sleep(0.1)
    else:
        raise SystemExit("the page never finished loading")

    size = await cdp.evaluate(STUB, page)
    print("  camera stub: %sx%s canvas stream" % tuple(size))

    await cdp.evaluate("document.getElementById('start').click()", page)
    for _ in range(50):
        live = await cdp.evaluate(
            "typeof state !== 'undefined' && !!state.track"
            " && document.getElementById('video').videoWidth", page)
        if live:
            break
        await asyncio.sleep(0.1)
    captured = await cdp.evaluate(
        "({w: document.getElementById('video').videoWidth,"
        " h: document.getElementById('video').videoHeight})", page)
    ok = captured["w"] == 1280 and captured["h"] == 960
    print("  [%s] camera started at %sx%s" % ("ok  " if ok else "FAIL",
                                              captured["w"], captured["h"]))
    if not ok:
        logs = await cdp.evaluate(
            "Array.from(document.querySelectorAll('#log p')).map(p => p.textContent)", page)
        print("  page log: %s" % logs)
        raise SystemExit("camera did not start")

    # The page takes its minimum height from /health; wait for it before the
    # floor check so the range is the host's, not the placeholder.
    for _ in range(50):
        if await cdp.evaluate("typeof state !== 'undefined' && !!state.health", page):
            break
        await asyncio.sleep(0.1)
    else:
        raise SystemExit("the page never received /health")

    # The synthetic camera has no printed tag. Give the browser locator a known
    # slanted camera pose (camera at [0,-1,1], looking at the origin) so the
    # raised-plane intersection has a visible x/y shift.
    pose = await cdp.evaluate("""(() => {
        const c = Math.cos(Math.PI / 4);
        const T = [[1, 0, 0, 0], [0, -c, c, -1], [0, -c, -c, 1], [0, 0, 0, 1]];
        window.__tapPose = T;
        ArucoPose.detectFrame = () => ({corners: [[0, 0], [1, 0], [1, 1], [0, 1]]});
        ArucoPose.poseFromCorners = () => ({T_base_cam: T});
        return T;
    })()""", page)
    if not pose or len(pose) != 4:
        raise SystemExit("could not install the browser pose stub")

    # The height badge belongs in the camera chip row, immediately after zoom.
    # The synthetic camera exposes no zoom, so inject one for this check.
    chips = await cdp.evaluate("""(() => {
        const track = state.track;
        const original = track.getSettings;
        try {
          track.getSettings = () => ({width: 1280, height: 960, frameRate: 15, zoom: 1.5});
        } catch (error) {
          return {error: String(error)};
        }
        try {
          renderChips();
          const box = document.getElementById('chips');
          const badge = document.getElementById('heightBadge');
          const zoom = Array.from(box.querySelectorAll('.chip'))
            .find((chip) => chip.textContent.trim().startsWith('zoom'));
          return {inRow: box.contains(badge),
                  afterZoom: !!zoom && zoom.nextElementSibling === badge,
                  zoom: zoom ? zoom.textContent : null};
        } finally {
          track.getSettings = original;
          renderChips();
        }
    })()""", page)
    chips_ok = bool(chips.get("inRow") and chips.get("afterZoom"))
    print("  [%s] height badge sits next to zoom in the chip row"
          % ("ok  " if chips_ok else "FAIL"))
    if not chips_ok:
        print("    inRow=%s afterZoom=%s zoom=%s"
              % (chips.get("inRow"), chips.get("afterZoom"), chips.get("zoom")))
        raise SystemExit("the height badge is not in the zoom row")

    # Tap a known image pixel through the real pointer handler. The browser
    # resolves the tip itself; there is no capture/locate round trip.
    box = await cdp.evaluate("""(() => {
        const video = document.getElementById('video');
        const rect = video.getBoundingClientRect();
        return {x: rect.x, y: rect.y, w: rect.width, h: rect.height,
                vw: video.videoWidth, vh: video.videoHeight};
    })()""", page)
    scale = min(box["w"] / box["vw"], box["h"] / box["vh"])
    offset_x = box["x"] + (box["w"] - box["vw"] * scale) / 2
    offset_y = box["y"] + (box["h"] - box["vh"] * scale) / 2
    u, v = 401.5, 303.5
    x, y = offset_x + u * scale, offset_y + v * scale
    for kind, pressed in (("mousePressed", 1), ("mouseReleased", 0)):
        await cdp.call("Input.dispatchMouseEvent", {
            "type": kind, "x": x, "y": y, "button": "left",
            "buttons": pressed, "clickCount": 1,
        }, page)

    marker = None
    for _ in range(100):
        marker = await cdp.evaluate(
            "markersOverlay.count ? markersOverlay.targets[0] : null", page)
        if marker:
            break
        await asyncio.sleep(0.1)
    hover = float(await cdp.evaluate("state.health.hover_m", page))
    same_tap = (marker is not None
                and abs(float(marker.get("pixel", [-1, -1])[0]) - u) < 1.0
                and abs(float(marker.get("pixel", [-1, -1])[1]) - v) < 1.0)
    # Direct intersection with the raised plane for the slanted test pose:
    # camera at [0,-1,1], 45 deg down, K fx=fy=1000, cx=640, cy=480.
    c = math.cos(math.pi / 4)
    xn = (u - 640.0) / 1000.0
    yn = (v - 480.0) / 1000.0
    distance = (1.0 - hover) / (c * (1.0 + yn))
    expected = [distance * xn, -1.0 + distance * c * (1.0 - yn), hover]
    point = marker.get("tip", [0, 0, 0]) if marker else [0, 0, 0]
    tip_ok = (marker is not None and "tip" in marker
              and abs(float(point[0]) - expected[0]) < 1e-6
              and abs(float(point[1]) - expected[1]) < 1e-6
              and abs(float(point[2]) - expected[2]) < 1e-6)
    print("  [%s] browser locate marker at tip [%.3f, %.3f, %.3f] m"
          % ("ok  " if same_tap and tip_ok else "FAIL",
             float(point[0]), float(point[1]), float(point[2])))
    if not marker or not same_tap or not tip_ok:
        logs = await cdp.evaluate(
            "Array.from(document.querySelectorAll('#log p')).map(p => p.textContent)", page)
        print("  marker: %s" % marker)
        print("  expected: %s" % expected)
        print("  page log: %s" % logs)
        raise SystemExit("the browser did not locate the tap as a tip point")

    if args.screenshot:
        shot = await cdp.call("Page.captureScreenshot", {"format": "png"}, page)
        Path(args.screenshot).write_bytes(base64.b64decode(shot["data"]))
        print("  screenshot: %s" % args.screenshot)

    # Drag away from the tap point: the badge follows the finger and the gesture
    # is not a marker. Two drags check both directions (100 px up = +15 cm, then
    # 60 px down = -9 cm).
    before_count = int(await cdp.evaluate("markersOverlay.count", page))
    gesture = await cdp.evaluate("""(async () => {
        const stage = document.getElementById('stage');
        const badge = document.getElementById('heightBadge');
        const rect = stage.getBoundingClientRect();
        const x = rect.left + rect.width / 2;
        const y = rect.top + rect.height / 2;
        async function drag(pointerId, deltaY) {
          const send = (type, clientY, buttons) => stage.dispatchEvent(new PointerEvent(type, {
            bubbles: true, cancelable: true, pointerId: pointerId, pointerType: 'touch',
            isPrimary: true, button: 0, buttons: buttons, clientX: x, clientY: clientY,
          }));
          send('pointerdown', y, 1);
          await new Promise((resolve) => setTimeout(resolve, 20));
          send('pointermove', y + deltaY, 1);
          await new Promise((resolve) => setTimeout(resolve, 40));
          const during = {label: badge.textContent,
                          adjusting: stage.classList.contains('adjusting')};
          send('pointerup', y + deltaY, 0);
          await new Promise((resolve) => setTimeout(resolve, 40));
          during.after = badge.textContent;
          during.still = stage.classList.contains('adjusting');
          return during;
        }
        const before = badge.textContent;
        const up = await drag(77, -100);
        const down = await drag(78, 60);
        let reachHeight = null;
        try { reachHeight = reachOverlay.height; } catch (error) { /* not a global */ }
        return {before: before, up: up, down: down, reachHeight: reachHeight};
    })()""", page)
    await asyncio.sleep(0.5)
    after_count = int(await cdp.evaluate("markersOverlay.count", page))
    # The badge starts at the host's configured hover height, so derive the
    # expectations from it (100 px up = +15 cm, then 60 px down = -9 cm).
    before_cm = centimetres(gesture["before"])
    expected_up = min(60.0, before_cm + 15.0)
    expected_down = max(0.0, expected_up - 9.0)
    height_ok = (abs(centimetres(gesture["up"]["label"]) - expected_up) < 0.01
                 and abs(centimetres(gesture["up"]["after"]) - expected_up) < 0.01
                 and gesture["up"]["adjusting"] and not gesture["up"]["still"]
                 and abs(centimetres(gesture["down"]["label"]) - expected_down) < 0.01
                 and abs(centimetres(gesture["down"]["after"]) - expected_down) < 0.01
                 and gesture["down"]["adjusting"] and not gesture["down"]["still"]
                 and abs(float(gesture["reachHeight"] or 0.0) - expected_down / 100.0) < 1e-6)
    print("  [%s] drag height %s -> %s -> %s"
          % ("ok  " if height_ok else "FAIL", gesture["before"], gesture["up"]["after"],
             gesture["down"]["after"]))
    if abs(centimetres(gesture["up"]["label"]) - expected_up) >= 0.01:
        print("    while adjusting: %s (expected %.1f cm), adjusting=%s"
              % (gesture["up"]["label"], expected_up, gesture["up"]["adjusting"]))
    if abs(centimetres(gesture["down"]["label"]) - expected_down) >= 0.01:
        print("    while adjusting down: %s (expected %.1f cm), adjusting=%s"
              % (gesture["down"]["label"], expected_down, gesture["down"]["adjusting"]))
    drag_ok = after_count == before_count
    print("  [%s] drag added no marker" % ("ok  " if drag_ok else "FAIL"))
    if not drag_ok:
        print("    marker count %s -> %s" % (before_count, after_count))
    if not height_ok or not drag_ok:
        raise SystemExit("the drag height gesture did not behave")

    # Each marker must carry the tip height it was tapped at, and the browser
    # payload must not carry the host-owned tip offset or tap-height fields.
    planned = await cdp.evaluate("""(async () => {
        const realFetch = window.fetch.bind(window);
        const seen = [];
        window.fetch = async (url, options) => {
          const path = String(url);
          if (path === '/sequence') {
            seen.push(JSON.parse(options.body));
            return {ok: true, status: 200, text: async () => '',
                    json: async () => ({ok: true, execute: false,
                      plan: {id: 'fake-plan', summary: {markers: 2}}})};
          }
          return realFetch(url, options);
        };
        try {
          markersOverlay.clear();
          const video = document.getElementById('video');
          const rect = video.getBoundingClientRect();
          const x = rect.left + rect.width / 2;
          const y = rect.top + rect.height / 2;
          await markersOverlay.addAt(x, y);
          heightControl.setHeight(0.25);
          await markersOverlay.addAt(x, y);
          await startSequence();
          const targets = seen.length ? (seen[0].targets || []) : [];
          return {
            height: heightControl.height,
            count: markersOverlay.count,
            points: targets.map((target) => target.tip[2]),
            clean: targets.every((target) => target.tip.length === 3
              && target.tip_offset_m === undefined
              && target.tap_height_m === undefined
              && target.surface_point === undefined
              && target.point_frame === undefined),
          };
        } finally {
          window.fetch = realFetch;
        }
    })()""", page)
    first_expected = centimetres(gesture["down"]["after"]) / 100.0
    second_expected = 0.25
    points = planned["points"] or []
    height_sent = (planned["count"] == 2 and len(points) == 2 and planned["clean"]
                   and abs(float(points[0]) - first_expected) < 1e-9
                   and abs(float(points[1]) - second_expected) < 1e-9)
    print("  [%s] each marker carries its tip height (%s m, host fields absent)"
          % ("ok  " if height_sent else "FAIL", points))
    if not height_sent:
        print("    height=%s count=%s points=%s clean=%s (expected %s, %s)"
              % (planned["height"], planned["count"], points, planned["clean"],
                 first_expected, second_expected))
        raise SystemExit("the markers did not carry their tip heights")

    # The badge must stop at the tip floor: min_z minus the worktop. On the
    # default bench (min_z 10 cm, plane 0) that is exactly 10 cm; tip_m is only
    # the flange offset and does not enter.
    floor = await cdp.evaluate("""(async () => {
        const stage = document.getElementById('stage');
        const badge = document.getElementById('heightBadge');
        const rect = stage.getBoundingClientRect();
        const x = rect.left + rect.width / 2;
        const y = rect.top + rect.height / 2;
        const send = (type, clientY, buttons) => stage.dispatchEvent(new PointerEvent(type, {
          bubbles: true, cancelable: true, pointerId: 79, pointerType: 'touch',
          isPrimary: true, button: 0, buttons: buttons, clientX: x, clientY: clientY,
        }));
        send('pointerdown', y, 1);
        await new Promise((resolve) => setTimeout(resolve, 20));
        send('pointermove', y + 300, 1);
        await new Promise((resolve) => setTimeout(resolve, 40));
        const label = badge.textContent;
        send('pointerup', y + 300, 0);
        await new Promise((resolve) => setTimeout(resolve, 40));
        const health = state.health || {};
        return {label: label,
                expected: Math.max(0, (health.min_z ?? 0.10) - (health.plane_z ?? 0))};
    })()""", page)
    floor_ok = abs(centimetres(floor["label"]) / 100.0 - float(floor["expected"])) < 0.006
    print("  [%s] height badge floor is %s (min_z - plane_z)"
          % ("ok  " if floor_ok else "FAIL", floor["label"]))
    if not floor_ok:
        print("    expected %.3f m, got %s" % (floor["expected"], floor["label"]))
        raise SystemExit("the height badge ignored the tip floor")

    print("\npage, browser locate, marker tip heights and height drag all work")
    return 0


# --------------------------------------------------------------------------- #
# The /live sidebar: a chat task must leave a complete, replayable trace
# --------------------------------------------------------------------------- #

LIVE_VIDEO_STUB = """
(() => {
  const canvas = document.createElement('canvas');
  canvas.width = 1280; canvas.height = 960;
  const context = canvas.getContext('2d');
  context.fillStyle = '#dedede'; context.fillRect(0, 0, 1280, 960);
  context.fillStyle = '#333'; context.fillRect(500, 420, 280, 220);
  context.fillStyle = '#c33'; context.fillRect(900, 700, 200, 140);
  context.strokeStyle = '#2a7'; context.lineWidth = 10;
  context.beginPath(); context.arc(640, 600, 70, 0, Math.PI * 2); context.stroke();
  const video = document.getElementById('video');
  video.srcObject = canvas.captureStream(15);
  return true;
})()
"""

# What the scripted model does, turn by turn: mark a pixel, plan without moving,
# ask for a fresh frame, answer; then answer again after the forced compaction.
LIVE_SCRIPT = [
    {"content": "Marking the dark box.",
     "tool_calls": [("add_marker", '{"u":640,"v":600,"height_m":0.12}')]},
    {"tool_calls": [("move_arm", '{"execute":false}')]},
    {"tool_calls": [("capture_view", "{}")]},
    {"content": "Planned one marker; nothing moved."},
    {"content": "My context was trimmed, but the trace keeps the earlier turns."},
]


def scripted_llm():
    """A local model endpoint that answers from LIVE_SCRIPT, SSE or JSON."""
    from aiohttp import web

    seen = []

    async def chat(request):
        payload = await request.json()
        seen.append(payload)
        step = LIVE_SCRIPT[len(seen) - 1] if len(seen) <= len(LIVE_SCRIPT) else {"content": "ok"}
        calls = step.get("tool_calls") or []
        if payload.get("stream"):
            response = web.StreamResponse(status=200,
                                          headers={"Content-Type": "text/event-stream"})
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
        return web.json_response({"model": "stub-model",
                                  "choices": [{"message": message,
                                               "finish_reason": "tool_calls" if calls else "stop"}],
                                  "usage": {"total_tokens": 12}})

    async def models(request):
        return web.json_response({"data": [{"id": "stub-model", "meta": {"n_ctx": 8192}}]})

    app = web.Application()
    app.router.add_post("/v1/chat/completions", chat)
    app.router.add_get("/v1/models", models)
    return app, seen


@asynccontextmanager
async def hosted_live_server(workdir: Path):
    """A fixed-rig host for the sidebar: MockEdge gateway, scripted model, traces."""
    sys.path.insert(0, str(APP))
    import capture_server
    import extrinsics
    import mock_arm
    import numpy as np
    from aiohttp import web

    write_synthetic_calib(workdir / "calib")
    # The same slanted pose the phone check stubs: camera at [0,-1,1], 45 deg
    # down, so the browser's ray-plane solve has a visible x/y shift.
    T = extrinsics.look_at(np.array([0.0, -1.0, 1.0]), np.zeros(3))
    (workdir / "calib" / "extrinsics-fixed.json").write_text(json.dumps({
        "format": "rebot-extrinsics", "version": 1,
        "T_base_cam": T.tolist(), "image_size": [1280, 960],
        "plane_z": 0.0, "tip_offset": [0.0, 0.0, 0.035],
        "mount": "fixed", "method": "tip-sweep", "rms_px": 0.5, "rms_mm": 2.0,
        "observations": [],
    }), encoding="utf-8")

    edge, edge_url, edge_state = mock_arm.serve(0)
    fake_llm, seen = scripted_llm()
    runner = web.AppRunner(fake_llm)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    llm_port = site._server.sockets[0].getsockname()[1]

    app = capture_server.build_app(server_args(workdir, planner=edge_url, marker_at=""))
    app["llm_url"] = "http://127.0.0.1:%d/v1/chat/completions" % llm_port
    app["llm_model"] = "stub-model"
    app["llm_token"] = "stub-key"
    try:
        async with hosted_server(app) as url:
            yield url, workdir / "traces", edge_state, seen
    finally:
        await runner.cleanup()
        edge.shutdown()


def read_trace_events(traces: Path):
    """Every event of every session under the traces dir (there is one)."""
    events = []
    if not traces.is_dir():
        return events
    for directory in sorted(traces.iterdir()):
        path = directory / "events.jsonl"
        if not path.is_file():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


async def poll_until(predicate, timeout=30.0, interval=0.2):
    deadline = time.time() + timeout
    while time.time() < deadline:
        value = predicate()
        if value:
            return value
        await asyncio.sleep(interval)
    return None


async def live_check(args, port: int) -> int:
    await wait_for_devtools(port)
    async with aiohttp.ClientSession() as session:
        async with session.get("http://127.0.0.1:%d/json/version" % port) as response:
            endpoint = (await response.json())["webSocketDebuggerUrl"]
        async with session.ws_connect(endpoint, max_msg_size=0) as ws:
            return await drive_live(cdp=Cdp(ws), args=args)


async def drive_live(cdp, args) -> int:
    print("\nlive sidebar: the chat task leaves a complete trace")
    workdir = HERE / ".browser-check-work" / "live"
    shutil.rmtree(workdir, ignore_errors=True)
    workdir.mkdir(parents=True, exist_ok=True)
    failures = []

    def report(name, ok, detail=""):
        print("  [%s] %s%s" % ("ok  " if ok else "FAIL", name,
                               ("  -- " + detail) if detail else ""))
        if not ok:
            failures.append(name)
        return bool(ok)

    async with hosted_live_server(workdir) as (url, traces, edge_state, seen):
        target = await cdp.call("Target.createTarget", {"url": "about:blank"})
        attached = await cdp.call("Target.attachToTarget",
                                  {"targetId": target["targetId"], "flatten": True})
        page = attached["sessionId"]
        await cdp.call("Page.enable", session=page)
        await cdp.call("Runtime.enable", session=page)
        await cdp.call("Page.navigate", {"url": url + "/live?token=check-token"}, page)
        for _ in range(100):
            if await cdp.evaluate("document.readyState", page) == "complete":
                break
            await asyncio.sleep(0.1)
        else:
            raise SystemExit("the live page never finished loading")

        # No MediaMTX here: the canvas stream goes straight into the video
        # element, and pinStream() records the geometry a connect would pin.
        await cdp.evaluate(LIVE_VIDEO_STUB, page)
        sized = await cdp.evaluate("""(async () => {
            for (let i = 0; i < 100; i += 1) {
              const video = document.getElementById('video');
              if (video.videoWidth === 1280 && video.videoHeight === 960) {
                pinStream();
                streamWarned = '';
                document.getElementById('hint').hidden = true;
                return {w: video.videoWidth, h: video.videoHeight};
              }
              await new Promise((resolve) => setTimeout(resolve, 100));
            }
            return null;
        })()""", page)
        if not report("the sidebar preview has the stub camera", bool(sized), str(sized)):
            return 1
        agent_ready = await cdp.evaluate("""(async () => {
            for (let i = 0; i < 100; i += 1) {
              if (typeof CHAT !== 'undefined' && CHAT.ready && state.health) return true;
              await new Promise((resolve) => setTimeout(resolve, 100));
            }
            return false;
        })()""", page)
        if not report("the host offers the agent and a fixed rig", bool(agent_ready)):
            return 1

        # A turned AND resized stream: the tap must be rotated onto the upright
        # video grid and then scaled onto the calibration grid. Rotating alone
        # leaves video pixels against a native-K camera -- a silent wrong tip.
        turned = await cdp.evaluate("""(() => {
            const calib = {image_size: [1080, 1920],
                           K: [[900, 0, 540], [0, 900, 960], [0, 0, 1]],
                           dist: [0, 0, 0, 0, 0]};
            const c = Math.cos(0.1), s = Math.sin(0.1);
            // camera at z=0.80 looking DOWN (optical axis = third column)
            const T = [[1, 0, 0, 0.30], [0, -c, s, 0.05], [0, -s, -c, 0.80]];
            const turn = 3;                       // landscape video, portrait calib
            const uL = 900, vL = 400;             // a tap in the 1280x720 video
            const upright = ReachGeometry.mapPixel(uL, vL, [1280, 720], turn);
            const a = BrowserLocate.tipTarget(calib, T, uL, vL, 0, 0.05, 1280, 720, turn);
            const b = BrowserLocate.tipTarget(calib, T, upright[0], upright[1], 0, 0.05,
                                              720, 1280, 0);
            const d = Math.hypot(a.tip[0] - b.tip[0], a.tip[1] - b.tip[1],
                                 a.tip[2] - b.tip[2]);
            return {delta_m: d, a: a.tip, b: b.tip};
        })()""", page)
        report("a turned+resized tap lands where the upright tap does",
               bool(turned) and turned["delta_m"] < 1e-6,
               "delta %.3g m" % (turned or {}).get("delta_m", -1))
        unknown_turn = await cdp.evaluate("""(() => {
            const saved = window.RebotConfig;
            window.RebotConfig = null;
            const value = streamTurn();
            window.RebotConfig = saved;
            return value === undefined ? 'undefined' : String(value);
        })()""", page)
        report("an unknown stream turn is undefined, never 0",
               unknown_turn == "undefined", str(unknown_turn))

        # A-M9: rotateFrame turns the frame clockwise onto the calibration grid,
        # matching mapPixel. A 3x2 frame with marked corners, turned once, must
        # land each corner where a 90 deg CW rotation puts it.
        rotated = await cdp.evaluate("""(() => {
            const w = 3, h = 2;
            const data = new Uint8ClampedArray(w * h * 4);
            const set = (x, y, v) => { const i = (y * w + x) * 4;
              data[i] = v; data[i + 1] = v; data[i + 2] = v; data[i + 3] = 255; };
            set(0, 0, 10); set(2, 0, 20); set(0, 1, 30); set(2, 1, 40);
            const r = BrowserLocate.rotateFrame(data, w, h, 1);
            const at = (x, y) => r.data[(y * r.width + x) * 4];
            // CW: (x,y) -> (h-1-y, x); new grid is 2 wide x 3 tall.
            return {w: r.width, h: r.height,
                    tl: at(0, 0), tr: at(1, 0), bl: at(0, 2), br: at(1, 2)};
        })()""", page)
        report("rotateFrame turns the frame clockwise onto the calibration grid",
               bool(rotated) and rotated["w"] == 2 and rotated["h"] == 3
               and rotated["tl"] == 30 and rotated["tr"] == 10
               and rotated["bl"] == 40 and rotated["br"] == 20,
               str(rotated))

        # A-M10: arm_to_pixel works without the Reach overlay toggled on (it
        # loads the rig on demand instead of reading the overlay's private cache).
        arm_pixel = await cdp.evaluate("""(async () => {
            try {
              const r = await toolArmToPixel({x: 0.30, y: 0.0, z: 0.10});
              return {ok: r.ok, u: r.u, v: r.v};
            } catch (e) { return {error: String(e && e.message || e)}; }
        })()""", page)
        report("arm_to_pixel projects without the overlay toggled on",
               bool(arm_pixel) and arm_pixel.get("ok") is True
               and isinstance(arm_pixel.get("u"), (int, float)),
               str(arm_pixel))
        session_id = await cdp.evaluate("CHAT.sessionId", page)
        report("the page stamps a capture-format session id",
               bool(session_id) and len(str(session_id)) == 23
               and str(session_id)[8] == "T" and str(session_id)[15] == "Z",
               str(session_id))

        # Task 1: mark, plan, fresh frame, final reply -- all through the real
        # chat input, the real tool loop and the real /sequence route.
        await cdp.evaluate("""(() => {
            const box = document.getElementById('chatText');
            box.value = 'Place a marker on the dark box and plan it.';
            box.dispatchEvent(new Event('input'));
            document.getElementById('chatSend').click();
            return true;
        })()""", page)
        done = await poll_until(
            lambda: any(e["type"] == "task_end" for e in read_trace_events(traces)))
        if not report("the first task runs to task_end", bool(done)):
            print("    chat log: %s" % await cdp.evaluate(
                "Array.from(document.querySelectorAll('#chatLog .msg'))"
                ".map((m) => m.textContent).join(' | ')", page))
            return 1
        events = read_trace_events(traces)
        types = {e["type"] for e in events}
        for wanted in ("session_start", "user_message", "tool_call", "tool_result",
                       "frame_delivered", "marker_solved", "sequence_result",
                       "model_turn", "plan", "gateway_call", "task_end"):
            report("the log carries %s" % wanted, wanted in types)
        report("the tool wrapper saw all three tools",
               [e["name"] for e in events if e["type"] == "tool_call"]
               == ["add_marker", "move_arm", "capture_view"])
        report("the plan links the gateway's plan_id to the session",
               any(e["type"] == "plan" and e.get("plan_id") == edge_state.last_plan
                   for e in events), str(edge_state.last_plan))
        report("the model turn stores the prompt with blob refs",
               any(e["type"] == "model_turn"
                   and '{"$blob"' in json.dumps(e.get("messages")) for e in events))
        report("the host's model key never enters the log",
               "stub-key" not in json.dumps(events))

        # The explicit verdict: Bad plus a note on the task's final reply.
        clicked = await cdp.evaluate("""(() => {
            const rows = document.querySelectorAll('.msg.agent .feedback');
            const row = rows[rows.length - 1];
            if (!row) return {error: 'no feedback row on the final reply'};
            row.querySelectorAll('button')[1].click();
            const input = row.querySelector('input');
            if (!input) return {error: 'no note input'};
            input.value = 'the marker was off the object';
            input.dispatchEvent(new Event('change'));
            return {ok: true};
        })()""", page)
        feedback = await poll_until(
            lambda: next((e for e in read_trace_events(traces)
                          if e["type"] == "feedback"), None))
        report("the feedback buttons record a verdict with the note",
               bool(feedback) and feedback.get("verdict") == "fail"
               and feedback.get("note") == "the marker was off the object",
               str(clicked if not feedback else feedback.get("verdict")))

        # Force a compaction on the next turn: a tiny context budget makes
        # prepareContext trim the history. The log must keep every prior turn.
        before = await cdp.evaluate("CHAT.history.length", page)
        await cdp.evaluate("""(() => {
            CHAT.contextTokens = 1500;
            CHAT.maxTokens = 512;
            const box = document.getElementById('chatText');
            box.value = 'What did you do so far?';
            box.dispatchEvent(new Event('input'));
            document.getElementById('chatSend').click();
            return true;
        })()""", page)
        done2 = await poll_until(
            lambda: sum(1 for e in read_trace_events(traces) if e["type"] == "task_end") >= 2)
        report("the second task completes after compaction", bool(done2))
        events = read_trace_events(traces)
        compactions = [e for e in events if e["type"] == "compaction"]
        report("compaction is logged with what it dropped",
               bool(compactions) and compactions[0].get("dropped_messages", 0) > 0,
               str(compactions[0]) if compactions else "no compaction event")
        after = await cdp.evaluate("CHAT.history.length", page)
        report("the page context really was trimmed", after < before,
               "%s -> %s messages" % (before, after))
        first_user = next((e for e in events if e["type"] == "user_message"), None)
        report("compaction lost no prior turn from the log",
               first_user is not None
               and first_user.get("text") == "Place a marker on the dark box and plan it."
               and sum(1 for e in events if e["type"] == "user_message") >= 2
               and sum(1 for e in events if e["type"] == "tool_result") == 3)

        # Every frame reference resolves to stored bytes: the replay contract.
        import re as _re
        refs = set(_re.findall(r'"\$blob":\s*"([0-9a-f]{64})"', json.dumps(events)))
        report("every $blob in the log resolves",
               bool(refs) and all((traces / "blobs" / (sha + ".jpg")).is_file()
                                  for sha in refs), "%d blob(s)" % len(refs))
        manifest_files = list(traces.glob("*/manifest.json"))
        report("the session manifest froze the environment",
               len(manifest_files) == 1
               and json.loads(manifest_files[0].read_text(encoding="utf-8")).get("rig_file")
               == "rig.json"
               and (manifest_files[0].parent / "rig.json").is_file()
               and (manifest_files[0].parent / "calib.json").is_file())

    if failures:
        print("\nlive sidebar trace check: %d failure(s): %s" % (len(failures), failures))
        return 1
    print("\nlive sidebar: user_message, tools, frames, compaction and feedback all traced")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Headless check of phone.html against a running server.")
    parser.add_argument("--url", default="http://127.0.0.1:8443/",
                        help="the page as the phone would open it, token included")
    parser.add_argument("--serve", action="store_true",
                        help="host the phone check's capture server in-process with a "
                             "synthetic calibration: no external server needed")
    parser.add_argument("--chrome", default="", help="browser executable")
    parser.add_argument("--connect", type=int, default=0,
                        help="attach to an already running --remote-debugging-port instead of "
                             "launching one (needed when the shell sandbox blocks browser IPC)")
    parser.add_argument("--profile", default=str(HERE / ".e2e-profile"))
    parser.add_argument("--screenshot", default=str(HERE / ".e2e-shot.png"))
    args = parser.parse_args(argv)
    return asyncio.run(run(args))


if __name__ == "__main__":
    sys.exit(main())
