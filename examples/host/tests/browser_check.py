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

If the shell cannot launch a browser itself (confined sandboxes block Chromium's
IPC), start one separately and attach:

    chrome --headless=new --remote-debugging-port=9333 --user-data-dir=/tmp/x about:blank
    python examples/host/tests/browser_check.py --connect 9333 --url "..."

Browser autodetect only probes the Windows Chrome/Edge install paths; anywhere
else pass `--chrome <path>`. It is a check of the page, not of the camera: the
lens, the focus and the intrinsics are still the phone's problem.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

import aiohttp

HERE = Path(__file__).resolve().parent
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


def centimetres(label: str) -> float:
    """Read the height badge ("10.5 cm") back as a number."""
    return float(str(label).replace("cm", "").strip())


def find_browser(explicit: str) -> Path:
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise SystemExit("no browser at %s" % explicit)
        return path
    for candidate in WINDOWS_CANDIDATES:
        if Path(candidate).is_file():
            return Path(candidate)
    raise SystemExit("no Chrome or Edge found; pass --chrome")


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
        return await drive(args, args.connect)

    browser = find_browser(args.chrome)
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
        return await drive(args, port)
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


def main(argv=None):
    parser = argparse.ArgumentParser(description="Headless check of phone.html against a running server.")
    parser.add_argument("--url", default="http://127.0.0.1:8443/",
                        help="the page as the phone would open it, token included")
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
