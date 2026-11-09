#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive the phone page in headless Chrome: tap -> capture -> file on disk.

The page is hand-written JavaScript with a tap-to-pixel mapping and a
`getUserMedia` path that no unit test can reach. This runs the real page in a
real Chromium over the DevTools protocol, replaces the camera with a canvas
stream (`getUserMedia` returns a 1280x960 test pattern), clicks a known pixel,
captures once, and checks the stored tap and image size.

    # terminal 1
    python examples/host/app/capture_server.py --token test-token
    # terminal 2
    python examples/host/tests/browser_check.py --url "https://127.0.0.1:8443/?token=test-token"

If the shell cannot launch a browser itself (confined sandboxes block Chromium's
IPC), start one separately and attach:

    chrome --headless=new --remote-debugging-port=9333 --user-data-dir=/tmp/x about:blank
    python examples/host/tests/browser_check.py --connect 9333 --url "..."

Needs Chrome or Edge on the machine. It is a check of the page, not of the
camera: the lens, the focus and the intrinsics are still the phone's problem.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import aiohttp

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
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

    # Tap a known image pixel through the real pointer handler.
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
    tap = await cdp.evaluate("state.tap", page)
    check = ("[%s] tap maps to the image pixel" % (
        "ok  " if tap and abs(tap["u"] - u) < 1.0 and abs(tap["v"] - v) < 1.0 else "FAIL"))
    print("  %s (%.1f, %.1f) -> expected (%.1f, %.1f)"
          % (check, tap["u"] if tap else -1, tap["v"] if tap else -1, u, v))
    if not tap or abs(tap["u"] - u) > 1.0:
        raise SystemExit("tap mapping is wrong")

    before = {p.name for p in Path(args.captures).glob("*") if p.is_dir()}
    await cdp.evaluate("document.getElementById('shot').click()", page)

    found = None
    for _ in range(100):
        now = {p.name for p in Path(args.captures).glob("*") if p.is_dir()}
        fresh = now - before
        if fresh:
            found = fresh.pop()
            break
        await asyncio.sleep(0.1)
    print("  [%s] capture landed as %s" % ("ok  " if found else "FAIL", found))
    if not found:
        logs = await cdp.evaluate(
            "Array.from(document.querySelectorAll('#log p')).map(p => p.textContent)", page)
        print("  page log: %s" % logs)
        raise SystemExit("no capture directory appeared")

    if args.screenshot:
        shot = await cdp.call("Page.captureScreenshot", {"format": "png"}, page)
        Path(args.screenshot).write_bytes(base64.b64decode(shot["data"]))
        print("  screenshot: %s" % args.screenshot)

    meta = json.loads((Path(args.captures) / found / "meta.json").read_text(encoding="utf-8"))
    same_tap = (meta.get("tap") or {}) and abs(meta["tap"]["u"] - u) < 1.0
    print("  [%s] meta tap and image_size [%s]"
          % ("ok  " if same_tap and meta.get("image_size") == [1280, 960] else "FAIL",
             meta.get("image_size")))
    if not same_tap or meta.get("image_size") != [1280, 960]:
        raise SystemExit("the stored metadata does not match what was tapped")

    print("\npage, tap mapping and upload all work")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Headless check of phone.html against a running server.")
    parser.add_argument("--url", default="http://127.0.0.1:8443/",
                        help="the page as the phone would open it, token included")
    parser.add_argument("--captures", default=str(ROOT / "state" / "captures"),
                        help="the server's capture directory, to see the upload arrive")
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
