#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Where a still comes from when nobody is holding the phone.

The manual flow taps a frame in a browser. The automatic flow has to fetch one
itself, so this is the seam: an RTSP stream (MediaMTX already serves the phone
at `rtsp://mediamtx:8554/phone`), a plain camera device, or an HTTP snapshot
endpoint of an IP camera.

    camera.open_source("rtsp://mediamtx:8554/phone")
    camera.open_source("http://cam.local/snapshot.jpg")
    camera.open_source("device:0")            # /dev/video0, on a Linux host

A stream is kept open for the whole calibration: reconnecting per frame costs a
second or two and can hand back a keyframe from before the arm moved.
"""

from __future__ import annotations

import time
import urllib.error
import urllib.request

import cv2
import numpy as np


class CameraError(Exception):
    """No frame; the message says which URL or device failed."""


class Source:
    """One camera, opened once per calibration and read many times."""

    name = ""

    def read(self, retries=3):
        raise NotImplementedError

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


class Snapshot(Source):
    """An HTTP endpoint that returns a JPEG, one request per frame.

    `latency` is accepted for symmetry with `Stream` and ignored: a snapshot is
    fetched when it is asked for, so it cannot be stale.
    """

    def __init__(self, url, timeout=8.0, latency=0.0):
        self.name = url
        self.timeout = float(timeout)
        self.latency = float(latency)

    def read(self, retries=3):
        last = None
        for attempt in range(max(1, retries)):
            try:
                with urllib.request.urlopen(self.name, timeout=self.timeout) as answer:
                    blob = answer.read()
                image = cv2.imdecode(np.frombuffer(blob, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is not None:
                    return image
                last = ValueError("the response is not a decodable image")
            except (urllib.error.URLError, TimeoutError, OSError) as error:
                last = error
            time.sleep(0.4 * (attempt + 1))
        raise CameraError("no snapshot from %s: %s" % (self.name, last))


class Stream(Source):
    """An RTSP/HTTP/device stream read through OpenCV."""

    def __init__(self, url, latency=0.25):
        self.name = str(url)
        target = int(url.split(":", 1)[1]) if self.name.startswith("device:") else self.name
        if self.name.startswith("device:"):
            self.capture = cv2.VideoCapture(target, cv2.CAP_DSHOW) if hasattr(cv2, "CAP_DSHOW") \
                else cv2.VideoCapture(target)
        else:
            self.capture = cv2.VideoCapture(target)
        try:
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except cv2.error:
            pass
        if not self.capture.isOpened():
            raise CameraError("cannot open the camera at %s" % self.name)
        self.latency = float(latency)
        # A stream hands back whatever is buffered; the first frames are often
        # black or from before the connection settled.
        for _ in range(5):
            self.capture.read()

    def read(self, retries=3):
        """A frame no older than `latency` seconds.

        The arm being still says nothing about the *camera*: a deep jitter buffer
        can hand back a frame from before the arm stopped, and no length of hold
        in the trajectory changes that. Frames arrive in real time, so draining
        the stream for `latency` seconds advances it by the same amount and the
        frame kept cannot be older than that -- cheaper and more honest than
        holding every pose a second longer on the chance the buffer is shallow.
        """
        last = None
        for _attempt in range(max(1, retries)):
            ok, frame = self.capture.read()
            if ok and frame is not None:
                deadline = time.time() + self.latency
                while time.time() < deadline:
                    if not self.capture.grab():
                        break
                ok, frame = self.capture.read()
                if ok and frame is not None:
                    return frame
                last = "read returned nothing after draining"
            else:
                last = "read returned nothing"
            time.sleep(0.3)
        raise CameraError("no frame from %s: %s" % (self.name, last))

    def close(self):
        try:
            self.capture.release()
        except Exception:      # noqa: BLE001 - a release failure must not mask a real error
            pass


def open_source(url, **kwargs):
    """Pick the reader from the URL: http(s) snapshot, else an OpenCV stream."""
    if not url:
        raise CameraError("no camera configured: set REBOT_CAMERA or pass camera=")
    url = str(url).strip()
    if url.startswith(("http://", "https://")) and not url.endswith((".mjpg", ".mjpeg")):
        return Snapshot(url, **kwargs)
    return Stream(url, **kwargs)
