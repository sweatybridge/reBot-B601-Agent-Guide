#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MediaMTX's control API: record a path, and find what it recorded.

The board has no recorder of its own and the capture server should not spend its
time decoding a stream it is not otherwise using, so the sweep is recorded by the
media server that already has the stream. This is the seam:

    rec = mediamtx.Recorder("http://mediamtx:9997", path="phone",
                            recordings=Path("state/recordings"))
    rec.enable()          # PATCH /v3/config/paths/patch/phone {"record": true}
    ... the sweep ...
    rec.disable()
    for segment in rec.segments(since=started): ...

The API is not published outside the compose network: it can reconfigure any
path, so only the containers should reach it.

Segment names are written by MediaMTX with *its* local time, and the container
runs in UTC (`TZ: UTC` in compose). The name is therefore parsed as UTC: parsing
it as the host's local time would put every recording eight hours away from the
pose log in, say, Singapore.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

STAMP = "%Y-%m-%d_%H-%M-%S-%f"


class MediaMtxError(Exception):
    """The media server could not be reached or refused the change."""


@dataclass
class Segment:
    """One recorded file and the wall-clock second it starts at."""

    path: Path
    start: float        # epoch seconds, UTC


def segment_start(name):
    """The epoch a segment file starts at, from its name; None if unparseable."""
    try:
        stamp = datetime.strptime(Path(name).stem, STAMP)
    except ValueError:
        return None
    return stamp.replace(tzinfo=timezone.utc).timestamp()


def path_from_camera(url):
    """The MediaMTX path a camera URL points at, or None if it is not RTSP.

    Only a stream MediaMTX is already serving can be recorded through it; a USB
    device or a snapshot endpoint has no path here.
    """
    parsed = urllib.parse.urlparse(str(url))
    if parsed.scheme not in ("rtsp", "rtsps"):
        return None
    name = parsed.path.strip("/")
    return name or None


class Recorder:
    """Start and stop recording one MediaMTX path, then list the segments."""

    def __init__(self, api, path="phone", recordings=None, timeout=10.0):
        self.api = str(api).rstrip("/")
        self.path = str(path)
        self.recordings = Path(recordings) if recordings is not None else None
        self.timeout = float(timeout)

    def _patch(self, body):
        url = "%s/v3/config/paths/patch/%s" % (self.api, self.path)
        request = urllib.request.Request(
            url, data=json.dumps(body).encode("utf-8"), method="PATCH",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as answer:
                return answer.status
        except urllib.error.HTTPError as error:
            detail = error.read()[:200].decode("utf-8", "replace")
            raise MediaMtxError("the media server refused %s: %s" % (body, detail)) from error
        except (urllib.error.URLError, OSError) as error:
            raise MediaMtxError(
                "cannot reach the media server's API at %s: %s -- is `api: yes` set in "
                "mediamtx.yml?" % (self.api, error)) from error

    def enable(self):
        self._patch({"record": True})

    def disable(self):
        self._patch({"record": False})

    def segments(self, since=None):
        """The recordings for this path, oldest first.

        Recording starts a new file, so a segment that begins before the sweep
        did is from an earlier run and is skipped.
        """
        if self.recordings is None:
            raise MediaMtxError("no recordings directory was configured")
        folder = self.recordings / self.path
        found = []
        for path in sorted(folder.glob("*.mp4")):
            start = segment_start(path.name)
            if start is None:
                continue
            if since is not None and start < float(since) - 2.0:
                continue
            found.append(Segment(path=path, start=start))
        return found
