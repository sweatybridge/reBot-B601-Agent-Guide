#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The pose log of a sweep, and the frames it implies.

The sweep is recorded by MediaMTX (`mediamtx.py` starts and stops it), so this
module has two jobs around that: sample the arm's pose while the passes happen,
and afterwards pick the recorded frame that shows each pose.

Picking is by *time*, because the recording comes from somewhere else and has no
per-frame log of ours. That is a step down from a frame index, and it is why the
sweep still holds at each pose: during a hold the pose is the same in every
sample, so a window of wall-clock time maps to a window of frames no matter what
the latency is. A pose is used only if its window is at least `MIN_HOLD_SECONDS`
long -- shorter than that is the arm passing through, not holding.

The pose log and the segment names share one clock: `time.time()` here, the
file name (parsed as UTC) there. They are both the host's wall clock, which is
what makes the mapping possible at all.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

LOG_HZ = 15.0
# A sample this close to the one before it is the arm standing still, not moving
# through: at 15 Hz, 0.008 rad per sample is 0.12 rad/s, well under the 0.35 rad/s
# the sweep moves at and well over encoder noise.
STILL_RAD = 0.008
# A window shorter than this is the arm passing through the pose, not holding it.
MIN_HOLD_SECONDS = 0.5
# How far a held pose may sit from the one that was commanded. The servo holds a
# gravity-loaded joint *below* its command -- there is no integral term -- so a
# steady offset of tens of mrad is normal, and it is not a failure: the fit uses
# the pose the arm actually held, measured, not the one that was asked for.
MAX_OFFSET_RAD = 0.15


class SweepVideoError(Exception):
    """The recording or the pose log could not give a frame."""


class PoseLog(threading.Thread):
    """Sample the arm's pose, at `hz`, for as long as the sweep runs."""

    def __init__(self, arm, path, hz=LOG_HZ):
        super().__init__(daemon=True)
        self.arm = arm
        self.path = Path(path)
        self.hz = max(1.0, float(hz))
        self._stopped = threading.Event()
        self.samples = 0
        self.error = None

    def run(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as log:
                started = time.time()
                period = 1.0 / self.hz
                while not self._stopped.is_set():
                    pose = self.arm.pose()
                    log.write(json.dumps({
                        "i": self.samples,
                        "t": round(time.time(), 4),
                        "q": [float(v) for v in pose["q"][:6]],
                    }) + "\n")
                    self.samples += 1
                    delay = started + self.samples * period - time.time()
                    if delay > 0:
                        time.sleep(delay)
        except Exception as error:      # noqa: BLE001 - carried to stop()
            self.error = error

    def stop(self):
        self._stopped.set()
        self.join(timeout=10)
        if self.error is not None:
            raise self.error
        if self.samples == 0:
            raise SweepVideoError("the pose log recorded nothing")
        return self.samples


def load_log(path):
    """The pose samples, as a list of {"i", "t", "q"}."""
    path = Path(path)
    if not path.is_file():
        raise SweepVideoError("no pose log at %s" % path)
    entries = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                entries.append(json.loads(line))
    if not entries:
        raise SweepVideoError("the pose log at %s is empty" % path)
    return entries


def _runs(flags):
    """Start and end indices of every True run."""
    runs = []
    start = None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            runs.append((start, index))
            start = None
    if start is not None:
        runs.append((start, len(flags)))
    return runs


def _holds(times, poses, still_rad=STILL_RAD, min_seconds=MIN_HOLD_SECONDS):
    """Windows where the arm stopped, as (start, end) index pairs.

    Stillness is measured between consecutive samples, so a run begins at the
    sample *before* the first still step -- the arm was already there.
    """
    still = np.zeros(len(poses), dtype=bool)
    for index in range(1, len(poses)):
        still[index] = float(np.max(np.abs(poses[index] - poses[index - 1]))) <= still_rad
    windows = []
    for start, end in _runs(still):
        begin = max(0, start - 1)
        if times[end - 1] - times[begin] >= float(min_seconds):
            windows.append((begin, end))
    return windows


def select(entries, targets, still_rad=STILL_RAD, min_seconds=MIN_HOLD_SECONDS,
           max_offset=MAX_OFFSET_RAD):
    """For each target pose, the middle of the stop that shows it.

    A hold is defined by *stopping*, not by being within a tolerance of the
    command: the loaded arm settles tens of mrad below what it was told. Holds and
    targets are matched by nearest -- in sweep order, so a hold cannot be claimed
    by a later pose -- and `max_offset` only rejects a stop that is nowhere near
    the pose it is supposed to be.

    The middle of the window is chosen so a latency of up to half the window
    cannot pull the frame out of it.
    """
    times = np.asarray([entry["t"] for entry in entries], dtype=float)
    poses = np.asarray([entry["q"] for entry in entries], dtype=float)
    holds = _holds(times, poses, still_rad=still_rad, min_seconds=min_seconds)
    chosen = []
    last = -1
    for number, target in enumerate(targets, start=1):
        wanted = np.asarray(target, dtype=float)
        match = None
        nearest = None
        for begin, end in holds:
            if begin <= last:
                continue
            offset = float(np.max(np.abs(poses[begin:end].mean(axis=0) - wanted)))
            if nearest is None or offset < nearest[0]:
                nearest = (offset, begin, end)
            if offset <= max_offset:
                match = (offset, begin, end)
                break
        if match is None:
            chosen.append({"pose": number, "missed": True,
                           "offset_mrad": None if nearest is None
                           else round(1000.0 * nearest[0], 1)})
            continue
        offset, begin, end = match
        middle = 0.5 * (times[begin] + times[end - 1])
        sample = int(np.argmin(np.abs(times - middle)))
        chosen.append({"pose": number, "t": float(times[sample]),
                       "window_s": round(float(times[end - 1] - times[begin]), 2),
                       "offset_mrad": round(1000.0 * offset, 1),
                       "error_mrad": round(float(1000.0 * np.max(np.abs(poses[sample] - wanted))), 1),
                       "q": entries[sample]["q"]})
        last = begin
    return chosen


def extract(segments, directory, targets, still_rad=STILL_RAD,
            min_seconds=MIN_HOLD_SECONDS, max_offset=MAX_OFFSET_RAD):
    """Save the frame nearest each chosen moment; return them in pose order.

    Each segment is decoded once, in order, and the wanted frames are kept on the
    way past. Seeking by frame number is unreliable across containers and this
    costs a single sequential decode per segment.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    entries = load_log(directory / "poses.jsonl")
    chosen = select(entries, targets, still_rad=still_rad, min_seconds=min_seconds,
                    max_offset=max_offset)
    missed = [item for item in chosen if item.get("missed")]
    if missed:
        parts = []
        for item in missed:
            if item["offset_mrad"] is None:
                parts.append("pose %d had no stop to match (the arm never came to rest)"
                             % item["pose"])
            else:
                parts.append("the nearest stop was %.0f mrad from pose %d"
                             % (item["offset_mrad"], item["pose"]))
        raise SweepVideoError("the arm never came to rest at %s: %s"
                              % (", ".join("pose %d" % item["pose"] for item in missed),
                                 "; ".join(parts)))
    if not segments:
        raise SweepVideoError(
            "nothing was recorded: check that the phone is publishing (the /live page) and "
            "that MediaMTX has the path")

    saved = {}
    for segment in segments:
        wanted = {}
        capture = cv2.VideoCapture(str(segment.path))
        try:
            fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
            if fps <= 0:
                raise SweepVideoError("the recording at %s has no frame rate" % segment.path)
            count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            for item in chosen:
                if item["pose"] in saved:
                    continue
                offset = item["t"] - segment.start
                if 0.0 <= offset < (count / fps if count else float("inf")):
                    index = int(round(offset * fps))
                    wanted.setdefault(index, []).append(item)
            if not wanted:
                continue

            index = 0
            while wanted:
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                for item in wanted.pop(index, []):
                    path = directory / ("pose-%d.jpg" % item["pose"])
                    if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94]):
                        raise SweepVideoError("cannot write %s" % path)
                    saved[item["pose"]] = dict(item, path=path, image=frame,
                                               segment=segment.path.name,
                                               frame=index)
                index += 1
        finally:
            capture.release()

    missing = [item["pose"] for item in chosen if item["pose"] not in saved]
    if missing:
        raise SweepVideoError(
            "the recording does not cover %s: the segment for that moment is missing or "
            "shorter than its log" % ", ".join("pose %d" % value for value in missing))

    result = [saved[number] for number in range(1, len(targets) + 1)]
    (directory / "poses.json").write_text(json.dumps(result, indent=2, default=str) + "\n",
                                          encoding="utf-8")
    return result
