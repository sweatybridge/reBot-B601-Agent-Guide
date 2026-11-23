"""Offline hold-frame extraction: clock alignment + stillness selection (pure).

The oracle's hold stills no longer have to be grabbed live during playback. The
executed-trajectory recording is permanent evidence (spec: retained for async
review + re-processing), and its fragments carry episode-relative timestamps
(`evidence.media_fragment.t_start/t_end`). A hold window is a span of the
trajectory clock; telemetry (`{cmd, meas, t}` samples) says when the arm was
actually still. So a hold still can be extracted AFTER execution:

    hold window (trajectory time) --(+t_offset)--> episode/video time
    -> fragments covering that span -> decoded frames
    -> pick the STILLEST frame (min measured joint speed), midpoint tiebreak.

This module is the pure decision logic: no I/O, no cv2, no network. It is given
decoded frame timestamps + telemetry samples and chooses the frame. The native
decode lives in `extract_service.py` (isolated activity, spec §2 crash-isolation
invariant); this module is unit-testable without a video.

Why this is at least as good as the live grab: the live path grabbed at the hold
midpoint and confirmed stillness with two pose reads 120 ms apart. Here stillness
comes from the full meas trajectory (richer than two samples), and we pick the
stillest frame in the window rather than hoping one live grab landed well. The
midpoint tiebreak reproduces the live choice when the whole window is still.

Parity note: the extracted frame need not be bit-identical to the old live grab
(it may be a different, stillest frame). The oracle gate is verdict agreement on
the same episode, not pixel identity -- the accuracy map is statistical anyway.
"""
from __future__ import annotations

import bisect
import math

# A frame this far outside the window (seconds) is still accepted, to tolerate
# fragment/frame timestamp rounding at the window edges.
_EDGE_TOL = 1e-6


def joint_speeds(samples):
    """Per-sample max joint speed (rad/s) from the meas trajectory.

    Central difference in the interior, one-sided at the ends; a zero-length
    interval reads as still (0.0), never a divide-by-zero. Returns a list of
    `(t, vmax)` aligned with `samples` (t in the samples' own clock).
    """
    n = len(samples)
    out = []
    for i in range(n):
        if n < 2:
            out.append((float(samples[i]["t"]), 0.0))
            continue
        if i == 0:
            a, b = samples[0], samples[1]
        elif i == n - 1:
            a, b = samples[n - 2], samples[n - 1]
        else:
            a, b = samples[i - 1], samples[i + 1]
        dt = float(b["t"]) - float(a["t"])
        if dt <= 1e-9:
            v = 0.0
        else:
            ma, mb = a["meas"], b["meas"]
            v = max(abs(float(mb[j]) - float(ma[j])) for j in range(len(ma))) / dt
        out.append((float(samples[i]["t"]), v))
    return out


def _speed_at(t, times, speeds):
    """Speed of the telemetry sample nearest to time `t` (both in video clock)."""
    if not times:
        return float("inf")
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return speeds[0]
    if i >= len(times):
        return speeds[-1]
    # nearest of the two bracketing samples
    return speeds[i - 1] if (t - times[i - 1]) <= (times[i] - t) else speeds[i]


def select_hold_frame(frame_times, samples, window, t_offset=0.0):
    """Choose the stillest decoded frame inside a hold window.

    `frame_times` are decoded frame timestamps in the episode/video clock;
    `window` has `t_start`/`t_end` in the trajectory clock; `t_offset` maps
    trajectory time -> video time (video_t = traj_t + t_offset; 0 when the
    recording clock zero is playback start). `samples` are telemetry in the
    trajectory clock.

    Returns `{t, score, midpoint, window:[w0,w1]}` for the chosen frame, or None
    if no decoded frame falls in the window (a missed sample is reported, never
    guessed -- §5.3). Selection: min measured joint speed; ties break toward the
    window midpoint (the live-grab choice), then toward the earlier frame.
    """
    w0 = float(window["t_start"]) + t_offset
    w1 = float(window["t_end"]) + t_offset
    if w1 < w0:
        w0, w1 = w1, w0
    mid = 0.5 * (w0 + w1)
    speeds = joint_speeds(samples)
    times = [t + t_offset for t, _ in speeds]
    vals = [v for _, v in speeds]
    cands = [float(tf) for tf in frame_times if w0 - _EDGE_TOL <= tf <= w1 + _EDGE_TOL]
    if not cands:
        return None

    def key(tf):
        return (_speed_at(tf, times, vals), abs(tf - mid), tf)

    best = min(cands, key=key)
    return {"t": best, "score": _speed_at(best, times, vals),
            "midpoint": mid, "window": [w0, w1]}


def fragments_for_window(fragments, t0, t1):
    """Fragments whose episode-relative span overlaps [t0, t1] (video clock).

    `fragments` are dicts with `t_start`/`t_end` (and `seq`); returned in seq
    order so the service decodes them in playlist order.
    """
    hit = [f for f in fragments if float(f["t_end"]) >= t0 and float(f["t_start"]) <= t1]
    return sorted(hit, key=lambda f: f.get("seq", 0))


def plan_extraction(windows, fragments, t_offset=0.0):
    """Per-window decode plan: the video-clock span and the fragments to decode.

    Pure scheduling helper for the service: returns one entry per hold window
    `{index, w0, w1, fragments:[seq...]}`. A window with no covering fragment is
    flagged `fragments: []` (the service reports it as unextractable, not fatal).
    """
    plan = []
    for window in windows:
        w0 = float(window["t_start"]) + t_offset
        w1 = float(window["t_end"]) + t_offset
        frags = fragments_for_window(fragments, w0, w1)
        plan.append({"index": window.get("index"), "w0": w0, "w1": w1,
                     "fragments": [f.get("seq") for f in frags]})
    return plan