#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The closed-loop oracle's pure core: hold windows, downsampling, attribution.

Everything here is a function of its arguments -- no I/O, no hardware, no
network -- so the live verdict (capture_server) and the offline labeler
(tests/trace_triage) cannot disagree, and every rule is unit-testable with a
synthetic document.

`attribute` implements the residual-signature table from the spec (§5.4): the
shape of the residual names the layer, not the size alone. The arm holding what
it was told while the camera disagrees is a camera->arm problem; a per-joint
error that tracks tau_g/kp is sag; a constant per-joint bias is joint zeros; a
z-only placement error the size of the object is the one-plane limitation, and
is labelled, never ticketed.
"""

from __future__ import annotations

import math

import rebot_ik

# --------------------------------------------------------------------------- #
# Thresholds -- one block, tunable from measured noise floors (§5.4).
# Conservative starting values: inside the arm's ~23 mm repeatability, above
# the fit's ~5 mm normal. Single source: rebot_ik.ORACLE (world.arm_config
# "oracle" group in PG mode); these alias it and apply_config re-syncs them.
# --------------------------------------------------------------------------- #
PIXEL_ERROR_MM = rebot_ik.ORACLE["pixel_error_mm"]        # predicted-vs-detected tip, at the work plane
TIP_ERROR_MM = rebot_ik.ORACLE["tip_error_mm"]            # FK(measured) vs FK(commanded) tip distance
HOLD_ERROR_RAD = rebot_ik.ORACLE["hold_error_rad"]        # 30 mrad: ~18 mm at 0.6 m, above the fit's normal
HOLD_MIN_S = 0.8             # a window shorter than this is tracking-only
MAX_SAMPLES = 2000           # wire budget for telemetry samples
WITNESS_DRIFT_PX = rebot_ik.ORACLE["witness_drift_px"]    # the capture path's own refusal limit
WITNESS_SCALE_TOL = rebot_ik.ORACLE["witness_scale_tol"]  # focus/zoom changes the field of view
SAG_CORRELATION = rebot_ik.ORACLE["sag_correlation"]      # |cos(hold_error, tau_g/kp)| to call it sag
TIP_VECTOR_COHERENCE = rebot_ik.ORACLE["tip_vector_coherence"]  # mean resultant length of the pixel-error directions:
                             # a rig/turns shift moves every hold the same way in
                             # camera space; a detector picking wrong blob ends at
                             # rotated wrists scatters. Below this, the pixel layer
                             # is a perception limitation, not a geometry ticket.
ZERO_BIAS_SPREAD_RAD = rebot_ik.ORACLE["zero_bias_spread_rad"]  # per-joint bias this constant across poses = zeros
REACH_FRACTION = rebot_ik.ORACLE["reach_fraction"]        # "near the boundary" for the reach-model signature
PLACEMENT_XY_MM = rebot_ik.ORACLE["placement_xy_mm"]      # x/y small enough to call a placement error z-only
PLACEMENT_Z_TOL_MM = rebot_ik.ORACLE["placement_z_tol_mm"]  # z matches the object height within this
CMD_TOL = 1e-9               # waypoints repeat a hold's command exactly
IK_RESIDUAL_MM = rebot_ik.ORACLE["ik_residual_mm"]        # the sequence planner's own acceptance bound: a
                             # solver residual above it missed its own gate
HOLD_TIME_TOL = 1e-6         # plan-clock slack matching a hold window to its
                             # marker's declared hold span


def _same_cmd(a, b):
    """Whether two commands hold the same arm pose.

    Only joints 1-6 define a hold: the gripper (motor 7) actuates right through
    a marker hold -- that is the grasp cycle, not motion -- so comparing all
    seven would find no holds at all in a pick sequence. Short lists (arm-only
    logs) compare in full.
    """
    if a is None or b is None or len(a) != len(b) or not a:
        return False
    return all(abs(float(x) - float(y)) <= CMD_TOL for x, y in zip(a[:6], b[:6]))


def hold_windows(waypoints, min_s=HOLD_MIN_S):
    """Runs of constant command at least `min_s` long, as index/time windows.

    The plan holds at every marker (plan_sequence builds the holds); a frame
    taken inside one shows the held pose regardless of stream latency, which is
    the same reason sweep_video.select picks window middles.
    """
    windows = []
    start = 0
    for index in range(1, len(waypoints) + 1):
        same = (index < len(waypoints)
                and _same_cmd(waypoints[index].get("q"), waypoints[index - 1].get("q")))
        if same:
            continue
        if index - start >= 2:
            t0 = float(waypoints[start]["t"])
            t1 = float(waypoints[index - 1]["t"])
            if t1 - t0 >= float(min_s):
                windows.append({"index": len(windows) + 1, "start": start, "end": index,
                                "t_start": t0, "t_end": t1,
                                "q_cmd": [float(v) for v in waypoints[start]["q"]]})
        start = index
    return windows


def match_hold_targets(windows, targets, time_tol=HOLD_TIME_TOL):
    """Which planned target each hold window belongs to, by the plan's clock.

    A marker sequence records each marker's hold span (`hold_t_start`/
    `hold_t_end`) as it builds it; a hold window lies inside exactly one such
    span. Matching by time is exact -- joint proximity would confuse a marker
    near the park pose with the gripper-opening start hold, and indexing would
    depend on whether that start hold exists at all. A target without a hold
    span (an older plan) is skipped; the start hold matches nothing.

    Pure: returns one entry per window -- the target dict, or None.
    """
    usable = [target for target in (targets or [])
              if target.get("hold_t_start") is not None
              and target.get("hold_t_end") is not None]
    matched = []
    for window in windows or []:
        t0 = float(window.get("t_start", 0.0))
        t1 = float(window.get("t_end", 0.0))
        best = None
        for target in usable:
            if (float(target["hold_t_start"]) - time_tol <= t0
                    and t1 <= float(target["hold_t_end"]) + time_tol):
                best = target
                break
        matched.append(best)
    return matched


def downsample(samples, limit=MAX_SAMPLES):
    """Thin samples to `limit`, keeping EVERY sample inside a hold window.

    Holds are where cmd is constant across consecutive samples -- those are the
    measurement points the oracle averages, so they are never dropped. The rest
    is strided uniformly; the first and last sample survive for context. If the
    holds alone exceed the budget they still all survive: the budget yields to
    the evidence. Mirrors downsample_samples in edge-rust/src/main.rs.
    """
    count = len(samples)
    if count <= limit:
        return list(samples)
    keep = [False] * count
    start = 0
    for index in range(1, count + 1):
        same = (index < count
                and _same_cmd(samples[index].get("cmd"), samples[index - 1].get("cmd")))
        if same:
            continue
        if index - start >= 2:
            for slot in range(start, index):
                keep[slot] = True
        start = index
    keep[0] = True
    keep[count - 1] = True
    kept = sum(keep)
    if kept < limit:
        free = [index for index in range(count) if not keep[index]]
        budget = limit - kept
        for step in range(budget):
            keep[free[step * len(free) // budget]] = True
    return [sample for index, sample in enumerate(samples) if keep[index]]


def _problem(kind, layer, detail):
    return {"kind": kind, "layer": layer, "detail": detail}


def _max_abs(values):
    return max((abs(float(v)) for v in values or []), default=0.0)


def _sag_correlation(hold):
    """|cos| between the per-joint hold error and the model's tau_g/kp sag.

    The controller has no integral term, so a load-dependent error lines up
    with the gravity prediction (held = command - sag: the sign is negative,
    hence the absolute cosine).
    """
    error = [float(v) for v in (hold.get("hold_error_rad") or [])][:6]
    sag = [float(v) for v in (hold.get("sag_rad") or [])][:6]
    if len(error) != 6 or len(sag) != 6:
        return 0.0
    ne = math.sqrt(sum(v * v for v in error))
    ns = math.sqrt(sum(v * v for v in sag))
    if ne < 1e-9 or ns < 1e-9:
        return 0.0
    return abs(sum(a * b for a, b in zip(error, sag)) / (ne * ns))


def _constant_bias(holds):
    """A per-joint bias that is pose- and load-independent: joint zeros."""
    errors = [[float(v) for v in (hold.get("hold_error_rad") or [])] for hold in holds]
    width = min(len(row) for row in errors)
    if width == 0:
        return False
    means = [sum(row[j] for row in errors) / len(errors) for j in range(width)]
    spread = [max(row[j] for row in errors) - min(row[j] for row in errors)
              for j in range(width)]
    return _max_abs(means) > HOLD_ERROR_RAD and max(spread) <= ZERO_BIAS_SPREAD_RAD


def attribute(oracle_doc):
    """Residual shape -> {layer, problems, confidence}. Pure (§5.4).

    `oracle_doc` is what capture_server writes to state/oracle/<plan_id>.json:
    per-hold residuals, the stream-turn provenance, the witness report, the
    telemetry's abort/stall flags, and any placement measurement. Every field
    is optional -- a missing measurement is reported as missing, never guessed.
    """
    doc = oracle_doc or {}
    holds = doc.get("holds") or []
    telemetry = doc.get("telemetry") or {}
    problems = []

    # Execution faults first: they explain the run, and they truncate the
    # evidence every later signature reads. Correlate, but not a correspondence
    # bug (§5.4, last row).
    abort = telemetry.get("abort")
    if abort:
        problems.append(_problem("follow-abort", "execution",
                                 "playback aborted at t=%s: %s"
                                 % (abort.get("t"), abort.get("reason"))))
    stall = telemetry.get("gripper_stall")
    if stall:
        problems.append(_problem("gripper-stall", "execution",
                                 "gripper stalled at t=%s (commanded %s, measured %s)"
                                 % (stall.get("t"), stall.get("commanded"),
                                    stall.get("measured"))))

    # The witness already refuses capture paths when the background moved;
    # markers bypass it, so the oracle re-checks it here (§5.4, row 3).
    witness_report = doc.get("witness") or {}
    drift = witness_report.get("drift_px")
    scale = witness_report.get("scale")
    if ((drift is not None and float(drift) > WITNESS_DRIFT_PX)
            or (scale is not None and abs(float(scale) - 1.0) > WITNESS_SCALE_TOL)):
        problems.append(_problem("stale-rig", "camera-arm",
                                 "the camera moved since the rig was fitted "
                                 "(drift %s px, scale %s)" % (drift, scale)))

    pixel_bad = [hold for hold in holds
                 if hold.get("pixel_error_mm") is not None
                 and float(hold["pixel_error_mm"]) > PIXEL_ERROR_MM]
    if pixel_bad:
        # A rig / K / turns error is a common camera-space shift: the error
        # vectors agree in direction. The green-tip heuristic at rotated wrists
        # picks different blob ends per pose and scatters -- that is perception,
        # labelled as a limitation (§7.5), never a geometry ticket.
        vectors = []
        for hold in pixel_bad:
            det, pred = hold.get("detected_tip_px"), hold.get("predicted_tip_px")
            if det and pred:
                dx, dy = float(det[0]) - float(pred[0]), float(det[1]) - float(pred[1])
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    vectors.append((dx / norm, dy / norm))
        coherence = (math.hypot(sum(v[0] for v in vectors), sum(v[1] for v in vectors))
                     / len(vectors)) if len(vectors) >= 2 else 1.0
        # Row 1: the arm held what it was told, the camera disagrees coherently.
        if coherence >= TIP_VECTOR_COHERENCE and all(
                hold.get("tip_error_mm") is not None
                and float(hold["tip_error_mm"]) <= TIP_ERROR_MM for hold in pixel_bad):
            problems.append(_problem("rig-mismatch", "camera-arm",
                                     "%d hold(s) with pixel error > %.0f mm while the tip "
                                     "held within %.0f mm: rig / K / turns"
                                     % (len(pixel_bad), PIXEL_ERROR_MM, TIP_ERROR_MM)))
        elif coherence < TIP_VECTOR_COHERENCE:
            problems.append(_problem("tip-detector", "perception",
                                     "%d hold(s) with pixel error > %.0f mm but scattered "
                                     "directions (coherence %.2f): the green-tip heuristic "
                                     "at rotated wrists, not a rig shift"
                                     % (len(pixel_bad), PIXEL_ERROR_MM, coherence)))
        # Row 2: a turns correction that was never measured (shape fallback) is
        # the quarter-turn sign. A turn measured against the base tag is not a
        # guess, so landscape alone does not indict it.
        source = (doc.get("stream_turn") or {}).get("source")
        if source != "measured" and (source == "shape" or doc.get("landscape")):
            problems.append(_problem("turns-correction", "camera-arm",
                                     "pixel error with stream_turn source %r%s"
                                     % (source, " in landscape" if doc.get("landscape") else "")))

    # Rows 4 and 5: per-joint hold error, load-dependent vs constant.
    biased = [hold for hold in holds
              if _max_abs(hold.get("hold_error_rad")) > HOLD_ERROR_RAD]
    if biased:
        if any(_sag_correlation(hold) >= SAG_CORRELATION for hold in biased):
            problems.append(_problem("sag", "arm-motor",
                                     "hold error tracks tau_g/kp: sag model / gains"))
        elif len(biased) >= 2 and _constant_bias(biased):
            problems.append(_problem("joint-zeros", "arm-motor",
                                     "constant per-joint bias across poses: joint zeros"))
        else:
            problems.append(_problem("hold-error", "arm-motor",
                                     "%d hold(s) above %.0f mrad, no load or bias signature"
                                     % (len(biased), 1000.0 * HOLD_ERROR_RAD)))

    # Row 6: tip error near the reach boundary with IK refusals in the tape.
    tip_bad = [hold for hold in holds
               if hold.get("tip_error_mm") is not None
               and float(hold["tip_error_mm"]) > TIP_ERROR_MM]
    if tip_bad:
        radius = doc.get("max_radius_m")
        reach = doc.get("max_reach_m")
        near = (radius is not None and reach is not None and float(reach) > 0
                and float(radius) >= REACH_FRACTION * float(reach))
        if near and int(doc.get("ik_refusals") or 0) > 0:
            problems.append(_problem("reach-model", "reach",
                                     "tip error grows near the reach boundary with IK "
                                     "refusals in the tape: reach model / --max-reach"))
        else:
            problems.append(_problem("tip-error", "arm-motor",
                                     "%d hold(s) with tip error > %.0f mm"
                                     % (len(tip_bad), TIP_ERROR_MM)))

    # The FK bridge (§7.4, Cartesian coverage): the plan records each marker's
    # raw IK solution, so the solver's residual |FK(q_ik) - flange_point| is
    # measurable separately from the arm's hold error. A command that misses
    # its target while the arm holds that command faithfully is an IK/model
    # problem, not a motor problem -- the sign pattern that separates them.
    ik_bad = [hold for hold in holds
              if hold.get("ik_residual_mm") is not None
              and float(hold["ik_residual_mm"]) > IK_RESIDUAL_MM]
    if ik_bad and all(hold.get("tip_error_mm") is not None
                      and float(hold["tip_error_mm"]) <= TIP_ERROR_MM for hold in ik_bad):
        problems.append(_problem("ik-residual", "ik",
                                 "%d hold(s) where the IK solution misses its target by "
                                 "> %.0f mm while the arm held within %.0f mm: solver / model"
                                 % (len(ik_bad), IK_RESIDUAL_MM, TIP_ERROR_MM)))

    # Row 7: z-only placement error the size of the object is the one-plane
    # assumption -- a limitation to report, never a bug ticket.
    placement = doc.get("placement") or {}
    error_mm = placement.get("error_mm")
    height = placement.get("object_height_mm")
    if (isinstance(error_mm, (list, tuple)) and len(error_mm) == 3 and height is not None
            and abs(float(error_mm[0])) <= PLACEMENT_XY_MM
            and abs(float(error_mm[1])) <= PLACEMENT_XY_MM
            and abs(float(error_mm[2]) - float(height)) <= PLACEMENT_Z_TOL_MM):
        problems.append(_problem("one-plane", "limitation",
                                 "placement error is z-only (%.1f mm) and matches the "
                                 "object height (%.1f mm): the one-plane assumption"
                                 % (float(error_mm[2]), float(height))))

    layer = problems[0]["layer"] if problems else "none"
    if not problems:
        measured = [hold for hold in holds if hold.get("pixel_error_mm") is not None]
        confidence = "high" if measured else ("medium" if holds else "low")
    else:
        confidence = "high"
        if abort or stall:
            confidence = "medium"   # the run was cut short; residuals are partial
        elif len(holds) <= 1 and any(p["layer"] in ("camera-arm", "arm-motor", "reach")
                                     for p in problems):
            confidence = "medium"   # one hold cannot separate pose from bias
    return {"layer": layer, "problems": problems, "confidence": confidence}