#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Physical hillclimb: identify what the data constrains, then climb only that.

Phase 4 of the trace-collector spec (§7). The oracle corpus says *where* the
loop is wrong; this module decides what may be updated about the rig and the
sag model, and drives the one gated excursion that could fix it. Three rules
from the spec are hard constraints, not preferences:

* **Propose, never auto-run.** `TriggerMonitor` watches witness drift, the EWMA
  of oracle pixel error, tag reprojection growth and rig staleness, and returns
  proposals. Running a session needs `POST /identify/run {confirm: true}` with
  the execute gate on and the `calibrating` single-flight flag clear.
* **Fit only what the data constrains (§7.2).** A direction whose 1-sigma is
  above `SIGMA_GATE_MM` is held at its prior value and flagged for excitation;
  it is never "updated" by a climb. `refine_zeros` stays off unless the
  observations carry arm-independent anchors (a base-marker or second-view
  label), and j1/j6 are gauges that are never fitted at all. A converged fit
  with a weak direction is reported PARTIAL, never success.
* **Verify before hot-load (§7.3a).** The candidate rig is written beside the
  live one as `extrinsics-candidate-<ts>.json` and only committed after
  `auto_calibrate.verify`'s gates *plus* hold-out poses the fit never saw. A
  refusal leaves the running rig untouched.

The sag/gain residual map (§7.3b) is learned from execution telemetry alone --
no extra motion -- and applied inside `sweep.aim` as an additive term that is
still clipped by `SAG_LIMIT_RAD`. It is keyed by rig fingerprint, kp and joint
zeros, and is not trusted outside the envelope it was fitted in.

`plane_z` / surface probing (§7.3c) is DEFERRED: it is the only component that
intentionally contacts the environment and needs its own clearance review. No
contact probing is implemented here; do not add one without that review.

Everything pure is unit-testable offline; the only arm motion is
`climb_session`, which goes through `auto_calibrate.run` -> `ArmGateway.
move_sequence` -> the gateway's existing `/move` check chain (command_ok,
path_safe, speed clamp, edge aborts). No new execute path, no gate changes.
"""

from __future__ import annotations

import json
import math
import time
from bisect import bisect_right
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import auto_calibrate
import extrinsics as ex
import oracle
import rebot_ik
import sweep


class IdentifyError(Exception):
    """The climb cannot run, or the update is not identifiable; the message is for the operator."""


# --------------------------------------------------------------------------- #
# Gates -- one block, like oracle.py's thresholds.
# --------------------------------------------------------------------------- #
SIGMA_GATE_MM = 8.0          # a direction pinned worse than this is flagged, not updated
HOLDOUT_MAX_PX = 6.0         # same bound verify() uses for leave-one-out
RIG_MAX_AGE_DAYS = 30.0      # a rig older than this is stale even without drift
TAG_GROWTH_PX = 2.0          # tag reprojection growth over the session baseline
EWMA_ALPHA = 0.3             # weight of the newest oracle pixel error
SAG_MIN_HOLDS = 30           # §7.3b: fewer holds cannot span the workspace
SAG_HUBER_RAD = 0.010        # 10 mrad: a stall is an outlier, not a measurement
SAG_VALIDATE_MAX_MRAD = 15.0 # hold-out rms above this is not a usable map
SAG_VALIDATE_RATIO = 2.0     # hold-out may be at most this times the train rms
SAG_SPAN_RADIUS_M = 0.10     # holds must span this much radius...
SAG_SPAN_HEIGHT_M = 0.05     # ...and this much height, or they are one pose
ACCURACY_MIN_N = 3           # §7.4: a cell validates only with 3 good holds
ACCURACY_RADIUS_EDGES = (0.10, 0.20, 0.30, 0.40, 0.50, 0.60)
ACCURACY_HEIGHT_EDGES = (0.00, 0.10, 0.20, 0.30, 0.40)
ACCURACY_SECTORS = 8
# Observation labels that count as arm-independent anchors (§7.2): a base-tag
# solve or a second view breaks the camera/zeros gauge that tip-only data has.
ANCHOR_LABELS = ("base-marker", "anchor", "second-view")


def _stamp():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _rig_theta(rig):
    """The solver vector of a rig: camera (as T_cam<-base), tip, zeros."""
    T_cam_base = ex.invert(np.asarray(rig.T_base_cam, dtype=float))
    theta = np.zeros(15)
    theta[0:3] = ex.log_so3(T_cam_base[:3, :3])
    theta[3:6] = T_cam_base[:3, 3]
    theta[6:9] = np.asarray(rig.tip_offset, dtype=float)
    if rig.joint_zeros is not None:
        theta[9:15] = np.asarray(rig.joint_zeros, dtype=float)
    return theta


# --------------------------------------------------------------------------- #
# §7.1 Triggers: propose, never auto-run
# --------------------------------------------------------------------------- #

class TriggerMonitor:
    """Watches the oracle corpus and proposes identification sessions.

    Pure state: `observe` is called with each oracle doc the server writes, and
    `rig_proposals` is the on-demand staleness check GET /identify runs. Nothing
    here moves the arm or starts a session -- the proposals are evidence for a
    human (or the agent) to act on with an explicit confirm.
    """

    def __init__(self, pixel_mm=oracle.PIXEL_ERROR_MM, alpha=EWMA_ALPHA):
        self.pixel_mm = float(pixel_mm)
        self.alpha = float(alpha)
        self.ewma = None
        self.tag_baseline = None
        self.proposals = []

    def observe(self, oracle_doc=None, rig=None):
        doc = oracle_doc or {}
        proposals = []

        witness_report = doc.get("witness") or {}
        drift = witness_report.get("drift_px")
        scale = witness_report.get("scale")
        if ((drift is not None and float(drift) > oracle.WITNESS_DRIFT_PX)
                or (scale is not None and abs(float(scale) - 1.0) > oracle.WITNESS_SCALE_TOL)):
            proposals.append({"kind": "stale-rig",
                              "reason": "witness drift %s px, scale %s: the camera moved"
                                        % (drift, scale)})

        errors = [float(h["pixel_error_mm"]) for h in doc.get("holds") or []
                  if h.get("pixel_error_mm") is not None]
        for value in errors:
            self.ewma = value if self.ewma is None else (
                self.alpha * value + (1.0 - self.alpha) * self.ewma)
        if self.ewma is not None and self.ewma > self.pixel_mm:
            proposals.append({"kind": "pixel-drift",
                              "reason": "oracle pixel error EWMA %.1f mm over %.1f mm"
                                        % (self.ewma, self.pixel_mm)})

        tag = doc.get("tag_reprojection_px")
        if tag is not None:
            tag = float(tag)
            if self.tag_baseline is None:
                self.tag_baseline = tag
            elif tag > self.tag_baseline + TAG_GROWTH_PX:
                proposals.append({"kind": "tag-growth",
                                  "reason": "tag reprojection grew %.1f -> %.1f px"
                                            % (self.tag_baseline, tag)})

        proposals.extend(self.rig_proposals(rig))
        self.proposals = proposals
        return proposals

    def rig_proposals(self, rig):
        if rig is None:
            return [{"kind": "missing-rig", "reason": "no fitted rig: fixed mode has no geometry"}]
        age = rig.age_days
        if age is not None and age > RIG_MAX_AGE_DAYS:
            return [{"kind": "stale-rig", "reason": "the rig is %.0f days old" % age}]
        return []


# --------------------------------------------------------------------------- #
# §7.2 The identifiability gate
# --------------------------------------------------------------------------- #

def has_anchors(observations):
    return any(str(o.label).startswith(ANCHOR_LABELS) for o in observations)


def select_free(fit_result, gate_mm=SIGMA_GATE_MM):
    """(free indices, flagged directions) under the sigma gate.

    j1 and j6 (indices 9 and 14) are exact gauges -- the fit machinery already
    excludes them, and this excludes them again so a crafted free list can
    never sneak them in.
    """
    free, flagged = [], []
    for k, j in enumerate(fit_result.free):
        if j in (9, 14):
            continue
        sigma_mm = 1000.0 * math.sqrt(max(0.0, float(fit_result.covariance[k, k])))
        if sigma_mm > float(gate_mm):
            flagged.append({"name": ex.NAMES[j], "sigma_mm": round(sigma_mm, 2)})
        else:
            free.append(j)
    return free, flagged


def identifiability(fit_result, gate_mm=SIGMA_GATE_MM):
    """What this fit may update, and what it must leave alone (§7.2)."""
    free, flagged = select_free(fit_result, gate_mm)
    weakest = fit_result.weakest or {}
    partial = bool(flagged) or float(weakest.get("sigma_mm", 0.0)) > float(gate_mm)
    return {
        "status": "partial" if partial else "ok",
        "identifiable": [ex.NAMES[j] for j in free],
        "flagged": flagged,
        "weakest": weakest,
        "gate_mm": float(gate_mm),
    }


def gated_fit(observations, K, dist, options=None, prior_rig=None, gate_mm=SIGMA_GATE_MM,
              calib_fingerprint="", device="", mount="fixed"):
    """Fit only in the directions the data constrains.

    An exploratory fit (the same machinery `auto_calibrate.run` uses, with the
    degeneracy gate lifted) measures what is observable. The update itself then
    runs Levenberg-Marquardt from the *prior* rig with only the identifiable
    parameters free, so a weak direction keeps the value it had -- it is
    flagged for excitation, not bent to match noisy clicks. `refine_zeros` is
    honoured only when the observations carry arm-independent anchors.
    """
    options = options or ex.FitOptions()
    observations = list(observations)
    anchors = has_anchors(observations)
    options = replace(options, refine_zeros=bool(options.refine_zeros and anchors),
                      max_weak_mm=1e9)
    exploratory = ex.fit(observations, K, dist, options,
                         calib_fingerprint=calib_fingerprint, device=device, mount=mount)
    report = identifiability(exploratory, gate_mm)
    free, _flagged = select_free(exploratory, gate_mm)

    if not free:
        # Nothing is identifiable: the update is the prior itself, and the
        # report says which directions need excitation before another climb.
        rig = prior_rig if prior_rig is not None else exploratory.rig
        return {"rig": rig, "report": report, "converged": exploratory.converged,
                "free": [], "iterations": exploratory.iterations}

    theta0 = _rig_theta(prior_rig) if prior_rig is not None else _rig_theta(exploratory.rig)
    model = ex._Model(observations, K, dist, options)
    model.free = free
    theta, residuals, weights, jacobian, iterations, converged = \
        ex._levenberg_marquardt(model, theta0, options)
    covariance, _variance = ex._covariance(model, residuals, weights, jacobian)
    sigma = {ex.NAMES[j]: math.sqrt(max(0.0, float(covariance[k, k])))
             for k, j in enumerate(free)}
    weakest = ex._conditioning(covariance, free)
    predicted = ex.predicted_error(theta, covariance, free, K, dist,
                                   options.plane_z, options)

    T_base_cam = ex.invert(ex.transform(ex.rodrigues(theta[0:3]), theta[3:6]))
    # The same physical sanity gates fit() applies before a rig can be written.
    if float(np.linalg.norm(T_base_cam[:3, 3])) > ex.MAX_CAMERA_M:
        raise IdentifyError("the gated fit ran away: the camera came out %.1f m from the base"
                            % float(np.linalg.norm(T_base_cam[:3, 3])))
    if float(np.linalg.norm(theta[6:9])) > ex.MAX_TIP_M:
        raise IdentifyError("the gated fit put the tip %.0f mm from the flange"
                            % (1000.0 * float(np.linalg.norm(theta[6:9]))))

    per_pose = np.linalg.norm(residuals.reshape(-1, 2), axis=1)
    rms_px = float(np.sqrt(np.mean(per_pose ** 2)))
    ranges = [float(np.linalg.norm(T_base_cam[:3, 3] - model.tip_in_base(
                  i, theta[6:9], theta[9:15] if options.refine_zeros else None)))
              for i in range(len(observations))]
    focal = float(np.mean([np.asarray(K, float)[0, 0], np.asarray(K, float)[1, 1]]))
    metres_per_pixel = float(np.mean(ranges)) / focal if focal else 0.0
    rig = ex.Rig(
        T_base_cam=T_base_cam, calib_fingerprint=str(calib_fingerprint),
        image_size=tuple(int(v) for v in (options.image_size or (0, 0))),
        plane_z=float(options.plane_z), tip_offset=np.asarray(theta[6:9], dtype=float),
        joint_zeros=(np.asarray(theta[9:15], dtype=float) if options.refine_zeros else None),
        rms_px=rms_px, rms_mm=rms_px * metres_per_pixel * 1000.0,
        max_px=float(np.max(per_pose)), observations=observations, sigma=sigma,
        weakest=weakest, predicted=predicted, device=str(device), mount=str(mount),
        fitted=_stamp(), iterations=int(iterations), converged=bool(converged),
    )
    # The report stays the exploratory one: removing the weak directions from
    # the free set makes the *update's* covariance look healthy by construction,
    # but the data still does not constrain them -- that is what must be said.
    return {"rig": rig, "report": report,
            "converged": bool(converged), "free": list(free), "iterations": int(iterations)}


# --------------------------------------------------------------------------- #
# §7.3a Active extrinsics climb: candidate scoring
# --------------------------------------------------------------------------- #

def candidate_poses(current_q, fit_result, K, dist, tip, seed=11, perturbations=24, top_k=8):
    """The top-k poses that most reduce the fit's weakest directions.

    Pool = sweep.POSES plus seeded perturbations, filtered by `pose_ok` and a
    `path_safe` chain from where the arm is now. Each survivor is scored by the
    covariance reduction its tip observation would add, projected onto the
    weakest eigendirections of the current fit, with a small D-optimality
    (log-det information) term to break ties -- task residuals alone are not
    persistently exciting, so the climb goes where the *uncertainty* is.
    """
    rng = np.random.default_rng(seed)
    base = [np.asarray(p, dtype=float) for p in sweep.POSES]
    pool = list(base)
    for _ in range(int(perturbations)):
        q = base[int(rng.integers(len(base)))]
        mask = rng.random(6) < 0.5
        pool.append(q + rng.normal(0.0, 0.12, 6) * mask)

    rig = fit_result.rig
    T_cam_base = ex.invert(np.asarray(rig.T_base_cam, dtype=float))
    safe = []
    previous = np.asarray(current_q, dtype=float).reshape(6)
    for q in pool:
        if not sweep.pose_ok(q) or not sweep.path_safe(previous, q):
            continue
        T = rebot_ik.forward(q)
        point = T[:3, :3] @ np.asarray(tip, dtype=float) + T[:3, 3]
        if ex.pixel(point, K, dist, T_cam_base) is None:
            continue        # behind the lens: no observation, no information
        safe.append(q)
        previous = q

    theta = _rig_theta(rig)
    free = [j for j in fit_result.free if j not in (9, 14)]
    covariance = np.asarray(fit_result.covariance, dtype=float)
    values, vectors = np.linalg.eigh(covariance)
    weak_dirs = vectors[:, np.argsort(-values)[:3]]
    information = np.linalg.pinv(covariance)
    logdet0 = float(np.linalg.slogdet(information)[1])
    fit_options = ex.FitOptions(refine_tip=True, refine_zeros=any(j >= 10 for j in free))

    scored = []
    for q in safe:
        model = ex._Model([ex.Observation(q=q, uv=(0.0, 0.0))], K, dist, fit_options)
        model.free = list(free)
        J = model.jacobian(theta)             # 2 x n_free, pixel per parameter
        added = information + J.T @ J         # ~1 px click noise
        C_new = np.linalg.pinv(added)
        reduction = sum(float(d @ (covariance - C_new) @ d) for d in weak_dirs.T)
        dopt = float(np.linalg.slogdet(added)[1]) - logdet0
        scored.append((reduction + 1e-3 * dopt, q))
    scored.sort(key=lambda item: -item[0])

    # Re-check the chain in score order: picking the top-k must not reorder the
    # legs into a path the capsule model never approved.
    chosen = []
    previous = np.asarray(current_q, dtype=float).reshape(6)
    for _score, q in scored:
        if len(chosen) >= int(top_k):
            break
        if not sweep.path_safe(previous, q):
            continue
        chosen.append(q)
        previous = q
    return chosen


def verify_candidate(rig, fit_observations, holdout_observations, calibration, options,
                     converged=True):
    """auto_calibrate.verify's gates PLUS poses the fit never saw (§7.3a)."""
    verdict = auto_calibrate.verify(rig, fit_observations, calibration, options, converged)
    reasons = list(verdict["reasons"])
    K, dist = calibration.K, calibration.dist
    holdout_px = []
    for observation in holdout_observations:
        T = rebot_ik.forward(np.asarray(observation.q, dtype=float))
        point = T[:3, :3] @ np.asarray(rig.tip_offset, dtype=float) + T[:3, 3]
        seen = ex.pixel(point, K, dist, ex.invert(np.asarray(rig.T_base_cam, dtype=float)))
        holdout_px.append(float("inf") if seen is None else
                          math.hypot(seen[0] - observation.uv[0], seen[1] - observation.uv[1]))
    if not holdout_observations:
        reasons.append("no hold-out poses: the climb must verify on poses the fit never saw")
    elif float(np.max(holdout_px)) > HOLDOUT_MAX_PX:
        reasons.append("the worst hold-out pose reprojects %.1f px (limit %.0f)"
                       % (float(np.max(holdout_px)), HOLDOUT_MAX_PX))
    return dict(verdict, ok=not reasons, reasons=reasons,
                holdout_px=[round(float(v), 2) for v in holdout_px])


def climb_session(arm, eye, calibration, options, captures_dir, recorder, current_rig,
                  calib_dir, progress=None, top_k=6, holdout=2, gate_mm=SIGMA_GATE_MM,
                  seed=11, mount="fixed", turn=0):
    """One gated identification excursion. Returns a report; never hot-loads.

    The arm moves only through `auto_calibrate.run` (one `/move` trajectory with
    holds, video + PoseLog exactly as the commissioning sweep), whose legs are
    timed by `sweep.seconds_for` and re-checked by the gateway. The candidate is
    written as `extrinsics-candidate-<ts>.json` and committed to the standard
    rig path only if verification passes *and* every direction was identifiable;
    a partial or refused climb leaves the running rig untouched. The caller
    (capture_server) does the hot-load, exactly as it does for /calibrate.
    """
    if current_rig is None or not current_rig.observations:
        raise IdentifyError("the climb refines an existing rig: fit one first (a sweep or the "
                            "base marker); there is nothing to score candidates against")
    K, dist = calibration.K, calibration.dist
    start = arm.pose()
    q_start = np.asarray(start["q"][:6], dtype=float)
    fit_options = ex.FitOptions(plane_z=float(options.plane_z),
                                image_size=tuple(int(v) for v in calibration.image_size),
                                work_radius=float(options.work_radius), max_weak_mm=1e9)
    base_fit = ex.fit(current_rig.observations, K, dist, fit_options,
                      calib_fingerprint=calibration.fingerprint, device=calibration.device)
    chosen = candidate_poses(q_start, base_fit, K, dist, current_rig.tip_offset,
                             seed=seed, top_k=int(top_k) + int(holdout))
    if len(chosen) < ex.MIN_POSES + int(holdout):
        raise IdentifyError("only %d safe, visible candidate poses from here: park the arm and "
                            "try again (need %d)" % (len(chosen), ex.MIN_POSES + int(holdout)))
    fit_poses = chosen[:int(top_k)]
    holdout_poses = chosen[int(top_k):]

    result = auto_calibrate.run(arm, eye, calibration, options, captures_dir,
                                recorder=recorder, mount=mount, progress=progress,
                                poses=fit_poses + holdout_poses,
                                seed_T=np.asarray(current_rig.T_base_cam, dtype=float),
                                turn=turn)
    fit_obs = result.observations[:int(top_k)]
    holdout_obs = result.observations[int(top_k):]
    gated = gated_fit(fit_obs, K, dist, fit_options, prior_rig=current_rig, gate_mm=gate_mm,
                      calib_fingerprint=calibration.fingerprint, device=calibration.device,
                      mount=mount)
    candidate = gated["rig"]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate_path = Path(calib_dir) / ("extrinsics-candidate-%s.json" % stamp)
    ex.save_rig(candidate, candidate_path)

    verdict = verify_candidate(candidate, fit_obs, holdout_obs, calibration, options,
                               gated["converged"])
    partial = gated["report"]["status"] != "ok"
    ok = bool(verdict["ok"]) and not partial
    report = {
        "ok": ok,
        "partial": partial,
        "poses": len(chosen),
        "holdout": len(holdout_obs),
        "seconds": round(result.seconds, 1),
        "rig": candidate.provenance(),
        "verification": verdict,
        "identifiability": gated["report"],
        "candidate": str(candidate_path),
        "written": None,
        "warnings": list(result.warnings),
    }
    if partial:
        report["error"] = ("partial identification: %s flagged for excitation, not updated"
                           % ", ".join(f["name"] for f in gated["report"]["flagged"]))
    if not ok:
        return report                       # old rig stays; the candidate is evidence

    rig_path = Path(calib_dir) / ("extrinsics-%s.json" % mount)
    auto_calibrate.attach_witness(candidate, result, calibration, rig_path)
    ex.save_rig(candidate, rig_path)
    report["written"] = str(rig_path)
    return report


# --------------------------------------------------------------------------- #
# §7.3b Sag / gain residual map: learned from telemetry, no extra motion
# --------------------------------------------------------------------------- #

def sag_holds(telemetry_docs, kp=None):
    """Per-hold residual samples from execution telemetry.

    Each doc is either a raw telemetry log or an oracle doc carrying one. Per
    hold window: sag_meas = cmd - meas, and the residual is what the gravity
    model (sweep.gravity_sag = tau_g/kp) did NOT explain. The feature is the raw
    gravity torque, so the affine correction is in rad per Nm.
    """
    default_kp = sweep.kp_vector() if kp is None else np.asarray(kp, dtype=float).reshape(6)
    holds = []
    for doc in telemetry_docs or []:
        telemetry = doc.get("telemetry") if isinstance(doc.get("telemetry"), dict) else doc
        samples = telemetry.get("samples") or []
        if not samples:
            continue
        gains = default_kp
        if isinstance(telemetry.get("kp"), dict):
            try:
                gains = np.asarray([float(telemetry["kp"][str(m)]) for m in range(1, 7)])
            except (KeyError, TypeError, ValueError):
                pass
        waypoints = [{"t": s.get("t"), "q": s.get("cmd")} for s in samples]
        for window in oracle.hold_windows(waypoints):
            inside = [s for s in samples
                      if window["t_start"] - 1e-6 <= float(s.get("t", -1))
                      <= window["t_end"] + 1e-6]
            if not inside:
                continue
            q_cmd = np.mean([s["cmd"] for s in inside], axis=0)[:6]
            q_meas = np.mean([s["meas"] for s in inside], axis=0)[:6]
            sag_meas = q_cmd - q_meas
            residual = sag_meas - sweep.gravity_sag(q_cmd, gains)
            tip = rebot_ik.forward(q_cmd)[:3, 3]
            holds.append({"q": q_cmd, "tau": rebot_ik.gravity_torques(q_cmd),
                          "r": residual, "tip": tip})
    return holds


def _huber_affine(x, y, delta=SAG_HUBER_RAD, iterations=20):
    """IRLS affine fit y ~ a*x + b; a stalled hold is down-weighted, not followed."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    A = np.column_stack([x, np.ones_like(x)])
    coef = np.linalg.lstsq(A, y, rcond=None)[0]
    for _ in range(int(iterations)):
        residual = y - A @ coef
        scale = np.where(np.abs(residual) > delta,
                         delta / np.maximum(np.abs(residual), 1e-12), 1.0)
        root = np.sqrt(scale)
        coef = np.linalg.lstsq(A * root[:, None], y * root, rcond=None)[0]
    return float(coef[0]), float(coef[1])


def _per_joint_rms(holds, per_motor):
    if not holds:
        return 0.0
    total = 0.0
    for hold in holds:
        for j in range(6):
            motor = per_motor[str(j + 1)]
            predicted = motor["a"] * hold["tau"][j] + motor["b"]
            total += (hold["r"][j] - predicted) ** 2
    return 1000.0 * math.sqrt(total / (len(holds) * 6))


def _tool_mm_per_rad(q):
    """Tip sensitivity per joint at a pose, by finite difference of the FK."""
    q = np.asarray(q, dtype=float).reshape(6)
    base = rebot_ik.forward(q)[:3, 3]
    out = []
    for j in range(6):
        moved = q.copy()
        moved[j] += 1e-4
        out.append(float(np.linalg.norm(rebot_ik.forward(moved)[:3, 3] - base)) / 1e-4)
    return np.asarray(out)


def fit_sag_map(holds, kp_key, rig_fingerprint, zeros_key="none", min_holds=SAG_MIN_HOLDS):
    """Fit the per-joint affine residual map, with a hold-out validation gate.

    Refuses below `min_holds`, refuses holds that do not span the workspace,
    and refuses a map whose held-out rms is worse than the train rms by more
    than SAG_VALIDATE_RATIO (or above SAG_VALIDATE_MAX_MRAD outright) -- that is
    an overfit map, and applying it would make the arm worse, not better.
    """
    reasons = []
    if len(holds) < int(min_holds):
        reasons.append("only %d holds (need %d): let more executions accumulate"
                       % (len(holds), int(min_holds)))
    tips = np.asarray([h["tip"] for h in holds], dtype=float).reshape(-1, 3)
    radius = np.hypot(tips[:, 0], tips[:, 1])
    if len(holds) and (float(radius.max() - radius.min()) < SAG_SPAN_RADIUS_M
                       or float(tips[:, 2].max() - tips[:, 2].min()) < SAG_SPAN_HEIGHT_M):
        reasons.append("the holds do not span the workspace (radius span %.2f m, height %.2f m)"
                       % (float(radius.max() - radius.min()) if len(holds) else 0.0,
                          float(tips[:, 2].max() - tips[:, 2].min()) if len(holds) else 0.0))
    if reasons:
        return {"ok": False, "reasons": reasons}

    # Deterministic split: every fifth hold is held out, so the validation is
    # reproducible and the train set still spans the same envelope.
    train = [h for i, h in enumerate(holds) if i % 5 != 4]
    test = [h for i, h in enumerate(holds) if i % 5 == 4]
    per_motor = {}
    for j in range(6):
        a, b = _huber_affine([h["tau"][j] for h in train], [h["r"][j] for h in train])
        per_motor[str(j + 1)] = {"a": a, "b": b}
    train_rms = _per_joint_rms(train, per_motor)
    test_rms = _per_joint_rms(test, per_motor)
    if test_rms > SAG_VALIDATE_MAX_MRAD or test_rms > max(SAG_VALIDATE_RATIO * train_rms, 1.0):
        return {"ok": False,
                "reasons": ["hold-out validation failed: %.1f mrad held out vs %.1f mrad train "
                            "(an overfit map is not applied)" % (test_rms, train_rms)],
                "rms_mrad": round(test_rms, 3)}

    q_all = np.asarray([h["q"] for h in holds], dtype=float)
    envelope = {
        "q_min": [float(v) for v in q_all.min(axis=0)],
        "q_max": [float(v) for v in q_all.max(axis=0)],
        "radius_m": [float(radius.min()), float(radius.max())],
        "height_m": [float(tips[:, 2].min()), float(tips[:, 2].max())],
    }
    # mm at the tool: per-joint hold-out rms times the FK sensitivity, summed in
    # quadrature at the mean pose. An approximation, labelled as one.
    sensitivity = _tool_mm_per_rad(q_all.mean(axis=0))
    per_joint = []
    for j in range(6):
        errors = [h["r"][j] - (per_motor[str(j + 1)]["a"] * h["tau"][j]
                               + per_motor[str(j + 1)]["b"]) for h in test]
        per_joint.append(math.sqrt(float(np.mean(np.square(errors)))) if errors else 0.0)
    validated_mm = 1000.0 * float(np.linalg.norm(np.asarray(per_joint) * sensitivity))
    return {
        "ok": True,
        "rms_mrad": round(test_rms, 3),
        "train_rms_mrad": round(train_rms, 3),
        "map": {
            "version": 1,
            "kp_key": str(kp_key),
            "rig_fingerprint": str(rig_fingerprint),
            "zeros_key": str(zeros_key),
            "per_motor": per_motor,
            "samples": len(holds),
            "rms_mrad": round(test_rms, 3),
            "validated_mm": round(validated_mm, 2),
            "envelope": envelope,
        },
    }


def zeros_key(rig):
    """The joint-zeros identity a sag map is keyed by (a zero change invalidates)."""
    return sweep.zeros_key(None if rig is None else rig.joint_zeros)


# The keying/application logic lives in sweep so planner_server can apply the
# map on /move without importing this module (OpenCV via auto_calibrate).
sag_correction = sweep.sag_correction
sag_map_path = sweep.sag_map_path
load_sag_map = sweep.load_sag_map


def save_sag_map(calib_dir, map_doc):
    path = sag_map_path(calib_dir, map_doc["rig_fingerprint"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(map_doc, indent=2) + "\n", encoding="utf-8")
    return path


# --------------------------------------------------------------------------- #
# §7.4 The accuracy envelope
# --------------------------------------------------------------------------- #

def accuracy_cell(point):
    """The radius x height x sector cell a tip point falls in."""
    x, y, z = (float(v) for v in np.asarray(point, dtype=float).reshape(3))
    radius = math.hypot(x, y)
    ri = min(max(bisect_right(ACCURACY_RADIUS_EDGES, radius) - 1, 0),
             len(ACCURACY_RADIUS_EDGES) - 2)
    hi = min(max(bisect_right(ACCURACY_HEIGHT_EDGES, z) - 1, 0),
             len(ACCURACY_HEIGHT_EDGES) - 2)
    sector = int(((math.atan2(y, x) + 2.0 * math.pi) % (2.0 * math.pi))
                 / (2.0 * math.pi / ACCURACY_SECTORS)) % ACCURACY_SECTORS
    return "r%d-h%d-s%d" % (ri, hi, sector)


def load_accuracy_map(path):
    path = Path(path)
    if not path.is_file():
        return {"version": 1, "cells": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def update_accuracy_map(path, oracle_doc, rig=None, threshold_mm=oracle.PIXEL_ERROR_MM):
    """Fold one oracle doc's holds into the map; called after the doc is written.

    A cell validates only with ACCURACY_MIN_N measured holds under threshold;
    a later over-threshold hold un-validates it, because the envelope is a
    measured claim and must stay honest. predicted_mm is the fit covariance's
    conservative corner bound -- per-cell prediction is not identifiable from
    holds alone, so the whole-grid bound is what is reported.
    """
    path = Path(path)
    try:
        doc = load_accuracy_map(path)
    except (json.JSONDecodeError, OSError):
        doc = {"version": 1, "cells": {}}
    cells = doc.setdefault("cells", {})
    predicted = (rig.predicted or {}).get("corner_mm") if rig is not None else None
    for hold in (oracle_doc or {}).get("holds") or []:
        error = hold.get("pixel_error_mm")
        tip = hold.get("tip_fk_cmd") or hold.get("tip_fk_meas")
        if error is None or tip is None:
            continue
        key = accuracy_cell(tip)
        cell = cells.setdefault(key, {"predicted_mm": None, "measured_mm": None,
                                      "n": 0, "n_good": 0, "validated_at": None})
        cell["n"] += 1
        error = float(error)
        cell["measured_mm"] = round(max(cell["measured_mm"] or 0.0, error), 2)
        if predicted is not None:
            cell["predicted_mm"] = predicted
        if error <= float(threshold_mm):
            cell["n_good"] += 1
            if cell["n_good"] >= ACCURACY_MIN_N and cell["validated_at"] is None:
                cell["validated_at"] = _stamp()
        else:
            cell["validated_at"] = None
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    return doc


def accuracy_warnings(map_doc, tips):
    """One warning per target whose cell has no validated accuracy (§7.4).

    A warning, never a refusal: the hard gates (reach band, min_z, IK) stay
    where they are; this only says the measured envelope does not cover the
    target yet.
    """
    cells = (map_doc or {}).get("cells") or {}
    warnings = []
    for index, tip in enumerate(tips, start=1):
        key = accuracy_cell(tip)
        cell = cells.get(key)
        if cell is None or not cell.get("validated_at"):
            warnings.append("target %d: cell %s is not validated (n=%s measured holds): "
                            "accuracy outside the measured envelope is not guaranteed"
                            % (index, key, (cell or {}).get("n", 0)))
    return warnings


def validated_fraction(map_doc):
    cells = (map_doc or {}).get("cells") or {}
    if not cells:
        return 0.0
    validated = sum(1 for cell in cells.values() if cell.get("validated_at"))
    return validated / len(cells)