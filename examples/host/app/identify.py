#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Continuous calibration: refit the tag rig, learn the sag map, cover the envelope.

Phase 4 of the trace-collector spec (§7). The oracle corpus says *where* the loop
is wrong; this module decides what may be updated, and drives the two gated
sessions that could fix it. Three rules from the spec are hard constraints, not
preferences:

* **Propose, never auto-run.** `TriggerMonitor` watches witness drift, the EWMA
  of oracle pixel error, tag reprojection growth, zero drift and rig staleness,
  and returns proposals. Running a session needs `POST /identify/run
  {confirm: true}` with the execute gate on and the `calibrating` single-flight
  flag clear.
* **The rig is arm-independent by construction (§7.2).** Camera extrinsics come
  from the base tag's PnP, which observes the camera against a printed square on
  the base -- never against the arm. So extrinsics are not coupled to joint zeros
  or the URDF, and the *"model becomes part of the calibration"* trap cannot
  open. The gate is the tag solve's own geometry (`verify_marker`: rms, side,
  grazing); a weak solve is REFUSED, not averaged in. Joint zeros are fixed at
  deployment and are never climbed from camera data: the oracle flags zero drift
  for re-zeroing, and nothing here absorbs it into the rig.
* **Verify before hot-load (§7.3a).** The candidate rig is written beside the
  live one as `extrinsics-candidate-<ts>.json` and only committed after every
  contributing solve passed `verify_marker` *plus* a held-out capture's tag
  reprojection. A refusal leaves the running rig untouched.

`refit_session` (§7.3a) takes repeated stills of the tag and averages the PnP
estimates. It moves nothing: no arm, no recorder, no sweep. The sag/gain residual
map (§7.3b) is learned from execution telemetry alone, and applied inside the
gravity-sag fixed point `sweep.aim` as an additive term still clipped by
`SAG_LIMIT_RAD`; it is keyed by rig fingerprint, kp and joint zeros, and is not
trusted outside the envelope it was fitted in.

`coverage_session` (§7.4) is the only arm motion here. It is controlled motion to
GATHER MEASUREMENTS -- not a rig-fit procedure, and it never writes a rig. Two
complementary forms, because they cover different things:

* joint-space (`coverage_poses`): safe poses from the sweep pool, chosen for the
  cells their tip lands in. The wrist diversity is what makes geometry
  observable, and every leg passes the sweep's own pose_ok/path_safe chain. It
  goes through `ArmGateway.move_sequence` -> the gateway's `/move` check chain.
* Cartesian (`cartesian_targets`): the unvalidated cells' centre points, planned
  and executed by the TASK planner exactly as a tap sequence is (/plan ->
  /execute, IK + peel + capsule checks). It targets cells directly instead of
  hoping a joint pose lands in one, and the plan records each marker's raw IK
  solution so the oracle can separate solver error from sag (the FK bridge:
  |target - FK(q_ik)| is the solver's, |FK(q_meas) - FK(q_cmd)| is the arm's).

Both forms' holds are measured by the same oracle that measures a task
execution. No new execute path, no gate changes.

The sag map and the accuracy map are the learned quantities, and both need
excitation: a sag direction with too few holds, or a cell nobody has measured, is
reported UNLEARNED (`unlearned`) and never extrapolated.

`plane_z` / surface probing (§7.3c) is DEFERRED: it is the only component that
intentionally contacts the environment and needs its own clearance review. No
contact probing is implemented here; do not add one without that review.
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
import base_marker
import extrinsics as ex
import oracle
import rebot_ik
import sweep


class IdentifyError(Exception):
    """The session cannot run, or the update is not identifiable; the message is for the operator."""


# --------------------------------------------------------------------------- #
# Gates -- one block, like oracle.py's thresholds. Single source: rebot_ik.LEARNING
# (world.arm_config learning group); apply_config re-syncs these on a table edit.
# --------------------------------------------------------------------------- #
RIG_MAX_AGE_DAYS = rebot_ik.LEARNING["rig_max_age_days"]      # a rig older than this is stale even without drift
TAG_GROWTH_PX = rebot_ik.LEARNING["tag_growth_px"]            # tag reprojection growth over the session baseline
EWMA_ALPHA = rebot_ik.LEARNING["ewma_alpha"]                  # weight of the newest oracle pixel error
MARKER_MIN_USABLE = rebot_ik.LEARNING["marker_min_usable"]    # §7.3a: fewer good solves than this is not an average
MARKER_SPREAD_MAX_MM = rebot_ik.LEARNING["marker_spread_max_mm"]  # solves disagreeing by more than this are not one camera
HOLDOUT_MAX_PX = rebot_ik.LEARNING["holdout_max_px"]          # §7.3a: the held-out capture's tag reprojection bound
TAG_PRIOR_DEPTH = rebot_ik.LEARNING["tag_prior_depth"]        # §7.3a: ~1% of range is a 4 cm tag's depth error at 1 m
SAG_MIN_HOLDS = rebot_ik.LEARNING["sag_min_holds"]            # §7.3b: fewer holds cannot span the workspace
SAG_HUBER_RAD = rebot_ik.LEARNING["sag_huber_rad"]            # 10 mrad: a stall is an outlier, not a measurement
SAG_VALIDATE_MAX_MRAD = rebot_ik.LEARNING["sag_validate_max_mrad"]  # hold-out rms above this is not a usable map
SAG_VALIDATE_RATIO = rebot_ik.LEARNING["sag_validate_ratio"]  # hold-out may be at most this times the train rms
SAG_SPAN_RADIUS_M = rebot_ik.LEARNING["sag_span_radius_m"]    # holds must span this much radius...
SAG_SPAN_HEIGHT_M = rebot_ik.LEARNING["sag_span_height_m"]    # ...and this much height, or they are one pose
ACCURACY_MIN_N = rebot_ik.LEARNING["accuracy_min_n"]          # §7.4: a cell validates only with 3 good holds
ACCURACY_RADIUS_EDGES = tuple(rebot_ik.LEARNING["accuracy_radius_edges"])
ACCURACY_HEIGHT_EDGES = tuple(rebot_ik.LEARNING["accuracy_height_edges"])
ACCURACY_SECTORS = rebot_ik.LEARNING["accuracy_sectors"]
COVERAGE_MAX_POSES = rebot_ik.LEARNING["coverage_max_poses"]  # §7.4: one trajectory's worth of measurement poses
COVERAGE_HOLD_S = rebot_ik.LEARNING["coverage_hold_s"]        # long enough for the oracle's two grabs at the middle
COVERAGE_PERTURBATIONS = rebot_ik.LEARNING["coverage_perturbations"]  # extra poses around sweep.POSES, to reach more cells
# Mirrors plan_to_target.Options.max_reach. Duplicated, not imported: identify.py
# runs in the capture image, which has no Pinocchio; the planner re-checks reach
# with its own value when it plans, so this only pre-filters obvious misses.
COVERAGE_MAX_REACH_M = rebot_ik.LEARNING["coverage_max_reach_m"]


def _stamp():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


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

    def __init__(self, pixel_mm=None, alpha=None):
        self.pixel_mm = float(oracle.PIXEL_ERROR_MM if pixel_mm is None else pixel_mm)
        self.alpha = float(EWMA_ALPHA if alpha is None else alpha)
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
        # §7.2: zero drift is a re-zeroing job, not a rig refit. It is proposed
        # separately so an operator is never sent to recalibrate a camera for an
        # error the arm's zeros caused.
        drift = zero_drift(doc)
        if drift is not None:
            proposals.append({"kind": drift["kind"], "reason": drift["detail"],
                              "action": drift["action"]})
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

def rig_identifiability(verdicts):
    """Which tag solves may become a rig, and which are refused (§7.2).

    `verdicts` is one `verify_marker` result per capture. A weak solve -- a tag
    too small to localise, or one seen nearly edge-on -- is REFUSED, not averaged
    in: averaging a bad solve with good ones does not cancel it, it spreads it
    through the rig. The report carries each solve's reprojection rms, tag side
    and grazing angle, so a marginal rig is visible rather than silent.

    The second half of the report is what this gate can never say. The tag is
    arm-independent, so it constrains the camera and nothing else: joint zeros
    and the tool offset are not observable from it, and claiming otherwise is the
    "model becomes part of the calibration" trap §7.2 exists to close.
    """
    usable, refused = [], []
    for index, verdict in enumerate(verdicts or [], start=1):
        verdict = verdict or {}
        marker = verdict.get("marker") or {}
        entry = {"capture": index, "rms_px": verdict.get("rms_px"),
                 "side_px": marker.get("side_px"), "grazing_deg": marker.get("grazing_deg"),
                 "camera_z_m": marker.get("camera_z_m")}
        if verdict.get("ok"):
            usable.append(entry)
        else:
            refused.append(dict(entry, reasons=list(verdict.get("reasons") or [])))
    return {
        "status": "ok" if len(usable) >= MARKER_MIN_USABLE else "refused",
        "captures": len(verdicts or []),
        "usable": usable,
        "refused": refused,
        "min_usable": MARKER_MIN_USABLE,
        "identifiable": ["camera extrinsics (T_base_cam)"],
        "not_identifiable": [
            "joint zeros: fixed at deployment; the tag cannot observe them",
            "tip offset: a known tool parameter (REBOT_TIP_MM), not fitted",
        ],
    }


def zero_drift(oracle_doc):
    """§7.2: joint-zero drift is FLAGGED FOR RE-ZEROING, never absorbed.

    The oracle's attribution already separates a constant per-joint bias across
    poses (`joint-zeros`) from a load-dependent one (`sag`). This turns that
    verdict into the operator-facing flag. Nothing in this module writes
    joint_zeros -- the tag rig carries none -- so a zero error cannot hide in the
    extrinsics, which is the whole point of an arm-independent rig.
    """
    problems = ((oracle_doc or {}).get("verdict") or {}).get("problems") or []
    flagged = [problem for problem in problems if problem.get("kind") == "joint-zeros"]
    if not flagged:
        return None
    return {"kind": "zero-drift",
            "layer": flagged[0].get("layer"),
            "detail": flagged[0].get("detail"),
            "action": "re-zero the arm (the deployment zero-calibration); refitting the rig "
                      "cannot fix this and must not be tried"}


def unlearned(sag_map=None, map_doc=None):
    """§7.2: what is NOT learned yet -- reported as a gap, never extrapolated.

    The sag map and the accuracy map are the only learned quantities here, and
    both need excitation. A map fitted from too few holds, and cells nobody has
    measured, are stated as gaps so no caller can mistake silence for coverage.
    """
    gaps = []
    if not sag_map:
        gaps.append({"quantity": "sag-map", "state": "unlearned",
                     "reason": "no residual map is fitted for this rig/kp/zeros key"})
    else:
        samples = int(sag_map.get("samples") or 0)
        if samples < SAG_MIN_HOLDS:
            gaps.append({"quantity": "sag-map", "state": "partial",
                         "reason": "%d holds, %d needed to span the workspace"
                                   % (samples, SAG_MIN_HOLDS)})
    cells = (map_doc or {}).get("cells") or {}
    total = ((len(ACCURACY_RADIUS_EDGES) - 1) * (len(ACCURACY_HEIGHT_EDGES) - 1)
             * ACCURACY_SECTORS)
    measured = sum(1 for cell in cells.values() if int(cell.get("n") or 0) > 0)
    validated = sum(1 for cell in cells.values() if cell.get("validated_at"))
    if validated < total:
        gaps.append({"quantity": "accuracy-map", "state": "unlearned",
                     "reason": "%d of %d cells validated (%d measured at all): accuracy "
                               "outside them is not guaranteed" % (validated, total, measured),
                     "validated": validated, "measured": measured, "cells": total})
    return gaps


# --------------------------------------------------------------------------- #
# §7.3a Camera rig refit: tag PnP, averaged, no arm motion
# --------------------------------------------------------------------------- #

def _rotation_to_quaternion(R):
    """wxyz quaternion from a rotation matrix, on the numerically safe branch."""
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def _quaternion_to_rotation(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def average_pnp(matrices):
    """The geodesic mean of several `T_cam_marker` estimates (§7.3a).

    Repeated stills of a static tag differ by corner-detection noise alone, so
    averaging cuts it. Rotation is averaged as quaternions by Markley's
    eigenvector method -- the exact geodesic mean. A component-wise average of
    rotation matrices is not a rotation, and an average of axis-angle vectors is
    not the geodesic mean, so neither is used here.
    """
    matrices = [np.asarray(T, dtype=float) for T in matrices]
    if len(matrices) == 1:
        return matrices[0].copy()
    quaternions = []
    for matrix in matrices:
        q = _rotation_to_quaternion(matrix[:3, :3])
        # q and -q are the same rotation. Flipping into the first one's hemisphere
        # stops the outer-product average from cancelling a cluster that happens
        # to straddle the sign.
        if quaternions and float(q @ quaternions[0]) < 0.0:
            q = -q
        quaternions.append(q)
    Q = np.asarray(quaternions, dtype=float)
    _values, vectors = np.linalg.eigh(Q.T @ Q)
    mean = vectors[:, -1]
    if float(mean @ Q[0]) < 0.0:
        mean = -mean
    out = np.eye(4)
    out[:3, :3] = _quaternion_to_rotation(mean / np.linalg.norm(mean))
    out[:3, 3] = np.mean([matrix[:3, 3] for matrix in matrices], axis=0)
    return out


def pnp_spread_mm(matrices):
    """How far the individual tag solves disagree, in mm (§7.3a).

    A static camera and a static tag give one answer plus noise. A spread beyond
    MARKER_SPREAD_MAX_MM means something moved mid-session -- the bracket, the
    tag, or the phone -- and averaging would hide it, so the session refuses.
    """
    if len(matrices) < 2:
        return 0.0
    centres = np.asarray([np.asarray(T, dtype=float)[:3, 3] for T in matrices])
    return float(1000.0 * np.max(np.linalg.norm(centres - centres.mean(axis=0), axis=1)))


def marker_reprojection(T_base_cam, corners, K, dist, marker):
    """Where a rig puts the tag's corners, against where they were detected.

    The tag's base placement is known, so a rig implies where its four corners
    must land in ANY still. Measured on a capture the average never saw, this is
    the check that averaging did not drift the camera (§7.3a's held-out gate).
    """
    import cv2

    T_base_marker = base_marker.base_from_marker(
        marker["at"], float(marker.get("yaw", 0.0)), mode="vertical")
    T_cam_marker = ex.invert(np.asarray(T_base_cam, dtype=float)) @ T_base_marker
    rvec, _jacobian = cv2.Rodrigues(T_cam_marker[:3, :3])
    points = base_marker.object_points(float(marker.get("size_m", base_marker.SIZE_M)))
    projected, _jacobian = cv2.projectPoints(
        points, rvec, T_cam_marker[:3, 3].reshape(3, 1),
        np.asarray(K, dtype=float), np.asarray(dist, dtype=float).reshape(-1))
    errors = np.linalg.norm(projected.reshape(-1, 2)
                            - np.asarray(corners, dtype=float).reshape(-1, 2), axis=1)
    return {"rms_px": round(float(np.sqrt(np.mean(errors ** 2))), 2),
            "worst_px": round(float(np.max(errors)), 2)}


def tag_prior_mm(rig, point):
    """The tag rig's own accuracy prior at a point, in mm (§7.3a, §7.4).

    A 4 cm tag solves to roughly 1% of range in depth (§7.3a), so the prior
    scales with the camera-to-point distance and nothing else. It is a PRIOR, not
    a measurement: the accuracy map's measured layer is what validates a cell,
    and the oracle catches a rig error regardless of how confident this looks.
    """
    if rig is None:
        return None
    camera = np.asarray(rig.T_base_cam, dtype=float)[:3, 3]
    range_m = float(np.linalg.norm(np.asarray(point, dtype=float).reshape(3) - camera))
    return round(TAG_PRIOR_DEPTH * range_m * 1000.0, 2)


def refit_session(eye, calibration, marker, calib_dir, plane_z=0.0, mount="fixed",
                  captures=5, holdout=1, settle_s=0.4, progress=None, q=None):
    """§7.3a: refit the camera rig from the base tag. NO ARM MOTION.

    Repeated stills of the tag, each gated by `verify_marker`; the survivors are
    averaged into a candidate rig written as `extrinsics-candidate-<ts>.json`.
    The candidate is committed only if a capture the average never saw still
    reprojects the tag within HOLDOUT_MAX_PX. A refusal leaves the running rig
    untouched and the candidate on disk as evidence; the caller (capture_server)
    does the hot-load, exactly as it does for /capture/pose.

    The tip offset is a known tool parameter and joint zeros come from the
    deployment zero-calibration (§7.2) -- neither is fitted here, and the tag
    could not observe them if it were.
    """
    if not isinstance(marker, dict) or marker.get("at") is None:
        raise IdentifyError("the base marker has no placement: set REBOT_MARKER_AT to its centre "
                            "and heading (see docs/base-marker.md)")
    captures, holdout = max(int(captures), 1), max(int(holdout), 0)
    started = time.time()
    solved, refused = [], []
    for index in range(captures + holdout):
        if progress is not None:
            progress(index + 1, "capture")
        if index:
            time.sleep(float(settle_s))
        frame = eye.read()
        if frame is None:
            refused.append({"capture": index + 1, "reasons": ["the camera returned no frame"]})
            continue
        try:
            run = auto_calibrate.solve_marker(frame, calibration, marker,
                                              plane_z=plane_z, mount=mount)
        except (auto_calibrate.AutoError, base_marker.MarkerError) as error:
            refused.append({"capture": index + 1, "reasons": [str(error)]})
            continue
        solved.append({"capture": index + 1, "run": run, "frame": frame})

    pool = [entry for entry in solved if entry["capture"] <= captures]
    held = [entry for entry in solved if entry["capture"] > captures]
    gate = rig_identifiability([entry["run"].verdict for entry in pool])
    usable = [entry for entry in pool if entry["run"].verdict["ok"]]
    if len(usable) < MARKER_MIN_USABLE:
        reasons = [reason for entry in gate["refused"] for reason in entry["reasons"]]
        reasons += [reason for entry in refused for reason in entry["reasons"]]
        raise IdentifyError("only %d of %d tag solves passed verify_marker (%d needed): %s"
                            % (len(usable), len(pool), MARKER_MIN_USABLE,
                               "; ".join(dict.fromkeys(reasons)) or "the tag was never detected"))

    matrices = [entry["run"].evidence["T_cam_marker"] for entry in usable]
    spread = pnp_spread_mm(matrices)
    if spread > MARKER_SPREAD_MAX_MM:
        raise IdentifyError("the tag solves disagree by %.1f mm (limit %.0f): the camera, the "
                            "bracket or the tag moved during the session, so there is no single "
                            "rig to average" % (spread, MARKER_SPREAD_MAX_MM))

    T_base_marker = base_marker.base_from_marker(
        marker["at"], float(marker.get("yaw", 0.0)), mode="vertical")
    T_base_cam = T_base_marker @ np.linalg.inv(average_pnp(matrices))
    if float(np.linalg.norm(T_base_cam[:3, 3])) > ex.MAX_CAMERA_M:
        raise IdentifyError("the averaged solve put the camera %.1f m from the base (limit %.0f)"
                            % (float(np.linalg.norm(T_base_cam[:3, 3])), ex.MAX_CAMERA_M))

    # The averaged rig's own evidence: the mean of what went into it. Gated again,
    # because a mean of marginal solves can still be marginal.
    evidence = {
        "rms_px": float(np.mean([float(e["run"].evidence["rms_px"]) for e in usable])),
        "side_px": float(np.mean([float(e["run"].evidence["side_px"]) for e in usable])),
        "grazing_deg": float(np.mean([float(e["run"].evidence["grazing_deg"]) for e in usable])),
        "id": usable[0]["run"].evidence.get("id"),
    }
    verdict = auto_calibrate.verify_marker(evidence, camera_at=T_base_cam[:3, 3])
    candidate = replace(usable[0]["run"].rig, T_base_cam=T_base_cam,
                        rms_px=evidence["rms_px"], max_px=evidence["rms_px"],
                        fitted=_stamp(), observations=[], method="base-marker")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate_path = Path(calib_dir) / ("extrinsics-candidate-%s.json" % stamp)
    candidate_path.parent.mkdir(parents=True, exist_ok=True)
    ex.save_rig(candidate, candidate_path)

    # The held-out gate: a capture the average never saw must still reproject the
    # tag under the averaged rig. This is what catches an average that drifted.
    holdout_px, warnings = [], []
    for entry in held:
        try:
            corners = base_marker.detect(entry["frame"],
                                         marker_id=(int(marker["id"]) if marker.get("id") is not None
                                                    else base_marker.MARKER_ID))
        except base_marker.MarkerError as error:
            warnings.append("held-out capture %d could not be checked (%s)"
                            % (entry["capture"], error))
            continue
        measured = marker_reprojection(T_base_cam, corners, calibration.K, calibration.dist,
                                       marker)
        holdout_px.append(measured)
    if not holdout_px:
        verdict = dict(verdict, ok=False,
                       reasons=list(verdict["reasons"]) + [
                           "no held-out capture could be checked: the refit must verify on a "
                           "still the average never saw"])
    elif max(entry["worst_px"] for entry in holdout_px) > HOLDOUT_MAX_PX:
        worst = max(entry["worst_px"] for entry in holdout_px)
        verdict = dict(verdict, ok=False,
                       reasons=list(verdict["reasons"]) + [
                           "the held-out capture's tag reprojects %.1f px off the averaged rig "
                           "(limit %.0f)" % (worst, HOLDOUT_MAX_PX)])

    report = {
        "ok": bool(verdict["ok"]),
        "method": "base-marker",
        "motion": False,
        "captures": captures + holdout,
        "usable": len(usable),
        "held_out": len(holdout_px),
        "spread_mm": round(spread, 2),
        "seconds": round(time.time() - started, 1),
        "rig": candidate.provenance(),
        "verification": verdict,
        "identifiability": gate,
        "holdout_px": holdout_px,
        "candidate": str(candidate_path),
        "written": None,
        "warnings": warnings,
    }
    if not report["ok"]:
        return report                          # old rig stays; the candidate is evidence

    rig_path = Path(calib_dir) / ("extrinsics-%s.json" % mount)
    run = auto_calibrate.MarkerRun(rig=candidate, verdict=verdict, evidence=evidence,
                                   seconds=report["seconds"], warnings=list(warnings))
    _rig, _verdict, written = auto_calibrate.commit_marker(
        run, calibration, rig_path, frame=usable[-1]["frame"], q=q)
    report["written"] = str(written) if written else None
    report["warnings"] = list(run.warnings)
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


def _huber_affine(x, y, delta=None, iterations=20):
    """IRLS affine fit y ~ a*x + b; a stalled hold is down-weighted, not followed."""
    delta = SAG_HUBER_RAD if delta is None else delta
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


def fit_sag_map(holds, kp_key, rig_fingerprint, zeros_key="none", min_holds=None):
    """Fit the per-joint affine residual map, with a hold-out validation gate.

    Refuses below `min_holds`, refuses holds that do not span the workspace,
    and refuses a map whose held-out rms is worse than the train rms by more
    than SAG_VALIDATE_RATIO (or above SAG_VALIDATE_MAX_MRAD outright) -- that is
    an overfit map, and applying it would make the arm worse, not better.
    """
    reasons = []
    min_holds = SAG_MIN_HOLDS if min_holds is None else min_holds
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
    tau_all = np.asarray([h["tau"] for h in holds], dtype=float)
    envelope = {
        "q_min": [float(v) for v in q_all.min(axis=0)],
        "q_max": [float(v) for v in q_all.max(axis=0)],
        "tau_min": [float(v) for v in tau_all.min(axis=0)],
        "tau_max": [float(v) for v in tau_all.max(axis=0)],
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
    text = json.dumps(map_doc, indent=2) + "\n"
    path = sag_map_path(calib_dir, map_doc["rig_fingerprint"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    (Path(calib_dir) / "sag-arm.json").write_text(text, encoding="utf-8")
    return path


def active_sag_map(calib_dir, kp_spec, rig_name="extrinsics-fixed.json"):
    """The sag map keyed to the active rig, or None (§7.2's unlearned report).

    The same keying `sweep.active_sag_extra` applies the map under, so what this
    reports is what the live /move path would actually use: a map fitted under a
    different rig, kp or zeros is None here exactly as it is inert there.
    """
    try:
        rig_doc = json.loads((Path(calib_dir) / rig_name).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    fingerprint = str(rig_doc.get("fingerprint") or "")
    if not fingerprint:
        return None
    return sweep.load_sag_map(calib_dir, fingerprint, kp_key=kp_spec,
                              zeros_key=sweep.zeros_key(rig_doc.get("joint_zeros")))


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


def update_accuracy_map(path, oracle_doc, rig=None, threshold_mm=None):
    """Fold one oracle doc's holds into the map; called after the doc is written.

    A cell validates only with ACCURACY_MIN_N measured holds under threshold;
    a later over-threshold hold un-validates it, because the envelope is a
    measured claim and must stay honest. predicted_mm is the TAG-RIG PRIOR at
    that cell's tip (§7.4): ~1% of range for a 4 cm tag, floored by the rig's own
    reprojection error. It is a prior, not a measurement -- the measured layer is
    what validates, and a cell with no measured holds stays unvalidated however
    confident the prior looks.
    """
    if threshold_mm is None:
        threshold_mm = oracle.PIXEL_ERROR_MM
    path = Path(path)
    try:
        doc = load_accuracy_map(path)
    except (json.JSONDecodeError, OSError):
        doc = {"version": 1, "cells": {}}
    cells = doc.setdefault("cells", {})
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
        predicted = tag_prior_mm(rig, tip)
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


def coverage_cells(map_doc):
    """Every cell of the grid, with how badly it still needs measuring (§7.4).

    need 3 = nobody has measured it at all; 2 = measured but fewer than
    ACCURACY_MIN_N holds; 1 = enough holds but at least one was over threshold,
    so it un-validated; 0 = validated. The grid is enumerated rather than read
    from the map's keys, because an absent cell and a never-visited one are the
    same gap and must both be reachable.
    """
    cells = (map_doc or {}).get("cells") or {}
    out = []
    for ri in range(len(ACCURACY_RADIUS_EDGES) - 1):
        for hi in range(len(ACCURACY_HEIGHT_EDGES) - 1):
            for sector in range(ACCURACY_SECTORS):
                key = "r%d-h%d-s%d" % (ri, hi, sector)
                cell = cells.get(key) or {}
                n = int(cell.get("n") or 0)
                if cell.get("validated_at"):
                    need = 0
                elif n >= ACCURACY_MIN_N:
                    need = 1
                elif n:
                    need = 2
                else:
                    need = 3
                out.append({"cell": key, "need": need, "n": n,
                            "measured_mm": cell.get("measured_mm"),
                            "radius_m": [ACCURACY_RADIUS_EDGES[ri], ACCURACY_RADIUS_EDGES[ri + 1]],
                            "height_m": [ACCURACY_HEIGHT_EDGES[hi], ACCURACY_HEIGHT_EDGES[hi + 1]]})
    return out


def coverage_poses(current_q, map_doc, tip, seed=11, top_k=None,
                   perturbations=None, per_cell=2):
    """The safe poses whose tip lands in the least-covered cells (§7.4).

    Pool = `sweep.POSES` plus seeded perturbations, filtered by `pose_ok` and a
    `path_safe` chain from where the arm is now -- the same filters the sweep
    uses, so a coverage pose is a pose the bench already considers safe. Each
    survivor is scored by `coverage_cells`' need for the cell its tip falls in.

    This is coverage OF THE GRID, not excitation of a fit: the score says which
    cells are empty, never which parameter directions are weak. There is no
    covariance and no Jacobian here, and a coverage session writes no rig -- it
    exists to give the oracle holds to measure, so the accuracy map's measured
    layer grows in the places nobody has been.
    """
    rng = np.random.default_rng(int(seed))
    top_k = COVERAGE_MAX_POSES if top_k is None else top_k
    perturbations = COVERAGE_PERTURBATIONS if perturbations is None else perturbations
    base = [np.asarray(pose, dtype=float) for pose in sweep.POSES]
    pool = list(base)
    for _ in range(int(perturbations)):
        pose = base[int(rng.integers(len(base)))]
        mask = rng.random(6) < 0.5
        pool.append(pose + rng.normal(0.0, 0.12, 6) * mask)

    tip = np.asarray(tip, dtype=float).reshape(3)
    # The gateway peels from rest before the first measurement leg, so the chain
    # the poses must be safe along starts at the lifted pose, not the parked one.
    safe, previous = [], sweep.unpark_pose(current_q)
    for pose in pool:
        if not sweep.pose_ok(pose) or not sweep.path_safe(previous, pose):
            continue
        T = rebot_ik.forward(pose)
        point = T[:3, :3] @ tip + T[:3, 3]
        safe.append({"q": pose, "tip": point, "cell": accuracy_cell(point)})
        previous = pose

    need = {entry["cell"]: entry["need"] for entry in coverage_cells(map_doc)}
    for entry in safe:
        entry["need"] = int(need.get(entry["cell"], 3))
    # Best-covered first would waste the trajectory on cells that are already
    # done; sort by need, then spread across cells so one trajectory measures as
    # many distinct gaps as it can reach.
    safe.sort(key=lambda entry: (-entry["need"], entry["cell"]))
    chosen, taken = [], {}
    for entry in safe:
        if len(chosen) >= int(top_k):
            break
        if taken.get(entry["cell"], 0) >= int(per_cell):
            continue
        if entry["need"] <= 0:
            continue        # already validated: measuring it again teaches nothing
        chosen.append(entry)
        taken[entry["cell"]] = taken.get(entry["cell"], 0) + 1

    # Re-check the chain in score order: picking the best cells must not reorder
    # the legs into a path the capsule model never approved.
    ordered, previous = [], sweep.unpark_pose(current_q)
    for entry in chosen:
        if not sweep.path_safe(previous, entry["q"]):
            continue
        ordered.append(entry)
        previous = entry["q"]
    return ordered


def coverage_plan(current_q, poses, gripper=0.0, hold_s=None):
    """The `/move` request a coverage session makes, and the cells it measures.

    Pure: the leg durations come from `sweep.seconds_for`, the same speed policy
    the gateway applies as its floor, so asking for exactly the policy value
    means the gateway cannot raise it and the oracle's hold windows are the ones
    this plan predicts. Nothing here moves the arm.

    The caller (capture_server) owns the orchestration, because the oracle hooks
    it needs are async: build the trajectory with a dry run, prepare the oracle
    from the REAL windows the gateway persisted, move, then finish. This module
    stays pure policy so both halves are unit-testable offline.
    """
    targets = [[float(v) for v in entry["q"]] for entry in poses]
    hold_s = COVERAGE_HOLD_S if hold_s is None else hold_s
    chain = [np.asarray(current_q, dtype=float).reshape(6)] + \
            [np.asarray(target, dtype=float) for target in targets]
    seconds = [sweep.seconds_for(start, end) for start, end in zip(chain, chain[1:])]
    return {
        "targets": targets,
        "seconds": [round(float(value), 3) for value in seconds],
        "hold": float(hold_s),
        "gripper": float(gripper),
        "cells": [entry["cell"] for entry in poses],
        "needs": [entry["need"] for entry in poses],
        "tips": [[float(v) for v in entry["tip"]] for entry in poses],
        "motion_s": round(float(sum(seconds)) + float(hold_s) * len(targets), 1),
        # §7.4: what this motion is for, stated so nobody mistakes it for a fit.
        "writes_rig": False,
        "purpose": "measure the unvalidated accuracy cells; the oracle validates them",
    }


def cartesian_targets(map_doc, top_k=None, min_z=None,
                      max_reach=None):
    """The centre points of the least-covered cells, as Cartesian targets (§7.4).

    The complement of `coverage_poses`: instead of FK-ing joint poses and hoping
    the tip lands in a gap, this aims AT the gap -- one point per unvalidated
    cell, at the cell's mid radius, mid height and mid sector angle, so
    `accuracy_cell(point)` round-trips to the cell it was chosen for. The task
    planner's IK then decides whether the arm can actually hold there; this
    function only applies the same reach band the planner enforces (min_z floor,
    max_reach) so obviously impossible cells are reported, not planned into.

    Returns (chosen, skipped): `chosen` is sorted by need (never-measured first)
    and capped at `top_k`, one point per cell; `skipped` counts the unvalidated
    cells the reach band excludes, so a session can say honestly which gaps it
    cannot reach from a Cartesian approach (the joint-space form may still reach
    some of them -- /move has no min_z floor, only the capsule check).

    Pure: no IK, no motion, no rig. A cell centre below the safety floor is
    NEVER clamped up into reach -- that would measure a different cell than the
    one the map says is unvalidated.
    """
    chosen, taken = [], set()
    top_k = COVERAGE_MAX_POSES if top_k is None else top_k
    min_z = sweep.DEFAULT_MIN_Z if min_z is None else min_z
    max_reach = COVERAGE_MAX_REACH_M if max_reach is None else max_reach
    skipped = {"below_min_z": 0, "beyond_reach": 0}
    entries = []
    for entry in coverage_cells(map_doc):
        if entry["need"] <= 0:
            continue        # validated: measuring it again teaches nothing
        r0, r1 = entry["radius_m"]
        h0, h1 = entry["height_m"]
        sector = int(entry["cell"].rsplit("-s", 1)[1])
        radius = 0.5 * (float(r0) + float(r1))
        height = 0.5 * (float(h0) + float(h1))
        angle = (sector + 0.5) * (2.0 * math.pi / ACCURACY_SECTORS)
        point = [radius * math.cos(angle), radius * math.sin(angle), height]
        if height < float(min_z) - 1e-9:
            skipped["below_min_z"] += 1
            continue
        if radius > float(max_reach):
            skipped["beyond_reach"] += 1
            continue
        entries.append({"cell": entry["cell"], "need": int(entry["need"]),
                        "point": [float(v) for v in point]})
    entries.sort(key=lambda item: (-item["need"], item["cell"]))
    for entry in entries:
        if len(chosen) >= int(top_k):
            break
        if entry["cell"] in taken:
            continue
        chosen.append(entry)
        taken.add(entry["cell"])
    return chosen, skipped
