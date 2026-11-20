#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Physical hillclimb on the whole-pose CAD projection error (spec phase 4).

The tip-only fit asks one point of the arm where it is. This module asks the
whole arm: the CAD visual meshes (Seeed's RS URDF, `cad/visuals.json`) are
posed by `rebot_ik.forward` at the *predicted held* joints, rendered through
the camera rig, and compared against the arm's lime covers segmented from the
recorded frames (`auto_calibrate.green_mask`). The cover set is the meshes
that empirically carry the arm's lime colour on this bench (COVER_MESHES,
selected from per-part coverage over eight diverse holds). The error is a
ONE-WAY contour chamfer in pixels -- measured green edge to predicted cover
surface, averaged over the holds of a cartesian run -- because the
segmentation sees only the lit, visible fraction of the covers.

What gets climbed:

* the sag residual map -- per joint, `sag_j = g_j * tau_g_j(q)/kp_j + b_j`,
  learned from the run's OWN TELEMETRY (the held-vs-commanded joint residual
  is directly observable; the same signal identify.fit_sag_map uses). The
  gains enter the prediction chain `q_cmd -> q_cmd - sag(q_cmd) -> FK -> CAD
  -> pixels`, so a wrong sag map moves the whole rendered arm.
* the camera rig -- a bounded SE(3) delta on the tag-fitted `T_base_cam`,
  climbed on the whole-pose projection error with the sag held at its
  telemetry estimate. Pixel error alone cannot identify the sag gains (a
  one-way surface objective is flat inside the covers), so the two signals
  are combined the way the branch's architecture separates them: telemetry
  for the joints, camera for the rig, one projection error validating both.

Joint zeros are NOT climbed (spec 7.2): the tag cannot observe them and a
constant sag bias is bounded to +-30 mrad and reported, never absorbed into a
zero. A sag direction whose gravity torque barely varies over the run's holds
is *unlearned* and frozen, not extrapolated.

The loop is hardware-in-the-loop and gated:

    plan cartesian run (/plan, Pinocchio IK + capsule checks)
      -> consent (--confirm) -> /execute through the planner's full gate
      -> MediaMTX records, pose log samples the edge at 15 Hz
      -> hold frames extracted, green covers segmented
      -> hillclimb rig + sag on the fit holds, validate on the held-out holds
      -> repeat with a fresh run until the error plateaus

Every accepted parameter change is traced with its error before/after, which
is what `--chart` draws. Candidates are written beside the live rig and are
never hot-loaded without passing the tag-reprojection gate and an explicit
commit; the running rig stays untouched until then.

Run inside the capture container (it has the planner, the recorder and the
camera on its network):

    docker compose exec -T capture python app/identify_cad.py --dry-run
    docker compose exec -T capture python app/identify_cad.py --confirm --rounds 4

The chart is rendered on the host from the trace (matplotlib lives there):

    python examples/host/app/identify_cad.py --chart <trace.json> --out chart.png
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import auto_calibrate  # noqa: E402
import base_marker  # noqa: E402
import calib as calib_mod  # noqa: E402
import camera as camera_mod  # noqa: E402
import extrinsics as ex  # noqa: E402
import locate_target as locator  # noqa: E402
import mediamtx  # noqa: E402
import rebot_ik  # noqa: E402
import sweep  # noqa: E402
import sweep_video  # noqa: E402

STATE = Path(os.environ.get("REBOT_STATE", HERE.parent / "state"))
CAD_DIR = HERE / "cad"


class IdentifyError(Exception):
    """The session cannot run, or a gate refused it; the message says which."""


# --------------------------------------------------------------------------- #
# CAD: the visual meshes, posed by the same chain the planner uses
# --------------------------------------------------------------------------- #

# rebot_ik.forward(with_links=True) returns link1..link6 then gripper_end.
LINK_FRAME = {"link1": 0, "link2": 1, "link3": 2, "link4": 3,
              "link5": 4, "link6": 5, "gripper_end": 6}

# The parts that carry the arm's lime colour ON THIS BENCH, selected from data:
# each of these meshes covered >= 5% of the measured green (median over eight
# diverse holds, min >= 2%) when rendered at the measured pose. The URDF's
# "green" files alone are not the whole story -- the link2/link3 shells and the
# cnc3/cnc7 brackets carry green plastic/anodising here too. The objective
# compares this set against the green segmentation, spread over link2, link3,
# link5 and the gripper, so the whole pose is constrained, not one point.
# Re-select with the per-part coverage diagnostic if the bench arm changes.
COVER_MESHES = ("pla2_black.STL", "pla2_green.STL",
                "pla3_black_without_seeed_badge.STL", "pla3_green.STL",
                "pla3_seeed_badge_with_counters.STL", "cnc3.STL",
                "pla5_green.STL", "pla7_green.STL", "cnc7.STL")


def load_stl(path):
    """Binary STL -> (N, 3, 3) float32 triangle vertices."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise IdentifyError("STL too short: %s" % path)
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) < 84 + count * 50:
        raise IdentifyError("STL truncated: %s (%d triangles)" % (path, count))
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(data, dtype=record, count=count, offset=84)
    return np.ascontiguousarray(triangles["v"], dtype=np.float32)


@dataclass
class Part:
    link: str
    frame: int | None          # None = base_link (identity)
    verts: np.ndarray          # (N, 3, 3) in the link frame
    cover: bool
    name: str = ""


class CadModel:
    """The URDF's visual geometry, grouped per link, green covers flagged."""

    def __init__(self, directory=CAD_DIR):
        manifest = json.loads((Path(directory) / "visuals.json").read_text(encoding="utf-8"))
        self.parts = []
        seen = {}
        for visual in manifest["visuals"]:
            name = Path(visual["mesh"]).name
            if name not in seen:
                seen[name] = load_stl(Path(directory) / manifest["mesh_dir"] / name)
            link = visual["link"]
            if link not in LINK_FRAME and link != "base_link":
                continue                     # gripper fingers: not green, skip
            if visual["xyz"] != [0.0, 0.0, 0.0] or visual["rpy"] != [0.0, 0.0, 0.0]:
                raise IdentifyError("visual %s has a non-zero origin; the renderer "
                                    "assumes the RS visuals sit on their link frame" % name)
            self.parts.append(Part(link=link, frame=LINK_FRAME.get(link),
                                   verts=seen[name], cover=name in COVER_MESHES, name=name))
        self.cover_parts = [p for p in self.parts if p.cover]
        if len(self.cover_parts) != len(COVER_MESHES):
            raise IdentifyError("expected %d cover meshes, found %d"
                                % (len(COVER_MESHES), len(self.cover_parts)))
        self.cover_tris = sum(len(p.verts) for p in self.cover_parts)

    def link_frames(self, q):
        _tool, links = rebot_ik.forward(np.asarray(q, float).reshape(6), with_links=True)
        return links


# --------------------------------------------------------------------------- #
# Rendering: software rasteriser, no OpenGL -- the container has only numpy/cv2
# --------------------------------------------------------------------------- #

def project_points(points_base, K, dist, T_cam_base):
    """Vectorised `extrinsics.pixel` for (N, 3); returns (N, 2) and a validity mask."""
    points_base = np.asarray(points_base, float).reshape(-1, 3)
    T = np.asarray(T_cam_base, float)
    cam = points_base @ T[:3, :3].T + T[:3, 3]
    z = cam[:, 2]
    valid = z > 1e-6
    zz = np.where(valid, z, 1.0)
    x, y = cam[:, 0] / zz, cam[:, 1] / zz
    k1, k2, p1, p2, k3 = ex._coefficients(dist)
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    K = np.asarray(K, float)
    uv = np.column_stack([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]])
    return uv, valid


def render_parts(model, parts, q, K, dist, T_cam_base, size, scale=1.0):
    """Rasterise the given parts' silhouette at `size` * scale. uint8 0/255."""
    width = max(8, int(round(size[0] * scale)))
    height = max(8, int(round(size[1] * scale)))
    links = model.link_frames(q)
    T = np.asarray(T_cam_base, float)
    polys = []
    for part in parts:
        frame = np.eye(4) if part.frame is None else links[part.frame]
        world = part.verts.reshape(-1, 3) @ frame[:3, :3].T + frame[:3, 3]
        uv, valid = project_points(world, K, dist, T)
        uv = uv.reshape(-1, 3, 2) * scale
        valid = valid.reshape(-1, 3)
        keep = valid.all(axis=1)
        tris = uv[keep]
        if not len(tris):
            continue
        lo = tris.min(axis=1)
        hi = tris.max(axis=1)
        inside = ((hi[:, 0] >= 0) & (lo[:, 0] < width)
                  & (hi[:, 1] >= 0) & (lo[:, 1] < height))
        tris = tris[inside]
        if len(tris):
            polys.extend(np.ascontiguousarray(tris, dtype=np.int32))
    mask = np.zeros((height, width), np.uint8)
    if polys:
        cv2.fillPoly(mask, polys, 255)
    return mask


def render_cover(model, q, K, dist, T_cam_base, size, scale=1.0):
    return render_parts(model, model.cover_parts, q, K, dist, T_cam_base, size, scale)


# --------------------------------------------------------------------------- #
# The objective: one-way contour chamfer, measured green -> predicted covers
# --------------------------------------------------------------------------- #

def mask_edges(mask):
    eroded = cv2.erode(mask, np.ones((3, 3), np.uint8))
    return (mask > 0) & (eroded == 0)


# The colour detector sees every lime object in the scene, not just the arm
# (a shelf, a toy). A measured blob this far from the predicted arm is not a
# correspondence -- the same rule auto_calibrate's GREEN_NEAREST applies to
# tip blobs. The gate is generous next to the tag prior's accuracy, so it
# removes background objects without constraining the climb; a frame whose
# arm blobs all fall outside it is reported unmeasurable, not fitted.
MEAS_GATE_PX = 80.0
# Held-out tag-reprojection bound for a climbed rig (spec 7.3a): the climb may
# move the rig off the tag prior by more than PnP noise, but not by centimetres.
CLIMB_TAG_MAX_PX = 3.0


def clean_measured(hold, model, theta, K, dist, T_prior, kp, size, scale):
    """Drop measured green blobs that sit nowhere near the predicted arm.

    Returns the number of blobs dropped. Uses the round's starting prediction
    only to decide *which blobs are the arm*; the error itself is still
    measured against the raw segmentation of those blobs.
    """
    if hold.meas_mask is None:
        return 0
    T_cam = ex.invert(rig_from_theta(theta, T_prior))
    q_pred = held_prediction(theta, hold.q_cmd, kp)
    pred = render_cover(model, q_pred, K, dist, T_cam, size, scale)
    dt = cv2.distanceTransform((pred == 0).astype(np.uint8), cv2.DIST_L2, 3)
    count, labels, stats, _cent = cv2.connectedComponentsWithStats(hold.meas_mask, 8)
    keep = np.zeros_like(hold.meas_mask)
    dropped = 0
    gate = MEAS_GATE_PX * scale
    for index in range(1, count):
        blob = labels == index
        if int(blob.sum()) < 25:
            continue
        edge = blob & (cv2.erode(blob.astype(np.uint8), np.ones((3, 3), np.uint8)) == 0)
        if not edge.any():
            continue
        if float(dt[edge].mean()) <= gate:
            keep[blob] = 255
        else:
            dropped += 1
    hold.meas_mask = keep
    hold.meas_edge = mask_edges(keep)
    hold.meas_dt = cv2.distanceTransform((keep == 0).astype(np.uint8), cv2.DIST_L2, 3)
    hold.green_px = int(keep.sum()) // 255
    return dropped


def mask_error(predicted, measured):
    """(chamfer px, IoU) or None when either side is too small to measure."""
    pe, me = predicted > 0, measured > 0
    if int(pe.sum()) < 40 or int(me.sum()) < 40:
        return None
    dt_m = cv2.distanceTransform((~me).astype(np.uint8), cv2.DIST_L2, 3)
    dt_p = cv2.distanceTransform((~pe).astype(np.uint8), cv2.DIST_L2, 3)
    e_p, e_m = mask_edges(predicted), mask_edges(measured)
    if not e_p.any() or not e_m.any():
        return None
    chamfer = 0.5 * (float(dt_m[e_p].mean()) + float(dt_p[e_m].mean()))
    iou = float((pe & me).sum()) / float((pe | me).sum())
    return chamfer, iou


def oneway_error(predicted, measured):
    """Mean distance from the measured green edge to the predicted surface."""
    pe, me = predicted > 0, measured > 0
    if int(pe.sum()) < 40 or int(me.sum()) < 40:
        return None
    e_m = mask_edges(measured)
    if not e_m.any():
        return None
    dt_p = cv2.distanceTransform((~pe).astype(np.uint8), cv2.DIST_L2, 3)
    return float(dt_p[e_m].mean())


# --------------------------------------------------------------------------- #
# Parameters: rig delta + sag affine, bounded, with identifiability gates
# --------------------------------------------------------------------------- #

PARAM_NAMES = (["rot_x", "rot_y", "rot_z", "tx", "ty", "tz"]
               + ["g%d" % j for j in range(1, 7)] + ["b%d" % j for j in range(1, 7)])
N_PARAMS = len(PARAM_NAMES)
#               rotation (rad)   translation (m)   sag gain      sag bias (rad)
STEP0 = np.array([0.0087] * 3 + [0.002] * 3 + [0.10] * 6 + [0.002] * 6)
FLOOR = np.array([0.00035] * 3 + [0.0001] * 3 + [0.005] * 6 + [0.0001] * 6)
LOWER = np.array([-0.14] * 3 + [-0.08] * 3 + [0.30] * 6 + [-0.03] * 6)
UPPER = np.array([+0.14] * 3 + [+0.08] * 3 + [2.50] * 6 + [+0.03] * 6)
# A joint whose gravity torque spans less than this over the run's holds cannot
# teach its sag gain: the excitation is not there (spec 7.2, "unlearned").
MIN_TORQUE_SPAN_NM = 0.05


def theta0():
    theta = np.zeros(N_PARAMS)
    theta[6:12] = 1.0                      # gains start at the pure gravity model
    return theta


def rig_from_theta(theta, T_prior):
    T = np.asarray(T_prior, float).copy()
    T[:3, :3] = T[:3, :3] @ ex.rodrigues(theta[:3])
    T[:3, 3] = T[:3, 3] + theta[3:6]
    return T


def sag_from_theta(theta, q, kp):
    """The climbed sag: g * tau_g/kp + b, clipped exactly like sweep.gravity_sag."""
    tau = rebot_ik.gravity_torques(np.asarray(q, float).reshape(6))
    sag = theta[6:12] * tau / np.asarray(kp, float) + theta[12:18]
    return np.clip(sag, -sweep.SAG_LIMIT_RAD, sweep.SAG_LIMIT_RAD)


def held_prediction(theta, q_cmd, kp):
    """held = command - sag: the sign measured on the reference arm (sweep.aim)."""
    return np.asarray(q_cmd, float) - sag_from_theta(theta, q_cmd, kp)


@dataclass
class Hold:
    """One measured stop of a cartesian run."""
    q_cmd: np.ndarray
    q_meas: np.ndarray
    frame: np.ndarray | None = None
    meas_mask: np.ndarray | None = None      # green segmentation, optimisation scale
    meas_edge: np.ndarray | None = None
    meas_dt: np.ndarray | None = None
    green_px: int = 0
    fit: bool = True
    note: str = ""

    def prepare(self, scale):
        if self.frame is None:
            return
        mask = auto_calibrate.green_mask(self.frame)
        self.green_px = int(mask.sum()) // 255
        if scale != 1.0:
            mask = cv2.resize(mask, (max(8, int(round(mask.shape[1] * scale))),
                                     max(8, int(round(mask.shape[0] * scale)))),
                              interpolation=cv2.INTER_AREA)
            mask = (mask > 127).astype(np.uint8) * 255
        self.meas_mask = mask
        self.meas_edge = mask_edges(mask)
        self.meas_dt = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 3)


class Objective:
    """Mean whole-pose projection error over a corpus of holds, at one scale."""

    def __init__(self, model, holds, K, dist, T_prior, kp, size, scale=0.5):
        self.model, self.holds = model, holds
        self.K, self.dist, self.T_prior, self.kp = K, dist, T_prior, kp
        self.size, self.scale = size, scale
        self.cache = {}
        self.evals = 0

    def per_hold(self, theta):
        """(one-way chamfer, IoU, two-way chamfer) per hold; None if unmeasurable.

        The climb error is ONE-WAY: measured green edge -> predicted cover
        surface. The segmentation sees only the lit, visible fraction of the
        covers, so the reverse direction (predicted -> measured) would punish
        the prediction for surfaces the camera cannot see, and its floor is
        segmentation noise, not geometry. Every measured green pixel must lie
        on the predicted arm; a wrong rig or sag moves the whole surface away
        from them, which is exactly the signal being climbed.
        """
        T_cam = ex.invert(rig_from_theta(theta, self.T_prior))
        out = []
        for hold in self.holds:
            if hold.meas_mask is None:
                out.append(None)
                continue
            q_pred = held_prediction(theta, hold.q_cmd, self.kp)
            pred = render_cover(self.model, q_pred, self.K, self.dist, T_cam,
                                self.size, self.scale)
            pe = pred > 0
            if int(pe.sum()) < 40 or not hold.meas_edge.any():
                out.append(None)
                continue
            dt_p = cv2.distanceTransform((~pe).astype(np.uint8), cv2.DIST_L2, 3)
            oneway = float(dt_p[hold.meas_edge].mean())
            e_p = mask_edges(pred)
            twoway = 0.5 * (oneway + float(hold.meas_dt[e_p].mean()))
            iou = float((pe & (hold.meas_mask > 0)).sum()) / float(
                (pe | (hold.meas_mask > 0)).sum())
            out.append((oneway, iou, twoway))
        return out

    def error(self, theta):
        key = np.asarray(theta, float).tobytes()
        if key in self.cache:
            return self.cache[key]
        self.evals += 1
        values = [v[0] for v in self.per_hold(theta) if v is not None]
        err = float(np.mean(values)) if values else float("inf")
        if len(self.cache) > 4096:
            self.cache.clear()
        self.cache[key] = err
        return err


def identifiability(holds, kp):
    """Which sag directions this corpus can teach: torque span per joint."""
    taus = np.array([rebot_ik.gravity_torques(h.q_cmd) for h in holds])
    span = taus.max(axis=0) - taus.min(axis=0) if len(taus) else np.zeros(6)
    frozen = []
    for j in range(6):
        if span[j] < MIN_TORQUE_SPAN_NM:
            frozen.extend([6 + j, 12 + j])
    return span, sorted(set(frozen))


def telemetry_sag_gains(holds, kp):
    """Per-joint sag gains from the run's own telemetry: held = cmd - g*tau_g/kp.

    The joint residual is directly observable (edge pose vs commanded waypoint),
    so the gains do not need the camera -- the same signal identify.fit_sag_map
    learns from, computed here per run. A joint whose modelled sag is under
    5 mrad in every hold cannot teach its gain: it stays at 1, unlearned.
    Returns (gains, per-joint sample counts, residual rms in mrad).
    """
    kp = np.asarray(kp, float)
    gains = np.ones(6)
    counts = np.zeros(6, int)
    residuals = [[] for _ in range(6)]
    for j in range(6):
        samples = []
        for h in holds:
            model_sag = rebot_ik.gravity_torques(h.q_cmd)[j] / kp[j]
            if abs(model_sag) < 0.005:
                continue
            measured = float(h.q_cmd[j] - h.q_meas[j])
            samples.append(measured / model_sag)
            residuals[j].append(measured - model_sag)
        counts[j] = len(samples)
        if len(samples) >= 3:
            gains[j] = float(np.clip(np.median(samples), LOWER[6 + j], UPPER[6 + j]))
    rms = [float(np.sqrt(np.mean(np.square(r)))) * 1000.0 if r else None
           for r in residuals]
    return gains, counts, rms


@dataclass
class Change:
    n: int
    param: str
    value_before: float
    value_after: float
    err_before: float
    err_after: float
    gain_px: float
    round: int = 0


def hillclimb(objective, theta, frozen=(), max_evals=1500, min_gain=0.03, round_no=0,
              val_objective=None, val_tolerance=0.01):
    """Coordinate pattern search; every accepted step is traced.

    Monotone by construction: a candidate is accepted only when the fit error
    drops by more than `min_gain` (above the rasterisation noise floor). With a
    `val_objective`, the held-out corpus must also not degrade by more than
    `val_tolerance` -- a step that only flatters the fit holds is refused on
    the spot, not rolled back after the fact. Steps halve on refusal and the
    search stops when every free step is under its floor.
    """
    theta = np.array(theta, float)
    steps = STEP0.copy()
    changes = []
    err = objective.error(theta)
    val_err = val_objective.error(theta) if val_objective is not None else None
    evals0 = objective.evals
    while objective.evals - evals0 < max_evals:
        accepted = 0
        for index in range(N_PARAMS):
            if index in frozen or steps[index] < FLOOR[index]:
                continue
            for sign in (+1.0, -1.0):
                candidate = theta.copy()
                candidate[index] = float(np.clip(candidate[index] + sign * steps[index],
                                                 LOWER[index], UPPER[index]))
                if candidate[index] == theta[index]:
                    continue
                trial = objective.error(candidate)
                if trial >= err - min_gain:
                    continue
                if val_objective is not None:
                    trial_val = val_objective.error(candidate)
                    if trial_val > val_err + val_tolerance:
                        continue
                else:
                    trial_val = None
                changes.append(Change(
                    n=len(changes) + 1, param=PARAM_NAMES[index],
                    value_before=float(theta[index]), value_after=float(candidate[index]),
                    err_before=float(err), err_after=float(trial),
                    gain_px=float(err - trial), round=round_no))
                theta, err = candidate, trial
                if trial_val is not None:
                    val_err = trial_val
                accepted += 1
                break
            else:
                steps[index] *= 0.5
        if accepted == 0:
            active = [i for i in range(N_PARAMS) if i not in frozen]
            if all(steps[i] < FLOOR[i] for i in active):
                break
    return theta, err, changes


# --------------------------------------------------------------------------- #
# The bench: planner, recorder, camera -- the same seams auto_calibrate uses
# --------------------------------------------------------------------------- #

class Bench:
    def __init__(self, planner=None, token=None, camera_url=None, recordings=None,
                 mtx_api=None, state=STATE):
        self.state = Path(state)
        self.calib_dir = Path(os.environ.get("REBOT_CALIB_DIR", self.state / "calib"))
        self.arm = auto_calibrate.ArmGateway(
            planner or os.environ.get("REBOT_PLANNER", "http://planner:8000"),
            token if token is not None else os.environ.get("REBOT_PLANNER_TOKEN", ""))
        self.camera_url = camera_url or os.environ.get("REBOT_CAMERA", "rtsp://mediamtx:8554/phone")
        self.recorder = mediamtx.Recorder(
            mtx_api or os.environ.get("REBOT_MEDIAMTX_API", "http://mediamtx:9997"),
            path=mediamtx.path_from_camera(self.camera_url) or "phone",
            recordings=Path(recordings or os.environ.get("REBOT_RECORDINGS",
                                                          self.state / "recordings")))
        self.marker = base_marker.parse_spec(os.environ.get("REBOT_MARKER_AT", ""),
                                             float(os.environ.get("REBOT_MARKER_SIZE", "0.04")))
        self.kp = sweep.kp_vector(os.environ.get("REBOT_KP", sweep.KP_DEFAULT))
        self.tip_m = float(os.environ.get("REBOT_TIP_MM", "35")) / 1000.0
        self.rig = ex.load_rig(self.calib_dir / "extrinsics-fixed.json")
        matches = sorted(self.calib_dir.glob("*.json"))
        calib_path = None
        for path in matches:
            if path.name.startswith("extrinsics"):
                continue
            try:
                candidate = calib_mod.load(path)
            except calib_mod.CalibError:
                continue
            if candidate.fingerprint == self.rig.calib_fingerprint:
                calib_path = path
                break
        if calib_path is None:
            raise IdentifyError("no intrinsics matching rig fingerprint %s in %s"
                                % (self.rig.calib_fingerprint, self.calib_dir))
        self.calibration = calib_mod.load(calib_path)
        self.calib_path = calib_path

    def grab(self):
        eye = camera_mod.open_source(self.camera_url)
        try:
            return eye.read()
        finally:
            eye.close()

    def grab_upright(self, turn, tries=12):
        """A still on the calibration grid, turned back. The phone renegotiates
        its stream size now and then (a 762x1356 window was observed mid-run);
        K belongs to the calibration grid, so wait for a frame that is on it."""
        expected = (int(self.calibration.image_size[0]), int(self.calibration.image_size[1]))
        for _ in range(max(1, tries)):
            frame = self.upright(self.grab(), turn)
            if (frame.shape[1], frame.shape[0]) == expected:
                return frame
            time.sleep(0.5)
        raise IdentifyError("the stream stayed off the calibration grid %dx%d; "
                            "cannot verify against the tag" % expected)

    def measure_turn(self, frame):
        """Quarter turns back to the calibration grid, decided by the base tag.

        The shape alone cannot tell 90 from 270 (locate_target.turn_back); the
        tag is a known point, so only the real correction lands the rig's
        prediction on it. Same test capture_server.measure_stream_turn runs.
        """
        size = (int(frame.shape[1]), int(frame.shape[0]))
        if calib_mod.turns_for_shapes(self.calibration.image_size, size) == 0:
            return 0, 0.0
        if self.marker is None:
            raise IdentifyError("the stream is a quarter turn from the calibration and "
                                "there is no base tag to decide which way")
        marker_id = base_marker.MARKER_ID if self.marker.get("id") is None else int(self.marker["id"])
        corners = base_marker.detect(frame, marker_id)
        centre = np.asarray(corners, float).reshape(-1, 2).mean(axis=0)
        predicted = ex.pixel(np.asarray(self.marker["at"], float), self.calibration.K,
                             self.calibration.dist, ex.invert(self.rig.T_base_cam))
        if predicted is None:
            raise IdentifyError("the rig projects the base tag behind the camera")
        best = None
        for turn in (1, 3):
            placed = calib_mod.map_pixel(predicted[0], predicted[1],
                                         self.calibration.image_size, (4 - turn) % 4)
            error = float(np.linalg.norm(np.asarray(placed) - centre))
            if best is None or error < best[1]:
                best = (turn, error)
        return best

    def upright(self, frame, turn):
        return locator.rotate_image(frame, turn) if turn else frame

    def plan_run(self, tips, hold_s=1.5):
        body = {"targets": [{"tip": [float(v) for v in point]} for point in tips],
                "gripper": "alternate", "tip_offset_m": self.tip_m,
                "options": {"hold_s": float(hold_s), "tool_axis": "z-", "min_z": sweep.DEFAULT_MIN_Z}}
        status, payload = self.arm._call("POST", "/plan", body, timeout=120)
        if status != 200:
            raise IdentifyError("the planner refused the run: %s"
                                % payload.get("error", status))
        return payload

    def execute_run(self, plan_id, workdir):
        """Record, log pose, execute, stop. Returns (segments, pose_log_path)."""
        workdir = Path(workdir)
        workdir.mkdir(parents=True, exist_ok=True)
        log_path = workdir / "poses.jsonl"
        started = time.time()
        self.recorder.enable()
        log = sweep_video.PoseLog(self.arm, log_path)
        log.start()
        try:
            status, payload = self.arm._call("POST", "/execute", {"id": plan_id}, timeout=900)
            if status != 200:
                raise IdentifyError("the edge refused the run: %s"
                                    % payload.get("error", status))
        finally:
            try:
                log.stop()
            except Exception as error:      # the recording is still worth keeping
                print("pose log stopped with: %s" % error, file=sys.stderr)
            self.recorder.disable()
        time.sleep(1.0)                     # let MediaMTX close the segment
        segments = self.recorder.segments(since=started)
        if not segments:
            raise IdentifyError("nothing was recorded: is the phone publishing?")
        return segments, log_path, payload


def extract_holds(segments, log_path, trajectory, turn, bench, workdir,
                  min_green=auto_calibrate.MIN_GREEN_PX):
    """The run's stops: (q_cmd from the plan, q_meas from the edge, frame).

    Holds are found by stillness in the pose log (sweep_video), matched forward
    to the nearest commanded waypoint -- the sag offset is tens of mrad, the
    markers are radians apart -- and the frame at the middle of each window is
    turned onto the calibration grid and segmented. Each segment is decoded
    once, in order: seeking by frame number is unreliable across containers
    (the same reason sweep_video.extract reads sequentially).
    """
    entries = sweep_video.load_log(log_path)
    times = np.asarray([e["t"] for e in entries], float)
    poses = np.asarray([e["q"] for e in entries], float)
    windows = sweep_video._holds(times, poses)
    waypoints = trajectory["waypoints"]
    wp_q = np.asarray([w["q"][:6] for w in waypoints], float)
    q_start = poses[0]
    pending, skipped = [], []
    pointer = 0
    for begin, end in windows:
        q_meas = poses[begin:end].mean(axis=0)
        if float(np.max(np.abs(q_meas - q_start))) < 0.10:
            continue                        # the start/return pose, not a marker
        if pointer >= len(wp_q):
            break
        offsets = np.max(np.abs(wp_q[pointer:] - q_meas), axis=1)
        local = int(np.argmin(offsets))
        index = pointer + local
        if offsets[local] > 0.25:
            skipped.append("a stop %.0f mrad from any commanded pose" % (1000 * offsets[local]))
            continue
        pointer = index + 1
        middle = 0.5 * (times[begin] + times[end - 1])
        pending.append({"q_cmd": wp_q[index].copy(), "q_meas": q_meas, "t": middle})

    # One sequential decode per segment; a wanted frame is kept on the way past.
    for segment in segments:
        capture = cv2.VideoCapture(str(segment.path))
        try:
            fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
            count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            if fps <= 0:
                continue
            duration = count / fps if count else float("inf")
            wanted = {}
            for item in pending:
                if "frame" in item:
                    continue
                offset = item["t"] - segment.start
                if 0.0 <= offset < duration:
                    wanted.setdefault(int(round(offset * fps)), []).append(item)
            if not wanted:
                continue
            index = 0
            while wanted:
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                for item in wanted.pop(index, []):
                    item["frame"] = frame
                index += 1
        finally:
            capture.release()

    holds = []
    expected = (int(bench.calibration.image_size[0]), int(bench.calibration.image_size[1]))
    for item in pending:
        if "frame" not in item:
            skipped.append("no recorded frame for the stop at t=%.1f" % item["t"])
            continue
        frame = bench.upright(item["frame"], turn)
        if (frame.shape[1], frame.shape[0]) != expected:
            skipped.append("stop at t=%.1f: frame %dx%d is not the calibration grid %dx%d "
                           "(the stream changed shape)" % (item["t"], frame.shape[1],
                                                           frame.shape[0], *expected))
            continue
        hold = Hold(q_cmd=item["q_cmd"], q_meas=item["q_meas"], frame=frame)
        mask = auto_calibrate.green_mask(frame)
        hold.green_px = int(mask.sum()) // 255
        if hold.green_px < min_green:
            skipped.append("stop at t=%.1f shows only %d green px" % (item["t"], hold.green_px))
            continue
        holds.append(hold)
    return holds, skipped


# The cartesian run: eight tip targets spanning the sweep's work envelope
# (radius 0.30-0.54 m, height 0.18-0.34 m, +-55 deg of sector), chosen for the
# tool-down wrist the marker sequence uses. The planner validates every marker
# (reach band, min_z, IK) and every leg (capsule model) before anything moves;
# a refused plan is a dry-run answer, not a surprise on the bench.
RUN_TIPS = (
    (0.34, +0.16, 0.30),
    (0.42, -0.20, 0.24),
    (0.44, +0.22, 0.32),
    (0.30, +0.05, 0.20),
    (0.32, -0.10, 0.18),
    (0.28, -0.30, 0.22),
    (0.46, +0.28, 0.34),
    (0.38, -0.32, 0.28),
)


def run_tips():
    return [np.asarray(point, float) for point in RUN_TIPS]


# --------------------------------------------------------------------------- #
# Session: rounds of execute -> measure -> climb -> validate
# --------------------------------------------------------------------------- #

def make_session_dir(state=STATE):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = Path(state) / "identify" / stamp
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def run_session(bench, model, rounds=4, confirm=False, scale=0.5, hold_s=1.5,
                max_evals=1500, plateau_px=0.05, session_dir=None):
    if not confirm:
        raise IdentifyError("arm motion needs --confirm (the deliberate gate); "
                            "use --dry-run to plan and preview without moving")
    health = bench.arm._call("GET", "/health", timeout=15)
    if health[0] != 200 or not health[1].get("execute"):
        raise IdentifyError("the planner has execution disabled (REBOT_EXECUTE=1)")
    edge = health[1].get("edge") or {}
    if not edge.get("ok") or not edge.get("can_up") or not edge.get("allow_execute"):
        raise IdentifyError("the edge is not ready to execute: %s" % edge)

    session = Path(session_dir) if session_dir else make_session_dir(bench.state)
    frame = bench.grab()
    turn, turn_err = bench.measure_turn(frame)
    print("stream turn: %d quarter turn(s), tag error %.1f px" % (turn, turn_err))
    upright = bench.grab_upright(turn)
    size = (upright.shape[1], upright.shape[0])
    K, dist = bench.calibration.K, bench.calibration.dist
    T_prior = np.asarray(bench.rig.T_base_cam, float)
    theta = theta0()
    # Start from the telemetry sag map when one is loaded for this rig/kp/zeros
    # (identify.fit_sag_map's schema): the climb only learns the residual the
    # arm-independent loop has not already explained.
    kp_key = os.environ.get("REBOT_KP", sweep.KP_DEFAULT)
    sag_prior = sweep.load_sag_map(bench.calib_dir, bench.rig.fingerprint,
                                   kp_key=kp_key, zeros_key=sweep.zeros_key(bench.rig.joint_zeros))
    if sag_prior:
        for j in range(6):
            entry = sag_prior["per_motor"][str(j + 1)]
            theta[6 + j] = float(np.clip(1.0 + entry["a"] * bench.kp[j], LOWER[6 + j], UPPER[6 + j]))
            theta[12 + j] = float(np.clip(entry["b"], LOWER[12 + j], UPPER[12 + j]))
        print("sag prior: loaded %s (%d samples)" % (sweep.sag_map_path(bench.calib_dir,
                                                                        bench.rig.fingerprint),
                                                     sag_prior.get("samples", 0)))
    tips = run_tips()
    trace = {
        "format": "rebot-identify-trace", "version": 1,
        "started": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "session": str(session), "objective": "whole-pose CAD cover one-way chamfer, px",
        "scale": scale, "kp": [float(v) for v in bench.kp],
        "rig_prior_fingerprint": bench.rig.fingerprint,
        "sag_prior": bool(sag_prior),
        "turns": turn, "turn_error_px": round(turn_err, 2),
        "tips": [[float(v) for v in point] for point in tips],
        "rounds": [], "params": PARAM_NAMES,
    }
    prev_all = None
    for round_no in range(1, rounds + 1):
        started = time.time()
        print("\n=== round %d/%d ===" % (round_no, rounds))
        plan = bench.plan_run(tips, hold_s=hold_s)
        plan_id = plan["id"]
        trajectory = json.loads(Path(plan["path"]).read_text(encoding="utf-8"))
        print("plan %s: %d markers, %.1f s" % (plan_id, plan["summary"]["markers"],
                                               plan["summary"]["duration_s"]))
        workdir = session / ("round-%d" % round_no)
        segments, log_path, executed = bench.execute_run(plan_id, workdir)
        holds, skipped = extract_holds(segments, log_path, trajectory, turn, bench, workdir)
        for note in skipped:
            print("  skipped: %s" % note)
        if len(holds) < 4:
            raise IdentifyError("round %d measured only %d holds; a partial corpus is "
                                "not a fit (need >= 4)" % (round_no, len(holds)))
        for index, hold in enumerate(holds):
            hold.fit = index % 2 == 0
            hold.prepare(scale)
        dropped = 0
        for hold in holds:
            dropped += clean_measured(hold, model, theta, K, dist, T_prior, bench.kp, size, scale)
        usable = [h for h in holds if h.meas_mask is not None and h.green_px >= 40]
        if dropped:
            print("  dropped %d background green blob(s) outside the %d px correspondence gate"
                  % (dropped, int(MEAS_GATE_PX)))
        fit = [h for h in usable if h.fit]
        val = [h for h in usable if not h.fit]
        if len(fit) < 2 or len(val) < 2:
            raise IdentifyError("round %d: fit/holdout split too small (%d/%d)"
                                % (round_no, len(fit), len(val)))
        span, frozen = identifiability(usable, bench.kp)
        # Sag comes from the run's telemetry (directly observable joint
        # residual); the projection climb then moves only the rig. Constant
        # biases stay at the telemetry prior: they are indistinguishable from
        # joint-zero drift in camera data, and zeros are never climbed (7.2).
        gains, counts, sag_rms = telemetry_sag_gains(usable, bench.kp)
        theta[6:12] = gains
        frozen = sorted(set(range(6, 18)))
        unlearned = ["j%d" % (j + 1) for j in range(6)
                     if counts[j] < 3 or span[j] < MIN_TORQUE_SPAN_NM]
        print("  %d holds (%d fit / %d holdout); telemetry sag gains %s (samples %s)"
              % (len(usable), len(fit), len(val), np.round(gains, 3), counts))
        print("  unlearned sag joints: %s" % (", ".join(unlearned) or "none"))

        obj_fit = Objective(model, fit, K, dist, T_prior, bench.kp, size, scale)
        obj_val = Objective(model, val, K, dist, T_prior, bench.kp, size, scale)
        obj_all = Objective(model, usable, K, dist, T_prior, bench.kp, size, scale)
        err_fit_before = obj_fit.error(theta)
        err_val_before = obj_val.error(theta)
        err_all_before = obj_all.error(theta)
        theta_start = theta.copy()
        theta, err_fit_after, changes = hillclimb(
            obj_fit, theta, frozen=frozen, max_evals=max_evals, round_no=round_no,
            val_objective=obj_val)
        err_val_after = obj_val.error(theta)
        err_all_after = obj_all.error(theta)
        # The holdout is the overfit guard: a round that only flatters its fit
        # holds is rolled back, and the rollback is reported, not hidden.
        accepted = err_val_after <= err_val_before + 0.02 and err_fit_after < err_fit_before
        if not accepted:
            theta = theta_start
            err_fit_after, err_val_after, err_all_after = (err_fit_before, err_val_before,
                                                           err_all_before)
        record = {
            "round": round_no, "plan_id": plan_id,
            "holds": len(usable), "fit_holds": len(fit), "val_holds": len(val),
            "skipped": skipped, "blobs_dropped": dropped,
            "unlearned": unlearned,
            "sag_gains": [float(v) for v in gains],
            "sag_samples": [int(v) for v in counts],
            "sag_rms_mrad": [None if v is None else round(v, 2) for v in sag_rms],
            "torque_span_nm": [round(float(v), 4) for v in span],
            "err_fit_before": round(err_fit_before, 4), "err_fit_after": round(err_fit_after, 4),
            "err_val_before": round(err_val_before, 4), "err_val_after": round(err_val_after, 4),
            "err_all_before": round(err_all_before, 4), "err_all_after": round(err_all_after, 4),
            "accepted": bool(accepted), "evals": obj_fit.evals,
            "changes": [vars(c) for c in changes],
            "theta_after": [float(v) for v in theta],
            "hold_q": [[float(v) for v in h.q_cmd] for h in usable],
            "per_hold_px": [None if v is None else round(v[0], 3)
                            for v in obj_all.per_hold(theta)],
            "seconds": round(time.time() - started, 1),
        }
        trace["rounds"].append(record)
        _save_round_overlay(bench, model, usable, theta, K, dist, T_prior, size,
                            turn, workdir)
        (session / "trace.json").write_text(json.dumps(trace, indent=1) + "\n", encoding="utf-8")
        print("  fit %.3f -> %.3f px, holdout %.3f -> %.3f px, all %.3f -> %.3f px, "
              "%d changes, %s" % (err_fit_before, err_fit_after, err_val_before, err_val_after,
                                  err_all_before, err_all_after, len(changes),
                                  "accepted" if accepted else "ROLLED BACK"))
        if prev_all is not None and abs(prev_all - err_all_after) < plateau_px and round_no >= 2:
            print("  plateau: run error moved %.3f px (< %.2f)" % (abs(prev_all - err_all_after),
                                                                   plateau_px))
            break
        prev_all = err_all_after

    trace["final"] = _finalise(bench, model, theta, trace, session, K, dist, T_prior, size, turn)
    (session / "trace.json").write_text(json.dumps(trace, indent=1) + "\n", encoding="utf-8")
    print("\nsession: %s" % session)
    print("final: %s" % json.dumps({k: trace["final"][k] for k in
                                    ("err_all_px", "tag_rms_px", "verify_ok", "candidate")}))
    return trace


def _save_round_overlay(bench, model, holds, theta, K, dist, T_prior, size, turn, workdir):
    """One diagnostic image: predicted green contour over the measured frame."""
    hold = holds[0]
    if hold.frame is None:
        return
    T_cam = ex.invert(rig_from_theta(theta, T_prior))
    q_pred = held_prediction(theta, hold.q_cmd, bench.kp)
    pred = render_cover(model, q_pred, K, dist, T_cam, size, 1.0)
    overlay = hold.frame.copy()
    contours, _ = cv2.findContours(pred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)
    meas = auto_calibrate.green_mask(hold.frame)
    contours, _ = cv2.findContours(meas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
    cv2.imwrite(str(Path(workdir) / "overlay.jpg"), overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])


def _finalise(bench, model, theta, trace, session, K, dist, T_prior, size, turn):
    """Verify-before-hotload: the climbed rig must still reproject the tag.

    The fresh tag solve must pass verify_marker (PnP quality), and the climbed
    rig's own corner reprojection must stay within the held-out bound of spec
    7.3a (3 px) -- the climb is allowed to move the rig off the tag prior by
    more than the PnP noise, but not by centimetres.
    """
    frame = bench.grab_upright(turn)
    T_climbed = rig_from_theta(theta, T_prior)
    result = {"theta": [float(v) for v in theta], "verify_ok": False}
    try:
        run = auto_calibrate.solve_marker(frame, bench.calibration, bench.marker,
                                          plane_z=float(bench.rig.plane_z))
        evidence = run.evidence
        marker_id = (base_marker.MARKER_ID if bench.marker.get("id") is None
                     else int(bench.marker["id"]))
        corners = np.asarray(base_marker.detect(frame, marker_id), float).reshape(-1, 2)
        object_pts = base_marker.object_points(float(bench.marker["size_m"]))
        from_marker = base_marker.base_from_marker(bench.marker["at"],
                                                   float(bench.marker.get("yaw", 0.0)),
                                                   mode="vertical")
        T_cam_marker = ex.invert(T_climbed) @ from_marker
        projected, valid = project_points(object_pts, K, dist, T_cam_marker)
        rms = float(np.sqrt(np.mean(np.sum((projected[valid] - corners[valid]) ** 2, axis=1))))
        side = float(np.mean(np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)))
        result.update(tag_rms_px=round(rms, 3), tag_side_px=round(side, 1),
                      tag_solve_rms_px=round(float(evidence["rms_px"]), 3),
                      tag_grazing_deg=round(float(evidence["grazing_deg"]), 1))
        gate = (rms <= CLIMB_TAG_MAX_PX
                and side >= auto_calibrate.MARKER_MIN_SIDE_PX
                and float(evidence["grazing_deg"]) <= auto_calibrate.MARKER_MAX_GRAZING_DEG
                and run.verdict["ok"])
        result["verify_ok"] = bool(gate)
        if not gate:
            result["verify_reasons"] = (run.verdict["reasons"]
                                        + ["climbed rig reprojects the tag %.2f px off" % rms])
    except (base_marker.MarkerError, auto_calibrate.AutoError, IdentifyError) as error:
        result["verify_reasons"] = ["the tag could not be re-read: %s" % error]

    # The candidate is written beside the live rig, never over it. Hot-load is
    # a separate, explicit commit once the gate above has passed.
    rig = ex.Rig(T_base_cam=T_climbed, calib_fingerprint=bench.rig.calib_fingerprint,
                 image_size=bench.rig.image_size, plane_z=bench.rig.plane_z,
                 tip_offset=np.asarray(bench.rig.tip_offset, float),
                 method="hillclimb", device=bench.rig.device, mount=bench.rig.mount,
                 fitted=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                 rms_px=float(result.get("tag_rms_px", 0.0)))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate = bench.calib_dir / ("extrinsics-candidate-%s.json" % stamp)
    ex.save_rig(rig, candidate)
    result["candidate"] = str(candidate)
    # Sag map in the spec 7.3(b) shape (identify.fit_sag_map's schema, so
    # sweep.load_sag_map / sag_correction can apply it): r = a*tau_g + b around
    # gravity_sag. Written as a candidate in the session dir, never hot-loaded.
    a = (theta[6:12] - 1.0) / bench.kp
    q_list = [q for r in trace["rounds"] for q in r.get("hold_q", [])]
    q_all = np.asarray(q_list, float) if q_list else None
    tips = np.asarray(trace["tips"], float)
    radius = np.hypot(tips[:, 0], tips[:, 1])
    envelope = {"radius_m": [float(radius.min()), float(radius.max())],
                "height_m": [float(tips[:, 2].min()), float(tips[:, 2].max())]}
    if q_all is not None and len(q_all):
        envelope["q_min"] = [float(v) for v in q_all.min(axis=0)]
        envelope["q_max"] = [float(v) for v in q_all.max(axis=0)]
    sag_map = {"version": 1, "kp_key": os.environ.get("REBOT_KP", sweep.KP_DEFAULT),
               "rig_fingerprint": rig.fingerprint,
               "zeros_key": sweep.zeros_key(bench.rig.joint_zeros),
               "per_motor": {str(j + 1): {"a": float(a[j]), "b": float(theta[12 + j])}
                             for j in range(6)},
               "samples": sum(r["holds"] for r in trace["rounds"]),
               "rms_mrad": None, "validated_mm": None,
               "envelope": envelope,
               "source": "identify_cad whole-pose projection hillclimb"}
    sag_path = session / ("sag-%s.json" % rig.fingerprint)
    sag_path.write_text(json.dumps(sag_map, indent=1) + "\n", encoding="utf-8")
    result["sag_map"] = str(sag_path)
    all_holds = sum(r["holds"] for r in trace["rounds"])
    result["err_all_px"] = trace["rounds"][-1]["err_all_after"] if trace["rounds"] else None
    result["holds_total"] = all_holds
    return result


# --------------------------------------------------------------------------- #
# Dry run: plan, preview the CAD pose at the arm's current joints, move nothing
# --------------------------------------------------------------------------- #

def dry_run(bench, model, session_dir=None):
    session = Path(session_dir) if session_dir else make_session_dir(bench.state)
    frame = bench.grab()
    turn, turn_err = bench.measure_turn(frame)
    upright = bench.grab_upright(turn)
    size = (upright.shape[1], upright.shape[0])
    pose = bench.arm.pose()
    q = np.asarray(pose["q"][:6], float)
    print("stream turn %d (tag error %.1f px); arm at [%s]"
          % (turn, turn_err, ", ".join("%+.3f" % v for v in q)))
    plan = bench.plan_run(run_tips())
    print("plan %s ok: %d markers, %.1f s, execute=%s -- NOT executed"
          % (plan["id"], plan["summary"]["markers"], plan["summary"]["duration_s"],
             plan.get("execute")))
    T_cam = ex.invert(bench.rig.T_base_cam)
    K, dist = bench.calibration.K, bench.calibration.dist
    # Whole-arm CAD overlay at the current pose: every visual mesh, not the tip.
    full = render_parts(model, model.parts, q, K, dist, T_cam, size, 1.0)
    overlay = upright.copy()
    green_fill = np.zeros_like(overlay)
    green_fill[full > 0] = (0, 200, 0)
    overlay = cv2.addWeighted(overlay, 1.0, green_fill, 0.35, 0)
    contours, _ = cv2.findContours(full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)
    cv2.imwrite(str(session / "cad-preview.jpg"), overlay, [cv2.IMWRITE_JPEG_QUALITY, 92])
    green = render_cover(model, q, K, dist, T_cam, size, 1.0)
    meas = auto_calibrate.green_mask(upright)
    hold = Hold(q_cmd=q, q_meas=q, frame=upright)
    hold.meas_mask = meas
    hold.meas_edge = mask_edges(meas)
    hold.meas_dt = cv2.distanceTransform((meas == 0).astype(np.uint8), cv2.DIST_L2, 3)
    dropped = clean_measured(hold, model, theta0(), K, dist, bench.rig.T_base_cam,
                             bench.kp, size, 1.0)
    err = oneway_error(green, hold.meas_mask)
    iou = mask_error(green, hold.meas_mask)
    report = {"turns": turn, "turn_error_px": round(turn_err, 2),
              "plan_id": plan["id"], "q": [float(v) for v in q],
              "green_px_measured": hold.green_px, "blobs_dropped": dropped,
              "green_px_predicted": int(green.sum()) // 255,
              "baseline_chamfer_px": None if err is None else round(err, 3),
              "baseline_iou": None if iou is None else round(iou[1], 4),
              "preview": str(session / "cad-preview.jpg")}
    (session / "dry-run.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print("baseline whole-pose error at the current pose: %s"
          % ("n/a (arm not visible enough)" if err is None
             else "%.3f px one-way chamfer, IoU %.3f" % (err, iou[1])))
    print("preview: %s" % report["preview"])
    return report


# --------------------------------------------------------------------------- #
# Chart: the incremental improvement from each accepted change
# --------------------------------------------------------------------------- #

def render_chart(trace_path, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    trace = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    rounds = trace["rounds"]
    if not rounds:
        raise IdentifyError("the trace has no rounds")
    fig, (ax, ax2, ax3) = plt.subplots(
        3, 1, figsize=(12, 9.5), gridspec_kw={"height_ratios": [2.6, 1.2, 1.2]})

    # Panel 1: the error after every accepted change, in order. Rolled-back
    # rounds contribute no steps -- their changes were reverted.
    xs, ys, colours = [0], [rounds[0]["err_fit_before"]], ["#444444"]
    x = 0
    boundaries = []
    for record in rounds:
        start = x
        if record["accepted"]:
            for change in record["changes"]:
                x += 1
                xs.append(x)
                ys.append(change["err_after"])
                rig_param = change["param"].startswith(("rot", "t"))
                colours.append("#1f77b4" if rig_param else "#d62728")
        boundaries.append((start, x, record))
    ax.step(xs, ys, where="post", color="#888888", linewidth=1.0, zorder=1)
    ax.scatter(xs[1:], ys[1:], c=colours[1:], s=26, zorder=3)
    ax.scatter([0], [ys[0]], c="#444444", s=40, marker="s", zorder=3, label="tag prior")
    top = max(ys) * 1.12
    ax.set_ylim(0, top)
    for start, end, record in boundaries:
        ax.axvspan(start + 0.5, end + 0.5, alpha=0.06,
                   color=("#2ca02c" if record["accepted"] else "#d62728"))
        ax.text((start + end) / 2 + 0.5, top * 0.97,
                "run %d (%d holds)%s" % (record["round"], record["holds"],
                                         "" if record["accepted"] else "\nrolled back"),
                ha="center", va="top", fontsize=8, color="#555555")
    ax.set_ylabel("fit-corpus projection error (px chamfer)")
    ax.set_xlabel("accepted change (blue = camera rig, red = sag map)")
    total = ys[0] - ys[-1]
    ax.set_title("Whole-pose CAD hillclimb: incremental improvement per change\n"
                 "%.3f px -> %.3f px  (-%.1f%% over %d accepted changes, %d hardware runs)"
                 % (ys[0], ys[-1], 100.0 * total / max(1e-9, ys[0]), len(xs) - 1, len(rounds)))
    ax.grid(alpha=0.25)
    ax.legend(loc="lower left", fontsize=8)

    # Panel 2: the gain of each accepted change -- the incremental view.
    gains, gcols, glabels = [], [], []
    for record in rounds:
        if not record["accepted"]:
            continue
        for change in record["changes"]:
            gains.append(change["gain_px"])
            gcols.append("#1f77b4" if change["param"].startswith(("rot", "t")) else "#d62728")
            glabels.append("%s %+.4g" % (change["param"],
                                         change["value_after"] - change["value_before"]))
    ax2.bar(range(1, len(gains) + 1), gains, color=gcols, width=0.8)
    ax2.set_ylabel("gain per change (px)")
    ax2.set_xlabel("accepted change index")
    ax2.grid(alpha=0.25, axis="y")
    if gains:
        best = int(np.argmax(gains))
        ax2.annotate(glabels[best], (best + 1, gains[best]),
                     textcoords="offset points", xytext=(0, 4), fontsize=7, ha="center")

    # Panel 3: the held-out run error, before/after each hardware round.
    numbers = [r["round"] for r in rounds]
    ax3.plot(numbers, [r["err_all_before"] for r in rounds], "o--", color="#2ca02c",
             label="all holds, before climb")
    ax3.plot(numbers, [r["err_all_after"] for r in rounds], "s-", color="#145214",
             label="all holds, after climb")
    ax3.plot(numbers, [r["err_val_before"] for r in rounds], "^:", color="#ff7f0e",
             label="holdout only, before")
    ax3.plot(numbers, [r["err_val_after"] for r in rounds], "v-", color="#8c4600",
             label="holdout only, after")
    ax3.set_ylabel("projection error (px)")
    ax3.set_xlabel("hardware run")
    ax3.set_xticks(numbers)
    ax3.grid(alpha=0.25)
    ax3.legend(loc="best", fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150)
    plt.close(fig)
    return str(out_path)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true",
                        help="plan the run and preview the CAD pose; move nothing")
    parser.add_argument("--confirm", action="store_true",
                        help="the deliberate gate: execute the runs on the arm")
    parser.add_argument("--rounds", type=int, default=4)
    parser.add_argument("--scale", type=float, default=0.5,
                        help="optimisation render scale (1 = full resolution)")
    parser.add_argument("--hold-s", type=float, default=1.5)
    parser.add_argument("--max-evals", type=int, default=1500)
    parser.add_argument("--session", default=None, help="reuse a session directory")
    parser.add_argument("--chart", metavar="TRACE", help="render the trace's chart and exit")
    parser.add_argument("--out", default="hillclimb-projection-error.png")
    args = parser.parse_args(argv)

    if args.chart:
        print("chart: %s" % render_chart(args.chart, args.out))
        return 0

    model = CadModel()
    print("CAD: %d visual parts, %d cover triangles"
          % (len(model.parts), model.cover_tris))
    bench = Bench()
    if args.dry_run or not args.confirm:
        dry_run(bench, model, session_dir=args.session)
        if not args.dry_run:
            print("\nnothing moved: pass --confirm to run the hillclimb on the arm")
        return 0
    run_session(bench, model, rounds=args.rounds, confirm=True, scale=args.scale,
                hold_s=args.hold_s, max_evals=args.max_evals, session_dir=args.session)
    return 0


if __name__ == "__main__":
    sys.exit(main())