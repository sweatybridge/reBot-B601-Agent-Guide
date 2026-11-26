"""Visual tip detection, frame orientation, and post-grasp verification, shared
by the capture server and the isolated workflow activity.

All functions here are pure functions of frames, joints, and camera/rig
calibration (OpenCV + numpy + rebot_ik), so they run identically in capture,
workflow activities, and offline tests without importing aiohttp or Pinocchio.
"""
from __future__ import annotations

import numpy as np

import auto_calibrate
import calib as calib_mod
import rebot_ik


# The B601 green gripper housing sits along the link6 -> gripper_end segment
# (`TOOL_XYZ = [0, 0, 0.16621]`), with its visible green centroid centered near
# ~48% of the segment (~80 mm from j6, ~86 mm behind the black fingertips).
_COLLAR_CENTER_FRAC = 0.48


def _project_point(p3, K, dist, T_base_cam):
    """Project one 3D base-frame point into pixels, or None if behind camera."""
    import cv2

    R = np.asarray(T_base_cam[:3, :3], dtype=float)
    t = np.asarray(T_base_cam[:3, 3], dtype=float)
    pc = R.T @ (np.asarray(p3, dtype=float).reshape(3) - t)
    if float(pc[2]) <= 0.05:
        return None
    uv, _ = cv2.projectPoints(
        pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
        np.asarray(K, dtype=float), np.asarray(dist, dtype=float).reshape(-1),
    )
    return uv.reshape(2)


def _arm_joint_pixels(q6, K, dist, T_base_cam):
    """Projected pixels for joints j1..j6, collar center, and gripper_end."""
    tool_T, frames = rebot_ik.forward(np.asarray(q6, dtype=float).reshape(6), with_frames=True)
    pts3d = [p for p, _z in frames]  # j1..j6 origins in base frame
    ee3d = tool_T[:3, 3]
    j6_3d = pts3d[5]
    collar3d = j6_3d + _COLLAR_CENTER_FRAC * (ee3d - j6_3d)
    joints_uv = [_project_point(p, K, dist, T_base_cam) for p in pts3d]
    ee_uv = _project_point(ee3d, K, dist, T_base_cam)
    collar_uv = _project_point(collar3d, K, dist, T_base_cam)
    return joints_uv, collar_uv, ee_uv


def _point_to_seg_dist(pts, a, b):
    """Per-point Euclidean distance from Nx2 `pts` to 2D segment a->b."""
    ab = b - a
    denom = float(ab @ ab)
    if denom <= 1e-12:
        return np.hypot(pts[:, 0] - a[0], pts[:, 1] - a[1])
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab[None, :]
    return np.hypot(pts[:, 0] - proj[:, 0], pts[:, 1] - proj[:, 1])


def _arm_green_support(frame, q6, K, dist, T_base_cam, radius_px=70.0):
    """Count green pixels lying near the projected arm skeleton (j2..ee)."""
    mask = auto_calibrate.green_mask(frame)
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return 0
    # Subsample dense masks for fast orientation scoring.
    step = max(1, len(xs) // 4000)
    pts = np.stack([xs[::step], ys[::step]], axis=1).astype(float)
    joints_uv, _collar_uv, ee_uv = _arm_joint_pixels(q6, K, dist, T_base_cam)
    chain = [p for p in joints_uv[1:] + [ee_uv] if p is not None]
    if len(chain) < 2:
        return 0
    min_d = np.full(len(pts), np.inf, dtype=float)
    for a, b in zip(chain[:-1], chain[1:]):
        d = _point_to_seg_dist(pts, a, b)
        min_d = np.minimum(min_d, d)
    return int((min_d <= radius_px).sum()) * step


def orient_frame(frame, q6, K, dist, T_base_cam, calib_size, hint_turn=None):
    """Rotate a landscape/portrait-mismatched frame onto the calibration grid.

    Returns `(upright_frame, turns, grid_scale)` where `turns` is in `{0, 1, 3}`
    (quarter turns clockwise) and `grid_scale = upright_w / calib_w`. When the
    frame aspect is swapped vs `calib_size` and `hint_turn` is None, scores
    CW90 (`1`) vs CCW90 (`3`) by green-arm overlap with the projected skeleton.
    """
    import cv2

    if frame is None:
        return None, 0, 1.0
    fh, fw = int(frame.shape[0]), int(frame.shape[1])
    if not calib_size or len(calib_size) < 2 or calib_size[0] <= 0 or calib_size[1] <= 0:
        return frame, 0, 1.0
    cw, ch = int(calib_size[0]), int(calib_size[1])
    if calib_mod.turns_for_shapes((cw, ch), (fw, fh)) == 0:
        return frame, 0, float(fw) / float(cw)

    # Frame is rotated +/- 90 degrees relative to calibration grid.
    upright_size = (fh, fw)
    scale = float(fh) / float(cw)
    K_scaled = calib_mod.scale_K(np.asarray(K, dtype=float), (cw, ch), upright_size)

    def _rot(t):
        if t == 1:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if t == 3:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    if hint_turn in (1, 3):
        return _rot(int(hint_turn)), int(hint_turn), scale

    c1 = _rot(1)
    c3 = _rot(3)
    s1 = _arm_green_support(c1, q6, K_scaled, dist, T_base_cam, radius_px=75.0 * scale)
    s3 = _arm_green_support(c3, q6, K_scaled, dist, T_base_cam, radius_px=75.0 * scale)
    chosen = 1 if s1 >= s3 else 3
    return (c1 if chosen == 1 else c3), chosen, scale


def _detect_tip_collar(frame, q6, K, dist, T_base_cam, scale=1.0):
    """Corridor-constrained green-collar detector for base-marker rigs (`t_tip ~ 0`).

    Isolates the green gripper housing along the `j6 -> ee` tool axis while
    excluding upstream link covers (`j2-j3`, `j3-j4`) that cause false positives
    when the arm folds. Maps the observed distal collar edge rigidly onto the
    black fingertips (`ee`), working uniformly in both horizontal and top-down
    orientations with zero systematic collar-to-tip bias.
    """
    import cv2

    joints_uv, p_collar_end, p_ee = _arm_joint_pixels(q6, K, dist, T_base_cam)
    p_j6 = joints_uv[5]
    if p_j6 is None or p_ee is None or p_collar_end is None:
        return None
    seg = p_ee - p_j6
    L = float(np.linalg.norm(seg))
    if L < 8.0 * float(scale):
        return None
    u_dir = seg / L
    v_dir = np.array([-u_dir[1], u_dir[0]], dtype=float)

    mask = auto_calibrate.green_mask(frame)
    area_scale = float(scale) * float(scale)
    min_blob = float(auto_calibrate.GREEN_MIN_BLOB_PX) * area_scale
    if int(mask.sum()) // 255 < auto_calibrate.MIN_GREEN_PX * area_scale:
        return None

    # Upstream link segments (j2-j3, j3-j4) for link-decal exclusion.
    upstream = []
    for a, b in ((joints_uv[1], joints_uv[2]), (joints_uv[2], joints_uv[3])):
        if a is not None and b is not None:
            upstream.append((a, b))

    perp_gate = max(75.0 * float(scale), 0.35 * L)
    along_min = -0.15 * L
    along_max = 1.05 * L
    collar_gate = max(65.0 * float(scale), 0.28 * L)
    collar_core = max(35.0 * float(scale), 0.18 * L)

    # Filter pixels into the gripper corridor before connected-component analysis
    # so a merged link+collar silhouette splits cleanly at the corridor boundary.
    h, w = mask.shape[:2]
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    pts = np.stack([xs, ys], axis=1).astype(float)
    rel = pts - p_j6
    along = rel @ u_dir
    perp = np.abs(rel @ v_dir)
    in_corr = (along >= along_min) & (along <= along_max) & (perp <= perp_gate)
    if not np.any(in_corr):
        return None

    c_pts = pts[in_corr]
    d_tool = _point_to_seg_dist(c_pts, p_j6, p_ee)
    d_collar = np.hypot(c_pts[:, 0] - p_collar_end[0], c_pts[:, 1] - p_collar_end[1])
    keep = np.ones(len(c_pts), dtype=bool)
    for a, b in upstream:
        d_up = _point_to_seg_dist(c_pts, a, b)
        # Drop pixels strictly closer to an upstream link unless inside collar core.
        keep &= (d_tool <= d_up + 1e-6) | (d_collar <= collar_core)
    if not np.any(keep):
        return None

    corr_mask = np.zeros((h, w), dtype=np.uint8)
    kept = c_pts[keep].astype(int)
    corr_mask[kept[:, 1], kept[:, 0]] = 255

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(corr_mask, 8)
    best = None  # (err_px, p_obs_collar, area)
    for idx in range(1, count):
        area = float(stats[idx, cv2.CC_STAT_AREA])
        if area < min_blob:
            continue
        p_obs = np.asarray(centroids[idx], dtype=float)
        err = float(np.linalg.norm(p_obs - p_collar_end))
        if err > collar_gate:
            continue
        if best is None or err < best[0]:
            best = (err, p_obs, area)

    if best is None:
        return None
    err, p_obs_collar, _area = best
    # Rigid collar-to-fingertip projection: removes the constant green-housing
    # offset while preserving the exact 2D residual (sag, zero bias, camera drift).
    uv_tip = p_obs_collar + (p_ee - p_collar_end)
    score = max(0.0, 1.0 - err / collar_gate)
    return {"detector": "green-collar", "uv": [float(uv_tip[0]), float(uv_tip[1])], "score": float(score)}


def detect_tip(frame, q6, K, dist, T_base_cam, t_tip, scale=1.0):
    """The tip pixel: corridor-constrained green collar (base-marker rig) or
    calibrated green tip with fragment-cleaned retry. None, never a guess.

    A failed detection is `detected: null` (§5.3): the oracle reports what it
    saw, and a missing measurement is not a zero error.
    """
    import cv2

    t_tip_arr = np.asarray(t_tip, dtype=float).reshape(-1)
    # Base-marker rigs have tip_offset = [0, 0, 0] (projecting to the black
    # fingertips `ee`), while the green gripper housing sits along `j6 -> ee`.
    if float(np.linalg.norm(t_tip_arr)) < 1e-3:
        return _detect_tip_collar(frame, q6, K, dist, T_base_cam, scale=scale)

    options = auto_calibrate.DetectOptions()
    area_scale = float(scale) * float(scale)
    size = (int(frame.shape[1]), int(frame.shape[0]))
    try:
        found = auto_calibrate.detect_tip_green(frame, q6, K, dist, T_base_cam, t_tip,
                                                options, size, scale=scale)
        return {"detector": "green", "uv": [found.u, found.v], "score": found.score}
    except auto_calibrate.AutoError:
        pass
    mask = auto_calibrate.green_mask(frame)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
    clean = np.zeros_like(mask)
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] >= auto_calibrate.GREEN_MIN_BLOB_PX * area_scale:
            clean[labels == index] = 255
    if int(clean.sum()) // 255 < auto_calibrate.MIN_GREEN_PX * area_scale:
        return None
    try:
        found = auto_calibrate.detect_tip_green(frame, q6, K, dist, T_base_cam, t_tip,
                                                options, size, mask=clean, scale=scale)
    except auto_calibrate.AutoError:
        return None
    return {"detector": "green", "uv": [found.u, found.v], "score": found.score}


def verify_grasp(pre_frame, post_frame, pick_uv, radius_px=65.0, scale=1.0,
                 post_q6=None, K=None, dist=None, T_base_cam=None):
    """Verify whether the target object left the pick zone after grasp + retract.

    Uses projected arm-skeleton occlusion gating (when `post_q6`, `K`, `dist`,
    `T_base_cam` are supplied) plus translation-invariant template matching
    (`TM_CCOEFF_NORMED` over a local ±55 px search window), aligned Lab color
    difference, and HSV histogram correlation around `pick_uv`. Tolerates a
    missed grasp nudging the object a few centimetres or casting an overhead
    arm shadow on the table while cleanly detecting object removal across
    arbitrary object colors.
    """
    import cv2

    if pre_frame is None or post_frame is None or pick_uv is None:
        return {"grasped": None, "confidence": "low", "reason": "missing_input"}
    if pre_frame.shape[:2] != post_frame.shape[:2]:
        post_frame = cv2.resize(post_frame, (pre_frame.shape[1], pre_frame.shape[0]),
                                interpolation=cv2.INTER_AREA)
    h, w = pre_frame.shape[:2]
    cx, cy = float(pick_uv[0]), float(pick_uv[1])
    r = max(20, int(round(float(radius_px) * float(scale))))
    shift_r = max(12, int(round(55.0 * float(scale))))
    x0, x1 = max(0, int(round(cx)) - r), min(w, int(round(cx)) + r)
    y0, y1 = max(0, int(round(cy)) - r), min(h, int(round(cy)) + r)
    if (x1 - x0) < 20 or (y1 - y0) < 20:
        return {"grasped": None, "confidence": "low", "reason": "pick_uv_out_of_frame",
                "pick_uv": [round(cx, 1), round(cy, 1)]}

    # Geometric arm-skeleton occlusion guard: black motor/wrist housings are
    # invisible to `green_mask`, so project links j2..ee in the post frame and
    # abstain whenever the arm itself sits directly over `pick_uv`.
    if post_q6 is not None and K is not None and dist is not None and T_base_cam is not None:
        try:
            joints_uv, _collar_uv, ee_uv = _arm_joint_pixels(post_q6, K, dist, T_base_cam)
            chain = [p for p in joints_uv[1:] + [ee_uv] if p is not None]
            if len(chain) >= 2:
                p_arr = np.asarray([[cx, cy]], dtype=float)
                min_arm_d = min(
                    float(_point_to_seg_dist(p_arr, a, b)[0])
                    for a, b in zip(chain[:-1], chain[1:])
                )
                occlude_gate = max(75.0, float(radius_px) + 15.0) * float(scale)
                if min_arm_d < occlude_gate:
                    return {
                        "grasped": None, "confidence": "low",
                        "reason": "arm_occludes_pick_zone",
                        "arm_dist_px": round(min_arm_d, 1),
                        "pick_uv": [round(cx, 1), round(cy, 1)],
                    }
        except Exception:  # noqa: BLE001 - fall back to pixel checks if FK/proj fails
            pass

    tpl = pre_frame[y0:y1, x0:x1]
    sx0, sx1 = max(0, x0 - shift_r), min(w, x1 + shift_r)
    sy0, sy1 = max(0, y0 - shift_r), min(h, y1 + shift_r)
    search = post_frame[sy0:sy1, sx0:sx1]

    # Guard against arm occlusion over the pick zone in either frame.
    pre_arm = auto_calibrate.green_mask(tpl) > 0
    post_center = post_frame[y0:y1, x0:x1]
    post_arm = auto_calibrate.green_mask(post_center) > 0
    valid_frac = float(np.mean((~pre_arm) & (~post_arm)))
    if valid_frac < 0.35:
        return {
            "grasped": None, "confidence": "low", "reason": "pick_zone_occluded",
            "valid_frac": round(valid_frac, 3), "pick_uv": [round(cx, 1), round(cy, 1)],
        }

    # Translation-invariant match: if a missed grasp bumped the object slightly,
    # the object patch still matches with high TM_CCOEFF_NORMED nearby.
    res_tm = cv2.matchTemplate(search, tpl, cv2.TM_CCOEFF_NORMED)
    _, tm_max, _, max_loc = cv2.minMaxLoc(res_tm)
    tm_max = float(tm_max)
    th, tw = tpl.shape[:2]
    best_patch = search[max_loc[1]:max_loc[1] + th, max_loc[0]:max_loc[0] + tw]

    tpl_lab = cv2.cvtColor(tpl, cv2.COLOR_BGR2LAB).astype(np.float32)
    bp_lab = cv2.cvtColor(best_patch, cv2.COLOR_BGR2LAB).astype(np.float32)
    valid = (~pre_arm) & (auto_calibrate.green_mask(best_patch) == 0)
    if int(valid.sum()) < 64:
        valid = np.ones((th, tw), dtype=bool)
    diff = tpl_lab[valid] - bp_lab[valid]
    de = np.sqrt(np.sum(diff * diff, axis=1))
    mean_de = float(np.mean(de))
    changed_frac = float(np.mean(de > 22.0))

    h1 = cv2.calcHist([cv2.cvtColor(tpl, cv2.COLOR_BGR2HSV)], [0, 1], None, [18, 16], [0, 180, 0, 256])
    h2 = cv2.calcHist([cv2.cvtColor(best_patch, cv2.COLOR_BGR2HSV)], [0, 1], None, [18, 16], [0, 180, 0, 256])
    cv2.normalize(h1, h1)
    cv2.normalize(h2, h2)
    hist_corr = float(cv2.compareHist(h1, h2, cv2.HISTCMP_CORREL))

    grasped = bool(tm_max < 0.72 and changed_frac >= 0.20 and mean_de >= 18.0)
    # Shadow-tolerant high-confidence miss bar: overhead arm shadows during
    # live post-retract checks raise Lab `changed_frac` to 0.19-0.36 while
    # gain-invariant `tm_max` (>= 0.75) and HSV `hist_corr` (>= 0.72) still
    # confirm the target object never left the pick zone.
    if (grasped and tm_max < 0.62 and changed_frac >= 0.25) or (
        (not grasped) and (
            (tm_max >= 0.80 and changed_frac <= 0.18)
            or (tm_max >= 0.75 and hist_corr >= 0.72)
        )
    ):
        confidence = "high"
    else:
        confidence = "medium"
    return {
        "grasped": grasped,
        "confidence": confidence,
        "tm_max": round(tm_max, 4),
        "mean_de": round(mean_de, 2),
        "changed_frac": round(changed_frac, 3),
        "hist_corr": round(hist_corr, 4),
        "valid_frac": round(valid_frac, 3),
        "pick_uv": [round(cx, 1), round(cy, 1)],
    }


def green_centroid(frame):
    """A geometry-free green centroid, for dry validation only.

    This exists so the durable chain can be exercised on a synthetic recording
    without a commissioned rig: it answers "where is the green cover in this
    frame", not "is the arm where the model predicts". The production detector
    above remains the only path used when `REBOT_WORKFLOW_DRY` is unset.
    """
    import cv2

    mask = auto_calibrate.green_mask(frame)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for index in range(1, count):
        area = stats[index, cv2.CC_STAT_AREA]
        if area < auto_calibrate.GREEN_MIN_BLOB_PX:
            continue
        if best is None or area > best[0]:
            best = (area, centroids[index])
    if best is None:
        return None
    return {"detector": "green-centroid", "uv": [float(best[1][0]), float(best[1][1])],
            "score": float(best[0])}
