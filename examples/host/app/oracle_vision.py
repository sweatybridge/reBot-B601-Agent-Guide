"""The green-tip pixel detector, shared by the capture server and the isolated
workflow activity.

This is deliberately the same code that used to live in `capture_server.
_oracle_detect`: the detector is a pure function of the frame, the measured
joints, and the calibration/rig, so it can run in either process. Keeping it in
one module is what lets the durable execute->oracle workflow reuse the capture
server's detector without importing the whole aiohttp server (or its camera and
routing dependencies) into the activity.
"""
from __future__ import annotations

import numpy as np

import auto_calibrate


def detect_tip(frame, q6, K, dist, T_base_cam, t_tip, scale=1.0):
    """The tip pixel: green, with a fragment-cleaned retry. None, never a guess.

    A failed detection is `detected: null` (§5.3): the oracle reports what it
    saw, and a missing measurement is not a zero error.

    There is no template fallback. Its template came from the PRE-execution
    still, where the arm is parked: at a hold pose that patch is background,
    and on the 20261120T064648Z coverage run it "matched" blank wall at 0.87
    and produced 16-221 mm pixel errors. A guess from background is worse than
    no measurement.

    Green itself can fail on a fragmented cover: the nearest component is a
    sliver below GREEN_MIN_BLOB_PX while the substantial cover sits a little
    farther out. The retry keeps only sizeable blobs -- still colour evidence
    from this frame, never background, never another frame.

    `scale` is the frame grid's ratio to the calibration grid (`Calib.scale`):
    a WebRTC bandwidth drop shrinks every blob, so the area gates follow s^2.
    """
    import cv2

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
