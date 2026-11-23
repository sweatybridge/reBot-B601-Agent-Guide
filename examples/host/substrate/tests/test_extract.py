"""Offline hold-frame extraction validation (deterministic synthetic rig).

No real episode has synchronized video+telemetry+verdict with the arm unpowered,
so this builds a known ground truth: encode a video whose green tip moves then
holds at exact positions, generate matching telemetry, and prove the extraction
activity (a) aligns hold windows to the right fragments, (b) picks a still frame,
and (c) yields the SAME tip pixel a live midpoint grab would -- the oracle parity
gate (verdict agreement, not pixel identity).

Part 1: pure unit checks of extract.py (no video).
Part 2: end-to-end through extract_service.py spawned as a SEPARATE process
        (the crash-isolation shape pg_durable's df.http targets).

Run in the capture container (cv2 + /tmp writable):
    python /app/substrate/tests/test_extract.py
"""
import base64
import json
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, "/app/substrate")
import cv2
import numpy as np
import extract

FPS = 30.0
W, H = 640, 480
Y = 240
R = 15
FAILS = []


def check(label, ok, detail=""):
    print(("OK   " if ok else "FAIL ") + label + (("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(label)


def marker_x(t):
    if t < 1.0:
        return 100.0 + 200.0 * (t / 1.0)
    if t < 1.8:
        return 300.0
    if t < 2.6:
        return 300.0 + 150.0 * ((t - 1.8) / 0.8)
    if t < 3.4:
        return 450.0
    return min(550.0, 450.0 + 100.0 * ((t - 3.4) / 0.6))


def draw(t):
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.circle(frame, (int(round(marker_x(t))), Y), R, (0, 255, 0), -1)
    return frame


def green_centroid(frame):
    b, g, r = frame[:, :, 0].astype(int), frame[:, :, 1].astype(int), frame[:, :, 2].astype(int)
    mask = ((g > 200) & (r < 100) & (b < 100)).astype(np.uint8)
    m = cv2.moments(mask)
    if m["m00"] == 0:
        return None
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])


def encode(frames, path):
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not vw.isOpened():
        raise RuntimeError("VideoWriter could not open (mp4v)")
    for f in frames:
        vw.write(f)
    vw.release()


def telemetry():
    samples = []
    for i in range(int(4.0 * FPS) + 1):
        t = i / FPS
        q = [marker_x(t) / 1000.0, 0, 0, 0, 0, 0, 0]
        samples.append({"cmd": q, "meas": list(q), "t": t})
    return samples


# --------------------------------------------------------------------------- #
# Part 1: pure logic
# --------------------------------------------------------------------------- #
def part1():
    # joint_speeds: a ramp reads positive, a plateau reads zero.
    ramp = [{"meas": [i * 0.1, 0, 0, 0, 0, 0, 0], "t": i * 0.1} for i in range(10)]
    sp = extract.joint_speeds(ramp)
    check("joint_speeds ramp ~1 rad/s", all(abs(v - 1.0) < 1e-6 for _, v in sp[1:-1]))
    still = [{"meas": [0.3] * 7, "t": i * 0.1} for i in range(10)]
    check("joint_speeds plateau = 0", all(v == 0.0 for _, v in extract.joint_speeds(still)))

    # stillness preference: pick the still plateau frame, not the moving ones.
    samples = []
    for i in range(21):
        t = i * 0.05
        # moving except a still plateau around t=0.5
        v = 0.0 if 0.45 <= t <= 0.55 else 0.3
        samples.append({"meas": [v, 0, 0, 0, 0, 0, 0], "t": t})
    # make meas actually integrate the speed so the plateau is flat
    pos = 0.0
    for s in samples:
        pos += s["meas"][0] * 0.05
        s["meas"][0] = pos
    win = {"t_start": 0.0, "t_end": 1.0}
    choice = extract.select_hold_frame([0.1, 0.5, 0.9], samples, win)
    check("selects stillest frame (0.5)", abs(choice["t"] - 0.5) < 1e-9, "t=%.3f score=%.4f" % (choice["t"], choice["score"]))

    # flat stillness -> midpoint tiebreak (the live-grab choice).
    flat = [{"meas": [0.3] * 7, "t": i * 0.05} for i in range(21)]
    choice = extract.select_hold_frame([0.2, 0.5, 0.8], flat, win)
    check("midpoint tiebreak when all still", abs(choice["t"] - 0.5) < 1e-9, "t=%.3f" % choice["t"])

    # no frame in window -> None (reported, not guessed).
    check("no frame in window -> None", extract.select_hold_frame([2.0, 3.0], flat, win) is None)

    # fragment selection by episode-relative span.
    frags = [{"seq": 0, "t_start": 0.0, "t_end": 2.0}, {"seq": 1, "t_start": 2.0, "t_end": 4.0}]
    check("fragments_for_window hold1", [f["seq"] for f in extract.fragments_for_window(frags, 1.0, 1.8)] == [0])
    check("fragments_for_window hold2", [f["seq"] for f in extract.fragments_for_window(frags, 2.6, 3.4)] == [1])
    plan = extract.plan_extraction([{"index": 1, "t_start": 1.0, "t_end": 1.8},
                                    {"index": 2, "t_start": 2.6, "t_end": 3.4}], frags)
    check("plan_extraction maps windows->fragments", plan[0]["fragments"] == [0] and plan[1]["fragments"] == [1])

    # t_offset shifts the window into the video clock.
    choice = extract.select_hold_frame([2.5, 3.0, 3.5], flat, {"t_start": 0.0, "t_end": 1.0}, t_offset=2.5)
    check("t_offset alignment", choice is not None and abs(choice["t"] - 3.0) < 1e-9)


# --------------------------------------------------------------------------- #
# Part 2: end-to-end through the isolated service
# --------------------------------------------------------------------------- #
def part2():
    tmp = "/tmp/extract_rig"
    os.makedirs(tmp, exist_ok=True)
    frames = [draw(i / FPS) for i in range(int(4.0 * FPS))]
    f0, f1 = tmp + "/frag0.mp4", tmp + "/frag1.mp4"
    encode(frames[:60], f0)          # episode [0, 2.0)
    encode(frames[60:], f1)          # episode [2.0, 4.0)
    fragments = []
    for seq, path, t0, t1 in ((0, f0, 0.0, 2.0), (1, f1, 2.0, 4.0)):
        with open(path, "rb") as fh:
            fragments.append({"seq": seq, "t_start": t0, "t_end": t1,
                              "data_b64": base64.b64encode(fh.read()).decode("ascii")})
    windows = [{"index": 1, "t_start": 1.0, "t_end": 1.8},
               {"index": 2, "t_start": 2.6, "t_end": 3.4}]
    req = {"fragments": fragments, "samples": telemetry(), "windows": windows,
           "t_offset": 0.0, "jpeg_quality": 92}

    port = 8799
    proc = subprocess.Popen([sys.executable, "/app/substrate/extract_service.py", "--port", str(port)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        base = "http://127.0.0.1:%d" % port
        for _ in range(50):
            try:
                urllib.request.urlopen(base + "/health", timeout=1)
                break
            except Exception:
                time.sleep(0.1)
        else:
            check("service health", False, "never came up")
            return
        check("service health", True)
        data = json.dumps(req).encode("utf-8")
        r = urllib.request.Request(base + "/extract", data=data,
                                   headers={"Content-Type": "application/json"})
        resp = json.loads(urllib.request.urlopen(r, timeout=60).read())
        check("both holds extracted", resp["missed"] == [] and len(resp["holds"]) == 2,
              "missed=%s" % resp["missed"])
        expected = {1: 300.0, 2: 450.0}
        for hold in resp["holds"]:
            idx = hold["index"]
            frame = cv2.imdecode(np.frombuffer(base64.b64decode(hold["frame_b64"]), np.uint8),
                                 cv2.IMREAD_COLOR)
            cx, cy = green_centroid(frame)
            w0, w1 = (1.0, 1.8) if idx == 1 else (2.6, 3.4)
            check("hold%d frame in window" % idx, w0 - 1e-6 <= hold["t"] <= w1 + 1e-6, "t=%.3f" % hold["t"])
            check("hold%d still (score~0)" % idx, hold["score"] < 1e-3, "score=%.5f" % hold["score"])
            check("hold%d tip at expected x" % idx, abs(cx - expected[idx]) < 2.0, "cx=%.2f want %.1f" % (cx, expected[idx]))
            # Parity vs the live midpoint grab: decode the midpoint frame from the
            # video and compare tip pixels. Both must show the held pose.
            mid = 0.5 * (w0 + w1)
            frag_path = f0 if idx == 1 else f1
            frag_t0 = 0.0 if idx == 1 else 2.0
            cap = cv2.VideoCapture(frag_path)
            target_idx = int(round((mid - frag_t0) * FPS))
            cap.set(cv2.CAP_PROP_POS_FRAMES, target_idx)
            ok, midframe = cap.read()
            cap.release()
            mx, my = green_centroid(midframe)
            check("hold%d tip == live midpoint grab" % idx,
                  abs(cx - mx) < 1.5 and abs(cy - my) < 1.5,
                  "extracted=(%.2f,%.2f) midpoint=(%.2f,%.2f)" % (cx, cy, mx, my))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()


if __name__ == "__main__":
    part1()
    part2()
    print("EXTRACT TESTS", "OK" if not FAILS else "FAILED: %s" % FAILS)
    sys.exit(0 if not FAILS else 1)