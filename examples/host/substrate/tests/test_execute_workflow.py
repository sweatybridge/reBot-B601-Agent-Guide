"""Dry validation of the durable execute -> oracle workflow (arm parked).

Runs the real pg_durable graph against the real activity service and a real
Postgres substrate, but the activity is in `REBOT_WORKFLOW_DRY=1`: it never
contacts the edge, never starts a recording, and synthesises the telemetry from
the plan. A deterministic synthetic recording supplies the hold stills, so the
offline extraction and detection path is exercised end to end without motion.

This is the bench-side proof that the chain is durable, at-most-once, and
idempotent; it is not a substitute for a real recorded episode (that needs a
powered arm).

Run inside the capture container (cv2 + psycopg + /app/substrate):
    REBOT_WORKFLOW_DRY=1 python /app/substrate/tests/test_execute_workflow.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, "/app/substrate")
import cv2
import numpy as np

import execute_workflow
from store import SubstrateStore

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres:rebot@substrate:5432/postgres")
ACTIVITY = os.environ.get("REBOT_WORKFLOW_TEST_URL", "http://capture:8766")
EXEC = "wf2_exec"
PLAN = "wf2_plan"
EPISODE = "wf2_ep"
WORKSPACE = "wf2"
PLAN_SHA = "wf2_plan_blob"
REC_SHA = "wf2_rec"
CALIB = "wf2_calib"
RIG = "wf2_rig"
FPS = 30.0
W, H = 640, 480
FAILS = []


def check(label, ok, detail=""):
    print(("OK   " if ok else "FAIL ") + label + (("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(label)


def draw(t):
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    # Still at x=300 through the hold window [0.4, 1.6]; moving elsewhere.
    if 0.4 <= t <= 1.6:
        x = 300.0
    else:
        x = 150.0 + 150.0 * min(1.0, abs(t - 1.0) / 0.6)
    cv2.circle(frame, (int(round(x)), 240), 15, (0, 255, 0), -1)
    return frame


def encode(path):
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not vw.isOpened():
        raise RuntimeError("VideoWriter could not open (mp4v)")
    for i in range(int(2.0 * FPS)):
        vw.write(draw(i / FPS))
    vw.release()


def cleanup(store):
    capture_execs = ("20261124T000000Z-abc123", "20261124T000001Z-def456",
                     "20261124T000002Z-aaa111", "20261124T000003Z-bbb222",
                     "20261124T000004Z-ccc333")
    with store._conn() as c:
        # Coverage-workflow fixture from the ad-hoc validation, if present.
        c.execute("DELETE FROM evidence.execution_hold WHERE exec_id='wfc_exec'")
        c.execute("DELETE FROM evidence.execution WHERE exec_id='wfc_exec'")
        c.execute("DELETE FROM evidence.oracle_hold WHERE verdict_id IN "
                  "(SELECT id FROM evidence.oracle_verdict WHERE exec_id='wfc_exec')")
        c.execute("DELETE FROM evidence.oracle_verdict WHERE exec_id='wfc_exec'")
        c.execute("DELETE FROM evidence.telemetry WHERE exec_id='wfc_exec'")
        c.execute("DELETE FROM evidence.media_fragment WHERE episode_id='wfc_ep'")
        c.execute("DELETE FROM evidence.still WHERE episode_id='wfc_ep'")
        c.execute("DELETE FROM evidence.plan WHERE plan_id='wfc_plan'")
        c.execute("DELETE FROM evidence.episode WHERE episode_id='wfc_ep'")
        for pid in capture_execs:
            c.execute("DELETE FROM evidence.execution_hold WHERE exec_id=%s", (pid,))
            c.execute("DELETE FROM evidence.media_fragment WHERE episode_id=%s",
                      ("exec-" + pid,))
            c.execute("DELETE FROM evidence.still WHERE episode_id=%s", ("exec-" + pid,))
            c.execute("DELETE FROM evidence.execution WHERE exec_id=%s OR plan_id=%s",
                      (pid, pid))
            c.execute("DELETE FROM evidence.oracle_hold WHERE verdict_id IN "
                      "(SELECT id FROM evidence.oracle_verdict WHERE exec_id=%s)", (pid,))
            c.execute("DELETE FROM evidence.oracle_verdict WHERE exec_id=%s", (pid,))
            c.execute("DELETE FROM evidence.telemetry WHERE exec_id=%s", (pid,))
            c.execute("DELETE FROM evidence.plan WHERE plan_id=%s", (pid,))
            c.execute("DELETE FROM evidence.episode WHERE episode_id=%s",
                      ("exec-" + pid,))
        c.execute("DELETE FROM evidence.execution_hold WHERE exec_id=%s", (EXEC,))
        c.execute("DELETE FROM evidence.execution WHERE exec_id=%s OR plan_id=%s",
                  (EXEC, PLAN))
        c.execute("DELETE FROM evidence.execution WHERE plan_id IN "
                  "(SELECT plan_id FROM evidence.plan WHERE episode_id=%s)", (EPISODE,))
        c.execute("DELETE FROM evidence.oracle_hold WHERE verdict_id IN "
                  "(SELECT id FROM evidence.oracle_verdict WHERE exec_id=%s)", (EXEC,))
        c.execute("DELETE FROM evidence.oracle_verdict WHERE exec_id=%s", (EXEC,))
        c.execute("DELETE FROM evidence.telemetry WHERE exec_id=%s", (EXEC,))
        c.execute("DELETE FROM evidence.media_fragment WHERE episode_id=%s", (EPISODE,))
        c.execute("DELETE FROM evidence.still WHERE episode_id=%s", (EPISODE,))
        c.execute("DELETE FROM evidence.oracle_hold WHERE verdict_id IN "
                  "(SELECT id FROM evidence.oracle_verdict WHERE plan_id IN "
                  " (SELECT plan_id FROM evidence.plan WHERE episode_id=%s))", (EPISODE,))
        c.execute("DELETE FROM evidence.oracle_verdict WHERE plan_id IN "
                  "(SELECT plan_id FROM evidence.plan WHERE episode_id=%s)", (EPISODE,))
        c.execute("DELETE FROM evidence.plan WHERE episode_id=%s", (EPISODE,))
        c.execute("DELETE FROM evidence.episode WHERE episode_id=%s", (EPISODE,))
        c.execute("DELETE FROM evidence.blob WHERE sha256 IN (%s,%s,'wf3_plan_blob')",
                  (PLAN_SHA, REC_SHA))
        c.execute("DELETE FROM learning.accuracy_cell WHERE rig_fingerprint=%s", (RIG,))
        c.execute("DELETE FROM learning.sag_map WHERE rig_fingerprint=%s", (RIG,))
        c.execute("DELETE FROM world.shared_rig WHERE rig_fingerprint=%s", (RIG,))
        c.execute("DELETE FROM world.calibration WHERE calib_fingerprint=%s", (CALIB,))


def seed(store):
    plan = {
        "waypoints": [
            {"q": [0.1, 0.2, 0.3, 0.2, 0.1, 0, 0], "t": 0.0},
            {"q": [0.2, 0.3, 0.4, 0.2, 0.1, 0, 0], "t": 0.4},
            {"q": [0.2, 0.3, 0.4, 0.2, 0.1, 0, 0], "t": 1.6},
            {"q": [0.1, 0.2, 0.3, 0.2, 0.1, 0, 0], "t": 2.0},
        ],
        "capture": {"targets": []},
    }
    raw = json.dumps(plan).encode("utf-8")
    with store._conn() as c:
        c.execute("INSERT INTO evidence.blob(sha256,kind,bytes,size_bytes,status) "
                  "VALUES (%s,'trajectory',%s,%s,'verified') ON CONFLICT DO NOTHING",
                  (PLAN_SHA, raw, len(raw)))
        c.execute("INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) "
                  "VALUES (%s,%s,0,0.04,0.05) ON CONFLICT DO NOTHING", (WORKSPACE, WORKSPACE))
        c.execute("INSERT INTO world.calibration(calib_fingerprint,K,dist,image_size,calib_doc) "
                  "VALUES (%s,%s,%s,%s,%s) ON CONFLICT DO NOTHING",
                  (CALIB, json.dumps([[600, 0, 320], [0, 600, 240], [0, 0, 1]]),
                   json.dumps([0, 0, 0, 0, 0]), json.dumps([W, H]),
                   json.dumps({"device": "synthetic", "image_size": [W, H],
                               "K": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
                               "dist": [0, 0, 0, 0, 0]})))
        c.execute("INSERT INTO world.shared_rig(rig_fingerprint,workspace_id,calib_fingerprint,"
                  "T_base_cam,image_size,tip_offset,method,provenance,status,rig_doc) "
                  "VALUES (%s,%s,%s,%s,%s,%s,'base-marker','{}','active',%s) "
                  "ON CONFLICT DO NOTHING",
                  (RIG, WORKSPACE, CALIB,
                   json.dumps([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
                   json.dumps([W, H]), json.dumps([0, 0, -0.035]),
                   json.dumps({"fingerprint": RIG})))
        c.execute("INSERT INTO evidence.episode(episode_id,workspace_id,arm_ids,kind) "
                  "VALUES (%s,%s,%s,'execution') ON CONFLICT DO NOTHING",
                  (EPISODE, WORKSPACE, json.dumps(["b601"])))
        c.execute("INSERT INTO evidence.plan(plan_id,episode_id,arm_id,blob_sha,metadata,snapshot) "
                  "VALUES (%s,%s,'b601',%s,'{}','{}') ON CONFLICT DO NOTHING",
                  (PLAN, EPISODE, PLAN_SHA))
        # A real 2 s recording; the hold window maps to the still dot.
        tmp = "/tmp/wf2_recording.mp4"
        encode(tmp)
        data = Path(tmp).read_bytes()
        c.execute("INSERT INTO evidence.blob(sha256,kind,bytes,size_bytes,status) "
                  "VALUES (%s,'fragment',%s,%s,'verified') ON CONFLICT DO NOTHING",
                  (REC_SHA, data, len(data)))
        c.execute("INSERT INTO evidence.media_fragment(episode_id,sha256,seq,t_start,t_end,codec) "
                  "VALUES (%s,%s,0,0,2.0,'mp4') ON CONFLICT (episode_id,seq) DO NOTHING",
                  (EPISODE, REC_SHA))
        return REC_SHA


def _activity_health(url):
    try:
        return json.loads(urllib.request.urlopen(url + "/health", timeout=2).read())
    except Exception:  # noqa: BLE001
        return None


def ensure_activity():
    """Use a dry activity service, or start one. Never run against a live one.

    The dry guard is deliberate: this test must never reach the gateway or the
    edge. If a non-dry service is already listening, refuse instead of risking a
    real play.
    """
    health = _activity_health(ACTIVITY) or _activity_health("http://127.0.0.1:8766")
    if health is not None:
        if not health.get("dry"):
            raise SystemExit("refusing: activity at %s is not REBOT_WORKFLOW_DRY=1" % ACTIVITY)
        return None
    env = dict(os.environ, REBOT_WORKFLOW_DRY="1", REBOT_EXECUTE="0", REBOT_PG_DSN=DSN)
    proc = subprocess.Popen(
        [sys.executable, "/app/substrate/workflow_service.py", "--port", "8766"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        if _activity_health("http://127.0.0.1:8766") is not None:
            return proc
        time.sleep(0.1)
    proc.terminate()
    raise SystemExit("the dry activity service never came up on :8766")


def main():
    store = SubstrateStore(DSN)
    cleanup(store)
    seed(store)
    proc = ensure_activity()
    try:
        return run_checks(store)
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                proc.kill()


def run_checks(store):

    receipt = execute_workflow.start_execute_workflow(
        store, EXEC, PLAN, "b601", EPISODE, ACTIVITY, WORKSPACE,
        timeout_s=10, poll_s=1, await_s=90, label="wf2_dry")
    check("workflow completed", receipt.get("status") == "completed",
          json.dumps(receipt)[:500])
    out = receipt.get("receipt") or {}
    check("SQL receipt is ok/executed/not-pending",
          bool(out.get("ok")) and bool(out.get("executed"))
          and not out.get("pending") and out.get("workflow_status") == "completed",
          json.dumps(out)[:300])
    check("receipt carries the extracted hold count", int(out.get("holds") or 0) == 1,
          "holds=%s" % out.get("holds"))
    health = execute_workflow.workflow_health(store, EXEC)
    ex = health["execution"] or {}
    check("execution verified + learned", ex.get("status") == "verified"
          and bool(ex.get("learned")), json.dumps(ex))
    check("one extracted hold still", len(health["stills"]) == 1,
          str(health["stills"]))
    check("verdict persisted", bool(health["verdict"]), str(health["verdict"]))
    check("attempts == 1 (played once)", int(ex.get("attempts") or 0) == 1,
          "attempts=%s" % ex.get("attempts"))

    # Replay the same instance start: the claim is refused and the learned flag
    # means no second play, no duplicate still, no second accuracy-cell fold.
    with store._conn() as c:
        before = c.execute("SELECT count(*) AS n FROM evidence.oracle_verdict "
                           "WHERE exec_id=%s", (EXEC,)).fetchone()["n"]
        cells = c.execute("SELECT coalesce(sum(n),0) AS n FROM learning.accuracy_cell "
                          "WHERE rig_fingerprint=%s", (RIG,)).fetchone()["n"]
    again = execute_workflow.start_execute_workflow(
        store, EXEC, PLAN, "b601", EPISODE, ACTIVITY, WORKSPACE,
        timeout_s=10, poll_s=1, await_s=90, label="wf2_replay")
    check("replay completed idempotently", again.get("status") == "completed",
          json.dumps(again)[:300])
    with store._conn() as c:
        after = c.execute("SELECT count(*) AS n FROM evidence.oracle_verdict "
                          "WHERE exec_id=%s", (EXEC,)).fetchone()["n"]
        cells2 = c.execute("SELECT coalesce(sum(n),0) AS n FROM learning.accuracy_cell "
                           "WHERE rig_fingerprint=%s", (RIG,)).fetchone()["n"]
        stills = c.execute("SELECT count(*) AS n FROM evidence.execution_hold "
                           "WHERE exec_id=%s", (EXEC,)).fetchone()["n"]
        attempts = c.execute("SELECT attempts FROM evidence.execution WHERE exec_id=%s",
                             (EXEC,)).fetchone()["attempts"]
    check("replay wrote no second verdict", after == before, "%s -> %s" % (before, after))
    check("replay folded no second cell sample", cells2 == cells, "%s -> %s" % (cells, cells2))
    check("replay did not duplicate the still", stills == 1, "stills=%s" % stills)
    check("replay did not re-claim the play", attempts == 1, "attempts=%s" % attempts)

    print("EXECUTE WORKFLOW DRY", "OK" if not FAILS else "FAILED: %s" % FAILS)
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
