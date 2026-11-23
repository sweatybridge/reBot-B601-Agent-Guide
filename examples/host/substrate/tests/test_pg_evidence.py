"""PgEvidenceMirror round-trip against the substrate (FK order: capture, plan, oracle,
telemetry). Run: python tests/test_pg_evidence.py  (container on localhost:5433).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import psycopg
from store import SubstrateStore
from pg_evidence import PgEvidenceMirror

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres@localhost:5433/postgres")
store = SubstrateStore(DSN)
mirror = PgEvidenceMirror(store, workspace_id="ws_ev_test")

with psycopg.connect(DSN) as c:
    c.execute("INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) "
              "VALUES ('arm_ev','E','u','z','kp50') ON CONFLICT DO NOTHING")
    # accuracy_cell_update now requires the learning config; seed arm_ev from b601.
    c.execute("INSERT INTO world.arm_config(arm_id, config_doc) "
              "SELECT 'arm_ev', config_doc FROM world.arm_config WHERE arm_id='b601' "
              "ON CONFLICT (arm_id) DO NOTHING")
    c.execute("INSERT INTO world.calibration(calib_fingerprint,K,dist,image_size) "
              "VALUES ('cal_ev','[[1]]','[0]','[1600,1200]') ON CONFLICT DO NOTHING")
    c.execute("INSERT INTO world.shared_rig(rig_fingerprint,workspace_id,calib_fingerprint,"
              "T_base_cam,image_size,tip_offset,method,provenance,status) "
              "VALUES ('rig_ev','ws_ev_test','cal_ev','[[1]]','[1600,1200]','[0]','base-marker','{}','active') "
              "ON CONFLICT DO NOTHING")
    c.execute("DELETE FROM learning.accuracy_cell WHERE arm_id='arm_ev'")

# capture -> episode + still + blob + manifest
frame = b"\xff\xd8capture-frame-bytes"
sha = mirror.mirror_capture("20261122T000000Z-cap001",
                            {"purpose": "extrinsics", "image_size": [1600, 1200]},
                            frame, kind="calibration", image_size=[1600, 1200])
assert store.get_blob(sha) == frame, "capture blob not bit-identical"
assert store.read_manifest("20261122T000000Z-cap001")["purpose"] == "extrinsics"

# plan -> evidence.plan + trajectory blob
store.create_episode("ep_exec", "ws_ev_test", ["arm_ev"], kind="execution")
traj = b'{"format":"rebot-joint-trajectory","waypoints":[]}'
plan_sha = mirror.mirror_plan("plan_ev1", "ep_exec", "arm_ev", traj,
                              metadata={"waypoints": 1}, snapshot={"rig": "rig_ev"})
assert store.get_blob(plan_sha) == traj

# oracle -> verdict + accuracy cells (plan FK satisfied)
doc = {"holds": [{"tip_fk_cmd": [0.15, 0.0, 0.05], "pixel_error_mm": 5.0}] * 3,
       "verdict": {"layer": "none", "confidence": "high", "problems": []}}
mirror.mirror_oracle("plan_ev1", "arm_ev", doc, rig_fingerprint="rig_ev")
with psycopg.connect(DSN) as c:
    v = c.execute("SELECT layer, confidence FROM evidence.oracle_verdict WHERE plan_id='plan_ev1'").fetchone()
    assert v == ("none", "high"), "verdict row: %s" % (v,)
    cell = c.execute("SELECT n, n_good, validated_at IS NOT NULL FROM learning.accuracy_cell "
                     "WHERE arm_id='arm_ev' AND rig_fingerprint='rig_ev'").fetchone()
    assert cell == (3, 3, True), "accuracy cell: %s" % (cell,)

# telemetry -> evidence.telemetry + samples blob
samples = b'{"samples":[{"t":0,"cmd":[0],"meas":[0]}]}'
tel_sha = mirror.mirror_telemetry("exec_ev1", "plan_ev1", "arm_ev", samples, "kp50", {"abort": None})
assert store.get_blob(tel_sha) == samples
with psycopg.connect(DSN) as c:
    t = c.execute("SELECT kp_key FROM evidence.telemetry WHERE exec_id='exec_ev1'").fetchone()
    assert t == ("kp50",), "telemetry row: %s" % (t,)

# calib: rig + sag mirror
class FakeRig:
    fingerprint = "rig_mirror1"
    calib_fingerprint = "cal_ev"
    T_base_cam = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    image_size = [1600, 1200]
    tip_offset = [0, 0, 0]
    method = "base-marker"
    rms_px = 0.5
    rms_mm = 1.0
    witness = {}

    def provenance(self):
        return {"fingerprint": self.fingerprint, "method": self.method}


mirror.mirror_rig(FakeRig())
with psycopg.connect(DSN) as c:
    r = c.execute("SELECT status FROM world.shared_rig WHERE rig_fingerprint='rig_mirror1'").fetchone()
    assert r == ("active",), "rig mirror: %s" % (r,)
mirror.mirror_sag("arm_ev", {"rig_fingerprint": "rig_mirror1", "kp_key": "kp50", "zeros_key": "none",
                             "per_motor": {"1": {"a": 0.001, "b": 0}}, "samples": 30,
                             "rms_mrad": 5.0, "validated_mm": 3.0, "envelope": {}})
with psycopg.connect(DSN) as c:
    s = c.execute("SELECT samples, status FROM learning.sag_map "
                  "WHERE arm_id='arm_ev' AND rig_fingerprint='rig_mirror1'").fetchone()
    assert s == (30, "active"), "sag mirror: %s" % (s,)

print("PG_EVIDENCE MIRROR ROUND-TRIP OK")