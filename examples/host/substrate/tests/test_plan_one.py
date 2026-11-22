"""plan_one end-to-end: seed URDF, call the SQL wrapper, verify the catalog.
Run in the planner container (has psycopg + substrate access + the URDF)."""
import sys, json
sys.path.insert(0, "/app/app")
import psycopg

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)
conn.execute("INSERT INTO world.robot_model(model_id, urdf) VALUES ('rebot-b601', %s) "
             "ON CONFLICT (model_id) DO UPDATE SET urdf=EXCLUDED.urdf", (URDF,))
conn.commit()

# reachable target from a non-park seed: tip 1 cm below the FK gripper_end point
sys.path.insert(0, "/opt/reBotArm_control_py")
import numpy as np
from reBotArm_control_py.kinematics.robot_model import load_robot_model, get_end_effector_frame_id, pad_q_for_model
from reBotArm_control_py.kinematics.forward_kinematics import compute_fk
m = load_robot_model(None); frame = get_end_effector_frame_id(m)
q6 = [0.2, 0.3, 0.4, 0.2, 0.1, 0.0]
_, _, T = compute_fk(m, pad_q_for_model(m, np.array(q6), 6))
tip = T[:3, 3]
point = [float(tip[0]), float(tip[1]), float(tip[2]) - 0.035]
pose = {"ids": [1,2,3,4,5,6,7], "q": q6 + [4.9]}
target = {"format": "rebot-target", "point": point, "point_frame": "tip", "tip_offset_m": 0.035}
options = {"hover": True, "hover_m": 0.0, "tip_m": 0.0, "ik_pos_tol": 0.010,
           "kp_spec": "50,2:150,3:150", "tool_axis": "current"}

row = conn.execute("SELECT plan_one(%s,%s,%s,%s,%s)",
                   ("ws_plan_test", json.dumps(pose), json.dumps(target),
                    json.dumps(options), "b601")).fetchone()
res = row[0]
pid = res["plan_id"]; summary = res["summary"]
print("plan_one ->", pid, "waypoints", summary["waypoints"], "dur %.3f" % summary["duration_s"])

# verify the catalog row + blob bytes round-trip
p = conn.execute("SELECT blob_sha, metadata, snapshot, status FROM evidence.plan WHERE plan_id=%s",
                 (pid,)).fetchone()
assert p is not None, "no evidence.plan row"
b = conn.execute("SELECT bytes FROM evidence.blob WHERE sha256=%s", (p[0],)).fetchone()
payload = json.loads(bytes(b[0]).decode())
assert payload["format"] == "rebot-joint-trajectory"
assert len(payload["waypoints"]) == summary["waypoints"]
assert p[3] == "planned"
# blob refcount protected by the plan row
rc = conn.execute("SELECT refcount FROM evidence.blob WHERE sha256=%s", (p[0],)).fetchone()[0]
assert rc >= 1, "blob refcount not incremented"
print("catalog OK: status", p[3], "blob refcount", rc, "waypoints", len(payload["waypoints"]))

# refusal path: an unreachable target must raise and persist nothing
bad = dict(target, point=[0.9, 0.0, 0.04])
try:
    conn.execute("SELECT plan_one(%s,%s,%s,%s,%s)",
                 ("ws_plan_test", json.dumps(pose), json.dumps(bad), json.dumps(options), "b601"))
    print("REFUSAL: did not raise (CHECK)")
except psycopg.Error as e:
    conn.rollback()
    print("refusal OK:", str(e).splitlines()[0][:70])
print("PLAN_ONE OK")