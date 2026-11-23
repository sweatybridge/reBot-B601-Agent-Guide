""" /move parity: traj.joint_move/joint_sequence + planner_server.move_one compute
vs rebot_joint_move/rebot_joint_sequence/rebot_move_build. Run in planner container.
"""
import sys, json, tempfile, os
sys.path.insert(0, "/app/app")
sys.path.insert(0, "/opt/reBotArm_control_py")
import numpy as np
import psycopg
import traj, sweep
import planner_server as ps

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)
CFG = json.dumps(conn.execute(
    "SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()[0])

def rust(sql, args):
    return conn.execute(sql, args).fetchone()[0]

# ── builders: exact parity (deterministic interpolation) ─────────────────── #
worst = 0.0
q_from = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0, 4.9]
q_to = [0.2, 0.4, 0.5, 0.1, 0.2, 0.1, 2.0]
ref = traj.joint_move(np.array(q_from), np.array(q_to), 2.0, control_hz=30.0, hold_s=0.5,
                      limits=[list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])
got = rust("SELECT rebot_joint_move(%s,%s,%s,%s::jsonb,%s,%s,%s::jsonb,%s::text)",
           (q_from, q_to, 2.0, CFG, 30.0, 0.5,
            json.dumps([list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]]), ""))
assert len(ref["waypoints"]) == len(got["waypoints"]), "joint_move count"
for a, b in zip(ref["waypoints"], got["waypoints"]):
    worst = max(worst, abs(a["t"] - b["t"]),
                max(abs(x - y) for x, y in zip(a["q"], b["q"])),
                max(abs(x - y) for x, y in zip(a["v"], b["v"])))
print("joint_move wp", len(got["waypoints"]), "maxdiff %.2e" % worst)

poses = [q_from, q_to, [0.1, 0.3, 0.4, 0.05, 0.15, 0.05, 4.9]]
ref = traj.joint_sequence([np.array(p) for p in poses], [2.0, 1.5], control_hz=30.0, hold_s=0.4,
                          limits=[list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])
got = rust("SELECT rebot_joint_sequence(%s::jsonb,%s::jsonb,%s::jsonb,%s,%s,%s::jsonb,%s::text)",
           (json.dumps(poses), json.dumps([2.0, 1.5]), CFG, 30.0, 0.4,
            json.dumps([list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]]), ""))
assert len(ref["waypoints"]) == len(got["waypoints"]), "joint_sequence count"
for a, b in zip(ref["waypoints"], got["waypoints"]):
    worst = max(worst, abs(a["t"] - b["t"]),
                max(abs(x - y) for x, y in zip(a["q"], b["q"])),
                max(abs(x - y) for x, y in zip(a["v"], b["v"])))
print("joint_sequence wp", len(got["waypoints"]), "maxdiff %.2e" % worst)

# DEFAULT_HZ parity: Python's default is now 50 Hz; the Rust cores' hz defaults
# must match when the argument is omitted (move_one relies on this).
ref = traj.joint_move(np.array(q_from), np.array(q_to), 2.0, hold_s=0.5,
                      limits=[list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])
got = rust("SELECT rebot_joint_move(%s,%s,%s,%s::jsonb, hold_s := %s, limits := %s::jsonb)",
           (q_from, q_to, 2.0, CFG, 0.5,
            json.dumps([list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])))
assert got["control_hz"] == traj.DEFAULT_HZ == 50.0, "joint_move default hz"
assert len(ref["waypoints"]) == len(got["waypoints"]), "joint_move default-hz count"
for a, b in zip(ref["waypoints"], got["waypoints"]):
    worst = max(worst, abs(a["t"] - b["t"]),
                max(abs(x - y) for x, y in zip(a["q"], b["q"])),
                max(abs(x - y) for x, y in zip(a["v"], b["v"])))
print("joint_move default-hz wp", len(got["waypoints"]), "maxdiff %.2e" % worst)
ref = traj.joint_sequence([np.array(p) for p in poses], [2.0, 1.5], hold_s=0.4,
                          limits=[list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])
got = rust("SELECT rebot_joint_sequence(%s::jsonb,%s::jsonb,%s::jsonb, hold_s := %s, limits := %s::jsonb)",
           (json.dumps(poses), json.dumps([2.0, 1.5]), CFG, 0.4,
            json.dumps([list(l) for l in sweep.rebot_ik.LIMITS] + [[-0.1, 5.0]])))
assert got["control_hz"] == traj.DEFAULT_HZ == 50.0, "joint_sequence default hz"
assert len(ref["waypoints"]) == len(got["waypoints"]), "joint_sequence default-hz count"
for a, b in zip(ref["waypoints"], got["waypoints"]):
    worst = max(worst, abs(a["t"] - b["t"]),
                max(abs(x - y) for x, y in zip(a["q"], b["q"])),
                max(abs(x - y) for x, y in zip(a["v"], b["v"])))
print("joint_sequence default-hz wp", len(got["waypoints"]), "maxdiff %.2e" % worst)
print("BUILDERS", "OK" if worst < 1e-9 else "CHECK")

# â”€â”€ move_build vs planner_server.move_one compute (dry_run, mocked edge) â”€â”€â”€ #
ps.EXECUTE = True
ps.CALIB_DIR = tempfile.mkdtemp()  # no rig -> model-only sag
ps.PG_DSN = ""; ps._pg_mirror = None  # reference writes a local file we can read
q_from6 = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0]
ps.dump_pose = lambda: {"ids": [1, 2, 3, 4, 5, 6, 7], "q": q_from6 + [4.9]}
ps.edge_request = lambda method, path, **kw: (200, {"can_up": True})

def compare_move(body, opts_extra):
    body = dict(body); body["dry_run"] = True
    tgt = [body["q"]] if "q" in body else body["targets"]
    res = ps.move_one(body)
    ref_payload = json.loads(open(res["path"]).read())
    pose = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": q_from6 + [4.9]}
    opts = {"kp_spec": "50,2:150,3:150", "gripper": 4.9}; opts.update(opts_extra)
    got = rust("SELECT rebot_move_build(%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb)",
               (URDF, json.dumps(pose), json.dumps(tgt), json.dumps(opts), json.dumps({}), CFG))
    rw, fw = got["payload"]["waypoints"], ref_payload["waypoints"]
    if len(rw) != len(fw):
        print("  move count differs rust", len(rw), "ref", len(fw)); return 1e9
    d = 0.0
    for a, b in zip(rw, fw):
        d = max(d, abs(a["t"] - b["t"]), max(abs(x - y) for x, y in zip(a["q"], b["q"])))
    return d

d1 = compare_move({"q": [0.2, 0.4, 0.5, 0.1, 0.2, 0.1], "hold": 0.5}, {"hold": 0.5})
print("move single maxdiff %.2e" % d1)
d2 = compare_move({"targets": [[0.2, 0.4, 0.5, 0.1, 0.2, 0.1], [0.1, 0.3, 0.4, 0.05, 0.15, 0.05]],
                   "hold": 0.4}, {"hold": 0.4})
print("move multi maxdiff %.2e" % d2)
ok = worst < 1e-9 and d1 < 1e-6 and d2 < 1e-6
print("MOVE PARITY", "OK" if ok else "CHECK")
sys.exit(0 if ok else 1)
