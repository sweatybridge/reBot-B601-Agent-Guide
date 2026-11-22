"""Oracle parity: hold_windows/match_hold_targets/downsample (exact) and
oracle_compute per-hold residuals vs the Python oracle_finish compute.
Run in the planner container.
"""
import sys, json, math
sys.path.insert(0, "/app/app")
sys.path.insert(0, "/opt/reBotArm_control_py")
import numpy as np
import psycopg
import oracle, traj, sweep, rebot_ik, locate_target, witness

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)
def rust(sql, args): return conn.execute(sql, args).fetchone()[0]

K = [[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]
dist = [0.0, 0.0, 0.0, 0.0, 0.0]
# camera at (0.3,0.2,0.8) looking down at the workspace
f = np.array([0.2, 0.1, -1.0]); f /= np.linalg.norm(f)
r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
d = np.cross(f, r)
T = np.eye(4); T[:3, 0] = r; T[:3, 1] = d; T[:3, 2] = f; T[:3, 3] = [0.3, 0.2, 0.8]
T_list = T.reshape(-1).tolist()
tip_offset = [0.0, 0.0, -0.035]
kp_spec = "50,2:150,3:150"
kp = sweep.kp_vector(kp_spec)

# synthetic trajectory with a hold
q_from = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0, 4.9]
q_to = [0.2, 0.4, 0.5, 0.1, 0.2, 0.1, 4.9]
limits = [list(l) for l in rebot_ik.LIMITS] + [[-0.1, 5.0]]
payload = traj.joint_move(np.array(q_from), np.array(q_to), 2.0, hold_s=1.0, limits=limits)
wps = payload["waypoints"]

# ── helpers: exact parity ─────────────────────────────────────────────────── #
w_py = oracle.hold_windows(wps)
w_sql = rust("SELECT hold_windows(%s::jsonb)", (json.dumps(wps),))
assert w_py == w_sql, "hold_windows mismatch:\n%s\n%s" % (w_py, w_sql)
print("hold_windows OK (%d windows)" % len(w_py))

targets = [{"point": [0.2, 0.0, 0.3], "flange_point": [0.2, 0.0, 0.335],
            "hold_t_start": w["t_start"], "hold_t_end": w["t_end"],
            "q_ik": q_to[:6]} for w in w_py]
m_py = oracle.match_hold_targets(w_py, targets)
m_sql = rust("SELECT match_hold_targets(%s::jsonb,%s::jsonb)", (json.dumps(w_py), json.dumps(targets)))
assert m_py == m_sql, "match_hold_targets mismatch"
print("match_hold_targets OK")

# samples: cmd = waypoint q, meas = q + small per-joint error
err = [0.001, -0.002, 0.003, -0.001, 0.002, -0.001]
samples = [{"t": w["t"], "cmd": w["q"][:6],
            "meas": [w["q"][j] + err[j] for j in range(6)]} for w in wps]
ds_py = oracle.downsample(samples, limit=20)
ds_sql = rust("SELECT downsample(%s::jsonb,%s)", (json.dumps(samples), 20))
assert ds_py == ds_sql, "downsample mismatch (%d vs %d)" % (len(ds_py), len(ds_sql))
print("downsample OK (%d kept)" % len(ds_py))

# ── oracle_compute per-hold residuals vs Python ───────────────────────────── #
class Rig: pass
rig = Rig(); rig.T_base_cam = T
detections = {}
for w in w_py:
    inside = [s for s in samples if w["t_start"] - 1e-6 <= s["t"] <= w["t_end"] + 1e-6]
    qm = np.mean([s["meas"] for s in inside], axis=0)
    tool = rebot_ik.forward(qm)
    tip_meas = tool[:3, 3]; R = tool[:3, :3]
    tip_point = tip_meas + R @ np.array(tip_offset)
    pred = locate_target.project(np.array(K), T, tip_point, np.array(dist))
    detections[str(w["index"])] = {"uv": [pred[0] + 1.0, pred[1] + 1.0],
                                   "score": 0.9, "detector": "green",
                                   "image_size": [1280, 720], "grid_scale": 1.0}

K_flat = [v for row in K for v in row]
res = rust("SELECT oracle_compute(%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb,%s,%s,%s,%s,%s,%s::jsonb)",
           (URDF, json.dumps(wps), json.dumps(samples), json.dumps(targets),
            json.dumps(detections), K_flat, dist, T_list, tip_offset, kp_spec, json.dumps({})))
holds_sql = res["holds"]

worst = 0.0
for w, h_sql in zip(w_py, holds_sql):
    inside = [s for s in samples if w["t_start"] - 1e-6 <= s["t"] <= w["t_end"] + 1e-6]
    qc = np.mean([s["cmd"] for s in inside], axis=0)
    qm = np.mean([s["meas"] for s in inside], axis=0)
    sag = sweep.gravity_sag(qc, kp)
    tip_cmd = rebot_ik.forward(qc)[:3, 3]; tip_meas = rebot_ik.forward(qm)[:3, 3]
    tip_err = 1000.0 * np.linalg.norm(tip_meas - tip_cmd)
    R = rebot_ik.forward(qm)[:3, :3]
    tip_point = tip_meas + R @ np.array(tip_offset)
    pred = locate_target.project(np.array(K), T, tip_point, np.array(dist))
    px_err = math.hypot(1.0, 1.0)
    mmpp = witness.millimetres_per_pixel(rig, np.array(K), point=tip_point)
    for key, val in [("hold_error_max_rad", float(np.max(np.abs(qm - qc)))),
                     ("tip_error_mm", round(tip_err, 3)),
                     ("pixel_error_px", round(px_err, 2)),
                     ("pixel_error_mm", round(px_err * mmpp, 2))]:
        worst = max(worst, abs(float(h_sql[key]) - val))
    for j in range(6):
        worst = max(worst, abs(h_sql["sag_rad"][j] - sag[j]),
                    abs(h_sql["q_cmd"][j] - qc[j]), abs(h_sql["q_meas"][j] - qm[j]))
print("oracle_compute maxdiff %.2e" % worst)
print("verdict layer:", res["verdict"]["layer"])
ok = worst < 1e-6
print("ORACLE PARITY", "OK" if ok else "CHECK")
sys.exit(0 if ok else 1)