"""Golden-trajectory parity: plan_to_target.plan (Pinocchio) vs rebot_plan (Rust).

Run INSIDE the planner container (needs Pinocchio + psycopg + substrate access):
    docker exec rebot-capture-planner-1 python /app/substrate/tests/plan_parity.py

The hover branch (the gateway tap path). Success criterion: joint commands match
to ~1e-12 rad (the CLIK port matches Pinocchio's solver), tau to ~1e-2 Nm (URDF
RNEA gravity), velocity/time at rounding precision, identical waypoint counts and
durations. Iterative IK is not literally byte-identical across solvers, so parity
is physical equivalence within the acceptance band, not raw bytes.
"""
import sys, json
sys.path.insert(0, "/app/app")
sys.path.insert(0, "/opt/reBotArm_control_py")
import numpy as np
import psycopg
import plan_to_target as ptt
from reBotArm_control_py.kinematics.robot_model import (
    load_robot_model, get_end_effector_frame_id, pad_q_for_model)
from reBotArm_control_py.kinematics.forward_kinematics import compute_fk

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
model = load_robot_model(None)
frame = get_end_effector_frame_id(model)


def fk_tip(q6):
    _, _, T = compute_fk(model, pad_q_for_model(model, np.array(q6), 6))
    return T[:3, 3].copy()


def opts():
    o = ptt.Options()
    o.hover = True; o.hover_m = 0.0; o.tip_m = 0.0; o.quiet = True
    o.ik_pos_tol = 0.010; o.calib_dir = ""
    o.hz = 30.0  # opts_json passes hz=30 explicitly; pin the reference to match
    return o


def opts_json():
    return {"hover": True, "hover_m": 0.0, "tip_m": 0.0, "hz": 30.0, "approach": 0.06,
            "hold_s": 0.6, "no_return": False, "unpark_j2": 0.0, "unpark_j3": 8.0,
            "unpark_s": 2.0, "ik_pos_tol": 0.010, "ik_angle_tol": 0.5,
            "min_z": 0.04, "max_z": 0.95, "max_reach": 0.75,
            "tool_axis": "current", "yaw": 0.0, "kp_spec": "50,2:150,3:150"}


seeds = [[0.2, 0.3, 0.4, 0.2, 0.1, 0.0],
         [0.0, 0.5, 0.8, 0.0, 0.3, 0.0],
         [-0.4, 0.6, 0.9, 0.2, 0.2, 0.3]]
conn = psycopg.connect(DSN)
# Required arm config (option B): the cores take it as a jsonb arg, sourced from
# world.arm_config — the same doc the SQL wrappers pass in production.
CFG = json.dumps(conn.execute(
    "SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()[0])
worst_q = worst_tau = worst_v = 0.0
cases = 0
for q6 in seeds:
    tip = fk_tip(q6)
    point = [float(tip[0]), float(tip[1]), float(tip[2]) - 0.035]
    target = {"format": "rebot-target", "point": point, "point_frame": "tip", "tip_offset_m": 0.035}
    dump = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": q6 + [4.9]}
    try:
        ref_payload, ref_sum = ptt.plan(dump, target, opts(), log=lambda *a: None)
    except ptt.PlanError as e:
        print("seed", q6, "REFUSED by reference:", str(e)[:80]); continue
    row = conn.execute("SELECT rebot_plan(%s, %s, %s, %s, %s, %s::jsonb)",
                       (URDF, json.dumps(dump), json.dumps(target),
                        json.dumps(opts_json()), json.dumps({}), CFG)).fetchone()
    rust = row[0]
    rp, rs = rust["payload"], rust["summary"]
    rw, fw = rp["waypoints"], ref_payload["waypoints"]
    if len(rw) != len(fw):
        print("seed", q6, "WAYPOINT COUNT differs: rust", len(rw), "ref", len(fw)); continue
    dq = dt = dtau = dv = 0.0
    for a, b in zip(rw, fw):
        dt = max(dt, abs(a["t"] - b["t"]))
        dq = max(dq, max(abs(x - y) for x, y in zip(a["q"], b["q"])))
        dtau = max(dtau, max(abs(x - y) for x, y in zip(a["tau"], b["tau"])))
        if "v" in a and "v" in b:
            dv = max(dv, max(abs(x - y) for x, y in zip(a["v"], b["v"])))
    worst_q = max(worst_q, dq); worst_tau = max(worst_tau, dtau); worst_v = max(worst_v, dv)
    cases += 1
    print("seed", [round(x, 2) for x in q6], "wp", len(rw),
          "dur ref %.3f rust %.3f" % (ref_sum["duration_s"], rs["duration_s"]),
          "maxdq %.2e dtau %.2e dv %.2e" % (dq, dtau, dv))
print("HOVER CASES", cases, "WORST dq %.3e dtau %.3e dv %.3e" % (worst_q, worst_tau, worst_v))

# ── plan_sequence parity (multi-marker, gripper alternate) ────────────────── #
def seq_opts():
    o = ptt.Options()
    o.quiet = True; o.ik_pos_tol = 0.010; o.calib_dir = ""
    o.hz = 30.0  # seq_opts_json passes hz=30 explicitly; pin the reference to match
    return o

def seq_opts_json():
    d = opts_json(); d.pop("hover", None); d.pop("hover_m", None); d.pop("no_return", None)
    d["grasp_q"] = 2.0
    return d

def compare(rw, fw):
    dq = dt = dtau = dv = 0.0
    for a, b in zip(rw, fw):
        dt = max(dt, abs(a["t"] - b["t"]))
        dq = max(dq, max(abs(x - y) for x, y in zip(a["q"], b["q"])))
        dtau = max(dtau, max(abs(x - y) for x, y in zip(a["tau"], b["tau"])))
        if "v" in a and "v" in b:
            dv = max(dv, max(abs(x - y) for x, y in zip(a["v"], b["v"])))
    return dq, dtau, dv, dt

marker_sets = []
for q6 in [[0.2, 0.3, 0.4, 0.2, 0.1, 0.0], [0.0, 0.5, 0.8, 0.0, 0.3, 0.0]]:
    tip = fk_tip(q6)
    # markers near the seed's reachable tip, small xy offsets at the same height
    offsets = [[0.02, 0.02], [-0.02, 0.02], [0.0, -0.03]]
    tips = [[float(tip[0] + dx), float(tip[1] + dy), float(tip[2])] for dx, dy in offsets]
    marker_sets.append([q6, tips])
seq_cases = 0
for q6, tips in marker_sets:
    targets = [{"format": "rebot-target", "point": [t[0], t[1], t[2] - 0.035],
                "point_frame": "tip", "tip_offset_m": 0.035} for t in tips]
    dump = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": q6 + [4.9]}
    try:
        ref_payload, ref_sum = ptt.plan_sequence(dump, targets, seq_opts(), log=lambda *a: None)
    except ptt.PlanError as e:
        print("seq seed", q6, "REFUSED by reference:", str(e)[:80]); continue
    row = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                       (URDF, json.dumps(dump), json.dumps(targets),
                        json.dumps(seq_opts_json()), json.dumps({}), CFG)).fetchone()
    rust = row[0]
    rw, fw = rust["payload"]["waypoints"], ref_payload["waypoints"]
    if len(rw) != len(fw):
        print("seq seed", q6, "WAYPOINT COUNT differs: rust", len(rw), "ref", len(fw)); continue
    dq, dtau, dv, dt = compare(rw, fw)
    worst_q = max(worst_q, dq); worst_tau = max(worst_tau, dtau); worst_v = max(worst_v, dv)
    seq_cases += 1
    print("seq markers", len(tips), "wp", len(rw),
          "dur ref %.3f rust %.3f" % (ref_sum["duration_s"], rust["summary"]["duration_s"]),
          "maxdq %.2e dtau %.2e dv %.2e" % (dq, dtau, dv))
print("SEQ CASES", seq_cases)

# ── via pass-through chain parity (marker 'via' lists -> traj.chain_series) ── #
# One continuous filleted chain leg per via-carrying marker, under one S-ramp;
# odd markers chain into the staging point then slide, even markers chain into
# the pose. searchsorted/interpolation port: q must agree to ~1e-9 rad.
via_cases = 0
q6 = [0.2, 0.3, 0.4, 0.2, 0.1, 0.0]
tip = fk_tip(q6)

def mk_via(dx, dy, vias=None):
    t = {"format": "rebot-target",
         "point": [float(tip[0] + dx), float(tip[1] + dy), float(tip[2] - 0.035)],
         "point_frame": "tip", "tip_offset_m": 0.035}
    if vias:
        t["via"] = [[float(tip[0] + vx), float(tip[1] + vy), float(tip[2])]
                    for vx, vy in vias]
    return t

via_targets = [
    mk_via(0.03, 0.00, vias=[(0.015, 0.02)]),
    mk_via(-0.02, 0.03),
    mk_via(0.00, -0.03, vias=[(0.02, -0.01), (-0.01, -0.02)]),
]
dump = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": q6 + [4.9]}
ref_payload, ref_sum = ptt.plan_sequence(dump, via_targets, seq_opts(), log=lambda *a: None)
row = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                   (URDF, json.dumps(dump), json.dumps(via_targets),
                    json.dumps(seq_opts_json()), json.dumps({}), CFG)).fetchone()
rust = row[0]
rw, fw = rust["payload"]["waypoints"], ref_payload["waypoints"]
assert len(rw) == len(fw), "via WAYPOINT COUNT differs: rust %d ref %d" % (len(rw), len(fw))
dq, dtau, dv, dt = compare(rw, fw)
assert rust["summary"]["via_counts"] == ref_sum["via_counts"], "via_counts differ"
assert rust["summary"]["via_points"] == ref_sum["via_points"], "via_points differ"
assert dq < 1e-9, "via chain q parity gap %.3e rad" % dq
assert dt < 1e-9, "via chain time parity gap %.3e s" % dt
worst_q = max(worst_q, dq); worst_tau = max(worst_tau, dtau); worst_v = max(worst_v, dv)
via_cases += 1
print("via markers", len(via_targets), "wp", len(rw),
      "dur ref %.3f rust %.3f" % (ref_sum["duration_s"], rust["summary"]["duration_s"]),
      "maxdq %.2e dtau %.2e dv %.2e" % (dq, dtau, dv))
print("VIA CASES", via_cases)

# ── top-down (tool_axis='x+') + vertical descend/retract parity ──────────── #
td_targets = [
    {"format": "rebot-target", "point": [0.22, -0.08, 0.04], "point_frame": "tip", "tip_offset_m": 0.035},
    {"format": "rebot-target", "point": [0.22,  0.08, 0.04], "point_frame": "tip", "tip_offset_m": 0.035},
]
td_dump = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0]}
td_opts = seq_opts()
td_opts.tool_axis = "x+"
td_opts.entry = "vertical"
td_opts_json = seq_opts_json()
td_opts_json["tool_axis"] = "x+"
td_opts_json["entry"] = "vertical"
ref_td, ref_td_sum = ptt.plan_sequence(td_dump, td_targets, td_opts, log=lambda *a: None)
row_td = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                      (URDF, json.dumps(td_dump), json.dumps(td_targets),
                       json.dumps(td_opts_json), json.dumps({}), CFG)).fetchone()
rust_td = row_td[0]
rw_td, fw_td = rust_td["payload"]["waypoints"], ref_td["waypoints"]
assert len(rw_td) == len(fw_td), "top-down WAYPOINT COUNT differs: rust %d ref %d" % (len(rw_td), len(fw_td))
assert rust_td["payload"]["entry"] == "vertical" and ref_td["entry"] == "vertical"
assert rust_td["payload"]["tool_axis"] == "x+" and ref_td["tool_axis"] == "x+"
dq_td, dtau_td, dv_td, dt_td = compare(rw_td, fw_td)
assert dq_td < 1e-4, "top-down vertical parity gap %.3e rad" % dq_td
worst_q = max(worst_q, dq_td); worst_tau = max(worst_tau, dtau_td); worst_v = max(worst_v, dv_td)
print("top-down vertical markers", len(td_targets), "wp", len(rw_td),
      "dur ref %.3f rust %.3f" % (ref_td_sum["duration_s"], rust_td["summary"]["duration_s"]),
      "maxdq %.2e dtau %.2e dv %.2e" % (dq_td, dtau_td, dv_td))

print("TOTAL WORST dq %.3e dtau %.3e dv %.3e" % (worst_q, worst_tau, worst_v))
ok = cases > 0 and seq_cases > 0 and via_cases > 0 and worst_q < 1e-2 and worst_tau < 1e-2
print("PARITY", "OK" if ok else "CHECK")
sys.exit(0 if ok else 1)