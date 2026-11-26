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

# ── automatic wider-pitch fallback parity (yellow band: <=30 deg horiz, 60 deg vert) ── #
for p_label, p_axis, p_entry, p_targets, expect_range in [
    ("horiz_yellow_30", "current", "front",
     [{"format": "rebot-target", "point": [0.42, 0.05, 0.04], "point_frame": "tip", "tip_offset_m": 0.035}],
     (10.0, 30.0)),
    ("vert_yellow_60", "x+", "vertical",
     [{"format": "rebot-target", "point": [0.48, -0.05, 0.04], "point_frame": "tip", "tip_offset_m": 0.035},
      {"format": "rebot-target", "point": [0.30,  0.05, 0.08], "point_frame": "tip", "tip_offset_m": 0.035}],
     (60.0, 80.0)),
]:
    p_opts = seq_opts()
    p_opts.tool_axis = p_axis
    p_opts.entry = p_entry
    p_opts_json = seq_opts_json()
    p_opts_json["tool_axis"] = p_axis
    p_opts_json["entry"] = p_entry
    ref_p, ref_p_sum = ptt.plan_sequence(td_dump, p_targets, p_opts, log=lambda *a: None)
    rust_p = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                          (URDF, json.dumps(td_dump), json.dumps(p_targets),
                           json.dumps(p_opts_json), json.dumps({}), CFG)).fetchone()[0]
    rw_p, fw_p = rust_p["payload"]["waypoints"], ref_p["waypoints"]
    assert len(rw_p) == len(fw_p), "%s WAYPOINT COUNT differs: rust %d ref %d" % (p_label, len(rw_p), len(fw_p))
    assert rust_p["payload"]["marker_pitch_deg"] == ref_p["marker_pitch_deg"], (
        "%s pitch differs: rust %s ref %s" % (p_label, rust_p["payload"]["marker_pitch_deg"], ref_p["marker_pitch_deg"]))
    for deg in rust_p["payload"]["marker_pitch_deg"]:
        assert expect_range[0] <= deg <= expect_range[1], "%s unexpected pitch %s" % (p_label, deg)
    dq_p, dtau_p, dv_p, _ = compare(rw_p, fw_p)
    assert dq_p < 1e-4, "%s parity gap %.3e rad" % (p_label, dq_p)
    worst_q = max(worst_q, dq_p); worst_tau = max(worst_tau, dtau_p); worst_v = max(worst_v, dv_p)
    print(p_label, "pitches", rust_p["payload"]["marker_pitch_deg"], "wp", len(rw_p),
          "maxdq %.2e dtau %.2e" % (dq_p, dtau_p))

# ── smooth sag-map envelope boundary (no peel-off / return step shock) ──── #
import numpy as np
sag_snap = {
    "rig_fingerprint": "test_rig",
    "zeros_key": "none",
    "sag_map": {
        "rig_fingerprint": "test_rig",
        "kp_key": "50,2:150,3:150",
        "zeros_key": "none",
        "envelope": {
            "q_min": [-0.7, 0.0, 0.0, -0.057, -0.08, -0.09],
            "q_max": [0.9, 2.77, 1.89, 1.57, 0.13, 0.08],
        },
        "per_motor": {
            "1": {"a": 0.0, "b": 0.0},
            "2": {"a": -0.004, "b": -0.006},
            "3": {"a": -0.0035, "b": -0.002},
            "4": {"a": -0.015, "b": 0.0},
            "5": {"a": 0.0, "b": 0.0},
            "6": {"a": 0.0, "b": 0.0},
        },
    },
}
sm_tgts = [{"format": "rebot-target", "point": [0.22, 0.0, 0.02], "point_frame": "tip", "tip_offset_m": 0.035}]
sm_opts_json = seq_opts_json()
sm_opts_json["tool_axis"] = "x+"
sm_opts_json["entry"] = "vertical"
sm_opts_json["min_z"] = 0.02
rust_sm = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                       (URDF, json.dumps(td_dump), json.dumps(sm_tgts),
                        json.dumps(sm_opts_json), json.dumps(sag_snap), CFG)).fetchone()[0]
q_sm = np.array([w["q"][:6] for w in rust_sm["payload"]["waypoints"]])
t_sm = np.array([w["t"] for w in rust_sm["payload"]["waypoints"]])
dt_sm = np.maximum(np.diff(t_sm), 1e-6)
vel_sm = np.diff(q_sm, axis=0) / dt_sm[:, None]
acc_sm = np.abs(np.diff(vel_sm, axis=0) / dt_sm[1:, None])
max_acc_sm = float(acc_sm.max())
assert max_acc_sm < 10.0, "sag envelope boundary shock remains: max_acc=%.2f rad/s2" % max_acc_sm
print("smooth sag boundary 2cm top-down: wp=%d max_vel=%.3f max_acc=%.2f rad/s2" % (
    len(q_sm), float(np.abs(vel_sm).max()), max_acc_sm))

# ── Issue #13: elevated 2-via arc (f95aff) jerk + Python/Rust parity ────── #
f9_tgts = [
    {"format": "rebot-target", "point": [0.462, -0.033, 0.04], "point_frame": "tip", "tip_offset_m": 0.0},
    {"format": "rebot-target", "point": [0.430,  0.250, 0.04], "point_frame": "tip", "tip_offset_m": 0.0,
     "via": [[0.462, -0.033, 0.16], [0.430, 0.250, 0.16]]},
]
f9_dump = {"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.9]}
f9_opts = seq_opts(); f9_opts.hz = 50.0; f9_opts.tool_axis = "z-"; f9_opts.entry = "front"
f9_opts_json = seq_opts_json(); f9_opts_json["hz"] = 50.0; f9_opts_json["tool_axis"] = "z-"; f9_opts_json["entry"] = "front"
ref_f9, _ = ptt.plan_sequence(f9_dump, f9_tgts, f9_opts, log=lambda *a: None)
rust_f9 = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                       (URDF, json.dumps(f9_dump), json.dumps(f9_tgts),
                        json.dumps(f9_opts_json), json.dumps({}), CFG)).fetchone()[0]
rw_f9, fw_f9 = rust_f9["payload"]["waypoints"], ref_f9["waypoints"]
assert len(rw_f9) == len(fw_f9), "f95aff WAYPOINT COUNT differs: rust %d ref %d" % (len(rw_f9), len(fw_f9))
dq_f9, dtau_f9, dv_f9, _ = compare(rw_f9, fw_f9)
assert dq_f9 < 1e-4, "f95aff parity gap %.3e rad" % dq_f9
q_f9 = np.array([w["q"][:6] for w in rw_f9])
t_f9 = np.array([w["t"] for w in rw_f9])
dt_f9 = np.maximum(np.diff(t_f9), 1e-6)
v_f9 = np.diff(q_f9, axis=0) / dt_f9[:, None]
a_f9 = np.diff(v_f9, axis=0) / dt_f9[1:, None]
j_f9 = np.abs(np.diff(a_f9, axis=0) / dt_f9[2:, None])
max_jerk_f9 = float(j_f9.max())
assert max_jerk_f9 < 120.0, "f95aff V-corner jerk spike remains: %.1f rad/s3" % max_jerk_f9
print("f95aff 2-via arc: wp=%d maxdq=%.2e max_jerk=%.1f rad/s3" % (len(rw_f9), dq_f9, max_jerk_f9))

# ── Issue #13: close-in horizontal folded J4 bind guard (b602e4) ────────── #
b6_tgts = [
    {"format": "rebot-target", "point": [0.32, 0.10, 0.045], "point_frame": "tip", "tip_offset_m": 0.0},
    {"format": "rebot-target", "point": [0.35, -0.10, 0.055], "point_frame": "tip", "tip_offset_m": 0.0},
]
b6_opts = seq_opts(); b6_opts.hz = 50.0; b6_opts.tool_axis = "z-"; b6_opts.entry = "front"
b6_opts_json = seq_opts_json(); b6_opts_json["hz"] = 50.0; b6_opts_json["tool_axis"] = "z-"; b6_opts_json["entry"] = "front"
b6_ref_ok = False
try:
    ref_b6, _ = ptt.plan_sequence(f9_dump, b6_tgts, b6_opts, log=lambda *a: None)
    b6_ref_ok = True
except ptt.PlanError:
    ref_b6 = None
b6_rust_ok = False
try:
    rust_b6 = conn.execute("SELECT rebot_plan_sequence(%s, %s, %s, %s, %s, %s::jsonb)",
                           (URDF, json.dumps(f9_dump), json.dumps(b6_tgts),
                            json.dumps(b6_opts_json), json.dumps({}), CFG)).fetchone()[0]
    b6_rust_ok = True
except Exception:
    conn.rollback()
    rust_b6 = None
assert b6_ref_ok == b6_rust_ok, "b602e4 accept/refuse mismatch: ref=%s rust=%s" % (b6_ref_ok, b6_rust_ok)
if b6_rust_ok:
    for wp in rust_b6["payload"]["waypoints"]:
        q_b6 = wp["q"][:6]
        assert not (q_b6[2] < np.radians(8.0) - 1e-4 and q_b6[3] < -0.10), "b602e4 folded J4 bind present: %s" % q_b6
print("b602e4 folded-wrist guard: accepted=%s (bind-free)" % b6_rust_ok)

print("TOTAL WORST dq %.3e dtau %.3e dv %.3e" % (worst_q, worst_tau, worst_v))
ok = cases > 0 and seq_cases > 0 and via_cases > 0 and worst_q < 1e-2 and worst_tau < 1e-2
print("PARITY", "OK" if ok else "CHECK")
sys.exit(0 if ok else 1)