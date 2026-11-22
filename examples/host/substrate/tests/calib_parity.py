"""Calibration fit parity: extrinsics.fit (Python) vs rebot_calibrate_fit (Rust).
Synthetic clicks from a known rig + poses; both solvers must recover the same rig.
Run in the capture container (has numpy + extrinsics + psycopg).
"""
import sys, json, math
sys.path.insert(0, "/app/app")
import numpy as np
import psycopg
import extrinsics as ex
import rebot_ik

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)

K = np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]])
dist = np.zeros(5)
tip_true = np.array([0.0, 0.0, -0.035])
T_true = ex.look_at([0.35, 0.25, 0.85], [0.25, 0.0, 0.05])  # T_base<-cam
T_cam_base_true = ex.invert(T_true)

# varied poses across the workspace, several heights + wrist orientations
qs = [[0.0, 0.4, 0.6, 0.0, 0.3, 0.0], [0.3, 0.6, 0.8, 0.2, 0.5, 0.3],
      [-0.3, 0.5, 0.7, -0.2, 0.4, -0.3], [0.5, 0.8, 0.9, 0.3, 0.6, 0.5],
      [-0.5, 0.7, 0.6, -0.3, 0.5, -0.5], [0.2, 1.0, 1.0, 0.1, 0.7, 0.2],
      [-0.2, 0.9, 0.8, -0.1, 0.6, -0.2], [0.4, 0.5, 1.1, 0.4, 0.3, 0.6],
      [0.1, 1.2, 0.9, -0.2, 0.8, 0.1], [-0.4, 1.1, 1.0, 0.2, 0.7, -0.4]]
obs = []
for q in qs:
    T = rebot_ik.forward(np.array(q))
    point = T[:3, :3] @ tip_true + T[:3, 3]
    uv = ex.pixel(point, K, dist, T_cam_base_true)
    assert uv is not None, "pose behind camera"
    obs.append({"q": q, "uv": list(uv)})

opts = ex.FitOptions(refine_tip=True, refine_zeros=False, plane_z=0.0, huber_px=2.0)
res_py = ex.fit([ex.Observation(q=np.array(o["q"]), uv=tuple(o["uv"])) for o in obs],
                K, dist, opts)
rig_py = res_py.rig

opts_json = {"refine_tip": True, "refine_zeros": False, "plane_z": 0.0, "huber_px": 2.0,
             "max_iterations": 120, "max_weak_mm": 60.0, "work_radius": 0.55}
res_rs = conn.execute("SELECT rebot_calibrate_fit(%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb)",
                      (URDF, json.dumps(obs), json.dumps(K.reshape(-1).tolist()),
                       json.dumps(dist.tolist()), json.dumps(opts_json))).fetchone()[0]

T_rs = np.array(res_rs["T_base_cam"]).reshape(4, 4)
tip_rs = np.array(res_rs["tip_offset"])
dT = float(np.max(np.abs(T_rs - rig_py.T_base_cam)))
dtip = float(np.max(np.abs(tip_rs - rig_py.tip_offset)))
drms = abs(res_rs["rms_px"] - rig_py.rms_px)
dweak = abs(res_rs["weakest"]["sigma_mm"] - rig_py.weakest["sigma_mm"])
print("T_base_cam maxdiff %.2e" % dT)
print("tip_offset maxdiff %.2e" % dtip)
print("rms_px py %.4f rs %.4f (diff %.2e)" % (rig_py.rms_px, res_rs["rms_px"], drms))
print("weakest sigma_mm py %.3f rs %.3f (diff %.2e)" % (rig_py.weakest["sigma_mm"], res_rs["weakest"]["sigma_mm"], dweak))
print("predicted py", rig_py.predicted.get("centre_mm"), "rs", res_rs["predicted"].get("centre_mm"))
print("converged py", res_py.converged, "rs", res_rs["converged"], "iters", res_rs["iterations"])
ok = dT < 1e-6 and dtip < 1e-6 and drms < 1e-4 and dweak < 1e-3
print("CALIB PARITY (exact)", "OK" if ok else "CHECK")

# noisy clicks exercise the Huber robust-loss path; both solvers must agree
rng = np.random.default_rng(7)
obs_noisy = [{"q": o["q"], "uv": [o["uv"][0] + float(rng.normal(0, 0.7)),
                                  o["uv"][1] + float(rng.normal(0, 0.7))]} for o in obs]
res_py2 = ex.fit([ex.Observation(q=np.array(o["q"]), uv=tuple(o["uv"])) for o in obs_noisy],
                 K, dist, opts)
res_rs2 = conn.execute("SELECT rebot_calibrate_fit(%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb)",
                       (URDF, json.dumps(obs_noisy), json.dumps(K.reshape(-1).tolist()),
                        json.dumps(dist.tolist()), json.dumps(opts_json))).fetchone()[0]
T_rs2 = np.array(res_rs2["T_base_cam"]).reshape(4, 4)
dT2 = float(np.max(np.abs(T_rs2 - res_py2.rig.T_base_cam)))
dtip2 = float(np.max(np.abs(np.array(res_rs2["tip_offset"]) - res_py2.rig.tip_offset)))
drms2 = abs(res_rs2["rms_px"] - res_py2.rig.rms_px)
print("noisy: T maxdiff %.2e tip %.2e rms py %.4f rs %.4f" % (dT2, dtip2, res_py2.rig.rms_px, res_rs2["rms_px"]))
ok2 = dT2 < 1e-6 and dtip2 < 1e-6 and drms2 < 1e-4
print("CALIB PARITY (noisy)", "OK" if ok2 else "CHECK")
sys.exit(0 if (ok and ok2) else 1)