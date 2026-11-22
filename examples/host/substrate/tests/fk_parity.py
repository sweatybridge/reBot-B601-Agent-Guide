"""FK parity: rebot_ik.forward (numpy) vs rebot_fk_tip + rebot_joint_points (Rust).
The /fk gateway endpoint switches to the substrate in PG mode; both must agree.
Run in the planner container (has rebot_ik + psycopg)."""
import sys, json
sys.path.insert(0, "/app/app")
import numpy as np
import psycopg
import rebot_ik

URDF = open("/app/app/cad/ReBot_Arm_RS.urdf").read()
DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)

qs = [[0.0, 0.5, 0.8, 0.0, 0.3, 0.0], [0.3, 0.6, 0.8, 0.2, 0.5, 0.3],
      [-0.3, 0.5, 0.7, -0.2, 0.4, -0.3], [0.5, 0.8, 0.9, 0.3, 0.6, 0.5],
      [-0.5, 0.7, 0.6, -0.3, 0.5, -0.5], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
worst_tip = worst_pts = 0.0
for q in qs:
    tool, links = rebot_ik.forward(np.asarray(q, dtype=float), with_links=True)
    tip_py = np.asarray(tool[:3, 3], dtype=float)
    pts_py = np.asarray([f[:3, 3] for f in links], dtype=float)
    tip_rs = np.asarray(conn.execute("SELECT rebot_fk_tip(%s,%s)", (URDF, q)).fetchone()[0])
    flat = conn.execute("SELECT rebot_joint_points(%s,%s)", (URDF, q)).fetchone()[0]
    pts_rs = np.asarray(flat, dtype=float).reshape(-1, 3)
    dt = float(np.max(np.abs(tip_py - tip_rs)))
    dp = float(np.max(np.abs(pts_py - pts_rs))) if pts_py.shape == pts_rs.shape else 9.9
    worst_tip = max(worst_tip, dt)
    worst_pts = max(worst_pts, dp)
    print("q %s tip %.2e pts %.2e (shapes %s/%s)" % (q[0], dt, dp, pts_py.shape, pts_rs.shape))
print("FK PARITY worst tip %.2e pts %.2e" % (worst_tip, worst_pts))
ok = worst_tip < 1e-9 and worst_pts < 1e-9
print("FK PARITY", "OK" if ok else "CHECK")
sys.exit(0 if ok else 1)