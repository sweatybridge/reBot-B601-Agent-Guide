"""Propagation check for the via/stream-rate config keys (isolated process;
never edits the table). Verifies apply_config re-syncs traj's aliases and the
call-time-resolved defaults, and that Rust refuses a non-integer sample count."""
import sys, json
sys.path.insert(0, "/app/app")
import psycopg
import rebot_ik
import traj

DSN = "postgresql://postgres:rebot@substrate:5432/postgres"
conn = psycopg.connect(DSN)
cfg = conn.execute("SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()[0]
if isinstance(cfg, str):
    cfg = json.loads(cfg)

# Edit the motion group in-memory and apply.
cfg["motion"]["default_hz"] = 40.0
cfg["motion"]["corner_radius_rad"] = 0.07
cfg["motion"]["chain_line_samples"] = 30.0
cfg["motion"]["chain_fillet_samples"] = 12.0
rebot_ik.apply_config(cfg)
assert traj.DEFAULT_HZ == 40.0, traj.DEFAULT_HZ
assert traj.CORNER_RADIUS_RAD == 0.07, traj.CORNER_RADIUS_RAD
assert traj.CHAIN_LINE_SAMPLES == 30 and isinstance(traj.CHAIN_LINE_SAMPLES, int)
assert traj.CHAIN_FILLET_SAMPLES == 12 and isinstance(traj.CHAIN_FILLET_SAMPLES, int)

# Call-time defaults pick up the edit (no explicit hz/radius args).
poses = [[0.0, 0.3, 0.4, 0.0, 0.1, 0.0], [0.1, 0.35, 0.45, 0.0, 0.1, 0.0],
         [0.2, 0.3, 0.4, 0.0, 0.1, 0.0]]
series = traj.chain_series(poses, 2.0)
assert len(series) == 80, len(series)  # 2 s * 40 Hz
mv = traj.joint_move([0.0] * 7, [0.1] * 7, 2.0)
assert mv["control_hz"] == 40.0, mv["control_hz"]
samples = traj.chain_samples(poses)
# start + 30 line + 12 fillet (one interior knot) + 30 line.
assert len(samples) == 1 + 30 + 12 + 30, len(samples)
print("python propagation OK (hz/radius/samples flow through call-time defaults)")

# Rust: the edited doc computes; a non-integer sample count is a hard refusal.
row = conn.execute("SELECT rebot_within_limits(%s, %s::jsonb, 0.0)",
                   ([0.0] * 6, json.dumps(cfg))).fetchone()
assert row[0] is True
bad = json.loads(json.dumps(cfg))
bad["motion"]["chain_line_samples"] = 40.5
try:
    conn.execute("SELECT rebot_within_limits(%s, %s::jsonb, 0.0)",
                 ([0.0] * 6, json.dumps(bad))).fetchone()
    print("FAIL: Rust accepted a non-integer chain_line_samples")
    sys.exit(1)
except psycopg.Error:
    conn.rollback()
    print("rust integer-gate refusal OK (40.5 samples raises)")
print("VIA CONFIG PROPAGATION OK")