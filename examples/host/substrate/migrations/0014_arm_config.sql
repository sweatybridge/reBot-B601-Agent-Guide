-- 0014: world.arm_config — the canonical, table-backed source of truth for the
-- per-arm physical model + motion tuning constants that were previously hardcoded
-- (and duplicated) across rebot_ik.py / sweep.py / witness.py / plan_to_target.py
-- and the Rust rebot_kinematics cores (plan.rs / lib.rs).
--
-- The seed values are BIT-IDENTICAL to the former hardcoded constants, so the
-- compute path is unchanged (parity preserved). They are deliberately NOT derived
-- from the URDF: LINK_INERTIALS is the simplified gravity model (gripper fingers
-- omitted, tool mass lumped into gripper_end) that differs from raw URDF RNEA by
-- ~2e-3 Nm; the table stores that model, not the URDF inertials.
--
-- arm_id is a loose text key (like world.robot_model), not an FK, so the config
-- can be seeded before/without a world.arm row. Consumers: the Python arm_config
-- loader (PG mode) and the Rust cores (config passed by plan_one / move_one).

CREATE TABLE IF NOT EXISTS world.arm_config (
  arm_id     text PRIMARY KEY,
  config_doc jsonb NOT NULL,
  updated_at timestamptz NOT NULL DEFAULT now()
);

INSERT INTO world.arm_config (arm_id, config_doc) VALUES ('b601', '{
  "gravity": 9.80665,
  "limits": [[-2.80,2.80],[0.0,3.14],[0.0,3.14],[-1.57,1.57],[-1.57,1.57],[-3.14,3.14]],
  "link_inertials": [
    ["link1",0.4655,[0.00817652,0.00122584,0.03314197]],
    ["link2",1.5520,[-0.11786481,-0.00342226,-0.02715519]],
    ["link3",1.2520,[0.10342715,-0.06641379,-0.02710687]],
    ["link4",0.4600,[0.04755445,-0.04411135,-0.03141119]],
    ["link5",0.2012,[0.01144006,0.00000479,0.03804875]],
    ["link6",0.1000,[0.00001802,0.00000002,0.00434806]],
    ["gripper_end",0.6500,[-0.11275169,0.00012844,-0.00000573]]
  ],
  "link_radii": [0.055,0.055,0.050,0.045,0.035,0.030,0.020],
  "tool": {"xyz":[0.0,0.0,0.16621],"rpy":[3.1416,-1.5708,0.0]},
  "gripper": {"motor7_open":4.90,"motor7_closed":-0.015,"rad_per_mm":0.0412},
  "collision": {"clearance":0.012,"link_radius_max":0.85},
  "motion": {
    "default_min_z":0.04,"sag_limit_rad":0.15,"limit_tol":0.002,"max_joint_step":0.5,
    "ramp_s":0.15,"min_cruise_speed":0.25,"min_seconds":0.3,"max_seconds":25.0,
    "peak_speed":0.5,"gripper_peak_speed":3.0
  }
}'::jsonb)
ON CONFLICT (arm_id) DO NOTHING;