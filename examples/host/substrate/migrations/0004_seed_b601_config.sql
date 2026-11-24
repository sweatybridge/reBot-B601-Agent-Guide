-- 0004_seed_b601_config.sql — complete required-config seed for arm 'b601'.
-- Combines physical model, motion/via/rate tuning, calibration gates, oracle
-- attribution thresholds, and learning gates in one idempotent statement.
-- Preserves any operator-tuned keys already present in world.arm_config.

WITH seed(doc) AS (
  VALUES ('{
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
      "peak_speed":0.5,"gripper_peak_speed":3.0,
      "default_hz":50.0,"corner_radius_rad":0.05,"chain_line_samples":40.0,"chain_fillet_samples":16.0
    },
    "calibration": {
      "min_poses":6,"max_camera_m":5.0,"max_tip_m":0.20
    },
    "oracle": {
      "pixel_error_mm":10.0,"tip_error_mm":8.0,"hold_error_rad":0.030,
      "witness_drift_px":2.0,"witness_scale_tol":0.05,"sag_correlation":0.8,
      "tip_vector_coherence":0.8,"zero_bias_spread_rad":0.005,"reach_fraction":0.9,
      "placement_xy_mm":10.0,"placement_z_tol_mm":5.0,"ik_residual_mm":10.0
    },
    "learning": {
      "rig_max_age_days":30.0,"tag_growth_px":2.0,"ewma_alpha":0.3,
      "marker_min_usable":3,"marker_spread_max_mm":6.0,"holdout_max_px":3.0,
      "tag_prior_depth":0.01,"sag_min_holds":30,"sag_huber_rad":0.010,
      "sag_validate_max_mrad":15.0,"sag_validate_ratio":2.0,
      "sag_span_radius_m":0.10,"sag_span_height_m":0.05,"accuracy_min_n":3,
      "accuracy_radius_edges":[0.10,0.20,0.30,0.40,0.50,0.60],
      "accuracy_height_edges":[0.00,0.10,0.20,0.30,0.40],"accuracy_sectors":8,
      "coverage_max_poses":12,"coverage_hold_s":2.0,"coverage_perturbations":24,
      "coverage_max_reach_m":0.75
    }
  }'::jsonb)
)
INSERT INTO world.arm_config (arm_id, config_doc)
SELECT 'b601', doc FROM seed
ON CONFLICT (arm_id) DO UPDATE
  SET config_doc = jsonb_set(
        (SELECT doc FROM seed) || world.arm_config.config_doc,
        '{motion}',
        ((SELECT doc->'motion' FROM seed) || COALESCE(world.arm_config.config_doc->'motion', '{}'::jsonb))
      ),
      updated_at = now();
