-- 0016: calibration gates group in world.arm_config (required-config consolidation).
-- The hand-eye fit sanity gates formerly hardcoded in calib.rs (MIN_POSES /
-- MAX_CAMERA_M / MAX_TIP_M) and extrinsics.py. Seed bit-identical to the former
-- constants -> the fit is unchanged. rebot_calibrate_fit now takes this group as a
-- required jsonb arg (passed by extrinsics._fit_pg from the table); a missing field
-- is a hard refusal. Idempotent merge into the existing config_doc.

UPDATE world.arm_config
   SET config_doc = config_doc || '{"calibration":{"min_poses":6,"max_camera_m":5.0,"max_tip_m":0.20}}'::jsonb,
       updated_at = now()
 WHERE arm_id = 'b601';