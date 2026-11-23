-- 0019: via-chain + stream-rate tuning in world.arm_config's motion group
-- (required-config consolidation). The constants formerly hardcoded in traj.py
-- (DEFAULT_HZ / CORNER_RADIUS_RAD / CHAIN_LINE_SAMPLES / CHAIN_FILLET_SAMPLES)
-- and mirrored in plan.rs. Seed bit-identical to the former constants -> plans
-- and moves are unchanged. The Rust cores now read them from the required
-- config doc (motion group); a missing field is a hard refusal. The sample
-- counts stay floats in the doc (motion-group convention) and are coerced to
-- ints at the use sites. Idempotent merge into the existing motion group.

UPDATE world.arm_config
   SET config_doc = jsonb_set(config_doc, '{motion}',
           (config_doc -> 'motion') || '{"default_hz":50.0,"corner_radius_rad":0.05,"chain_line_samples":40.0,"chain_fillet_samples":16.0}'::jsonb),
       updated_at = now()
 WHERE arm_id = 'b601';