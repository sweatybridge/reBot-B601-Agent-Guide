-- self-collision validated against an independent Python capsule model (sweep.joint_points+RADII).
DO $$
DECLARE free boolean; gap float8;
BEGIN
  free := rebot_self_collision_free(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, -0.5, 0.0]::float8[]);
  gap := rebot_min_self_clearance(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, -0.5, 0.0]::float8[]);
  IF free <> true THEN RAISE EXCEPTION 'self_collision_free mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, -0.5, 0.0]: %', free; END IF;
  IF abs(gap - (0.00530132668523969)) > 1e-4 THEN RAISE EXCEPTION 'min_clearance mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, -0.5, 0.0]: % vs 0.00530132668523969', gap; END IF;
  free := rebot_self_collision_free(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, 0.5, 0.0]::float8[]);
  gap := rebot_min_self_clearance(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, 0.5, 0.0]::float8[]);
  IF free <> true THEN RAISE EXCEPTION 'self_collision_free mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, 0.5, 0.0]: %', free; END IF;
  IF abs(gap - (0.005298980322451029)) > 1e-4 THEN RAISE EXCEPTION 'min_clearance mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, 0.5, 0.0]: % vs 0.005298980322451029', gap; END IF;
  free := rebot_self_collision_free(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, 1.2, 0.0]::float8[]);
  gap := rebot_min_self_clearance(pg_read_file('/tmp/rebot.urdf'), ARRAY[0.1, 0.3, 0.3, 0.0, 1.2, 0.0]::float8[]);
  IF free <> true THEN RAISE EXCEPTION 'self_collision_free mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, 1.2, 0.0]: %', free; END IF;
  IF abs(gap - (0.0014721366182105476)) > 1e-4 THEN RAISE EXCEPTION 'min_clearance mismatch for ARRAY[0.1, 0.3, 0.3, 0.0, 1.2, 0.0]: % vs 0.0014721366182105476', gap; END IF;
  RAISE NOTICE 'SELF-COLLISION MATCHES REFERENCE OK (% clear, % colliding)', 3, 0;
END $$;
