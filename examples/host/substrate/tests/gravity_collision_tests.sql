-- Gravity dynamics + collision checking, validated against the pure-numpy
-- reference (rebot_ik.gravity_torques / sweep.path_safe) at q=[0.2,0.3,0.4,0.2,0.1,0].
-- Prereq: /tmp/rebot.urdf present (see kinematics_tests.sql).
DO $$
DECLARE
  u text := pg_read_file('/tmp/rebot.urdf');
  q float8[] := ARRAY[0.2,0.3,0.4,0.2,0.1,0.0];
  fold float8[] := ARRAY[0.0,2.5,0.5,0.0,0.0,0.0];
  ref_g float8[] := ARRAY[0.0, 1.845377, 5.741341, 1.520166, -0.019008, -0.000782];
  g float8[];
  pts float8[];
  i int;
BEGIN
  -- Gravity torques must match the reference model within 1e-3 Nm.
  g := rebot_gravity_torques(u, q);
  RAISE NOTICE 'gravity torques: %', g;
  IF array_length(g,1) <> 6 THEN RAISE EXCEPTION 'gravity must return 6 torques'; END IF;
  FOR i IN 1..6 LOOP
    IF abs(g[i]-ref_g[i]) > 1e-3 THEN
      RAISE EXCEPTION 'gravity[%] mismatch: rust=% ref=%', i, g[i], ref_g[i];
    END IF;
  END LOOP;

  -- Joint points: 7 points (21 floats); tool point matches the reference.
  pts := rebot_joint_points(u, q);
  IF array_length(pts,1) <> 21 THEN RAISE EXCEPTION 'joint_points must return 21 floats'; END IF;
  IF abs(pts[19]-0.288373)>1e-3 OR abs(pts[20]-(-0.037856))>1e-3 OR abs(pts[21]-0.395204)>1e-3 THEN
    RAISE EXCEPTION 'tool point mismatch: %,%,%', pts[19], pts[20], pts[21];
  END IF;

  -- Collision gate: a high pose to itself is clear; folding down is not.
  IF NOT rebot_path_safe(u, q, q, 40) THEN
    RAISE EXCEPTION 'path_safe(q,q) should be true';
  END IF;
  IF rebot_path_safe(u, q, fold, 40) THEN
    RAISE EXCEPTION 'path_safe(q,fold) should be false (worktop dip)';
  END IF;

  RAISE NOTICE 'GRAVITY + COLLISION MATCH REFERENCE OK';
END $$;