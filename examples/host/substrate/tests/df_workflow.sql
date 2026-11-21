-- A pg_durable workflow whose SQL step calls the rebot_kinematics extension.
-- Proves durable orchestration (df.* / duroxide) composes with in-worker
-- memory-safe kinematics: the step is checkpointed and resumable.
--
-- Integration constraint: pg_durable step results cannot be FLOAT8[] columns
-- (supported: scalars, text, jsonb, json, ...). Kinematics functions return
-- float8[], so wrap them in to_jsonb() when used as a durable step result.
--
-- Prereq: /tmp/rebot.urdf present (see kinematics_tests.sql).
SELECT df.start(
  df.sql('SELECT to_jsonb(rebot_fk(pg_read_file(''/tmp/rebot.urdf''), ARRAY[0,0,0,0,0,0,0,0]::float8[], ''joint6'')) AS pose'),
  'rebot_fk_demo'
) AS iid \gset
SELECT df.await_instance(:'iid', 30) AS awaited;
SELECT df.status(:'iid') AS status;
SELECT df.result(:'iid') AS result;
-- Self-check by label (psql does not interpolate variables inside dollar-quoting).
DO $$
DECLARE iid text; res text;
BEGIN
  SELECT id INTO iid FROM df.instances
    WHERE label='rebot_fk_demo' ORDER BY created_at DESC LIMIT 1;
  res := df.result(iid);
  IF res IS NULL OR res NOT LIKE '%0.1355%' THEN
    RAISE EXCEPTION 'workflow result missing FK pose: %', res;
  END IF;
  RAISE NOTICE 'PG_DURABLE + KINEMATICS WORKFLOW OK';
END $$;