-- 0020 workflow foundation tests: at-most-once motion guard + idempotent learning.
-- Run: psql -v ON_ERROR_STOP=1 -f tests/execute_workflow_tests.sql
-- Rerun-safe: cleans its wf_* fixtures first.

-- Cleanup (FK order: children first).
DELETE FROM evidence.oracle_hold WHERE verdict_id IN (SELECT id FROM evidence.oracle_verdict WHERE exec_id LIKE 'wf_%');
DELETE FROM evidence.execution WHERE exec_id LIKE 'wf_%';
DELETE FROM evidence.oracle_verdict WHERE exec_id LIKE 'wf_%';
DELETE FROM learning.accuracy_cell WHERE rig_fingerprint = 'wf_rig';
DELETE FROM evidence.telemetry WHERE exec_id LIKE 'wf_%';
DELETE FROM evidence.media_fragment WHERE episode_id = 'wf_ep';
DELETE FROM evidence.plan WHERE plan_id = 'wf_plan';
DELETE FROM evidence.episode WHERE episode_id = 'wf_ep';
DELETE FROM evidence.blob WHERE sha256 = 'wf_blob';
DELETE FROM world.shared_rig WHERE rig_fingerprint = 'wf_rig';
DELETE FROM world.calibration WHERE calib_fingerprint = 'wf_calib';

-- Fixtures.
INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size)
VALUES ('wf_calib', '[[600,0,320],[0,600,240],[0,0,1]]', '[0,0,0,0,0]', '[640,480]');
INSERT INTO world.shared_rig(rig_fingerprint, workspace_id, calib_fingerprint, T_base_cam,
                             image_size, tip_offset, method, provenance, status)
VALUES ('wf_rig', 'default', 'wf_calib',
        '[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]', '[640,480]', '[0,0,0]',
        'base-marker', '{}', 'superseded');
INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
VALUES ('wf_blob', 'telemetry', convert_to('{"waypoints":[]}', 'UTF8'), 16, 'verified');
INSERT INTO evidence.episode(episode_id, workspace_id, kind) VALUES ('wf_ep', 'default', 'execution');
INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot)
VALUES ('wf_plan', 'wf_ep', 'b601', 'wf_blob', '{}', '{}');
INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
VALUES ('wf_exec', 'wf_plan', 'b601', 'wf_blob', '50', '{}');

DO $$
DECLARE
  windows jsonb := '[{"index":1,"t_start":1.0,"t_end":1.8}]'::jsonb;
  doc jsonb := '{"verdict":{"layer":"none","confidence":"high","problems":[]},
                 "holds":[{"pixel_error_mm":5.0,"tip_fk_cmd":[0.2,0.0,0.1]}]}'::jsonb;
  v jsonb; n1 bigint; n2 bigint; ex jsonb;
BEGIN
  -- 1. Idempotent creation.
  IF (execution_prepare('wf_exec','wf_plan','b601','wf_ep')->>'status') <> 'pending' THEN
    RAISE EXCEPTION 'prepare should return pending'; END IF;
  IF (execution_prepare('wf_exec','wf_plan','b601','wf_ep')->>'status') <> 'pending' THEN
    RAISE EXCEPTION 'prepare should be idempotent'; END IF;
  RAISE NOTICE 'OK execution_prepare idempotent';

  -- 2. AT-MOST-ONCE motion guard: first claim true, every later claim false.
  IF execution_claim_play('wf_exec') <> true THEN
    RAISE EXCEPTION 'first claim must succeed'; END IF;
  IF execution_claim_play('wf_exec') <> false THEN
    RAISE EXCEPTION 'second claim must be refused (at-most-once)'; END IF;
  IF execution_claim_play('wf_exec') <> false THEN
    RAISE EXCEPTION 'third claim must be refused'; END IF;
  RAISE NOTICE 'OK at-most-once claim guard';

  -- 3. mark_played transitions playing->played once.
  IF execution_mark_played('wf_exec','wf_blob') <> true THEN
    RAISE EXCEPTION 'mark_played should succeed'; END IF;
  IF execution_mark_played('wf_exec','wf_blob') <> false THEN
    RAISE EXCEPTION 'mark_played should be idempotent'; END IF;
  RAISE NOTICE 'OK mark_played idempotent';

  -- 4. wait_fragments gate: false before ingest, true after the span covers the hold.
  IF wait_fragments('wf_ep', windows) <> false THEN
    RAISE EXCEPTION 'wait_fragments should be false before ingest'; END IF;
  INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end)
  VALUES ('wf_ep','wf_blob',0,0.0,4.0);
  IF wait_fragments('wf_ep', windows) <> true THEN
    RAISE EXCEPTION 'wait_fragments should be true once covered'; END IF;
  IF wait_fragments('wf_ep', '[{"index":2,"t_start":5.0,"t_end":5.5}]'::jsonb) <> false THEN
    RAISE EXCEPTION 'wait_fragments should be false for an uncovered window'; END IF;
  RAISE NOTICE 'OK wait_fragments gate';

  -- 5. oracle_learn folds cells once.
  v := oracle_learn('wf_exec','b601','wf_rig', doc);
  IF v->>'layer' <> 'none' THEN RAISE EXCEPTION 'verdict layer mismatch'; END IF;
  SELECT coalesce(sum(n),0) INTO n1 FROM learning.accuracy_cell WHERE rig_fingerprint='wf_rig';
  IF n1 <> 1 THEN RAISE EXCEPTION 'expected 1 folded hold, got %', n1; END IF;
  RAISE NOTICE 'OK oracle_learn folds cells (n=%)', n1;

  -- 6. IDEMPOTENT replay: same verdict, NO double-count.
  v := oracle_learn('wf_exec','b601','wf_rig', doc);
  SELECT coalesce(sum(n),0) INTO n2 FROM learning.accuracy_cell WHERE rig_fingerprint='wf_rig';
  IF n2 <> n1 THEN RAISE EXCEPTION 'replay double-counted cells: % -> %', n1, n2; END IF;
  IF v->>'layer' <> 'none' THEN RAISE EXCEPTION 'replay verdict mismatch'; END IF;
  RAISE NOTICE 'OK oracle_learn idempotent replay (n still %)', n2;

  -- 7. Ledger reflects verified + learned.
  ex := execution_get('wf_exec');
  IF ex->>'status' <> 'verified' OR (ex->>'learned')::boolean <> true THEN
    RAISE EXCEPTION 'ledger should be verified/learned, got %', ex; END IF;
  IF ex->>'verdict_id' IS NULL THEN RAISE EXCEPTION 'verdict_id should be set'; END IF;
  RAISE NOTICE 'OK ledger verified/learned';

  -- 8. oracle_learn refuses without persisted telemetry.
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id) VALUES ('wf_notelem','wf_plan','b601');
  BEGIN
    PERFORM oracle_learn('wf_notelem','b601','wf_rig', doc);
    RAISE EXCEPTION 'oracle_learn should require telemetry';
  EXCEPTION WHEN OTHERS THEN
    IF SQLERRM NOT LIKE '%requires persisted telemetry%' THEN RAISE; END IF;
  END;
  RAISE NOTICE 'OK oracle_learn requires telemetry';

  -- 9. dead-letter guard: a claimed-but-failed play goes dead, never auto-replays.
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id) VALUES ('wf_dead','wf_plan','b601');
  IF execution_claim_play('wf_dead') <> true THEN RAISE EXCEPTION 'claim wf_dead'; END IF;
  PERFORM execution_mark_dead('wf_dead');
  IF execution_claim_play('wf_dead') <> false THEN
    RAISE EXCEPTION 'dead execution must not be reclaimable'; END IF;
  IF execution_get('wf_dead')->>'status' <> 'dead' THEN RAISE EXCEPTION 'should be dead'; END IF;
  RAISE NOTICE 'OK dead-letter no-replay';
END $$;

SELECT 'EXECUTE_WORKFLOW TESTS PASSED' AS result;