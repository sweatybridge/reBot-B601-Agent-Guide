-- 0021 workflow tests: prepare -> claim -> played -> ready -> stills -> finalize.
-- Run: psql -v ON_ERROR_STOP=1 -f tests/execute_oracle_workflow_tests.sql
-- Rerun-safe: cleans its ewf_* fixtures first.

DELETE FROM evidence.execution_hold WHERE exec_id LIKE 'ewf_%';
DELETE FROM evidence.execution WHERE exec_id LIKE 'ewf_%';
DELETE FROM evidence.oracle_hold WHERE verdict_id IN
  (SELECT id FROM evidence.oracle_verdict WHERE exec_id LIKE 'ewf_%');
DELETE FROM evidence.oracle_verdict WHERE exec_id LIKE 'ewf_%';
DELETE FROM evidence.telemetry WHERE exec_id LIKE 'ewf_%'
   OR blob_sha = 'ewf_tel_blob';
DELETE FROM evidence.media_fragment WHERE episode_id = 'ewf_ep';
DELETE FROM evidence.still WHERE episode_id = 'ewf_ep';
DELETE FROM evidence.plan WHERE plan_id = 'ewf_plan';
DELETE FROM evidence.episode WHERE episode_id = 'ewf_ep';
DELETE FROM evidence.blob WHERE sha256 IN ('ewf_plan_blob', 'ewf_tel_blob', 'ewf_rec');
DELETE FROM learning.accuracy_cell WHERE rig_fingerprint = 'ewf_rig';
DELETE FROM learning.sag_map WHERE rig_fingerprint = 'ewf_rig';
DELETE FROM world.shared_rig WHERE rig_fingerprint = 'ewf_rig';
DELETE FROM world.calibration WHERE calib_fingerprint = 'ewf_calib';

INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
VALUES ('ewf', 'ewf', 0, 0.04, 0.05) ON CONFLICT DO NOTHING;
INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size)
VALUES ('ewf_calib', '[[600,0,320],[0,600,240],[0,0,1]]', '[0,0,0,0,0]', '[640,480]');
INSERT INTO world.shared_rig(rig_fingerprint, workspace_id, calib_fingerprint, T_base_cam,
                             image_size, tip_offset, method, provenance, status, rig_doc)
VALUES ('ewf_rig', 'ewf', 'ewf_calib',
        '[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]', '[640,480]', '[0,0,-0.035]',
        'base-marker', '{}', 'active', '{"fingerprint":"ewf_rig"}');
INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
VALUES ('ewf_ep', 'ewf', '["b601"]', 'execution');
INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status) VALUES
 ('ewf_plan_blob', 'trajectory',
  convert_to('{"waypoints":[{"q":[0.2,0.3,0.4,0.2,0.1,0,0],"t":0.0},{"q":[0.2,0.3,0.4,0.2,0.1,0,0],"t":1.0},{"q":[0.2,0.3,0.4,0.2,0.1,0,0],"t":1.2}],"capture":{"targets":[]}}', 'UTF8'),
  100, 'verified'),
 ('ewf_tel_blob', 'telemetry',
  convert_to('{"version":2,"samples":[{"t":0.0,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]},{"t":1.0,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]}],"kp":{"2":150,"3":150}}', 'UTF8'),
  100, 'verified'),
 ('ewf_rec', 'fragment', '\x00'::bytea, 1, 'verified');
INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot)
VALUES ('ewf_plan', 'ewf_ep', 'b601', 'ewf_plan_blob', '{}', '{}');
INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
VALUES ('ewf_exec', 'ewf_plan', 'b601', 'ewf_tel_blob', '150', '{}');

DO $$
DECLARE
  r jsonb; d jsonb; n1 bigint; n2 bigint;
  jpg text := '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAf/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIQAxAAAAF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABBQJ//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAwEBPwF//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAgEBPwF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQAGPwJ//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABPyF//9oADAMBAAIAAwAAABAf/8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAwEBPxB//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAgEBPxB//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABPxB//9k=';
BEGIN
  r := execution_prepare('ewf_exec', 'ewf_plan', 'b601', 'ewf_ep', 'ewf', 10);
  IF r->>'status' <> 'pending' THEN RAISE EXCEPTION 'prepare status %', r; END IF;
  IF jsonb_array_length(r->'windows') <> 1 THEN RAISE EXCEPTION 'expected one hold window'; END IF;
  RAISE NOTICE 'OK prepare (window %..%)',
    r->'windows'->0->>'t_start', r->'windows'->0->>'t_end';

  IF execution_claim_play('ewf_exec') <> true THEN RAISE EXCEPTION 'first claim'; END IF;
  IF execution_claim_play('ewf_exec') <> false THEN RAISE EXCEPTION 'replay claim'; END IF;
  RAISE NOTICE 'OK at-most-once claim';

  PERFORM execution_mark_played('ewf_exec', 'ewf_tel_blob');
  PERFORM execution_add_segment('ewf_exec', 0, 'ewf_rec', 2.0, 'mp4');
  PERFORM execution_set_recording('ewf_exec', 0.0);
  IF NOT execution_ready('ewf_exec') THEN RAISE EXCEPTION 'not ready after ingest'; END IF;
  IF execution_waiting('ewf_exec') THEN RAISE EXCEPTION 'waiting after ready'; END IF;
  RAISE NOTICE 'OK recording readiness gate';

  r := execution_store_stills('ewf_exec', jsonb_build_object(
        'holds', jsonb_build_array(jsonb_build_object(
          'index', 1, 't', 1.0, 'score', 0.0, 'width', 640, 'height', 480, 'frame_b64', jpg)),
        'missed', '[]'::jsonb));
  IF (r->>'stored')::int <> 1 THEN RAISE EXCEPTION 'stored %', r; END IF;
  -- Idempotent: the same hold does not add a second still/row.
  r := execution_store_stills('ewf_exec', jsonb_build_object(
        'holds', jsonb_build_array(jsonb_build_object(
          'index', 1, 't', 1.0, 'score', 0.0, 'width', 640, 'height', 480, 'frame_b64', jpg)),
        'missed', '[]'::jsonb));
  IF (SELECT count(*) FROM evidence.execution_hold WHERE exec_id='ewf_exec') <> 1 THEN
    RAISE EXCEPTION 'duplicate execution_hold';
  END IF;
  RAISE NOTICE 'OK extract stills idempotent';

  d := execution_detect_request('ewf_exec');
  IF jsonb_array_length(d->'holds') <> 1 THEN RAISE EXCEPTION 'detect request %', d; END IF;
  IF (d->'holds'->0->>'grid_scale')::float8 <> 1.0 THEN RAISE EXCEPTION 'grid scale'; END IF;
  RAISE NOTICE 'OK detect request carries frame + geometry';

  r := execution_finalize('ewf_exec', jsonb_build_object('detections', jsonb_build_object(
        '1', jsonb_build_object('uv', jsonb_build_array(320.0, 240.0), 'score', 1.0,
                                'detector', 'test', 'image_size', jsonb_build_array(1, 1),
                                'grid_scale', 1.0,
                                'frame', (SELECT still_sha FROM evidence.execution_hold
                                           WHERE exec_id='ewf_exec' AND hold_index=1)))));
  IF (execution_get('ewf_exec')->>'status') <> 'verified' THEN
    RAISE EXCEPTION 'finalize did not verify';
  END IF;
  IF NOT (execution_get('ewf_exec')->>'learned')::boolean THEN
    RAISE EXCEPTION 'finalize did not learn';
  END IF;
  SELECT count(*) INTO n1 FROM evidence.oracle_verdict WHERE exec_id='ewf_exec';
  IF n1 <> 1 THEN RAISE EXCEPTION 'expected one verdict, got %', n1; END IF;
  RAISE NOTICE 'OK finalize verified + learned';

  -- Replay: oracle_learn returns the recorded verdict, no second row/fold.
  SELECT coalesce(sum(n),0) INTO n1 FROM learning.accuracy_cell WHERE rig_fingerprint='ewf_rig';
  PERFORM execution_finalize('ewf_exec', jsonb_build_object('detections', '{}'::jsonb));
  SELECT count(*) INTO n2 FROM evidence.oracle_verdict WHERE exec_id='ewf_exec';
  IF n2 <> 1 THEN RAISE EXCEPTION 'replay added a verdict'; END IF;
  SELECT coalesce(sum(n),0) INTO n2 FROM learning.accuracy_cell WHERE rig_fingerprint='ewf_rig';
  IF n2 <> n1 THEN RAISE EXCEPTION 'replay double-counted cells: % -> %', n1, n2; END IF;
  RAISE NOTICE 'OK finalize replay idempotent (cells %)', n2;

  -- The play claim is one-shot even if the status is manually reset: the
  -- attempts counter, not only `status`, is the guard.
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, status, attempts)
  VALUES ('ewf_retry', 'ewf_plan', 'b601', 'pending', 1);
  IF execution_claim_play('ewf_retry') <> false THEN
    RAISE EXCEPTION 'attempts=1 row reclaimed motion'; END IF;
  RAISE NOTICE 'OK one-shot play claim (attempts guard)';

  -- An abort WITH telemetry is played-but-failed: measured, not dead.
  r := execution_prepare('ewf_abort', 'ewf_plan', 'b601', 'ewf_ep', 'ewf', 10);
  IF NOT execution_claim_play('ewf_abort') THEN RAISE EXCEPTION 'abort claim'; END IF;
  r := execution_after_play('ewf_abort', jsonb_build_object(
        'ok', false, 'claimed', true, 'executed', false, 'error', 'edge aborted',
        'telemetry', jsonb_build_object('version', 2, 'abort', jsonb_build_object('t', 0.4),
                                        'samples', '[]'::jsonb, 'kp', jsonb_build_object('2', 150))));
  IF r->>'status' <> 'played' THEN RAISE EXCEPTION 'abort should be played, got %', r; END IF;
  IF r->>'error' <> 'edge aborted' THEN RAISE EXCEPTION 'abort error lost'; END IF;
  IF NOT EXISTS (SELECT 1 FROM evidence.telemetry WHERE exec_id='ewf_abort') THEN
    RAISE EXCEPTION 'abort telemetry not stored';
  END IF;
  RAISE NOTICE 'OK abort-with-telemetry is measured, not dead';

  -- A play with no telemetry at all is dead-lettered.
  r := execution_prepare('ewf_notelem2', 'ewf_plan', 'b601', 'ewf_ep', 'ewf', 10);
  IF NOT execution_claim_play('ewf_notelem2') THEN RAISE EXCEPTION 'notelem claim'; END IF;
  r := execution_after_play('ewf_notelem2', jsonb_build_object('ok', false, 'claimed', true,
                                                              'executed', false, 'error', 'no log'));
  IF r->>'status' <> 'dead' THEN RAISE EXCEPTION 'no-telemetry abort should be dead, got %', r; END IF;
  RAISE NOTICE 'OK no-telemetry play dead-letters';
END $$;

SELECT 'EXECUTE ORACLE WORKFLOW TESTS PASSED' AS result;
