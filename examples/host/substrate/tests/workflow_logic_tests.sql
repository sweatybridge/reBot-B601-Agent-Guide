-- 0022 tests: the decisions that moved from Python into Postgres.
-- Run: psql -v ON_ERROR_STOP=1 -f tests/workflow_logic_tests.sql
-- Rerun-safe: cleans its wfl_* fixtures first.

DELETE FROM evidence.execution_hold WHERE exec_id LIKE 'wfl_%';
DELETE FROM evidence.execution WHERE exec_id LIKE 'wfl_%';
DELETE FROM evidence.telemetry WHERE exec_id LIKE 'wfl_%';
DELETE FROM evidence.media_fragment WHERE episode_id = 'wfl_ep';
DELETE FROM evidence.still WHERE episode_id = 'wfl_ep';
DELETE FROM evidence.plan WHERE plan_id = 'wfl_plan';
DELETE FROM evidence.episode WHERE episode_id = 'wfl_ep';
DELETE FROM evidence.blob WHERE sha256 IN ('wfl_plan_blob', 'wfl_tel_blob', 'wfl_rec');
DELETE FROM learning.accuracy_cell WHERE rig_fingerprint = 'wfl_rig';
DELETE FROM learning.sag_map WHERE rig_fingerprint = 'wfl_rig';
DELETE FROM world.shared_rig WHERE rig_fingerprint = 'wfl_rig';
DELETE FROM world.calibration WHERE calib_fingerprint = 'wfl_calib';

INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
VALUES ('wfl', 'wfl', 0, 0.04, 0.05) ON CONFLICT DO NOTHING;
INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size)
VALUES ('wfl_calib', '[[600,0,320],[0,600,240],[0,0,1]]', '[0,0,0,0,0]', '[640,480]');
INSERT INTO world.shared_rig(rig_fingerprint, workspace_id, calib_fingerprint, T_base_cam,
                             image_size, tip_offset, method, provenance, status, rig_doc)
VALUES ('wfl_rig', 'wfl', 'wfl_calib',
        '[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]', '[640,480]', '[0,0,-0.035]',
        'base-marker', '{}', 'active', '{"fingerprint":"wfl_rig"}');
INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
VALUES ('wfl_ep', 'wfl', '["b601"]', 'execution');
INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status) VALUES
 ('wfl_plan_blob', 'trajectory',
  convert_to('{"waypoints":[{"q":[0.1,0.2,0.3,0.2,0.1,0,0],"t":0.0},{"q":[0.2,0.3,0.4,0.2,0.1,0,0],"t":0.4},{"q":[0.2,0.3,0.4,0.2,0.1,0,0],"t":1.6},{"q":[0.1,0.2,0.3,0.2,0.1,0,0],"t":2.0}]}', 'UTF8'),
  100, 'verified'),
 ('wfl_tel_blob', 'telemetry',
  convert_to('{"version":2,"samples":[{"t":0.0,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]},{"t":0.4,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]},{"t":1.6,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]},{"t":2.0,"cmd":[0.2,0.3,0.4,0.2,0.1,0,0],"meas":[0.2,0.3,0.4,0.2,0.1,0,0]}],"kp":{"2":150,"3":150}}', 'UTF8'),
  100, 'verified'),
 ('wfl_rec', 'fragment', '\x00'::bytea, 1, 'verified');
INSERT INTO evidence.plan(plan_id, episode_id, arm_id, blob_sha, metadata, snapshot)
VALUES ('wfl_plan', 'wfl_ep', 'b601', 'wfl_plan_blob', '{}', '{}');
INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
VALUES ('wfl_exec', 'wfl_plan', 'b601', 'wfl_tel_blob', '150', '{}');

DO $$
DECLARE
  r jsonb;
  w jsonb;
  v float8;
  n int;
BEGIN
  PERFORM execution_prepare('wfl_exec', 'wfl_plan', 'b601', 'wfl_ep', 'wfl', 10);
  -- Fragment timing is cumulative in the catalog.
  r := episode_add_segment('wfl_ep', 0, 'wfl_rec', 2.0);
  IF (r->>'t_start')::float8 <> 0 OR (r->>'t_end')::float8 <> 2 THEN
    RAISE EXCEPTION 'segment 0 span %', r; END IF;
  r := episode_add_segment('wfl_ep', 1, 'wfl_rec', 1.0);
  IF (r->>'t_start')::float8 <> 2 OR (r->>'t_end')::float8 <> 3 THEN
    RAISE EXCEPTION 'segment 1 span %', r; END IF;
  -- Idempotent replay returns the existing row, no second fragment.
  r := episode_add_segment('wfl_ep', 1, 'wfl_rec', 1.0);
  IF NOT (r->>'existing')::boolean THEN RAISE EXCEPTION 'segment replay'; END IF;
  RAISE NOTICE 'OK cumulative + idempotent segment timing';

  -- Extraction windows: hold 0.4..1.6 overlaps only fragment 0 (0..2).
  SELECT extraction_windows('wfl_exec') INTO r;
  w := r->'windows'->0;
  IF (w->>'index')::int <> 1
     OR (w->>'w0')::float8 <> 0.4 OR (w->>'w1')::float8 <> 1.6
     OR jsonb_array_length(w->'fragments') <> 1 OR (w->'fragments'->>0)::int <> 0 THEN
    RAISE EXCEPTION 'extraction window %', r; END IF;
  RAISE NOTICE 'OK extraction window -> fragment plan';

  -- Joint speed: a 0.1 rad ramp every 0.1 s is 1 rad/s.
  v := (extract_joint_speeds('[{"t":0,"meas":[0,0,0,0,0,0]},{"t":0.1,"meas":[0.1,0,0,0,0,0]},{"t":0.2,"meas":[0.2,0,0,0,0,0]}]'::jsonb)->1->>'v')::float8;
  IF abs(v - 1.0) > 1e-9 THEN RAISE EXCEPTION 'joint speed %', v; END IF;
  v := (extract_joint_speeds('[{"t":0,"meas":[0.3,0,0,0,0,0]},{"t":0.1,"meas":[0.3,0,0,0,0,0]}]'::jsonb)->0->>'v')::float8;
  IF v <> 0 THEN RAISE EXCEPTION 'plateau speed %', v; END IF;
  -- Nearest-sample lookup picks the closer of the bracketing samples.
  IF extract_speed_at(0.5, '[{"t":0.0,"v":1},{"t":0.6,"v":0}]'::jsonb) <> 0 THEN
    RAISE EXCEPTION 'nearest speed sample'; END IF;
  RAISE NOTICE 'OK joint speeds';

  -- Stillest selection: flat window -> midpoint tiebreak (1.0).
  r := select_hold_frame('wfl_exec', 1, ARRAY[0.5, 1.0, 1.5]::float8[]);
  IF r IS NULL OR abs((r->>'t')::float8 - 1.0) > 1e-9 THEN
    RAISE EXCEPTION 'midpoint tiebreak %', r; END IF;
  -- No frame in the window -> NULL (reported, not guessed).
  IF select_hold_frame('wfl_exec', 1, ARRAY[5.0, 6.0]::float8[]) IS NOT NULL THEN
    RAISE EXCEPTION 'a frame outside the window was chosen'; END IF;
  RAISE NOTICE 'OK stillest-frame selection';

  -- Dry telemetry is synthesized from the plan.
  SELECT execution_dry_telemetry('wfl_plan', '50,2:150,3:150') INTO r;
  IF jsonb_array_length(r->'samples') <> 4 THEN RAISE EXCEPTION 'dry samples %', r; END IF;
  IF (r->'kp'->>'1')::float8 <> 50 OR (r->'kp'->>'3')::float8 <> 150 THEN
    RAISE EXCEPTION 'dry kp %', r->'kp'; END IF;
  RAISE NOTICE 'OK dry telemetry synthesis';

  -- Receipt: no workflow instance, played ledger -> executed, not pending.
  PERFORM execution_prepare('wfl_exec', 'wfl_plan', 'b601', 'wfl_ep', 'wfl', 10);
  IF NOT execution_claim_play('wfl_exec') THEN RAISE EXCEPTION 'claim'; END IF;
  PERFORM execution_mark_played('wfl_exec', 'wfl_tel_blob');
  r := execution_receipt('wfl_exec');
  IF NOT (r->>'executed')::boolean OR (r->>'pending')::boolean OR NOT (r->>'ok')::boolean THEN
    RAISE EXCEPTION 'receipt %', r; END IF;
  IF (r->>'holds')::int <> 0 OR r->'ledger'->>'status' <> 'played' THEN
    RAISE EXCEPTION 'receipt ledger %', r; END IF;
  RAISE NOTICE 'OK execution receipt';

  -- t_offset shifts the video-clock window.
  PERFORM execution_set_recording('wfl_exec', 0.25);
  SELECT extraction_windows('wfl_exec') INTO r;
  IF (r->'windows'->0->>'w0')::float8 <> 0.65 OR (r->'windows'->0->>'w1')::float8 <> 1.85 THEN
    RAISE EXCEPTION 't_offset window %', r; END IF;
  RAISE NOTICE 'OK t_offset shifts the extraction window';
END $$;

SELECT 'WORKFLOW LOGIC TESTS PASSED' AS result;
