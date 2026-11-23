-- 0021: durable execute -> oracle workflow (offline extraction, no live grabs).
--
-- 0020 encoded the two replay hazards: motion is at-most-once (claim before
-- play) and learning is idempotent (oracle_learn).  This migration wires the
-- chain that used to live in capture_server's grab timer into one pg_durable
-- graph:
--
--   prepare -> claim+play (isolated activity) -> wait for the recording
--           -> extract stills offline -> detect (isolated activity)
--           -> oracle_compute + oracle_learn
--
-- The hold stills are now extracted from the permanent executed-trajectory
-- recording (`evidence.media_fragment`), not grabbed live at playback time.
-- `evidence.execution_hold` records the mapping from a hold to its extracted
-- still; the still itself remains in `evidence.still`/`evidence.blob`.
--
-- Every step is idempotent so the durable graph can be replayed:
--   * execution_prepare / execution_claim_play / execution_mark_played
--   * execution_store_stills (content-addressed blobs; unique execution_hold)
--   * execution_finalize -> oracle_compute -> oracle_learn (learned flag)
--
-- The activity service (substrate/workflow_service.py) owns native OpenCV /
-- playback work and is reached with df.http(), so a decode/edge crash fails the
-- step, never the Postgres cluster (spec §2 crash-isolation invariant).

ALTER TABLE evidence.execution
  ADD COLUMN IF NOT EXISTS t_offset            double precision NOT NULL DEFAULT 0.0,
  ADD COLUMN IF NOT EXISTS hold_windows        jsonb NOT NULL DEFAULT '[]'::jsonb,
  ADD COLUMN IF NOT EXISTS record_deadline     timestamptz,
  ADD COLUMN IF NOT EXISTS record_timeout_s   integer NOT NULL DEFAULT 120,
  ADD COLUMN IF NOT EXISTS play_started_at     timestamptz,
  ADD COLUMN IF NOT EXISTS recording_started_at timestamptz,
  ADD COLUMN IF NOT EXISTS error               text;

-- One extracted still per hold. The still row already carries the blob FK; this
-- table only adds the (execution, hold_index) -> still mapping the detector and
-- oracle need. The refcount trigger is attached for the same reason as the
-- other sha columns (a blob must stay while this row references it).
CREATE TABLE IF NOT EXISTS evidence.execution_hold (
  exec_id     text NOT NULL REFERENCES evidence.execution(exec_id) ON DELETE CASCADE,
  hold_index  integer NOT NULL,
  still_sha   text NOT NULL REFERENCES evidence.blob(sha256),
  t           double precision,
  score       double precision,
  created_at  timestamptz NOT NULL DEFAULT now(),
  PRIMARY KEY (exec_id, hold_index)
);

DROP TRIGGER IF EXISTS blob_ref_exechold ON evidence.execution_hold;
CREATE TRIGGER blob_ref_exechold AFTER INSERT OR DELETE ON evidence.execution_hold
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');

-- Flatten a jsonb number array of any nesting depth (K is 3x3, T is 4x4, dist
-- is 1x5) into the float8[] the kinematics functions take.
CREATE OR REPLACE FUNCTION jsonb_numbers(p jsonb) RETURNS float8[] AS $$
  WITH RECURSIVE walk(v, path) AS (
    SELECT CASE WHEN jsonb_typeof(p) = 'string' THEN (p #>> '{}')::jsonb ELSE p END,
           ARRAY[]::int[]
    UNION ALL
    SELECT child, walk.path || t.idx::int
      FROM walk
      CROSS JOIN LATERAL jsonb_array_elements(
        CASE WHEN jsonb_typeof(walk.v) = 'array' THEN walk.v ELSE '[]'::jsonb END)
        WITH ORDINALITY AS t(child, idx)
  )
  SELECT array_agg((v #>> '{}')::float8 ORDER BY path)
    FROM walk
   WHERE jsonb_typeof(v) <> 'array';
$$ LANGUAGE sql IMMUTABLE;

-- Idempotent creation of the execution ledger row for a plan. The graph's
-- first step calls this; capture only supplies plan/arm/episode identity.
CREATE OR REPLACE FUNCTION execution_prepare(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_workspace text DEFAULT 'default', p_timeout_s integer DEFAULT 120
) RETURNS jsonb AS $$
DECLARE
  waypoints jsonb;
  windows jsonb;
BEGIN
  INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
  VALUES (p_workspace, p_workspace, 0, 0.04, 0.05) ON CONFLICT DO NOTHING;
  INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
  VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT DO NOTHING;
  IF p_episode_id IS NOT NULL THEN
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (p_episode_id, p_workspace, jsonb_build_array(p_arm_id), 'execution')
    ON CONFLICT DO NOTHING;
  END IF;

  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id,
                                 record_deadline, record_timeout_s)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id,
          now() + make_interval(secs => greatest(1, p_timeout_s)),
          greatest(1, p_timeout_s))
  ON CONFLICT (exec_id) DO UPDATE
    SET episode_id = COALESCE(evidence.execution.episode_id, EXCLUDED.episode_id),
        record_timeout_s = COALESCE(evidence.execution.record_timeout_s, EXCLUDED.record_timeout_s);

  SELECT convert_from(b.bytes, 'UTF8')::jsonb -> 'waypoints'
    INTO waypoints
    FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
   WHERE p.plan_id = p_plan_id;
  IF waypoints IS NULL THEN
    RAISE EXCEPTION 'execution_prepare: no plan % (or no trajectory blob)', p_plan_id;
  END IF;
  windows := hold_windows(waypoints);
  UPDATE evidence.execution
     SET hold_windows = windows
   WHERE exec_id = p_exec_id AND hold_windows = '[]'::jsonb;

  RETURN execution_get(p_exec_id) || jsonb_build_object('windows', windows);
END $$ LANGUAGE plpgsql;

-- Claim the play BEFORE moving. Replaces the 0020 version to stamp the play
-- clock, which the recording alignment needs (video_t = traj_t + t_offset).
CREATE OR REPLACE FUNCTION execution_claim_play(p_exec_id text) RETURNS boolean AS $$
DECLARE claimed boolean;
BEGIN
  UPDATE evidence.execution
     SET status = 'playing', attempts = attempts + 1,
         play_started_at = COALESCE(play_started_at, now()), updated_at = now()
   WHERE exec_id = p_exec_id AND status = 'pending';
  GET DIAGNOSTICS claimed = ROW_COUNT;
  RETURN claimed;
END $$ LANGUAGE plpgsql;

-- Post-play bookkeeping. The isolated activity returns the gateway response
-- (including telemetry, which the gateway also persisted). If the gateway did
-- not persist telemetry (mock / dry activity), store the returned block here so
-- the oracle and learning steps have the exact evidence. A refusal/abort with
-- telemetry is played-but-failed: it is measured and marked `error`, never
-- replayed. A play with no telemetry at all is dead-lettered.
CREATE OR REPLACE FUNCTION execution_after_play(p_exec_id text, p_response jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  tel jsonb;
  raw text;
  sha text;
  kp text;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;

  -- The activity owns the claim guard. A replay returns claimed=false without
  -- commanding the arm; return the ledger as-is (never dead-letter a replay).
  IF p_response IS NOT NULL AND p_response ? 'claimed'
     AND COALESCE((p_response->>'claimed')::boolean, true) = false THEN
    RETURN execution_get(p_exec_id);
  END IF;

  -- A refusal/abort WITH telemetry is still measured: the edge persisted the
  -- abort log, and the oracle needs it to attribute an execution fault. Only a
  -- play with no telemetry is dead-lettered (nothing measurable happened, or a
  -- crash left an ambiguous claim; either way, never auto-replay).
  IF p_response IS NULL THEN
    UPDATE evidence.execution
       SET status = 'dead', error = 'no play response', updated_at = now()
     WHERE exec_id = p_exec_id AND status IN ('pending','playing');
    RETURN execution_get(p_exec_id);
  END IF;

  SELECT blob_sha INTO sha FROM evidence.telemetry WHERE exec_id = p_exec_id;
  IF sha IS NULL THEN
    tel := p_response -> 'telemetry';
    IF tel IS NOT NULL AND jsonb_typeof(tel) = 'object' THEN
      raw := tel::text;
      sha := encode(digest(convert_to(raw, 'UTF8'), 'sha256'), 'hex');
      INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
      VALUES (sha, 'telemetry', convert_to(raw, 'UTF8'), octet_length(convert_to(raw, 'UTF8')), 'verified')
      ON CONFLICT (sha256) DO NOTHING;
      kp := COALESCE(
        (SELECT string_agg(key || ':' || value, ',' ORDER BY key::int)
           FROM jsonb_each_text(tel -> 'kp')),
        'default');
      INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
      VALUES (p_exec_id, ex.plan_id, ex.arm_id, sha, kp, '{}')
      ON CONFLICT (exec_id) DO NOTHING;
    END IF;
  END IF;

  IF sha IS NULL THEN
    UPDATE evidence.execution
       SET status = 'dead',
           error = COALESCE(p_response->>'error', 'no telemetry after play'),
           updated_at = now()
     WHERE exec_id = p_exec_id AND status IN ('pending','playing');
    RETURN execution_get(p_exec_id);
  END IF;

  PERFORM execution_mark_played(p_exec_id, sha);
  UPDATE evidence.execution
     SET record_deadline = now() + make_interval(secs => greatest(1, ex.record_timeout_s)),
         error = CASE
           WHEN COALESCE((p_response->>'ok')::boolean, false)
                AND COALESCE((p_response->>'executed')::boolean, false) THEN NULL
           ELSE COALESCE(p_response->>'error', 'play aborted')
         END,
         updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Recording readiness: telemetry is persisted and the ingested fragments cover
-- every hold window in the video clock. No holds -> ready once telemetry lands.
CREATE OR REPLACE FUNCTION execution_ready(p_exec_id text) RETURNS boolean AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND OR ex.episode_id IS NULL THEN RETURN false; END IF;
  IF ex.status NOT IN ('playing','played','verified') THEN RETURN false; END IF;
  IF NOT EXISTS (SELECT 1 FROM evidence.telemetry WHERE exec_id = p_exec_id) THEN
    RETURN false;
  END IF;
  IF ex.hold_windows IS NULL OR ex.hold_windows = '[]'::jsonb THEN
    RETURN true;
  END IF;
  RETURN wait_fragments(ex.episode_id, ex.hold_windows, ex.t_offset);
END $$ LANGUAGE plpgsql STABLE;

-- The loop condition: true while still waiting and before the record deadline.
-- The worker sleeps between evaluations; this is the only polling in the graph.
CREATE OR REPLACE FUNCTION execution_waiting(p_exec_id text) RETURNS boolean AS $$
  SELECT NOT execution_ready(p_exec_id)
     AND (record_deadline IS NULL OR now() < record_deadline)
    FROM evidence.execution WHERE exec_id = p_exec_id;
$$ LANGUAGE sql STABLE;

CREATE OR REPLACE FUNCTION execution_fail(p_exec_id text, p_reason text) RETURNS jsonb AS $$
BEGIN
  UPDATE evidence.execution
     SET status = 'dead', error = p_reason, updated_at = now()
   WHERE exec_id = p_exec_id AND status IN ('pending','playing','played');
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Attach an ingested recording to the execution: one fragment (seq 0) spanning
-- the recording, and the clock offset that maps trajectory time to video time.
CREATE OR REPLACE FUNCTION execution_add_recording(
  p_exec_id text, p_sha text, p_duration double precision, p_t_offset double precision,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE ex evidence.execution%ROWTYPE;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;
  INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end, codec)
  VALUES (ex.episode_id, p_sha, 0, 0.0, greatest(0.0, p_duration), p_codec)
  ON CONFLICT (episode_id, seq) DO NOTHING;
  UPDATE evidence.execution
     SET t_offset = p_t_offset, recording_started_at = now(), updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- The multi-segment form: the activity ingests each MediaMTX segment itself
-- (seq order, cumulative spans) and then only records the clock offset here.
CREATE OR REPLACE FUNCTION execution_set_recording(p_exec_id text, p_t_offset double precision)
RETURNS jsonb AS $$
BEGIN
  UPDATE evidence.execution
     SET t_offset = p_t_offset, recording_started_at = now(), updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Persist the extract activity's stills. Blob-first, content-addressed, and
-- idempotent: a replay inserts the same blob (ON CONFLICT) and the same
-- (execution, hold) mapping (ON CONFLICT DO NOTHING), never a duplicate still.
CREATE OR REPLACE FUNCTION execution_store_stills(p_exec_id text, p_extract jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  hold jsonb;
  data bytea;
  sha text;
  idx int;
  stored int := 0;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;

  FOR hold IN SELECT * FROM jsonb_array_elements(COALESCE(p_extract->'holds', '[]'::jsonb)) LOOP
    idx := (hold->>'index')::int;
    data := decode(replace(hold->>'frame_b64', E'\n', ''), 'base64');
    sha := encode(digest(data, 'sha256'), 'hex');
    INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
    VALUES (sha, 'still', data, octet_length(data), 'verified')
    ON CONFLICT (sha256) DO NOTHING;
    IF NOT EXISTS (
      SELECT 1 FROM evidence.still s
       WHERE s.episode_id = ex.episode_id AND s.sha256 = sha AND s.purpose = 'hold'
         AND COALESCE(s.t, -1) = COALESCE((hold->>'t')::float8, -1)) THEN
      INSERT INTO evidence.still(episode_id, sha256, arm_id, purpose, t, width, height)
      VALUES (ex.episode_id, sha, ex.arm_id, 'hold', (hold->>'t')::float8,
              NULLIF(hold->>'width', '')::int, NULLIF(hold->>'height', '')::int);
    END IF;
    INSERT INTO evidence.execution_hold(exec_id, hold_index, still_sha, t, score)
    VALUES (p_exec_id, idx, sha, (hold->>'t')::float8, (hold->>'score')::float8)
    ON CONFLICT (exec_id, hold_index) DO NOTHING;
    stored := stored + 1;
  END LOOP;
  RETURN jsonb_build_object('stored', stored, 'missed', COALESCE(p_extract->'missed', '[]'::jsonb));
END $$ LANGUAGE plpgsql;

-- Everything the isolated detector needs, in one small JSON envelope: the
-- extracted JPEG and the geometry/pose for that hold. The service never reads
-- the database directly, so this is the complete activity contract.
CREATE OR REPLACE FUNCTION execution_detect_request(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  tel jsonb;
  rig record;
  cal record;
  holds jsonb := '[]'::jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  SELECT convert_from(b.bytes, 'UTF8')::jsonb INTO tel
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  SELECT r.* INTO rig
    FROM evidence.episode e
    JOIN world.shared_rig r ON r.workspace_id = e.workspace_id AND r.status = 'active'
   WHERE e.episode_id = ex.episode_id
   ORDER BY r.fitted DESC NULLS LAST LIMIT 1;
  IF rig IS NULL THEN RETURN jsonb_build_object('holds', '[]'::jsonb, 'error', 'no active rig'); END IF;
  SELECT * INTO cal FROM world.calibration WHERE calib_fingerprint = rig.calib_fingerprint;

  SELECT jsonb_agg(jsonb_build_object(
           'index', h.hold_index,
           'frame_b64', replace(encode(b.bytes, 'base64'), E'\n', ''),
           'frame', h.still_sha,
           'image_size', jsonb_build_array(s.width, s.height),
           'grid_scale', CASE WHEN (cal.image_size->>0)::float8 > 0
                              THEN s.width::float8 / (cal.image_size->>0)::float8 ELSE 1.0 END,
           't', h.t,
           'q_meas', COALESCE((
             SELECT jsonb_build_array(
                      avg((smp->'meas'->>0)::float8), avg((smp->'meas'->>1)::float8),
                      avg((smp->'meas'->>2)::float8), avg((smp->'meas'->>3)::float8),
                      avg((smp->'meas'->>4)::float8), avg((smp->'meas'->>5)::float8))
               FROM jsonb_array_elements(COALESCE(tel->'samples', '[]'::jsonb)) smp
              WHERE (smp->>'t')::float8 BETWEEN
                    (SELECT (w->>'t_start')::float8 FROM jsonb_array_elements(ex.hold_windows) w
                      WHERE (w->>'index')::int = h.hold_index)
                AND (SELECT (w->>'t_end')::float8 FROM jsonb_array_elements(ex.hold_windows) w
                      WHERE (w->>'index')::int = h.hold_index)),
             '[0,0,0,0,0,0]'::jsonb)))
    INTO holds
    FROM evidence.execution_hold h
    JOIN evidence.still s ON s.episode_id = ex.episode_id AND s.sha256 = h.still_sha
                         AND s.purpose = 'hold'
    JOIN evidence.blob b ON b.sha256 = h.still_sha
   WHERE h.exec_id = p_exec_id;

  RETURN jsonb_build_object(
    'holds', COALESCE(holds, '[]'::jsonb),
    'K', cal.K, 'dist', cal.dist, 'T_base_cam', rig.T_base_cam,
    'tip_offset', rig.tip_offset,
    'image_size', cal.image_size);
END $$ LANGUAGE plpgsql STABLE;

-- Final compute: the SQL port of oracle_finish's residual path. The detector
-- has already produced per-hold pixels; this gathers plan/telemetry/rig/URDF,
-- calls oracle_compute, and folds the verdict through the idempotent
-- oracle_learn. Replaying finalize returns the recorded verdict, never a second
-- accuracy-cell fold.
CREATE OR REPLACE FUNCTION execution_finalize(p_exec_id text, p_detections jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  plan_json jsonb;
  tel jsonb;
  rig record;
  cal record;
  urdf_text text;
  waypoints jsonb;
  targets jsonb;
  samples jsonb;
  kp text;
  res jsonb;
  doc jsonb;
  recorded_doc jsonb;
  det jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.status NOT IN ('playing','played','verified') THEN
    RAISE EXCEPTION 'execution_finalize: % is %, not played', p_exec_id, ex.status;
  END IF;
  IF ex.learned AND ex.verdict_id IS NOT NULL THEN
    -- Replay: the verdict and the accuracy fold are write-once. Return the
    -- recorded doc instead of re-running the residual compute on new pixels.
    SELECT v.doc INTO recorded_doc FROM evidence.oracle_verdict v WHERE v.id = ex.verdict_id;
    RETURN jsonb_build_object('doc', recorded_doc, 'verdict', recorded_doc->'verdict',
                             'replayed', true);
  END IF;

  SELECT convert_from(b.bytes, 'UTF8')::jsonb INTO plan_json
    FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
   WHERE p.plan_id = ex.plan_id;
  SELECT convert_from(b.bytes, 'UTF8')::jsonb INTO tel
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  IF plan_json IS NULL OR tel IS NULL THEN
    RAISE EXCEPTION 'execution_finalize: % has no plan/telemetry evidence', p_exec_id;
  END IF;
  waypoints := COALESCE(plan_json->'waypoints', '[]'::jsonb);
  targets := COALESCE(plan_json->'capture'->'targets', plan_json->'targets', '[]'::jsonb);
  samples := COALESCE(tel->'samples', '[]'::jsonb);
  kp := COALESCE((SELECT string_agg(key || ':' || value, ',' ORDER BY key::int)
                    FROM jsonb_each_text(tel->'kp')), 'default');
  det := COALESCE(p_detections->'detections', '{}'::jsonb);

  SELECT r.* INTO rig
    FROM evidence.episode e
    JOIN world.shared_rig r ON r.workspace_id = e.workspace_id AND r.status = 'active'
   WHERE e.episode_id = ex.episode_id
   ORDER BY r.fitted DESC NULLS LAST LIMIT 1;
  IF rig IS NULL THEN RAISE EXCEPTION 'execution_finalize: % has no active rig', p_exec_id; END IF;
  SELECT * INTO cal FROM world.calibration WHERE calib_fingerprint = rig.calib_fingerprint;
  SELECT rm.urdf INTO urdf_text FROM world.robot_model rm ORDER BY rm.created_at LIMIT 1;
  IF urdf_text IS NULL THEN RAISE EXCEPTION 'execution_finalize: world.robot_model is empty'; END IF;

  res := oracle_compute(
    urdf_text, waypoints, samples, targets, det,
    jsonb_numbers(cal.K), jsonb_numbers(cal.dist),
    jsonb_numbers(rig.T_base_cam), jsonb_numbers(rig.tip_offset), kp,
    jsonb_build_object('plan_id', ex.plan_id, 'exec_id', p_exec_id,
                       'source', 'durable_offline_extract',
                       'created', to_char(now() AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS"Z"')));
  doc := (res->'doc') || jsonb_build_object('verdict', res->'verdict');
  PERFORM oracle_learn(p_exec_id, ex.arm_id, rig.rig_fingerprint, doc);
  RETURN jsonb_build_object('doc', doc, 'verdict', res->'verdict');
END $$ LANGUAGE plpgsql;

-- Adopt an execution that was played outside the durable graph (coverage /
-- calibration moves). It records the evidence and lets the same extraction/
-- detect/finalize chain run without inventing a second execution model.
CREATE OR REPLACE FUNCTION execution_adopt(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_t_offset double precision, p_workspace text DEFAULT 'default'
) RETURNS jsonb AS $$
DECLARE sha text;
BEGIN
  INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m)
  VALUES (p_workspace, p_workspace, 0, 0.04, 0.05) ON CONFLICT DO NOTHING;
  INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key)
  VALUES (p_arm_id, p_arm_id, 'unknown', 'none', 'default') ON CONFLICT DO NOTHING;
  IF p_episode_id IS NOT NULL THEN
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (p_episode_id, p_workspace, jsonb_build_array(p_arm_id), 'execution')
    ON CONFLICT DO NOTHING;
  END IF;
  SELECT blob_sha INTO sha FROM evidence.telemetry WHERE exec_id = p_exec_id;
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id, status,
                                 played_at, telemetry_sha, t_offset, hold_windows)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id, 'played', now(), sha,
          p_t_offset, COALESCE((
            SELECT hold_windows(convert_from(b.bytes, 'UTF8')::jsonb->'waypoints')
              FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
             WHERE p.plan_id = p_plan_id), '[]'::jsonb))
  ON CONFLICT (exec_id) DO NOTHING;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Build and start the durable graph. Keeping the graph construction in SQL
-- means the same definition is used by capture, tests, and psql; Python only
-- supplies the activity URL and awaits the instance.
CREATE OR REPLACE FUNCTION execute_workflow_start(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_activity_url text, p_workspace text DEFAULT 'default',
  p_timeout_s integer DEFAULT 120, p_poll_s integer DEFAULT 2,
  p_label text DEFAULT 'execute_oracle'
) RETURNS text AS $$
DECLARE
  url text := rtrim(p_activity_url, '/');
  body text := jsonb_build_object('exec_id', p_exec_id)::text;
  prep text;
  play text;
  wait text;
  finish text;
BEGIN
  prep := df.sql(format(
    'SELECT execution_prepare(%L,%L,%L,%L,%L,%s)',
    p_exec_id, p_plan_id, p_arm_id, p_episode_id, p_workspace, p_timeout_s));

  -- The activity owns the claim guard: it transitions pending -> playing
  -- BEFORE moving and returns claimed=false on a replay, so the graph can
  -- always call it and the arm still moves at most once.
  play := df.seq(
    df.as(df.http(url || '/execute', 'POST', body, '{}'::jsonb, greatest(30, p_timeout_s)), 'play'),
    df.sql(format('SELECT execution_after_play(%L, ($play.body)::jsonb)', p_exec_id))
  );

  wait := df.loop(df.sleep(greatest(1, p_poll_s)),
                  format('SELECT execution_waiting(%L)', p_exec_id));

  finish := df.if(
    format('SELECT execution_ready(%L)', p_exec_id),
    df.seq(
      df.seq(
        df.as(df.http(url || '/extract', 'POST', body, '{}'::jsonb, 300), 'extract'),
        df.sql(format('SELECT execution_store_stills(%L, ($extract.body)::jsonb)', p_exec_id))
      ),
      df.seq(
        df.as(df.http(url || '/detect', 'POST', body, '{}'::jsonb, 300), 'detect'),
        df.sql(format('SELECT execution_finalize(%L, ($detect.body)::jsonb)', p_exec_id))
      )
    ),
    df.sql(format('SELECT execution_fail(%L, %L)', p_exec_id,
                  'no recording/telemetry before the deadline'))
  );

  RETURN df.start(df.seq(prep, df.seq(play, df.seq(wait, finish))), p_label);
END $$ LANGUAGE plpgsql;
