-- 0003_execute_workflow.sql — durable execute -> oracle workflow (pg_durable).
--
-- Fixes all P0/P1 workflow bugs and consolidates round trips (Items 2, 5, 6):
--   * No infinite loop on dead/crashed/unclaimed plays (execution_prepare sets
--     record_deadline on conflict; execution_waiting requires status IN
--     ('playing','played') AND record_deadline IS NOT NULL).
--   * Video-vs-trajectory clock alignment in select_hold_frame & extraction_plan
--     (subtracts ex.t_offset before looking up trajectory telemetry speeds).
--   * extraction_plan(exec_id) precomputes target stillest timestamp per hold
--     in SQL and returns only overlapping fragment metadata (3+N -> 1 round trip).
--   * execution_ingest_segments ingests all recording segments atomically under
--     an episode advisory lock (1+2S+1 -> 1 round trip).
--   * execution_dry_telemetry parses KP spec properly (comma-delimited, id:val).
--   * execution_finalize passes telemetry flags (abort/gripper_stall) AND arm_id
--     to oracle_compute, and oracle_learn folds accuracy + monitor_observe.
--   * execution_receipt requires ex.error IS NULL for ok=true and never fails open.
--   * execute_workflow_start deduplicates in-flight executions; coverage_workflow_start
--     stamps the real df.start() instance id (no literal '{sys_instance_id}').

-- Clean up superseded/overloaded signatures from earlier iterations.
DROP FUNCTION IF EXISTS execute_workflow_start(text,text,text,text,text,text,integer,integer,text);
DROP FUNCTION IF EXISTS execution_set_workflow(text,text);

CREATE OR REPLACE FUNCTION execution_get(p_exec_id text) RETURNS jsonb AS $$
  SELECT to_jsonb(e) FROM evidence.execution e WHERE exec_id = p_exec_id;
$$ LANGUAGE sql STABLE;

-- Idempotent prepare: ensures FK parents, creates/updates ledger row with
-- non-null record_deadline, and extracts hold windows from the plan blob.
CREATE OR REPLACE FUNCTION execution_prepare(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_workspace text DEFAULT 'default', p_timeout_s integer DEFAULT 120
) RETURNS jsonb AS $$
DECLARE
  waypoints jsonb;
  windows jsonb;
  tout int := greatest(1, COALESCE(p_timeout_s, 120));
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  IF p_episode_id IS NOT NULL THEN
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
      VALUES (p_episode_id, p_workspace, jsonb_build_array(p_arm_id), 'execution')
      ON CONFLICT DO NOTHING;
  END IF;

  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id,
                                 record_deadline, record_timeout_s)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id,
          now() + make_interval(secs => tout), tout)
  ON CONFLICT (exec_id) DO UPDATE SET
    episode_id       = COALESCE(evidence.execution.episode_id, EXCLUDED.episode_id),
    record_timeout_s = COALESCE(evidence.execution.record_timeout_s, EXCLUDED.record_timeout_s),
    record_deadline  = COALESCE(evidence.execution.record_deadline, EXCLUDED.record_deadline),
    updated_at       = now();

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

-- AT-MOST-ONCE motion guard: claim pending -> playing before moving the arm.
-- Also stamps play_started_at and pushes record_deadline out by record_timeout_s.
CREATE OR REPLACE FUNCTION execution_claim_play(p_exec_id text) RETURNS boolean AS $$
DECLARE claimed boolean;
BEGIN
  UPDATE evidence.execution
     SET status = 'playing',
         attempts = attempts + 1,
         play_started_at = COALESCE(play_started_at, now()),
         record_deadline = COALESCE(record_deadline, now() + make_interval(secs => greatest(1, record_timeout_s))),
         updated_at = now()
   WHERE exec_id = p_exec_id AND status = 'pending' AND attempts = 0;
  GET DIAGNOSTICS claimed = ROW_COUNT;
  RETURN claimed;
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_mark_played(p_exec_id text, p_telemetry_sha text)
RETURNS boolean AS $$
DECLARE done boolean;
BEGIN
  UPDATE evidence.execution
     SET status = 'played', played_at = now(), telemetry_sha = p_telemetry_sha,
         updated_at = now()
   WHERE exec_id = p_exec_id AND status = 'playing';
  GET DIAGNOSTICS done = ROW_COUNT;
  RETURN done;
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_fail(p_exec_id text, p_reason text) RETURNS jsonb AS $$
BEGIN
  UPDATE evidence.execution
     SET status = 'dead',
         error = COALESCE(NULLIF(error, ''), p_reason),
         updated_at = now()
   WHERE exec_id = p_exec_id AND status IN ('pending','playing','played');
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Compatibility wrapper so callers/tests using execution_mark_dead still work.
DROP FUNCTION IF EXISTS execution_mark_dead(text);
DROP FUNCTION IF EXISTS execution_begin(text, text, text, text);
DROP FUNCTION IF EXISTS execution_add_recording(text, text, float8, float8, text);
CREATE OR REPLACE FUNCTION execution_mark_dead(p_exec_id text, p_reason text DEFAULT 'activity failed after play claim')
RETURNS void AS $$
BEGIN
  PERFORM execution_fail(p_exec_id, p_reason);
END $$ LANGUAGE plpgsql;

-- Helper: serialize tel->'kp' object cleanly into '1:50,2:150,...'
CREATE OR REPLACE FUNCTION _kp_key_from_tel(p_tel jsonb) RETURNS text AS $$
BEGIN
  IF p_tel IS NULL OR jsonb_typeof(p_tel->'kp') <> 'object' THEN
    RETURN 'default';
  END IF;
  RETURN COALESCE(
    (SELECT string_agg(k || ':' || v, ',' ORDER BY k::int)
       FROM jsonb_each_text(p_tel->'kp') e(k, v)
      WHERE k ~ '^[0-9]+$'),
    'default');
END $$ LANGUAGE plpgsql IMMUTABLE;

-- Post-play bookkeeping.
CREATE OR REPLACE FUNCTION execution_after_play(p_exec_id text, p_response jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  tel jsonb;
  raw bytea;
  sha text;
  kp text;
  tout int;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  tout := greatest(1, COALESCE(ex.record_timeout_s, 120));

  -- Replay after claim: never auto-replay; if telemetry already landed recover
  -- playing -> played, otherwise ensure record_deadline is bounded (no infinite loop).
  IF p_response IS NOT NULL AND p_response ? 'claimed'
     AND COALESCE((p_response->>'claimed')::boolean, true) = false THEN
    IF ex.status = 'playing' THEN
      SELECT blob_sha INTO sha FROM evidence.telemetry WHERE exec_id = p_exec_id;
      IF sha IS NOT NULL THEN
        PERFORM execution_mark_played(p_exec_id, sha);
        UPDATE evidence.execution
           SET record_deadline = now() + make_interval(secs => tout), updated_at = now()
         WHERE exec_id = p_exec_id;
      ELSE
        UPDATE evidence.execution
           SET status = 'dead',
               error = COALESCE(error, 'activity crashed after play claim with no telemetry'),
               updated_at = now()
         WHERE exec_id = p_exec_id;
      END IF;
    END IF;
    RETURN execution_get(p_exec_id);
  END IF;

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
      raw := convert_to(tel::text, 'UTF8');
      sha := encode(digest(raw, 'sha256'), 'hex');
      INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
        VALUES (sha, 'telemetry', raw, octet_length(raw), 'verified')
        ON CONFLICT (sha256) DO UPDATE SET created_at = now();
      kp := _kp_key_from_tel(tel);
      INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary)
        VALUES (p_exec_id, ex.plan_id, ex.arm_id, sha, kp, '{}'::jsonb)
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
     SET record_deadline = now() + make_interval(secs => tout),
         error = CASE
           WHEN COALESCE((p_response->>'ok')::boolean, false)
                AND COALESCE((p_response->>'executed')::boolean, false) THEN NULL
           ELSE COALESCE(p_response->>'error', 'play aborted')
         END,
         updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Gap-aware fragment readiness gate: verifies every hold window [w0, w1] is
-- covered by contiguous/overlapping fragments (not just min..max span).
CREATE OR REPLACE FUNCTION wait_fragments(
  p_episode_id text, p_windows jsonb, p_t_offset float8 DEFAULT 0.0
) RETURNS boolean AS $$
DECLARE
  w jsonb;
  w0 float8; w1 float8;
  f record;
  covered float8;
  ws jsonb := CASE WHEN jsonb_typeof(p_windows) = 'array' THEN p_windows ELSE '[]'::jsonb END;
BEGIN
  IF jsonb_array_length(ws) = 0 THEN RETURN true; END IF;
  IF NOT EXISTS (SELECT 1 FROM evidence.media_fragment WHERE episode_id = p_episode_id) THEN
    RETURN false;
  END IF;
  FOR w IN SELECT jsonb_array_elements(ws) LOOP
    w0 := (w->>'t_start')::float8 + COALESCE(p_t_offset, 0.0);
    w1 := (w->>'t_end')::float8 + COALESCE(p_t_offset, 0.0);
    IF w1 < w0 THEN SELECT w1, w0 INTO w0, w1; END IF;
    covered := NULL;
    FOR f IN
      SELECT t_start, t_end FROM evidence.media_fragment
       WHERE episode_id = p_episode_id
         AND t_end >= w0 - 1e-6 AND t_start <= w1 + 1e-6
       ORDER BY t_start, t_end
    LOOP
      IF covered IS NULL THEN
        IF f.t_start > w0 + 1e-6 THEN RETURN false; END IF;
        covered := f.t_end;
      ELSE
        IF f.t_start > covered + 0.05 THEN RETURN false; END IF;
        covered := greatest(covered, f.t_end);
      END IF;
    END LOOP;
    IF covered IS NULL OR covered < w1 - 1e-6 THEN
      RETURN false;
    END IF;
  END LOOP;
  RETURN true;
END $$ LANGUAGE plpgsql STABLE;

CREATE OR REPLACE FUNCTION execution_ready(p_exec_id text) RETURNS boolean AS $$
DECLARE ex evidence.execution%ROWTYPE;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND OR ex.episode_id IS NULL THEN RETURN false; END IF;
  IF ex.status NOT IN ('playing','played','verified') THEN RETURN false; END IF;
  IF NOT EXISTS (SELECT 1 FROM evidence.telemetry WHERE exec_id = p_exec_id) THEN
    RETURN false;
  END IF;
  IF ex.hold_windows IS NULL OR jsonb_typeof(ex.hold_windows) <> 'array' OR ex.hold_windows = '[]'::jsonb THEN
    RETURN true;
  END IF;
  -- Aborted plays with telemetry are ready to finalize immediately if no recording arrived
  -- (or if recording already arrived), so the oracle can attribute the execution fault.
  IF ex.error IS NOT NULL AND ex.status = 'played'
     AND NOT EXISTS (SELECT 1 FROM evidence.media_fragment WHERE episode_id = ex.episode_id) THEN
    RETURN true;
  END IF;
  RETURN wait_fragments(ex.episode_id, ex.hold_windows, ex.t_offset);
END $$ LANGUAGE plpgsql STABLE;

-- Loop condition: ONLY wait while status is actively in-flight ('playing'/'played'),
-- not yet ready, AND before a concrete record_deadline (eliminates infinite loop).
CREATE OR REPLACE FUNCTION execution_waiting(p_exec_id text) RETURNS boolean AS $$
  SELECT COALESCE((
    SELECT status IN ('playing', 'played')
       AND NOT execution_ready(p_exec_id)
       AND record_deadline IS NOT NULL
       AND now() < record_deadline
      FROM evidence.execution WHERE exec_id = p_exec_id
  ), false);
$$ LANGUAGE sql STABLE;

-- ---------------------------------------------------------------------------
-- Recording segment ingestion (locked, race-free; Item 6 single-call batch)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION episode_add_segment(
  p_episode_id text, p_seq integer, p_sha text, p_duration double precision,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE
  t0 float8;
  t1 float8;
  existing record;
BEGIN
  PERFORM pg_advisory_xact_lock(hashtext('seg:' || p_episode_id));
  SELECT t_start, t_end INTO existing
    FROM evidence.media_fragment
   WHERE episode_id = p_episode_id AND seq = p_seq;
  IF FOUND THEN
    RETURN jsonb_build_object('seq', p_seq, 't_start', existing.t_start,
                              't_end', existing.t_end, 'existing', true);
  END IF;
  SELECT COALESCE(max(t_end), 0.0) INTO t0
    FROM evidence.media_fragment WHERE episode_id = p_episode_id;
  t1 := t0 + greatest(0.0, COALESCE(p_duration, 0.0));
  INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end, codec)
    VALUES (p_episode_id, p_sha, p_seq, t0, t1, COALESCE(p_codec, 'mp4'));
  RETURN jsonb_build_object('seq', p_seq, 't_start', t0, 't_end', t1, 'existing', false);
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_add_segment(
  p_exec_id text, p_seq integer, p_sha text, p_duration double precision,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE ep text;
BEGIN
  SELECT episode_id INTO ep FROM evidence.execution WHERE exec_id = p_exec_id;
  IF ep IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;
  RETURN episode_add_segment(ep, p_seq, p_sha, p_duration, p_codec);
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_set_recording(p_exec_id text, p_t_offset double precision)
RETURNS jsonb AS $$
BEGIN
  UPDATE evidence.execution
     SET t_offset = COALESCE(p_t_offset, 0.0), recording_started_at = now(), updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Item 6: ingest all segments + set t_offset in ONE atomic SQL call.
-- p_durations[i] corresponds to p_blobs[i] (1-indexed, seq = i-1).
CREATE OR REPLACE FUNCTION execution_ingest_segments(
  p_exec_id text, p_blobs bytea[], p_durations float8[], p_t_offset float8,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE
  ep text;
  n int := COALESCE(array_length(p_blobs, 1), 0);
  i int;
  sha text;
  added jsonb;
  last_end float8 := 0.0;
BEGIN
  SELECT episode_id INTO ep FROM evidence.execution WHERE exec_id = p_exec_id;
  IF ep IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;
  PERFORM pg_advisory_xact_lock(hashtext('seg:' || ep));
  FOR i IN 1..n LOOP
    sha := encode(digest(p_blobs[i], 'sha256'), 'hex');
    INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
      VALUES (sha, 'fragment', p_blobs[i], octet_length(p_blobs[i]), 'verified')
      ON CONFLICT (sha256) DO UPDATE SET created_at = now();
    added := episode_add_segment(ep, i - 1, sha, COALESCE(p_durations[i], 10.0), p_codec);
    last_end := COALESCE((added->>'t_end')::float8, last_end);
  END LOOP;
  UPDATE evidence.execution
     SET t_offset = COALESCE(p_t_offset, 0.0), recording_started_at = now(), updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN jsonb_build_object(
    'segments', n, 'duration_s', round(last_end::numeric, 3),
    't_offset', round(COALESCE(p_t_offset, 0.0)::numeric, 3));
END $$ LANGUAGE plpgsql;

-- Same atomic batch ingest for coverage episodes (before execution row exists).
CREATE OR REPLACE FUNCTION episode_ingest_segments(
  p_episode_id text, p_workspace text, p_arm_id text,
  p_blobs bytea[], p_durations float8[], p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE
  n int := COALESCE(array_length(p_blobs, 1), 0);
  i int;
  sha text;
  added jsonb;
  last_end float8 := 0.0;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
    VALUES (p_episode_id, p_workspace,
            CASE WHEN p_arm_id IS NOT NULL THEN jsonb_build_array(p_arm_id) ELSE '[]'::jsonb END,
            'execution')
    ON CONFLICT (episode_id) DO NOTHING;
  PERFORM pg_advisory_xact_lock(hashtext('seg:' || p_episode_id));
  FOR i IN 1..n LOOP
    sha := encode(digest(p_blobs[i], 'sha256'), 'hex');
    INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
      VALUES (sha, 'fragment', p_blobs[i], octet_length(p_blobs[i]), 'verified')
      ON CONFLICT (sha256) DO UPDATE SET created_at = now();
    added := episode_add_segment(p_episode_id, i - 1, sha, COALESCE(p_durations[i], 10.0), p_codec);
    last_end := COALESCE((added->>'t_end')::float8, last_end);
  END LOOP;
  RETURN jsonb_build_object('segments', n, 'duration_s', round(last_end::numeric, 3));
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Dry telemetry builder (fixed KP spec parser: comma-separated, id:value)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION execution_dry_telemetry(
  p_plan_id text, p_kp_spec text DEFAULT '50,2:150,3:150'
) RETURNS jsonb AS $$
DECLARE
  waypoints jsonb;
  samples jsonb;
  kp jsonb := '{}'::jsonb;
  base_kp float8 := 50.0;
  part text;
  trimmed text;
  kv text[];
  m int;
BEGIN
  SELECT convert_from(b.bytes, 'UTF8')::jsonb -> 'waypoints'
    INTO waypoints
    FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
   WHERE p.plan_id = p_plan_id;
  IF waypoints IS NULL OR jsonb_typeof(waypoints) <> 'array' THEN
    RAISE EXCEPTION 'execution_dry_telemetry: no plan % (or no trajectory blob)', p_plan_id;
  END IF;
  SELECT COALESCE(jsonb_agg(
           jsonb_build_object('t', (e->>'t')::float8, 'cmd', e->'q', 'meas', e->'q')
           ORDER BY (e->>'t')::float8), '[]'::jsonb)
    INTO samples
    FROM jsonb_array_elements(waypoints) e
   WHERE e ? 'q';

  FOREACH part IN ARRAY string_to_array(COALESCE(NULLIF(trim(p_kp_spec), ''), '50,2:150,3:150'), ',') LOOP
    trimmed := trim(part);
    IF trimmed = '' THEN CONTINUE; END IF;
    IF position(':' IN trimmed) > 0 THEN
      kv := string_to_array(trimmed, ':');
      IF array_length(kv, 1) = 2 AND trim(kv[1]) ~ '^[1-6]$' THEN
        kp := kp || jsonb_build_object(trim(kv[1]), (trim(kv[2]))::float8);
      END IF;
    ELSE
      base_kp := trimmed::float8;
    END IF;
  END LOOP;
  FOR m IN 1..6 LOOP
    IF NOT (kp ? m::text) THEN
      kp := kp || jsonb_build_object(m::text, base_kp);
    END IF;
  END LOOP;

  RETURN jsonb_build_object(
    'format', 'rebot-trajectory-log', 'version', 2, 'dry', true,
    'samples', samples, 'kp', kp);
END $$ LANGUAGE plpgsql STABLE;

-- ---------------------------------------------------------------------------
-- Frame extraction planning & stillness scoring (Item 2 + P0 t_offset fix)
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION extract_joint_speeds(p_samples jsonb) RETURNS jsonb AS $$
DECLARE
  arr jsonb := CASE WHEN jsonb_typeof(p_samples) = 'array' THEN p_samples ELSE '[]'::jsonb END;
BEGIN
  RETURN COALESCE((
    WITH pts AS (
      SELECT ord,
             (s->>'t')::float8 AS t,
             ARRAY(SELECT jsonb_array_elements_text(s->'meas')::float8) AS q
        FROM jsonb_array_elements(arr) WITH ORDINALITY x(s, ord)
       WHERE jsonb_typeof(s->'meas') = 'array' AND jsonb_array_length(s->'meas') > 0
         AND (s->>'t') IS NOT NULL
    ),
    diffs AS (
      SELECT ord, t,
             t - lag(t) OVER (ORDER BY t, ord) AS dt,
             q,
             lag(q) OVER (ORDER BY t, ord) AS prev_q
        FROM pts
    )
    SELECT jsonb_agg(
             jsonb_build_object(
               't', t,
               'v', CASE
                 WHEN prev_q IS NULL OR dt IS NULL OR dt <= 1e-9 THEN 0.0
                 ELSE COALESCE((
                   SELECT max(abs(q[k] - prev_q[k])) / dt
                     FROM generate_series(1, least(array_length(q, 1), array_length(prev_q, 1))) k
                 ), 0.0)
               END
             ) ORDER BY t, ord)
      FROM diffs
  ), '[]'::jsonb);
END $$ LANGUAGE plpgsql IMMUTABLE;

CREATE OR REPLACE FUNCTION extract_speed_at(p_t float8, p_speeds jsonb) RETURNS float8 AS $$
DECLARE
  n int;
  i int;
  tprev float8;
  tnext float8;
BEGIN
  IF p_speeds IS NULL OR jsonb_typeof(p_speeds) <> 'array' THEN RETURN NULL; END IF;
  n := jsonb_array_length(p_speeds);
  IF n = 0 THEN RETURN NULL; END IF;
  SELECT min(ord) - 1 INTO i
    FROM jsonb_array_elements(p_speeds) WITH ORDINALITY e(v, ord)
   WHERE (v->>'t')::float8 >= p_t;
  IF i IS NULL THEN i := n - 1; END IF;
  IF i <= 0 THEN RETURN (p_speeds->0->>'v')::float8; END IF;
  IF i >= n THEN RETURN (p_speeds->(n - 1)->>'v')::float8; END IF;
  tprev := (p_speeds->(i - 1)->>'t')::float8;
  tnext := (p_speeds->i->>'t')::float8;
  IF (p_t - tprev) <= (tnext - p_t) THEN
    RETURN (p_speeds->(i - 1)->>'v')::float8;
  END IF;
  RETURN (p_speeds->i->>'v')::float8;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- Fixed P0 clock alignment: subtracts ex.t_offset from video-clock tf before
-- looking up trajectory-clock speeds, and never emits Infinity to jsonb.
CREATE OR REPLACE FUNCTION select_hold_frame(
  p_exec_id text, p_window_index integer, p_frame_times float8[]
) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  samples jsonb;
  speeds jsonb;
  w jsonb;
  w0 float8; w1 float8; mid float8;
  choice jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  SELECT (convert_from(b.bytes, 'UTF8')::jsonb) -> 'samples'
    INTO samples
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  IF samples IS NULL THEN RETURN NULL; END IF;
  SELECT e INTO w
    FROM jsonb_array_elements(COALESCE(ex.hold_windows, '[]'::jsonb)) e
   WHERE (e->>'index')::int = p_window_index;
  IF w IS NULL THEN RETURN NULL; END IF;
  w0 := (w->>'t_start')::float8 + ex.t_offset;
  w1 := (w->>'t_end')::float8 + ex.t_offset;
  IF w1 < w0 THEN SELECT w1, w0 INTO w0, w1; END IF;
  mid := 0.5 * (w0 + w1);
  speeds := extract_joint_speeds(samples);
  SELECT jsonb_build_object('t', tf, 'score', v, 'midpoint', mid,
                            'window', jsonb_build_array(w0, w1))
    INTO choice
    FROM (
      SELECT tf,
             extract_speed_at(tf - ex.t_offset, speeds) AS v,
             abs(tf - mid) AS d
        FROM unnest(COALESCE(p_frame_times, ARRAY[]::float8[])) tf
       WHERE tf >= w0 - 1e-6 AND tf <= w1 + 1e-6
    ) q
   ORDER BY v NULLS LAST, d, tf
   LIMIT 1;
  RETURN choice;
END $$ LANGUAGE plpgsql STABLE;

CREATE OR REPLACE FUNCTION extraction_windows(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  wins jsonb := '[]'::jsonb;
  w jsonb;
  w0 float8; w1 float8;
  seqs jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN
    RETURN jsonb_build_object('episode_id', NULL, 't_offset', ex.t_offset, 'windows', '[]'::jsonb);
  END IF;
  FOR w IN SELECT * FROM jsonb_array_elements(COALESCE(ex.hold_windows, '[]'::jsonb)) LOOP
    w0 := (w->>'t_start')::float8 + ex.t_offset;
    w1 := (w->>'t_end')::float8 + ex.t_offset;
    IF w1 < w0 THEN SELECT w1, w0 INTO w0, w1; END IF;
    SELECT COALESCE(jsonb_agg(f.seq ORDER BY f.seq), '[]'::jsonb) INTO seqs
      FROM evidence.media_fragment f
     WHERE f.episode_id = ex.episode_id
       AND f.t_end >= w0 - 1e-6 AND f.t_start <= w1 + 1e-6;
    wins := wins || jsonb_build_array(jsonb_build_object(
      'index', (w->>'index')::int,
      't_start', (w->>'t_start')::float8,
      't_end', (w->>'t_end')::float8,
      'w0', w0, 'w1', w1,
      'fragments', seqs));
  END LOOP;
  RETURN jsonb_build_object('episode_id', ex.episode_id, 't_offset', ex.t_offset, 'windows', wins);
END $$ LANGUAGE plpgsql STABLE;

-- Item 2: Single-call extraction plan — computes target stillest timestamp
-- `target_t` (video clock) and `score` per hold directly from telemetry speeds
-- AND returns only the overlapping fragment seqs (eliminating N per-hold SQL
-- calls and full-window frame buffering in workflow_service.py).
CREATE OR REPLACE FUNCTION extraction_plan(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  samples jsonb;
  speeds jsonb := '[]'::jsonb;
  wins jsonb := '[]'::jsonb;
  w jsonb;
  w0 float8; w1 float8; mid float8;
  t_start float8; t_end float8;
  best_t float8; best_score float8;
  seqs jsonb;
  needed_seqs int[] := ARRAY[]::int[];
  frags jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN
    RETURN jsonb_build_object('episode_id', NULL, 't_offset', ex.t_offset,
                              'windows', '[]'::jsonb, 'needed_seqs', '[]'::jsonb);
  END IF;
  SELECT (convert_from(b.bytes, 'UTF8')::jsonb) -> 'samples'
    INTO samples
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  IF samples IS NOT NULL THEN
    speeds := extract_joint_speeds(samples);
  END IF;

  FOR w IN SELECT * FROM jsonb_array_elements(COALESCE(ex.hold_windows, '[]'::jsonb)) LOOP
    t_start := (w->>'t_start')::float8;
    t_end   := (w->>'t_end')::float8;
    w0 := t_start + ex.t_offset;
    w1 := t_end + ex.t_offset;
    IF w1 < w0 THEN SELECT w1, w0 INTO w0, w1; END IF;
    mid := 0.5 * (w0 + w1);

    -- Pick the stillest telemetry sample inside [t_start, t_end], tiebreak by midpoint.
    SELECT (s->>'t')::float8 + ex.t_offset, (s->>'v')::float8
      INTO best_t, best_score
      FROM jsonb_array_elements(speeds) s
     WHERE (s->>'t')::float8 >= least(t_start, t_end) - 1e-6
       AND (s->>'t')::float8 <= greatest(t_start, t_end) + 1e-6
     ORDER BY (s->>'v')::float8 ASC, abs(((s->>'t')::float8 + ex.t_offset) - mid) ASC, (s->>'t')::float8 ASC
     LIMIT 1;
    IF best_t IS NULL THEN
      best_t := mid;
      best_score := extract_speed_at(mid - ex.t_offset, speeds);
    END IF;

    SELECT COALESCE(jsonb_agg(f.seq ORDER BY f.seq), '[]'::jsonb) INTO seqs
      FROM evidence.media_fragment f
     WHERE f.episode_id = ex.episode_id
       AND f.t_end >= w0 - 1e-6 AND f.t_start <= w1 + 1e-6;
    needed_seqs := needed_seqs || ARRAY(
      SELECT f.seq FROM evidence.media_fragment f
       WHERE f.episode_id = ex.episode_id
         AND f.t_end >= w0 - 1e-6 AND f.t_start <= w1 + 1e-6
    );
    wins := wins || jsonb_build_array(jsonb_build_object(
      'index', (w->>'index')::int,
      't_start', t_start, 't_end', t_end,
      'w0', w0, 'w1', w1,
      'target_t', best_t, 'score', best_score, 'midpoint', mid,
      'fragments', seqs));
  END LOOP;

  SELECT COALESCE(jsonb_agg(jsonb_build_object(
           'seq', f.seq, 't_start', f.t_start, 't_end', f.t_end, 'codec', f.codec)
           ORDER BY f.seq), '[]'::jsonb)
    INTO frags
    FROM evidence.media_fragment f
   WHERE f.episode_id = ex.episode_id
     AND f.seq = ANY(needed_seqs);

  RETURN jsonb_build_object(
    'episode_id', ex.episode_id,
    't_offset', ex.t_offset,
    'windows', wins,
    'fragments', frags,
    'needed_seqs', to_jsonb(ARRAY(SELECT DISTINCT x FROM unnest(needed_seqs) x ORDER BY x)));
END $$ LANGUAGE plpgsql STABLE;

-- ---------------------------------------------------------------------------
-- Store stills, build detect request, finalize + idempotent oracle_learn
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION execution_store_stills(p_exec_id text, p_extract jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  hold jsonb;
  data bytea;
  sha text;
  idx int;
  ht float8;
  stored int := 0;
  ins int;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;

  FOR hold IN SELECT * FROM jsonb_array_elements(
    CASE WHEN jsonb_typeof(p_extract->'holds') = 'array' THEN p_extract->'holds' ELSE '[]'::jsonb END
  ) LOOP
    idx := (hold->>'index')::int;
    ht  := (hold->>'t')::float8;
    data := decode(replace(hold->>'frame_b64', E'\n', ''), 'base64');
    sha := encode(digest(data, 'sha256'), 'hex');
    INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status)
      VALUES (sha, 'still', data, octet_length(data), 'verified')
      ON CONFLICT (sha256) DO UPDATE SET created_at = now();
    IF NOT EXISTS (
      SELECT 1 FROM evidence.still s
       WHERE s.episode_id = ex.episode_id AND s.sha256 = sha AND s.purpose = 'hold'
         AND s.t IS NOT DISTINCT FROM ht) THEN
      INSERT INTO evidence.still(episode_id, sha256, arm_id, purpose, t, width, height)
      VALUES (ex.episode_id, sha, ex.arm_id, 'hold', ht,
              NULLIF(hold->>'width', '')::int, NULLIF(hold->>'height', '')::int);
    END IF;
    INSERT INTO evidence.execution_hold(exec_id, hold_index, still_sha, t, score)
      VALUES (p_exec_id, idx, sha, ht, (hold->>'score')::float8)
      ON CONFLICT (exec_id, hold_index) DO NOTHING;
    GET DIAGNOSTICS ins = ROW_COUNT;
    stored := stored + ins;
  END LOOP;
  RETURN jsonb_build_object(
    'stored', stored,
    'missed', CASE WHEN jsonb_typeof(p_extract->'missed') = 'array' THEN p_extract->'missed' ELSE '[]'::jsonb END);
END $$ LANGUAGE plpgsql;

-- Fixed P1 bugs: LATERAL still join (no Cartesian fan-out), HAVING count(*)>0
-- (no [null,...] -> NaN), 1e-6 tolerance, deterministic ORDER BY hold_index.
CREATE OR REPLACE FUNCTION execution_detect_request(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  tel jsonb;
  rig record;
  cal record;
  holds jsonb := '[]'::jsonb;
  samples_arr jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  SELECT convert_from(b.bytes, 'UTF8')::jsonb INTO tel
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  samples_arr := CASE WHEN jsonb_typeof(tel->'samples') = 'array' THEN tel->'samples' ELSE '[]'::jsonb END;

  SELECT r.* INTO rig
    FROM evidence.episode e
    JOIN world.shared_rig r ON r.workspace_id = e.workspace_id AND r.status = 'active'
   WHERE e.episode_id = ex.episode_id
   ORDER BY r.fitted DESC NULLS LAST LIMIT 1;
  IF rig IS NULL THEN
    RETURN jsonb_build_object('holds', '[]'::jsonb, 'error', 'no active rig');
  END IF;
  SELECT * INTO cal FROM world.calibration WHERE calib_fingerprint = rig.calib_fingerprint;

  SELECT jsonb_agg(jsonb_build_object(
           'index', h.hold_index,
           'frame_b64', replace(encode(b.bytes, 'base64'), E'\n', ''),
           'frame', h.still_sha,
           'image_size', jsonb_build_array(s.width, s.height),
           'grid_scale', CASE WHEN (cal.image_size->>0)::float8 > 0 AND s.width IS NOT NULL
                              THEN s.width::float8 / (cal.image_size->>0)::float8 ELSE 1.0 END,
           't', h.t,
           'q_meas', COALESCE((
             SELECT jsonb_build_array(
                      avg((smp->'meas'->>0)::float8), avg((smp->'meas'->>1)::float8),
                      avg((smp->'meas'->>2)::float8), avg((smp->'meas'->>3)::float8),
                      avg((smp->'meas'->>4)::float8), avg((smp->'meas'->>5)::float8))
               FROM jsonb_array_elements(samples_arr) smp
              WHERE (smp->>'t')::float8 BETWEEN
                    (SELECT (w->>'t_start')::float8 - 1e-6 FROM jsonb_array_elements(ex.hold_windows) w
                      WHERE (w->>'index')::int = h.hold_index LIMIT 1)
                AND (SELECT (w->>'t_end')::float8 + 1e-6 FROM jsonb_array_elements(ex.hold_windows) w
                      WHERE (w->>'index')::int = h.hold_index LIMIT 1)
             HAVING count(*) > 0),
             '[0,0,0,0,0,0]'::jsonb))
           ORDER BY h.hold_index)
    INTO holds
    FROM evidence.execution_hold h
    JOIN evidence.blob b ON b.sha256 = h.still_sha
    LEFT JOIN LATERAL (
      SELECT width, height FROM evidence.still st
       WHERE st.episode_id = ex.episode_id AND st.sha256 = h.still_sha AND st.purpose = 'hold'
       ORDER BY abs(COALESCE(st.t, 0) - COALESCE(h.t, 0)), st.id DESC
       LIMIT 1
    ) s ON true
   WHERE h.exec_id = p_exec_id;

  RETURN jsonb_build_object(
    'holds', COALESCE(holds, '[]'::jsonb),
    'K', cal.K, 'dist', cal.dist, 'T_base_cam', rig.T_base_cam,
    'tip_offset', rig.tip_offset, 'image_size', cal.image_size);
END $$ LANGUAGE plpgsql STABLE;

-- Idempotent learning: verdict + accuracy cells + monitor_observe ONCE per exec_id.
CREATE OR REPLACE FUNCTION oracle_learn(
  p_exec_id text, p_arm_id text, p_rig_fingerprint text, p_doc jsonb,
  p_threshold_mm float8 DEFAULT NULL
) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  v jsonb; cfg jsonb; vid bigint;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.learned THEN
    SELECT doc->'verdict' INTO v FROM evidence.oracle_verdict WHERE id = ex.verdict_id;
    RETURN coalesce(v, '{}'::jsonb);
  END IF;
  IF NOT EXISTS (SELECT 1 FROM evidence.telemetry WHERE exec_id = p_exec_id) THEN
    RAISE EXCEPTION 'oracle_learn requires persisted telemetry for %', p_exec_id;
  END IF;
  IF ex.status NOT IN ('playing', 'played', 'verified') THEN
    RAISE EXCEPTION 'oracle_learn: execution % has status %, not played', p_exec_id, ex.status;
  END IF;
  v := coalesce(p_doc->'verdict', '{}'::jsonb);
  INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc)
  VALUES (ex.plan_id, p_exec_id, p_arm_id,
          coalesce(v->>'layer', 'none'), coalesce(v->>'confidence', 'low'),
          coalesce(v->'problems', '[]'::jsonb), p_doc)
  RETURNING id INTO vid;
  SELECT config_doc->'learning' INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  PERFORM accuracy_cell_update(p_arm_id, p_rig_fingerprint, p_doc, cfg, p_threshold_mm);
  PERFORM monitor_observe(p_arm_id, p_rig_fingerprint, p_doc);
  UPDATE evidence.execution
     SET learned = true, verdict_id = vid, status = 'verified', updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN v;
END $$ LANGUAGE plpgsql;

-- Finalize: passes telemetry fault flags + ex.arm_id into oracle_compute.
CREATE OR REPLACE FUNCTION execution_finalize(p_exec_id text, p_detections jsonb)
RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  plan_json jsonb;
  tel jsonb;
  tel_meta jsonb;
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
    SELECT v.doc INTO recorded_doc FROM evidence.oracle_verdict v WHERE v.id = ex.verdict_id;
    RETURN jsonb_build_object('doc', recorded_doc, 'verdict', recorded_doc->'verdict', 'replayed', true);
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
  waypoints := CASE WHEN jsonb_typeof(plan_json->'waypoints') = 'array' THEN plan_json->'waypoints' ELSE '[]'::jsonb END;
  targets := CASE
    WHEN jsonb_typeof(plan_json->'capture'->'targets') = 'array' THEN plan_json->'capture'->'targets'
    WHEN jsonb_typeof(plan_json->'targets') = 'array' THEN plan_json->'targets'
    ELSE '[]'::jsonb END;
  samples := CASE WHEN jsonb_typeof(tel->'samples') = 'array' THEN tel->'samples' ELSE '[]'::jsonb END;
  kp := _kp_key_from_tel(tel);
  det := CASE WHEN jsonb_typeof(p_detections->'detections') = 'object' THEN p_detections->'detections' ELSE '{}'::jsonb END;

  -- Forward execution fault metadata so oracle_attribute reports abort/gripper-stall.
  tel_meta := jsonb_strip_nulls(jsonb_build_object(
    'version', tel->'version', 'abort', tel->'abort', 'gripper_stall', tel->'gripper_stall',
    'tick', tel->'tick', 'follow_error_max', tel->'follow_error_max',
    'kp', tel->'kp', 'kd', tel->'kd'));

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
    jsonb_build_object(
      'plan_id', ex.plan_id, 'exec_id', p_exec_id,
      'telemetry', tel_meta,
      'source', 'durable_offline_extract',
      'created', to_char(now() AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS"Z"')),
    ex.arm_id);
  doc := (res->'doc') || jsonb_build_object('verdict', res->'verdict');
  PERFORM oracle_learn(p_exec_id, ex.arm_id, rig.rig_fingerprint, doc, NULL);
  RETURN jsonb_build_object('doc', doc, 'verdict', res->'verdict');
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_adopt(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_t_offset double precision, p_workspace text DEFAULT 'default'
) RETURNS jsonb AS $$
DECLARE sha text;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  IF p_episode_id IS NOT NULL THEN
    INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind)
      VALUES (p_episode_id, p_workspace, jsonb_build_array(p_arm_id), 'execution')
      ON CONFLICT DO NOTHING;
  END IF;
  SELECT blob_sha INTO sha FROM evidence.telemetry WHERE exec_id = p_exec_id;
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id, status,
                                 played_at, telemetry_sha, t_offset, hold_windows)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id, 'played', now(), sha,
          COALESCE(p_t_offset, 0.0), COALESCE((
            SELECT hold_windows(convert_from(b.bytes, 'UTF8')::jsonb->'waypoints')
              FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
             WHERE p.plan_id = p_plan_id), '[]'::jsonb))
  ON CONFLICT (exec_id) DO NOTHING;
  RETURN execution_get(p_exec_id);
END $$ LANGUAGE plpgsql;

-- Execution receipt: requires ex.error IS NULL for ok=true and never fails open
-- when df.status() throws.
CREATE OR REPLACE FUNCTION execution_receipt(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  ws text;
  ws_err boolean := false;
  v jsonb;
  d jsonb;
  n integer;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN
    RETURN jsonb_build_object('ok', false, 'executed', false, 'error', 'unknown execution');
  END IF;
  IF ex.workflow_instance IS NOT NULL THEN
    BEGIN
      ws := df.status(ex.workflow_instance);
    EXCEPTION WHEN OTHERS THEN
      ws := NULL;
      ws_err := true;
    END;
  END IF;
  SELECT doc, doc -> 'verdict' INTO d, v
    FROM evidence.oracle_verdict WHERE id = ex.verdict_id;
  SELECT count(*) INTO n FROM evidence.execution_hold WHERE exec_id = p_exec_id;
  RETURN jsonb_build_object(
    'ok', ex.status IN ('played','verified')
          AND ex.error IS NULL
          AND NOT ws_err
          AND COALESCE(ws, 'completed') = 'completed',
    'executed', ex.status IN ('playing','played','verified'),
    'pending', ws IS NOT NULL AND ws NOT IN ('completed','failed','cancelled'),
    'workflow', ex.workflow_instance,
    'workflow_status', ws,
    'ledger', execution_get(p_exec_id),
    'verdict', v,
    'doc', d,
    'holds', n,
    'error', ex.error);
END $$ LANGUAGE plpgsql STABLE;

-- Item 5: Single-call await + receipt after df.start() commits.
CREATE OR REPLACE FUNCTION execution_await_receipt(
  p_iid text, p_exec_id text, p_wait_s integer DEFAULT 300
) RETURNS jsonb AS $$
DECLARE
  st text;
BEGIN
  PERFORM df.await_instance(p_iid, greatest(1, COALESCE(p_wait_s, 300)));
  st := df.status(p_iid);
  RETURN jsonb_build_object(
    'instance', p_iid,
    'status', st,
    'receipt', execution_receipt(p_exec_id));
END $$ LANGUAGE plpgsql;

-- Shared extract -> detect -> finalize durable subgraph.
CREATE OR REPLACE FUNCTION _oracle_finish_graph(p_exec_id text, p_url text) RETURNS text AS $$
DECLARE
  url text := rtrim(p_url, '/');
  body text := jsonb_build_object('exec_id', p_exec_id)::text;
BEGIN
  RETURN df.seq(
    df.seq(
      df.as(df.http(url || '/extract', 'POST', body, '{}'::jsonb, 300), 'extract'),
      df.sql(format('SELECT execution_store_stills(%L, ($extract.body)::jsonb)', p_exec_id))
    ),
    df.seq(
      df.as(df.http(url || '/detect', 'POST', body, '{}'::jsonb, 300), 'detect'),
      df.sql(format('SELECT execution_finalize(%L, ($detect.body)::jsonb)', p_exec_id))
    )
  );
END $$ LANGUAGE plpgsql IMMUTABLE;

-- Coverage workflow: stamps the real df.start() instance id after starting.
CREATE OR REPLACE FUNCTION coverage_workflow_start(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_t_offset double precision, p_activity_url text,
  p_workspace text DEFAULT 'default', p_label text DEFAULT NULL
) RETURNS text AS $$
DECLARE
  toff float8 := COALESCE(p_t_offset, 0.0);
  iid text;
BEGIN
  IF toff = 'NaN'::float8 OR toff = 'Infinity'::float8 OR toff = '-Infinity'::float8 THEN
    toff := 0.0;
  END IF;
  PERFORM execution_adopt(p_exec_id, p_plan_id, p_arm_id, p_episode_id, toff, p_workspace);
  iid := df.start(
    _oracle_finish_graph(p_exec_id, p_activity_url),
    COALESCE(p_label, 'coverage:' || p_exec_id));
  UPDATE evidence.execution
     SET workflow_instance = iid, updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN iid;
END $$ LANGUAGE plpgsql;

-- Execute -> oracle workflow: deduplicates concurrent starts for an active
-- execution so a second call can never kill a live moving arm!
CREATE OR REPLACE FUNCTION execute_workflow_start(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_activity_url text, p_workspace text DEFAULT 'default',
  p_timeout_s integer DEFAULT 120, p_play_timeout_s integer DEFAULT 900,
  p_poll_s integer DEFAULT 2, p_label text DEFAULT 'execute_oracle'
) RETURNS text AS $$
DECLARE
  url text := rtrim(p_activity_url, '/');
  body text := jsonb_build_object('exec_id', p_exec_id)::text;
  tout int := greatest(1, COALESCE(p_timeout_s, 120));
  play_tout int := greatest(30, COALESCE(p_play_timeout_s, 900));
  existing evidence.execution%ROWTYPE;
  prep text;
  play text;
  wait text;
  finish text;
  iid text;
BEGIN
  PERFORM _ensure_workspace_arm(p_workspace, p_arm_id);
  SELECT * INTO existing FROM evidence.execution WHERE exec_id = p_exec_id FOR UPDATE;
  IF FOUND AND existing.workflow_instance IS NOT NULL
     AND existing.status IN ('pending', 'playing', 'played', 'verified') THEN
    RETURN existing.workflow_instance;
  END IF;

  prep := df.sql(format(
    'SELECT execution_prepare(%L,%L,%L,%L,%L,%s)',
    p_exec_id, p_plan_id, p_arm_id, p_episode_id, p_workspace, tout));

  play := df.seq(
    df.as(df.http(url || '/execute', 'POST', body, '{}'::jsonb, play_tout), 'play'),
    df.sql(format('SELECT execution_after_play(%L, ($play.body)::jsonb)', p_exec_id))
  );

  wait := df.loop(df.sleep(greatest(1, COALESCE(p_poll_s, 2))),
                  format('SELECT execution_waiting(%L)', p_exec_id));

  finish := df.if(
    format('SELECT execution_ready(%L)', p_exec_id),
    _oracle_finish_graph(p_exec_id, url),
    df.sql(format('SELECT execution_fail(%L, %L)', p_exec_id,
                  'no recording/telemetry before the deadline'))
  );

  iid := df.start(df.seq(prep, df.seq(play, df.seq(wait, finish))), p_label);
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id,
                                 workflow_instance, record_timeout_s, record_deadline)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id, iid, tout,
          now() + make_interval(secs => play_tout + tout))
  ON CONFLICT (exec_id) DO UPDATE SET
    workflow_instance = EXCLUDED.workflow_instance,
    record_timeout_s  = EXCLUDED.record_timeout_s,
    record_deadline   = COALESCE(evidence.execution.record_deadline, EXCLUDED.record_deadline),
    updated_at        = now();
  RETURN iid;
END $$ LANGUAGE plpgsql;
