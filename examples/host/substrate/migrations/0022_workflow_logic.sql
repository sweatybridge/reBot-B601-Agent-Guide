-- 0022: push workflow decision logic out of Python and into Postgres.
--
-- 0021 wired the graph; this migration moves the remaining pure decisions that
-- were still in the activity/capture Python:
--
--   * extract planning + stillest-frame selection (ports of extract.py)
--   * episode fragment timing (cumulative spans; no Python accumulator)
--   * dry-play telemetry synthesis
--   * the execution receipt / status composition used by capture
--   * the coverage execute->oracle graph (adopt -> extract -> detect -> finalize)
--
-- Python is left with what must be native or I/O-bound: HTTP to the edge,
-- MediaMTX recording control, cv2/FFmpeg decode, JPEG encode, the OpenCV tip
-- detector, and file reads. Everything that is a decision over relational data
-- is a SQL function here.

ALTER TABLE evidence.execution
  ADD COLUMN IF NOT EXISTS workflow_instance text;

-- ---------------------------------------------------------------------------
-- Receipt: one JSON answer for capture. Includes the durable instance's own
-- status (df.status) so the caller does not parse df.result or re-derive
-- ok/executed/pending.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION execution_receipt(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  ws text;
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
    END;
  END IF;
  SELECT doc, doc -> 'verdict' INTO d, v
    FROM evidence.oracle_verdict WHERE id = ex.verdict_id;
  SELECT count(*) INTO n FROM evidence.execution_hold WHERE exec_id = p_exec_id;
  RETURN jsonb_build_object(
    'ok', ex.status IN ('played','verified')
          AND COALESCE(ws, 'completed') IN ('completed'),
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

-- ---------------------------------------------------------------------------
-- Dry-play telemetry (no edge): build the cmd/meas block from the plan's
-- waypoints. Used only when REBOT_WORKFLOW_DRY=1; the activity refuses to enter
-- dry mode on a host that can execute.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION execution_dry_telemetry(
  p_plan_id text, p_kp_spec text DEFAULT '50,2:150,3:150'
) RETURNS jsonb AS $$
DECLARE
  waypoints jsonb;
  samples jsonb;
  kp jsonb := '{}'::jsonb;
  part text;
  kv text[];
BEGIN
  SELECT convert_from(b.bytes, 'UTF8')::jsonb -> 'waypoints'
    INTO waypoints
    FROM evidence.plan p JOIN evidence.blob b ON b.sha256 = p.blob_sha
   WHERE p.plan_id = p_plan_id;
  IF waypoints IS NULL THEN
    RAISE EXCEPTION 'execution_dry_telemetry: no plan % (or no trajectory blob)', p_plan_id;
  END IF;
  SELECT COALESCE(jsonb_agg(
           jsonb_build_object('t', (e->>'t')::float8,
                              'cmd', e->'q', 'meas', e->'q')
           ORDER BY (e->>'t')::float8), '[]'::jsonb)
    INTO samples
    FROM jsonb_array_elements(waypoints) e
   WHERE e ? 'q';
  FOREACH part IN ARRAY string_to_array(COALESCE(p_kp_spec, ''), ':') LOOP
    IF position(',' IN part) > 0 THEN
      kv := string_to_array(part, ',');
      kp := kp || jsonb_build_object(kv[2], (kv[1])::float8);
    END IF;
  END LOOP;
  IF NOT kp ? '1' THEN
    kp := kp || jsonb_build_object('1', COALESCE((kp->>'2')::float8, 50.0));
  END IF;
  RETURN jsonb_build_object(
    'format', 'rebot-trajectory-log', 'version', 2, 'dry', true,
    'samples', samples, 'kp', kp);
END $$ LANGUAGE plpgsql STABLE;

-- ---------------------------------------------------------------------------
-- Fragment timing: appending a segment computes its cumulative episode span
-- from the existing rows. The activity still reads/hashes the bytes; the clock
-- arithmetic is here so no Python accumulator can drift from the catalog.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION episode_add_segment(
  p_episode_id text, p_seq integer, p_sha text, p_duration double precision,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE
  t0 float8;
  t1 float8;
BEGIN
  SELECT t_start, t_end INTO t0, t1
    FROM evidence.media_fragment
   WHERE episode_id = p_episode_id AND seq = p_seq;
  IF FOUND THEN
    RETURN jsonb_build_object('seq', p_seq, 't_start', t0, 't_end', t1, 'existing', true);
  END IF;
  SELECT COALESCE(max(t_end), 0.0) INTO t0
    FROM evidence.media_fragment WHERE episode_id = p_episode_id;
  t1 := t0 + greatest(0.0, p_duration);
  INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end, codec)
  VALUES (p_episode_id, p_sha, p_seq, t0, t1, p_codec)
  ON CONFLICT (episode_id, seq) DO NOTHING;
  RETURN jsonb_build_object('seq', p_seq, 't_start', t0, 't_end', t1, 'existing', false);
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_add_segment(
  p_exec_id text, p_seq integer, p_sha text, p_duration double precision,
  p_codec text DEFAULT 'mp4'
) RETURNS jsonb AS $$
DECLARE ex evidence.execution%ROWTYPE;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  IF ex.episode_id IS NULL THEN RAISE EXCEPTION 'execution % has no episode', p_exec_id; END IF;
  RETURN episode_add_segment(ex.episode_id, p_seq, p_sha, p_duration, p_codec);
END $$ LANGUAGE plpgsql;

-- ---------------------------------------------------------------------------
-- Extract planning (ports of extract.fragments_for_window / plan_extraction):
-- per hold, the video-clock span and the fragments to decode, in seq order.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION extraction_windows(p_exec_id text) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  w jsonb;
  out jsonb := '[]'::jsonb;
  w0 float8;
  w1 float8;
  seqs jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  FOR w IN SELECT * FROM jsonb_array_elements(COALESCE(ex.hold_windows, '[]'::jsonb)) LOOP
    w0 := (w->>'t_start')::float8 + ex.t_offset;
    w1 := (w->>'t_end')::float8 + ex.t_offset;
    SELECT COALESCE(jsonb_agg(f.seq ORDER BY f.seq), '[]'::jsonb)
      INTO seqs
      FROM evidence.media_fragment f
     WHERE f.episode_id = ex.episode_id
       AND f.t_end >= w0
       AND f.t_start <= w1;
    out := out || jsonb_build_array(jsonb_build_object(
      'index', (w->>'index')::int, 'w0', w0, 'w1', w1, 'fragments', seqs));
  END LOOP;
  RETURN jsonb_build_object('windows', out, 't_offset', ex.t_offset);
END $$ LANGUAGE plpgsql STABLE;

-- Port of extract.joint_speeds: per-sample max joint speed (rad/s), central
-- difference in the interior, one-sided at the ends; zero dt reads as still.
CREATE OR REPLACE FUNCTION extract_joint_speeds(p_samples jsonb) RETURNS jsonb AS $$
DECLARE
  n int := jsonb_array_length(COALESCE(p_samples, '[]'::jsonb));
  out jsonb := '[]'::jsonb;
  i int;
  a jsonb;
  b jsonb;
  dt float8;
  v float8;
  j int;
BEGIN
  IF n = 0 THEN RETURN out; END IF;
  FOR i IN 0..(n-1) LOOP
    IF n < 2 THEN
      out := out || jsonb_build_array(jsonb_build_object(
        't', (p_samples->i->>'t')::float8, 'v', 0.0));
      CONTINUE;
    END IF;
    IF i = 0 THEN
      a := p_samples->0; b := p_samples->1;
    ELSIF i = n - 1 THEN
      a := p_samples->(n-2); b := p_samples->(n-1);
    ELSE
      a := p_samples->(i-1); b := p_samples->(i+1);
    END IF;
    dt := (b->>'t')::float8 - (a->>'t')::float8;
    IF dt <= 1e-9 THEN
      v := 0.0;
    ELSE
      v := 0.0;
      IF jsonb_array_length(a->'meas') > 0 THEN
        FOR j IN 0..(jsonb_array_length(a->'meas') - 1) LOOP
          v := greatest(v, abs((b->'meas'->j)::float8 - (a->'meas'->j)::float8));
        END LOOP;
      END IF;
      v := v / dt;
    END IF;
    out := out || jsonb_build_array(jsonb_build_object(
      't', (p_samples->i->>'t')::float8, 'v', v));
  END LOOP;
  RETURN out;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- Port of extract._speed_at: speed of the nearest telemetry sample.
CREATE OR REPLACE FUNCTION extract_speed_at(p_t float8, p_speeds jsonb) RETURNS float8 AS $$
DECLARE
  n int := jsonb_array_length(COALESCE(p_speeds, '[]'::jsonb));
  i int;
  tprev float8;
  tnext float8;
BEGIN
  IF n = 0 THEN RETURN 'Infinity'::float8; END IF;
  SELECT min(ord) - 1 INTO i
    FROM jsonb_array_elements(p_speeds) WITH ORDINALITY e(v, ord)
   WHERE (v->>'t')::float8 >= p_t;
  IF i IS NULL THEN i := n - 1; END IF;
  IF i <= 0 THEN RETURN (p_speeds->0->>'v')::float8; END IF;
  IF i >= n THEN RETURN (p_speeds->(n-1)->>'v')::float8; END IF;
  tprev := (p_speeds->(i-1)->>'t')::float8;
  tnext := (p_speeds->i->>'t')::float8;
  IF (p_t - tprev) <= (tnext - p_t) THEN
    RETURN (p_speeds->(i-1)->>'v')::float8;
  END IF;
  RETURN (p_speeds->i->>'v')::float8;
END $$ LANGUAGE plpgsql IMMUTABLE;

-- Port of extract.select_hold_frame. The decoder supplies the decoded frame
-- timestamps; this function owns the window mask and the stillness/midpoint
-- tiebreak. Returns NULL when no frame is in the window (reported, not guessed).
CREATE OR REPLACE FUNCTION select_hold_frame(
  p_exec_id text, p_window_index integer, p_frame_times float8[]
) RETURNS jsonb AS $$
DECLARE
  ex evidence.execution%ROWTYPE;
  samples jsonb;
  speeds jsonb;
  w jsonb;
  w0 float8;
  w1 float8;
  mid float8;
  choice jsonb;
BEGIN
  SELECT * INTO ex FROM evidence.execution WHERE exec_id = p_exec_id;
  IF NOT FOUND THEN RAISE EXCEPTION 'unknown execution %', p_exec_id; END IF;
  SELECT (convert_from(b.bytes, 'UTF8')::jsonb) -> 'samples'
    INTO samples
    FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256 = t.blob_sha
   WHERE t.exec_id = p_exec_id;
  IF samples IS NULL THEN RETURN NULL; END IF;
  SELECT * INTO w
    FROM jsonb_array_elements(COALESCE(ex.hold_windows, '[]'::jsonb)) e
   WHERE (e->>'index')::int = p_window_index;
  IF w IS NULL THEN RETURN NULL; END IF;
  w0 := (w->>'t_start')::float8 + ex.t_offset;
  w1 := (w->>'t_end')::float8 + ex.t_offset;
  IF w1 < w0 THEN
    SELECT w1, w0 INTO w0, w1;
  END IF;
  mid := 0.5 * (w0 + w1);
  speeds := extract_joint_speeds(samples);
  SELECT jsonb_build_object('t', tf, 'score', v, 'midpoint', mid,
                            'window', jsonb_build_array(w0, w1))
    INTO choice
    FROM (
      SELECT tf, extract_speed_at(tf, speeds) AS v, abs(tf - mid) AS d
        FROM unnest(COALESCE(p_frame_times, ARRAY[]::float8[])) tf
       WHERE tf >= w0 - 1e-6 AND tf <= w1 + 1e-6
    ) q
   ORDER BY v, d, tf
   LIMIT 1;
  RETURN choice;
END $$ LANGUAGE plpgsql STABLE;

-- ---------------------------------------------------------------------------
-- Coverage: the identify session already moved the arm; this graph adopts the
-- execution and runs the same extract -> store -> detect -> finalize chain.
-- No play/claim and no wait: the recording was ingested before starting it.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION execution_set_workflow(p_exec_id text, p_iid text) RETURNS void AS $$
  UPDATE evidence.execution SET workflow_instance = p_iid WHERE exec_id = p_exec_id;
$$ LANGUAGE sql;

CREATE OR REPLACE FUNCTION coverage_workflow_start(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text,
  p_t_offset double precision, p_activity_url text,
  p_workspace text DEFAULT 'default', p_label text DEFAULT NULL
) RETURNS text AS $$
DECLARE
  url text := rtrim(p_activity_url, '/');
  body text := jsonb_build_object('exec_id', p_exec_id)::text;
  iid text;
BEGIN
  iid := df.start(
    df.seq(
      df.sql(format('SELECT execution_adopt(%L,%L,%L,%L,%s,%L)',
                    p_exec_id, p_plan_id, p_arm_id, p_episode_id, p_t_offset, p_workspace)),
      df.seq(
        df.sql(format('SELECT execution_set_workflow(%L, ''{sys_instance_id}'')', p_exec_id)),
        df.seq(
          df.seq(
            df.as(df.http(url || '/extract', 'POST', body, '{}'::jsonb, 300), 'extract'),
            df.sql(format('SELECT execution_store_stills(%L, ($extract.body)::jsonb)', p_exec_id))
          ),
          df.seq(
            df.as(df.http(url || '/detect', 'POST', body, '{}'::jsonb, 300), 'detect'),
            df.sql(format('SELECT execution_finalize(%L, ($detect.body)::jsonb)', p_exec_id))
          )
        )
      )
    ),
    COALESCE(p_label, 'coverage:' || p_exec_id));
  RETURN iid;
END $$ LANGUAGE plpgsql;

-- 0021's graph, now stamping the instance id so execution_receipt can read
-- df.status without the caller remembering it.
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
  iid text;
BEGIN
  prep := df.sql(format(
    'SELECT execution_prepare(%L,%L,%L,%L,%L,%s)',
    p_exec_id, p_plan_id, p_arm_id, p_episode_id, p_workspace, p_timeout_s));

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

  iid := df.start(df.seq(prep, df.seq(play, df.seq(wait, finish))), p_label);
  -- The graph's first step creates/updates the ledger row after commit; stamp
  -- the instance now so the row exists for execution_receipt immediately.
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id,
                                 workflow_instance, record_timeout_s)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id, iid, greatest(1, p_timeout_s))
  ON CONFLICT (exec_id) DO UPDATE
    SET workflow_instance = EXCLUDED.workflow_instance,
        record_timeout_s = EXCLUDED.record_timeout_s;
  RETURN iid;
END $$ LANGUAGE plpgsql;
