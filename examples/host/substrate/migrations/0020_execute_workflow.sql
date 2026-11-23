-- 0020: durable execute->oracle workflow foundation.
--
-- The execute->oracle chain becomes a pg_durable workflow (offline hold-frame
-- extraction replaces the live grab timer). Durable replay raises two safety
-- problems this migration encodes:
--
--   1. MOTION IS AT-MOST-ONCE. A blind retry of the /play step would move the arm
--      again. `execution_claim_play` claims the play BEFORE moving (pending ->
--      playing). A retry sees a non-pending status and does NOT re-move; a crash
--      after the claim leaves the execution in 'playing' -> dead-letter + operator
--      review, never auto-replay. The claim is the guard.
--
--   2. LEARNING IS IDEMPOTENT. accuracy_cell_update increments n (not idempotent).
--      `oracle_learn` folds cells + writes the verdict ONCE per execution, gated by
--      a `learned` flag under a row lock; a replay returns the recorded verdict
--      without re-folding. Verdict write-once per exec_id likewise.
--
-- The ledger is also the durable execution state (status/telemetry/verdict lineage).

CREATE TABLE IF NOT EXISTS evidence.execution (
  exec_id       text PRIMARY KEY,             -- == evidence.telemetry.exec_id
  plan_id       text NOT NULL REFERENCES evidence.plan(plan_id),
  arm_id        text NOT NULL REFERENCES world.arm(arm_id),
  episode_id    text,                         -- recording episode (fragments)
  status        text NOT NULL DEFAULT 'pending'
                CHECK (status IN ('pending','playing','played','verified','dead')),
  attempts      int  NOT NULL DEFAULT 0,
  played_at     timestamptz,
  telemetry_sha text REFERENCES evidence.blob(sha256),
  learned       boolean NOT NULL DEFAULT false,
  verdict_id    bigint REFERENCES evidence.oracle_verdict(id),
  created_at    timestamptz NOT NULL DEFAULT now(),
  updated_at    timestamptz NOT NULL DEFAULT now()
);

-- Idempotent execution creation (the workflow's first step). Returns the status.
CREATE OR REPLACE FUNCTION execution_begin(
  p_exec_id text, p_plan_id text, p_arm_id text, p_episode_id text DEFAULT NULL
) RETURNS text AS $$
BEGIN
  INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id)
  VALUES (p_exec_id, p_plan_id, p_arm_id, p_episode_id)
  ON CONFLICT (exec_id) DO NOTHING;
  RETURN (SELECT status FROM evidence.execution WHERE exec_id = p_exec_id);
END $$ LANGUAGE plpgsql;

-- AT-MOST-ONCE motion guard: claim pending -> playing. Returns true ONLY for the
-- caller that transitioned pending -> playing (the one allowed to move the arm).
-- Any other status (playing/played/verified/dead) returns false -> do NOT move.
CREATE OR REPLACE FUNCTION execution_claim_play(p_exec_id text) RETURNS boolean AS $$
DECLARE claimed boolean;
BEGIN
  UPDATE evidence.execution
     SET status = 'playing', attempts = attempts + 1, updated_at = now()
   WHERE exec_id = p_exec_id AND status = 'pending';
  GET DIAGNOSTICS claimed = ROW_COUNT;
  RETURN claimed;
END $$ LANGUAGE plpgsql;

-- Record a successful play (playing -> played) with its telemetry blob.
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

-- Mark an execution dead (poison job / failed play after claim). No auto-replay.
CREATE OR REPLACE FUNCTION execution_mark_dead(p_exec_id text) RETURNS void AS $$
BEGIN
  UPDATE evidence.execution SET status='dead', updated_at=now()
   WHERE exec_id = p_exec_id AND status IN ('pending','playing');
END $$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION execution_get(p_exec_id text) RETURNS jsonb AS $$
  SELECT to_jsonb(e) FROM evidence.execution e WHERE exec_id = p_exec_id;
$$ LANGUAGE sql STABLE;

-- Recording readiness gate: true when the ingested fragments' span covers every
-- hold window (video clock = trajectory clock + t_offset). The workflow waits on
-- this before extraction so it never reads a half-ingested recording.
CREATE OR REPLACE FUNCTION wait_fragments(
  p_episode_id text, p_windows jsonb, p_t_offset float8 DEFAULT 0.0
) RETURNS boolean AS $$
DECLARE
  w jsonb; w0 float8; w1 float8; fmin float8; fmax float8;
BEGIN
  SELECT min(t_start), max(t_end) INTO fmin, fmax
    FROM evidence.media_fragment WHERE episode_id = p_episode_id;
  IF fmin IS NULL THEN RETURN false; END IF;
  FOR w IN SELECT jsonb_array_elements(coalesce(p_windows, '[]'::jsonb)) LOOP
    w0 := (w->>'t_start')::float8 + p_t_offset;
    w1 := (w->>'t_end')::float8 + p_t_offset;
    IF fmin > w0 + 1e-6 OR fmax < w1 - 1e-6 THEN RETURN false; END IF;
  END LOOP;
  RETURN true;
END $$ LANGUAGE plpgsql STABLE;

-- IDEMPOTENT learning: write the verdict + fold accuracy cells ONCE per execution.
-- Requires telemetry persisted first (oracle_verdict.exec_id FK). A replay (learned
-- already true) returns the recorded verdict without re-folding cells. The row lock
-- + learned flag make the check-and-fold atomic against concurrent retries.
CREATE OR REPLACE FUNCTION oracle_learn(
  p_exec_id text, p_arm_id text, p_rig_fingerprint text, p_doc jsonb,
  p_threshold_mm float8 DEFAULT 10.0
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
  v := coalesce(p_doc->'verdict', '{}'::jsonb);
  INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc)
  VALUES (ex.plan_id, p_exec_id, p_arm_id,
          coalesce(v->>'layer','none'), coalesce(v->>'confidence','low'),
          coalesce(v->'problems','[]'::jsonb), p_doc)
  RETURNING id INTO vid;
  SELECT config_doc->'learning' INTO cfg FROM world.arm_config WHERE arm_id = p_arm_id;
  PERFORM accuracy_cell_update(p_arm_id, p_rig_fingerprint, p_doc, cfg, p_threshold_mm);
  UPDATE evidence.execution
     SET learned = true, verdict_id = vid, status = 'verified', updated_at = now()
   WHERE exec_id = p_exec_id;
  RETURN v;
END $$ LANGUAGE plpgsql;