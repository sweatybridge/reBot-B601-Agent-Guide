-- 0001_schema.sql — idempotent durable Postgres substrate schema.
-- Safe to run on both a fresh database and an existing substrate cluster.
-- Uses built-in to_jsonb() for blob refcount triggers (no hstore dependency).
SET client_min_messages = warning;

CREATE EXTENSION IF NOT EXISTS vector;
CREATE EXTENSION IF NOT EXISTS pgcrypto;

CREATE SCHEMA IF NOT EXISTS world;
CREATE SCHEMA IF NOT EXISTS evidence;
CREATE SCHEMA IF NOT EXISTS learning;
CREATE SCHEMA IF NOT EXISTS coord;
CREATE SCHEMA IF NOT EXISTS search;

-- Idempotent enum creation helper block.
DO $$
BEGIN
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='world' AND t.typname='rig_status') THEN
    CREATE TYPE world.rig_status AS ENUM ('active','superseded');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='world' AND t.typname='tag_kind') THEN
    CREATE TYPE world.tag_kind AS ENUM ('base','destination');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='blob_kind') THEN
    CREATE TYPE evidence.blob_kind AS ENUM ('still','fragment','trajectory','telemetry','other');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='blob_status') THEN
    CREATE TYPE evidence.blob_status AS ENUM ('pending','verified','missing');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='episode_kind') THEN
    CREATE TYPE evidence.episode_kind AS ENUM ('execution','chat','calibration','coverage');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='still_purpose') THEN
    CREATE TYPE evidence.still_purpose AS ENUM ('hold','user_message','frame_delivered','witness','pre_exec','post_exec');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='event_src') THEN
    CREATE TYPE evidence.event_src AS ENUM ('page','server','edge');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='evidence' AND t.typname='plan_status') THEN
    CREATE TYPE evidence.plan_status AS ENUM ('planned','executing','done','aborted','expired');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='learning' AND t.typname='map_status') THEN
    CREATE TYPE learning.map_status AS ENUM ('active','superseded','unlearned');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='learning' AND t.typname='failure_class') THEN
    CREATE TYPE learning.failure_class AS ENUM ('protocol','geometry','perception','limitation','infra','coordination');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='learning' AND t.typname='scope') THEN
    CREATE TYPE learning.scope AS ENUM ('arm','pair','fleet');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='coord' AND t.typname='rendezvous_state') THEN
    CREATE TYPE coord.rendezvous_state AS ENUM ('pending','a_at','b_at','confirmed','released','failed');
  END IF;
  IF NOT EXISTS (SELECT 1 FROM pg_type t JOIN pg_namespace n ON n.oid=t.typnamespace WHERE n.nspname='coord' AND t.typname='object_state') THEN
    CREATE TYPE coord.object_state AS ENUM ('free','held','placed');
  END IF;
END $$;

-- ---------------------------------------------------------------------------
-- World plane
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS world.workspace (
  workspace_id text PRIMARY KEY,
  name         text NOT NULL,
  plane_z      double precision NOT NULL,
  min_z        double precision NOT NULL DEFAULT 0.04,
  hover_m      double precision NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS world.arm (
  arm_id           text PRIMARY KEY,
  name             text NOT NULL,
  urdf_fingerprint text NOT NULL,
  zeros_key        text NOT NULL,
  kp_key           text NOT NULL,
  joint_ids        jsonb NOT NULL DEFAULT '[1,2,3,4,5,6,7]',
  created_at       timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS world.robot_model (
  model_id   text PRIMARY KEY,
  urdf       text NOT NULL,
  created_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS world.arm_config (
  arm_id     text PRIMARY KEY,
  config_doc jsonb NOT NULL CHECK (jsonb_typeof(config_doc) = 'object'),
  updated_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS world.calibration (
  calib_fingerprint text PRIMARY KEY,
  K                 jsonb NOT NULL,
  dist              jsonb NOT NULL,
  image_size        jsonb NOT NULL,
  calib_doc         jsonb,
  is_default        boolean NOT NULL DEFAULT false,
  created_at        timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE world.calibration ADD COLUMN IF NOT EXISTS calib_doc jsonb;
ALTER TABLE world.calibration ADD COLUMN IF NOT EXISTS is_default boolean NOT NULL DEFAULT false;
CREATE UNIQUE INDEX IF NOT EXISTS calibration_default_idx
  ON world.calibration(is_default) WHERE is_default;

CREATE TABLE IF NOT EXISTS world.tag (
  tag_id       text PRIMARY KEY,
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  kind         world.tag_kind NOT NULL,
  side_mm      double precision NOT NULL,
  pose_world   jsonb,
  created_at   timestamptz NOT NULL DEFAULT now()
);

-- ---------------------------------------------------------------------------
-- Evidence plane (blob store, episodes, events, plans, telemetry, verdicts)
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS evidence.blob (
  sha256       text PRIMARY KEY,
  kind         evidence.blob_kind NOT NULL,
  bytes        bytea NOT NULL,
  size_bytes   bigint NOT NULL,
  status       evidence.blob_status NOT NULL DEFAULT 'pending',
  refcount     integer NOT NULL DEFAULT 0 CHECK (refcount >= 0),
  created_at   timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE evidence.blob ALTER COLUMN bytes SET STORAGE EXTERNAL;

-- Shared overhead camera rig (references evidence.blob via witness_sha).
CREATE TABLE IF NOT EXISTS world.shared_rig (
  rig_fingerprint   text PRIMARY KEY,
  workspace_id      text NOT NULL REFERENCES world.workspace(workspace_id),
  calib_fingerprint text NOT NULL REFERENCES world.calibration(calib_fingerprint),
  T_base_cam        jsonb NOT NULL,
  image_size        jsonb NOT NULL,
  tip_offset        jsonb NOT NULL,
  method            text NOT NULL,
  rms_px            double precision,
  rms_mm            double precision,
  witness           jsonb,
  provenance        jsonb NOT NULL,
  rig_doc           jsonb,
  witness_sha       text REFERENCES evidence.blob(sha256),
  status            world.rig_status NOT NULL DEFAULT 'active',
  superseded_by     text REFERENCES world.shared_rig(rig_fingerprint),
  fitted            timestamptz,
  created_at        timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE world.shared_rig ADD COLUMN IF NOT EXISTS rig_doc jsonb;
ALTER TABLE world.shared_rig ADD COLUMN IF NOT EXISTS witness_sha text REFERENCES evidence.blob(sha256);
CREATE UNIQUE INDEX IF NOT EXISTS shared_rig_active_idx
  ON world.shared_rig(workspace_id) WHERE status = 'active';

CREATE TABLE IF NOT EXISTS evidence.episode (
  episode_id   text PRIMARY KEY,
  session_id   text,
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  arm_ids      jsonb NOT NULL DEFAULT '[]',
  kind         evidence.episode_kind NOT NULL,
  plan_id      text,
  started_at   timestamptz NOT NULL DEFAULT now(),
  ended_at     timestamptz
);

CREATE TABLE IF NOT EXISTS evidence.media_fragment (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  sha256       text NOT NULL REFERENCES evidence.blob(sha256),
  seq          integer NOT NULL,
  t_start      double precision NOT NULL,
  t_end        double precision NOT NULL,
  codec        text,
  width        integer,
  height       integer,
  UNIQUE (episode_id, seq)
);

CREATE TABLE IF NOT EXISTS evidence.still (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  sha256       text NOT NULL REFERENCES evidence.blob(sha256),
  arm_id       text REFERENCES world.arm(arm_id),
  purpose      evidence.still_purpose NOT NULL,
  t            double precision,
  width        integer,
  height       integer,
  created_at   timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS still_episode_idx ON evidence.still(episode_id);
CREATE INDEX IF NOT EXISTS still_sha_idx ON evidence.still(sha256);

CREATE TABLE IF NOT EXISTS evidence.event (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  seq          integer NOT NULL,
  pseq         integer,
  t            timestamptz NOT NULL DEFAULT now(),
  src          evidence.event_src NOT NULL,
  type         text NOT NULL,
  arm_id       text REFERENCES world.arm(arm_id),
  payload      jsonb NOT NULL DEFAULT '{}',
  blob_sha     text REFERENCES evidence.blob(sha256),
  UNIQUE (episode_id, seq)
);
CREATE INDEX IF NOT EXISTS event_type_idx ON evidence.event(type);
CREATE INDEX IF NOT EXISTS event_t_idx ON evidence.event(t);

CREATE TABLE IF NOT EXISTS evidence.manifest (
  episode_id        text PRIMARY KEY REFERENCES evidence.episode(episode_id),
  snapshot          jsonb NOT NULL,
  rig_fingerprint   text,
  calib_fingerprint text,
  code_version      text,
  live_html_sha     text,
  tool_schemas_sha  text,
  created_at        timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS evidence.plan_group (
  group_id     text PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  arm_ids      jsonb NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS evidence.plan (
  plan_id      text PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  group_id     text REFERENCES evidence.plan_group(group_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  blob_sha     text NOT NULL REFERENCES evidence.blob(sha256),
  metadata     jsonb NOT NULL,
  snapshot     jsonb NOT NULL,
  status       evidence.plan_status NOT NULL DEFAULT 'planned',
  created_at   timestamptz NOT NULL DEFAULT now(),
  expires_at   timestamptz
);
DO $$
BEGIN
  IF NOT EXISTS (SELECT 1 FROM pg_constraint WHERE conname = 'episode_plan_fk') THEN
    ALTER TABLE evidence.episode
      ADD CONSTRAINT episode_plan_fk FOREIGN KEY (plan_id) REFERENCES evidence.plan(plan_id);
  END IF;
END $$;

CREATE TABLE IF NOT EXISTS evidence.telemetry (
  exec_id      text PRIMARY KEY,
  plan_id      text NOT NULL REFERENCES evidence.plan(plan_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  blob_sha     text NOT NULL REFERENCES evidence.blob(sha256),
  kp_key       text NOT NULL,
  summary      jsonb NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS evidence.oracle_verdict (
  id           bigserial PRIMARY KEY,
  plan_id      text NOT NULL REFERENCES evidence.plan(plan_id),
  exec_id      text REFERENCES evidence.telemetry(exec_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  layer        text NOT NULL,
  confidence   text NOT NULL,
  problems     jsonb NOT NULL DEFAULT '[]',
  doc          jsonb NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS oracle_verdict_plan_idx ON evidence.oracle_verdict(plan_id);
CREATE UNIQUE INDEX IF NOT EXISTS oracle_verdict_exec_uidx
  ON evidence.oracle_verdict(exec_id) WHERE exec_id IS NOT NULL;

CREATE TABLE IF NOT EXISTS evidence.oracle_hold (
  id               bigserial PRIMARY KEY,
  verdict_id       bigint NOT NULL REFERENCES evidence.oracle_verdict(id) ON DELETE CASCADE,
  hold_index       integer NOT NULL,
  q_cmd            jsonb NOT NULL,
  q_meas           jsonb NOT NULL,
  hold_error_rad   jsonb,
  tip_fk_cmd       jsonb,
  tip_fk_meas      jsonb,
  tip_error_mm     double precision,
  pixel_error_mm   double precision,
  detected_tip_px  jsonb,
  predicted_tip_px jsonb,
  ik_residual_mm   double precision,
  still_sha        text REFERENCES evidence.blob(sha256),
  UNIQUE (verdict_id, hold_index)
);
CREATE INDEX IF NOT EXISTS oracle_hold_verdict_idx ON evidence.oracle_hold(verdict_id);

-- Durable execution ledger (pg_durable execute -> oracle workflow state).
CREATE TABLE IF NOT EXISTS evidence.execution (
  exec_id              text PRIMARY KEY,
  plan_id              text NOT NULL REFERENCES evidence.plan(plan_id),
  arm_id               text NOT NULL REFERENCES world.arm(arm_id),
  episode_id           text,
  status               text NOT NULL DEFAULT 'pending'
                       CHECK (status IN ('pending','playing','played','verified','dead')),
  attempts             int NOT NULL DEFAULT 0,
  played_at            timestamptz,
  telemetry_sha        text REFERENCES evidence.blob(sha256),
  learned              boolean NOT NULL DEFAULT false,
  verdict_id           bigint REFERENCES evidence.oracle_verdict(id),
  play_started_at      timestamptz,
  recording_started_at timestamptz,
  t_offset             double precision NOT NULL DEFAULT 0.0,
  hold_windows         jsonb NOT NULL DEFAULT '[]'::jsonb,
  record_deadline      timestamptz,
  record_timeout_s     integer NOT NULL DEFAULT 120,
  workflow_instance    text,
  error                text,
  created_at           timestamptz NOT NULL DEFAULT now(),
  updated_at           timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE evidence.execution
  ADD COLUMN IF NOT EXISTS play_started_at      timestamptz,
  ADD COLUMN IF NOT EXISTS recording_started_at timestamptz,
  ADD COLUMN IF NOT EXISTS t_offset             double precision NOT NULL DEFAULT 0.0,
  ADD COLUMN IF NOT EXISTS hold_windows         jsonb NOT NULL DEFAULT '[]'::jsonb,
  ADD COLUMN IF NOT EXISTS record_deadline      timestamptz,
  ADD COLUMN IF NOT EXISTS record_timeout_s     integer NOT NULL DEFAULT 120,
  ADD COLUMN IF NOT EXISTS workflow_instance    text,
  ADD COLUMN IF NOT EXISTS error                text;

CREATE TABLE IF NOT EXISTS evidence.execution_hold (
  exec_id     text NOT NULL REFERENCES evidence.execution(exec_id) ON DELETE CASCADE,
  hold_index  integer NOT NULL,
  still_sha   text NOT NULL REFERENCES evidence.blob(sha256),
  t           double precision,
  score       double precision,
  created_at  timestamptz NOT NULL DEFAULT now(),
  PRIMARY KEY (exec_id, hold_index)
);

-- ---------------------------------------------------------------------------
-- Learning, coordination, search & KV planes
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS learning.sag_map (
  map_id          bigserial PRIMARY KEY,
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  rig_fingerprint text NOT NULL REFERENCES world.shared_rig(rig_fingerprint),
  kp_key          text NOT NULL,
  zeros_key       text NOT NULL,
  per_motor       jsonb NOT NULL,
  samples         integer NOT NULL,
  rms_mrad        double precision,
  validated_mm    double precision,
  envelope        jsonb NOT NULL,
  status          learning.map_status NOT NULL DEFAULT 'active',
  created_at      timestamptz NOT NULL DEFAULT now()
);
CREATE UNIQUE INDEX IF NOT EXISTS sag_map_active_idx
  ON learning.sag_map(arm_id, rig_fingerprint, kp_key, zeros_key)
  WHERE status = 'active';

CREATE TABLE IF NOT EXISTS learning.accuracy_cell (
  cell_id         bigserial PRIMARY KEY,
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  rig_fingerprint text NOT NULL REFERENCES world.shared_rig(rig_fingerprint),
  cell_key        text NOT NULL,
  radius_idx      integer NOT NULL,
  height_idx      integer NOT NULL,
  sector          integer NOT NULL,
  predicted_mm    double precision,
  measured_mm     double precision,
  n               integer NOT NULL DEFAULT 0,
  n_good          integer NOT NULL DEFAULT 0,
  validated_at    timestamptz,
  updated_at      timestamptz NOT NULL DEFAULT now(),
  UNIQUE (arm_id, rig_fingerprint, cell_key)
);

CREATE TABLE IF NOT EXISTS learning.monitor_state (
  arm_id              text PRIMARY KEY,
  ewma_pixel_error_mm double precision,
  ewma_tag_rms_px     double precision,
  last_doc            jsonb,
  updated_at          timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS learning.clearance_cell (
  id               bigserial PRIMARY KEY,
  arm_a            text NOT NULL REFERENCES world.arm(arm_id),
  arm_b            text NOT NULL REFERENCES world.arm(arm_id),
  workspace_id     text NOT NULL REFERENCES world.workspace(workspace_id),
  cell_key         text NOT NULL,
  min_clearance_mm double precision,
  n                integer NOT NULL DEFAULT 0,
  validated_at     timestamptz,
  updated_at       timestamptz NOT NULL DEFAULT now(),
  UNIQUE (arm_a, arm_b, workspace_id, cell_key),
  CHECK (arm_a < arm_b)
);

CREATE TABLE IF NOT EXISTS coord.rendezvous (
  id           bigserial PRIMARY KEY,
  group_id     text NOT NULL REFERENCES evidence.plan_group(group_id),
  arm_a        text NOT NULL REFERENCES world.arm(arm_id),
  arm_b        text NOT NULL REFERENCES world.arm(arm_id),
  state        coord.rendezvous_state NOT NULL DEFAULT 'pending',
  updated_at   timestamptz NOT NULL DEFAULT now(),
  CHECK (arm_a <> arm_b)
);

CREATE TABLE IF NOT EXISTS coord.object_track (
  object_id    text PRIMARY KEY,
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  kind         text,
  pose_world   jsonb,
  held_by_arm  text REFERENCES world.arm(arm_id),
  state        coord.object_state NOT NULL DEFAULT 'free',
  updated_at   timestamptz NOT NULL DEFAULT now(),
  CHECK ((state = 'held') = (held_by_arm IS NOT NULL))
);

CREATE TABLE IF NOT EXISTS search.embedding_model (
  model_id   text PRIMARY KEY,
  name       text NOT NULL,
  version    text NOT NULL,
  dim        integer NOT NULL,
  kind       text NOT NULL,
  created_at timestamptz NOT NULL DEFAULT now(),
  UNIQUE (name, version)
);

CREATE TABLE IF NOT EXISTS search.frame_embedding (
  id         bigserial PRIMARY KEY,
  still_sha  text NOT NULL REFERENCES evidence.blob(sha256),
  model_id   text NOT NULL REFERENCES search.embedding_model(model_id),
  embedding  vector(768) NOT NULL,
  created_at timestamptz NOT NULL DEFAULT now(),
  UNIQUE (still_sha, model_id)
);
CREATE INDEX IF NOT EXISTS frame_embedding_hnsw_idx
  ON search.frame_embedding USING hnsw (embedding vector_cosine_ops);

CREATE TABLE IF NOT EXISTS substrate_kv (
  key        text PRIMARY KEY,
  value      jsonb NOT NULL,
  updated_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS world.invalidation (
  id             bigserial PRIMARY KEY,
  source_kind    text NOT NULL,
  source_id      text NOT NULL,
  target_kind    text NOT NULL,
  target_id      bigint NOT NULL,
  invalidated_at timestamptz NOT NULL DEFAULT now()
);

-- ---------------------------------------------------------------------------
-- Content-addressed blob refcount triggers (INSERT + UPDATE + DELETE)
-- Uses built-in to_jsonb() — no hstore dependency, safe under empty search_path.
-- ---------------------------------------------------------------------------
CREATE OR REPLACE FUNCTION evidence.blob_ref_delta() RETURNS trigger AS $$
DECLARE
  col     text := TG_ARGV[0];
  new_sha text;
  old_sha text;
BEGIN
  IF TG_OP = 'INSERT' THEN
    new_sha := to_jsonb(NEW) ->> col;
    IF new_sha IS NOT NULL THEN
      UPDATE evidence.blob SET refcount = refcount + 1 WHERE sha256 = new_sha;
    END IF;
  ELSIF TG_OP = 'DELETE' THEN
    old_sha := to_jsonb(OLD) ->> col;
    IF old_sha IS NOT NULL THEN
      UPDATE evidence.blob SET refcount = greatest(0, refcount - 1) WHERE sha256 = old_sha;
    END IF;
  ELSIF TG_OP = 'UPDATE' THEN
    new_sha := to_jsonb(NEW) ->> col;
    old_sha := to_jsonb(OLD) ->> col;
    IF new_sha IS DISTINCT FROM old_sha THEN
      IF old_sha IS NOT NULL THEN
        UPDATE evidence.blob SET refcount = greatest(0, refcount - 1) WHERE sha256 = old_sha;
      END IF;
      IF new_sha IS NOT NULL THEN
        UPDATE evidence.blob SET refcount = refcount + 1 WHERE sha256 = new_sha;
      END IF;
    END IF;
  END IF;
  RETURN NULL;
END $$ LANGUAGE plpgsql;

-- Attach AFTER INSERT OR UPDATE OR DELETE triggers to EVERY blob-referencing table.
DROP TRIGGER IF EXISTS blob_ref_still ON evidence.still;
CREATE TRIGGER blob_ref_still AFTER INSERT OR UPDATE OR DELETE ON evidence.still
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('sha256');

DROP TRIGGER IF EXISTS blob_ref_frag ON evidence.media_fragment;
CREATE TRIGGER blob_ref_frag AFTER INSERT OR UPDATE OR DELETE ON evidence.media_fragment
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('sha256');

DROP TRIGGER IF EXISTS blob_ref_plan ON evidence.plan;
CREATE TRIGGER blob_ref_plan AFTER INSERT OR UPDATE OR DELETE ON evidence.plan
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('blob_sha');

DROP TRIGGER IF EXISTS blob_ref_telem ON evidence.telemetry;
CREATE TRIGGER blob_ref_telem AFTER INSERT OR UPDATE OR DELETE ON evidence.telemetry
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('blob_sha');

DROP TRIGGER IF EXISTS blob_ref_hold ON evidence.oracle_hold;
CREATE TRIGGER blob_ref_hold AFTER INSERT OR UPDATE OR DELETE ON evidence.oracle_hold
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');

DROP TRIGGER IF EXISTS blob_ref_event ON evidence.event;
CREATE TRIGGER blob_ref_event AFTER INSERT OR UPDATE OR DELETE ON evidence.event
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('blob_sha');

DROP TRIGGER IF EXISTS blob_ref_exec ON evidence.execution;
CREATE TRIGGER blob_ref_exec AFTER INSERT OR UPDATE OR DELETE ON evidence.execution
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('telemetry_sha');

DROP TRIGGER IF EXISTS blob_ref_exec_hold ON evidence.execution_hold;
CREATE TRIGGER blob_ref_exec_hold AFTER INSERT OR UPDATE OR DELETE ON evidence.execution_hold
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');

DROP TRIGGER IF EXISTS blob_ref_rig_witness ON world.shared_rig;
CREATE TRIGGER blob_ref_rig_witness AFTER INSERT OR UPDATE OR DELETE ON world.shared_rig
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('witness_sha');

DROP TRIGGER IF EXISTS blob_ref_femb ON search.frame_embedding;
CREATE TRIGGER blob_ref_femb AFTER INSERT OR UPDATE OR DELETE ON search.frame_embedding
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');

-- Recompute exact refcounts across all referencing tables (idempotent upgrade repair).
CREATE OR REPLACE FUNCTION evidence.reconcile_blob_refcounts() RETURNS bigint AS $$
DECLARE updated bigint;
BEGIN
  WITH refs AS (
    SELECT sha256 AS sha FROM evidence.still
    UNION ALL SELECT sha256 FROM evidence.media_fragment
    UNION ALL SELECT blob_sha FROM evidence.plan
    UNION ALL SELECT blob_sha FROM evidence.telemetry
    UNION ALL SELECT still_sha FROM evidence.oracle_hold WHERE still_sha IS NOT NULL
    UNION ALL SELECT blob_sha FROM evidence.event WHERE blob_sha IS NOT NULL
    UNION ALL SELECT telemetry_sha FROM evidence.execution WHERE telemetry_sha IS NOT NULL
    UNION ALL SELECT still_sha FROM evidence.execution_hold
    UNION ALL SELECT witness_sha FROM world.shared_rig WHERE witness_sha IS NOT NULL
    UNION ALL SELECT still_sha FROM search.frame_embedding
  ),
  counts AS (
    SELECT sha, count(*)::int AS c FROM refs GROUP BY sha
  )
  UPDATE evidence.blob b
     SET refcount = COALESCE(c.c, 0)
    FROM (SELECT b2.sha256, cnt.c FROM evidence.blob b2 LEFT JOIN counts cnt ON cnt.sha = b2.sha256) c
   WHERE b.sha256 = c.sha256
     AND b.refcount IS DISTINCT FROM COALESCE(c.c, 0);
  GET DIAGNOSTICS updated = ROW_COUNT;
  RETURN updated;
END $$ LANGUAGE plpgsql;

SELECT evidence.reconcile_blob_refcounts();

CREATE OR REPLACE FUNCTION evidence.gc_orphan_blobs(grace interval DEFAULT interval '1 hour')
RETURNS bigint AS $$
  WITH gone AS (
    DELETE FROM evidence.blob
    WHERE refcount <= 0 AND created_at < now() - grace
    RETURNING 1)
  SELECT count(*) FROM gone;
$$ LANGUAGE sql;

-- Scoped rig invalidation: only touches clearance_cells in p_rig's workspace
-- and logs only the sag_map rows newly transitioned from active -> superseded.
CREATE OR REPLACE FUNCTION world.invalidate_rig(p_rig text) RETURNS void AS $$
DECLARE ws text;
BEGIN
  SELECT workspace_id INTO ws FROM world.shared_rig WHERE rig_fingerprint = p_rig;
  WITH gone AS (
    UPDATE learning.sag_map SET status = 'superseded'
     WHERE rig_fingerprint = p_rig AND status = 'active'
     RETURNING map_id
  )
  INSERT INTO world.invalidation(source_kind, source_id, target_kind, target_id)
    SELECT 'rig', p_rig, 'sag_map', map_id FROM gone;
  UPDATE learning.accuracy_cell SET validated_at = NULL, n_good = 0
   WHERE rig_fingerprint = p_rig;
  IF ws IS NOT NULL THEN
    UPDATE learning.clearance_cell SET validated_at = NULL
     WHERE workspace_id = ws;
  END IF;
END $$ LANGUAGE plpgsql;
