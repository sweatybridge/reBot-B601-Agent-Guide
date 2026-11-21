-- 0001_substrate.sql — durable Postgres substrate schema.
-- Extracted from rebot-durable-state-spec.md (§4+§5) in dependency order:
--   world -> evidence -> learning -> coord -> orch -> search -> functions.
-- Apply with: psql --single-transaction -v ON_ERROR_STOP=1 -f 0001_substrate.sql
SET client_min_messages = warning;

-- ===== section block 0 =====
CREATE EXTENSION IF NOT EXISTS vector;
CREATE EXTENSION IF NOT EXISTS pgcrypto;
CREATE EXTENSION IF NOT EXISTS hstore;
CREATE SCHEMA IF NOT EXISTS world;
CREATE SCHEMA IF NOT EXISTS evidence;
CREATE SCHEMA IF NOT EXISTS learning;
CREATE SCHEMA IF NOT EXISTS coord;
CREATE SCHEMA IF NOT EXISTS orch;
CREATE SCHEMA IF NOT EXISTS search;

-- ===== section block 3 =====
CREATE TABLE world.workspace (
  workspace_id text PRIMARY KEY,
  name         text NOT NULL,
  plane_z      double precision NOT NULL,
  min_z        double precision NOT NULL DEFAULT 0.04,
  hover_m      double precision NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE world.arm (
  arm_id           text PRIMARY KEY,
  name             text NOT NULL,
  urdf_fingerprint text NOT NULL,
  zeros_key        text NOT NULL,                -- sweep.zeros_key(joint_zeros)
  kp_key           text NOT NULL,
  joint_ids        jsonb NOT NULL DEFAULT '[1,2,3,4,5,6,7]',
  created_at       timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE world.calibration (
  calib_fingerprint text PRIMARY KEY,            -- calib.py fingerprint
  K                 jsonb NOT NULL,              -- intrinsics matrix
  dist              jsonb NOT NULL,
  image_size        jsonb NOT NULL,
  created_at        timestamptz NOT NULL DEFAULT now()
);

CREATE TYPE world.rig_status AS ENUM ('active','superseded');
-- The ONE shared overhead camera rig per workspace (T_base_cam in world frame W).
CREATE TABLE world.shared_rig (
  rig_fingerprint   text PRIMARY KEY,            -- extrinsics.fingerprint (sha256[:12])
  workspace_id      text NOT NULL REFERENCES world.workspace(workspace_id),
  calib_fingerprint text NOT NULL REFERENCES world.calibration(calib_fingerprint),
  T_base_cam        jsonb NOT NULL,              -- 4x4
  image_size        jsonb NOT NULL,
  tip_offset        jsonb NOT NULL,
  method            text NOT NULL,               -- base-marker | multi-pose
  rms_px            double precision,
  rms_mm            double precision,
  witness           jsonb,
  provenance        jsonb NOT NULL,              -- extrinsics.provenance()
  status            world.rig_status NOT NULL DEFAULT 'active',
  superseded_by     text REFERENCES world.shared_rig(rig_fingerprint),
  fitted            timestamptz,
  created_at        timestamptz NOT NULL DEFAULT now()
);
-- At most one active rig per workspace.
CREATE UNIQUE INDEX shared_rig_active_idx
  ON world.shared_rig(workspace_id) WHERE status = 'active';

-- Each arm's base transform in W, solved against a given shared rig.
CREATE TABLE world.arm_base_transform (
  id              bigserial PRIMARY KEY,
  workspace_id    text NOT NULL REFERENCES world.workspace(workspace_id),
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  rig_fingerprint text NOT NULL REFERENCES world.shared_rig(rig_fingerprint),
  T_world_base    jsonb NOT NULL,                -- 4x4
  status          world.rig_status NOT NULL DEFAULT 'active',
  fitted          timestamptz,
  UNIQUE (workspace_id, arm_id, rig_fingerprint)
);

CREATE TYPE world.tag_kind AS ENUM ('base','destination');
CREATE TABLE world.tag (
  tag_id       text PRIMARY KEY,
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  kind         world.tag_kind NOT NULL,
  side_mm      double precision NOT NULL,
  pose_world   jsonb,
  created_at   timestamptz NOT NULL DEFAULT now()
);

-- ===== section block 1 =====
CREATE TYPE evidence.blob_kind AS ENUM ('still','fragment','trajectory','telemetry','other');
CREATE TYPE evidence.blob_status AS ENUM ('pending','verified','missing');

-- The content store. bytes is the exact delivered payload (bit-identical replay).
CREATE TABLE evidence.blob (
  sha256       text PRIMARY KEY,                 -- hex digest = identity
  kind         evidence.blob_kind NOT NULL,
  bytes        bytea NOT NULL,
  size_bytes   bigint NOT NULL,
  status       evidence.blob_status NOT NULL DEFAULT 'pending',
  refcount     integer NOT NULL DEFAULT 0,       -- maintained by trigger (§5.1)
  created_at   timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE evidence.blob ALTER COLUMN bytes SET STORAGE EXTERNAL;

-- A recorded execution / task / calibration window.
CREATE TYPE evidence.episode_kind AS ENUM ('execution','chat','calibration','coverage');
CREATE TABLE evidence.episode (
  episode_id   text PRIMARY KEY,
  session_id   text,                             -- sidebar conversation, may span episodes
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  arm_ids      jsonb NOT NULL DEFAULT '[]',      -- one arm, or both for cooperation
  kind         evidence.episode_kind NOT NULL,
  plan_id      text,                             -- FK added after evidence.plan exists
  started_at   timestamptz NOT NULL DEFAULT now(),
  ended_at     timestamptz
);

-- HLS fragment metadata; bytes live in blob. Append-only, immutable.
CREATE TABLE evidence.media_fragment (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  sha256       text NOT NULL REFERENCES evidence.blob(sha256),
  seq          integer NOT NULL,                 -- playlist order
  t_start      double precision NOT NULL,        -- episode-relative seconds
  t_end        double precision NOT NULL,
  codec        text, width integer, height integer,
  UNIQUE (episode_id, seq)
);

-- A still frame with its capture context.
CREATE TYPE evidence.still_purpose AS ENUM
  ('hold','user_message','frame_delivered','witness','pre_exec','post_exec');
CREATE TABLE evidence.still (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  sha256       text NOT NULL REFERENCES evidence.blob(sha256),
  arm_id       text REFERENCES world.arm(arm_id),-- null = shared-camera scene still
  purpose      evidence.still_purpose NOT NULL,
  t            double precision,                 -- episode-relative seconds
  width integer, height integer,
  created_at   timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX still_episode_idx ON evidence.still(episode_id);
CREATE INDEX still_sha_idx ON evidence.still(sha256);

-- Append-only event log (replaces events.jsonl). INSERT-only, never UPDATE.
CREATE TYPE evidence.event_src AS ENUM ('page','server','edge');
CREATE TABLE evidence.event (
  id           bigserial PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  seq          integer NOT NULL,                 -- server-assigned monotonic
  pseq         integer,                          -- page-assigned, for late-batch merge
  t            timestamptz NOT NULL DEFAULT now(),
  src          evidence.event_src NOT NULL,
  type         text NOT NULL,                    -- user_message, tool_call, model_turn, ...
  arm_id       text REFERENCES world.arm(arm_id),
  payload      jsonb NOT NULL DEFAULT '{}',      -- redacted at write (trace.py:_redact)
  blob_sha     text REFERENCES evidence.blob(sha256),
  UNIQUE (episode_id, seq)
);
CREATE INDEX event_type_idx ON evidence.event(type);
CREATE INDEX event_t_idx ON evidence.event(t);

-- Frozen environment snapshot at first sight of an episode (replaces manifest.json).
CREATE TABLE evidence.manifest (
  episode_id        text PRIMARY KEY REFERENCES evidence.episode(episode_id),
  snapshot          jsonb NOT NULL,              -- gates, versions, plane_z, tip_m, ...
  rig_fingerprint   text,
  calib_fingerprint text,
  code_version      text,                        -- git commit
  live_html_sha     text,
  tool_schemas_sha  text,
  created_at        timestamptz NOT NULL DEFAULT now()
);

-- ===== section block 2 =====
-- A synchronized multi-arm plan group (single-arm plans have group_id null).
CREATE TABLE evidence.plan_group (
  group_id     text PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  arm_ids      jsonb NOT NULL,
  created_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TYPE evidence.plan_status AS ENUM ('planned','executing','done','aborted','expired');
CREATE TABLE evidence.plan (
  plan_id      text PRIMARY KEY,
  episode_id   text NOT NULL REFERENCES evidence.episode(episode_id),
  group_id     text REFERENCES evidence.plan_group(group_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  blob_sha     text NOT NULL REFERENCES evidence.blob(sha256), -- exact trajectory bytes
  metadata     jsonb NOT NULL,                   -- waypoints, duration_s, worst_ik_m, peak_tau_nm, hz
  snapshot     jsonb NOT NULL,                   -- rig fp, calib fp, q at plan time, gains
  status       evidence.plan_status NOT NULL DEFAULT 'planned',
  created_at   timestamptz NOT NULL DEFAULT now(),
  expires_at   timestamptz                       -- gateway 600 s plan expiry
);
ALTER TABLE evidence.episode
  ADD CONSTRAINT episode_plan_fk FOREIGN KEY (plan_id) REFERENCES evidence.plan(plan_id);

-- Full cmd/meas samples (bytes) + queryable scalar summary.
CREATE TABLE evidence.telemetry (
  exec_id      text PRIMARY KEY,
  plan_id      text NOT NULL REFERENCES evidence.plan(plan_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  blob_sha     text NOT NULL REFERENCES evidence.blob(sha256),
  kp_key       text NOT NULL,
  summary      jsonb NOT NULL,                   -- abort, gripper_stall, tick, follow_error_max
  created_at   timestamptz NOT NULL DEFAULT now()
);

-- Closed-loop verdict (oracle.attribute output).
CREATE TABLE evidence.oracle_verdict (
  id           bigserial PRIMARY KEY,
  plan_id      text NOT NULL REFERENCES evidence.plan(plan_id),
  exec_id      text REFERENCES evidence.telemetry(exec_id),
  arm_id       text NOT NULL REFERENCES world.arm(arm_id),
  layer        text NOT NULL,                    -- camera-arm|arm-motor|perception|reach|ik|limitation|none
  confidence   text NOT NULL,                    -- high|medium|low
  problems     jsonb NOT NULL DEFAULT '[]',      -- [{kind, layer, detail}]
  doc          jsonb NOT NULL,                   -- full oracle doc (stream_turn, placement, ...)
  created_at   timestamptz NOT NULL DEFAULT now()
);

-- Per-hold residual rows: the learning feed, queryable without parsing the doc.
CREATE TABLE evidence.oracle_hold (
  id              bigserial PRIMARY KEY,
  verdict_id      bigint NOT NULL REFERENCES evidence.oracle_verdict(id) ON DELETE CASCADE,
  hold_index      integer NOT NULL,
  q_cmd           jsonb NOT NULL,                -- [7]
  q_meas          jsonb NOT NULL,                -- [7]
  hold_error_rad  jsonb,                         -- [7]
  tip_fk_cmd      jsonb,                         -- [3]
  tip_fk_meas     jsonb,                         -- [3]
  tip_error_mm    double precision,
  pixel_error_mm  double precision,
  detected_tip_px jsonb,                         -- [2] or null
  predicted_tip_px jsonb,                        -- [2]
  ik_residual_mm  double precision,
  still_sha       text REFERENCES evidence.blob(sha256),
  UNIQUE (verdict_id, hold_index)
);
CREATE INDEX oracle_hold_verdict_idx ON evidence.oracle_hold(verdict_id);

-- ===== section block 4 =====
CREATE TYPE learning.map_status AS ENUM ('active','superseded','unlearned');

-- Sag / gain residual map (identify.fit_sag_map output), keyed rig+kp+zeros.
CREATE TABLE learning.sag_map (
  map_id          bigserial PRIMARY KEY,
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  rig_fingerprint text NOT NULL REFERENCES world.shared_rig(rig_fingerprint),
  kp_key          text NOT NULL,
  zeros_key       text NOT NULL,
  per_motor       jsonb NOT NULL,                -- {"1":{a,b}, ... "6":{a,b}}
  samples         integer NOT NULL,
  rms_mrad        double precision,
  validated_mm    double precision,
  envelope        jsonb NOT NULL,                -- q_min/q_max/radius_m/height_m
  status          learning.map_status NOT NULL DEFAULT 'active',
  created_at      timestamptz NOT NULL DEFAULT now()
);
CREATE UNIQUE INDEX sag_map_active_idx
  ON learning.sag_map(arm_id, rig_fingerprint, kp_key, zeros_key)
  WHERE status = 'active';

-- The hold corpus the map is fit from (passive, from execution telemetry).
CREATE TABLE learning.sag_hold (
  id              bigserial PRIMARY KEY,
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  exec_id         text NOT NULL REFERENCES evidence.telemetry(exec_id),
  rig_fingerprint text NOT NULL,
  kp_key          text NOT NULL,
  zeros_key       text NOT NULL,
  q               jsonb NOT NULL,                -- [7] hold pose
  tip             jsonb NOT NULL,                -- [3]
  tau             jsonb NOT NULL,                -- [6] gravity torque feature
  r               jsonb NOT NULL,                -- [6] residual sag_meas - gravity_sag
  approach_dir    text,                          -- stiction: average both directions
  created_at      timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX sag_hold_key_idx ON learning.sag_hold(arm_id, rig_fingerprint, kp_key, zeros_key);

-- Accuracy envelope cell (identify.update_accuracy_map), per arm through shared rig.
CREATE TABLE learning.accuracy_cell (
  cell_id         bigserial PRIMARY KEY,
  arm_id          text NOT NULL REFERENCES world.arm(arm_id),
  rig_fingerprint text NOT NULL REFERENCES world.shared_rig(rig_fingerprint),
  cell_key        text NOT NULL,                 -- "r#-h#-s#"
  radius_idx      integer NOT NULL,
  height_idx      integer NOT NULL,
  sector          integer NOT NULL,
  predicted_mm    double precision,              -- tag-rig prior
  measured_mm     double precision,              -- worst observed
  n               integer NOT NULL DEFAULT 0,
  n_good          integer NOT NULL DEFAULT 0,
  validated_at    timestamptz,                   -- null until n_good >= 3; reset on bad hold
  updated_at      timestamptz NOT NULL DEFAULT now(),
  UNIQUE (arm_id, rig_fingerprint, cell_key)
);

-- ===== section block 5 =====
-- Learned rendezvous poses in W; specific to (arm_a, arm_b, workspace).
CREATE TABLE learning.exchange_pose (
  id            bigserial PRIMARY KEY,
  arm_a         text NOT NULL REFERENCES world.arm(arm_id),
  arm_b         text NOT NULL REFERENCES world.arm(arm_id),
  workspace_id  text NOT NULL REFERENCES world.workspace(workspace_id),
  pose_world    jsonb NOT NULL,                  -- exchange tip pose in W
  object_class  text,
  success_n     integer NOT NULL DEFAULT 0,
  fail_n        integer NOT NULL DEFAULT 0,
  validated_at  timestamptz,
  created_at    timestamptz NOT NULL DEFAULT now(),
  CHECK (arm_a < arm_b)                          -- canonical unordered pair
);

-- Coupled-clearance map in RELATIVE-TIP-POSE space (6D). Open fork §10: 6D vs 14D.
CREATE TABLE learning.clearance_cell (
  id            bigserial PRIMARY KEY,
  arm_a         text NOT NULL REFERENCES world.arm(arm_id),
  arm_b         text NOT NULL REFERENCES world.arm(arm_id),
  workspace_id  text NOT NULL REFERENCES world.workspace(workspace_id),
  cell_key      text NOT NULL,                   -- discretized relative tip pose
  min_clearance_mm double precision,             -- observed minimum inter-arm distance
  n             integer NOT NULL DEFAULT 0,
  validated_at  timestamptz,                     -- unlearned until enough coordinated holds
  updated_at    timestamptz NOT NULL DEFAULT now(),
  UNIQUE (arm_a, arm_b, workspace_id, cell_key),
  CHECK (arm_a < arm_b)
);

-- Rendezvous sync residuals from coordinated executions.
CREATE TABLE learning.rendezvous_residual (
  id            bigserial PRIMARY KEY,
  group_id      text NOT NULL REFERENCES evidence.plan_group(group_id),
  arm_a         text NOT NULL REFERENCES world.arm(arm_id),
  arm_b         text NOT NULL REFERENCES world.arm(arm_id),
  sync_offset_ms double precision,
  grasp_success boolean,
  residual      jsonb,
  created_at    timestamptz NOT NULL DEFAULT now()
);

-- ===== section block 6 =====
CREATE TYPE learning.failure_class AS ENUM
  ('protocol','geometry','perception','limitation','infra','coordination');
CREATE TYPE learning.scope AS ENUM ('arm','pair','fleet');

-- Dedup key (spec §5.5), extended with scope.
CREATE TABLE learning.signature (
  signature     text PRIMARY KEY,                -- hash(class, layer, kind, error_class, frame_sha, code_version)
  class         learning.failure_class NOT NULL,
  layer         text,
  problem_kind  text,
  error_class   text,
  frame_sha     text REFERENCES evidence.blob(sha256),
  code_version  text NOT NULL,
  scope         learning.scope NOT NULL,
  first_seen    timestamptz NOT NULL DEFAULT now(),
  last_seen     timestamptz NOT NULL DEFAULT now(),
  occurrences   integer NOT NULL DEFAULT 1
);

CREATE TYPE learning.case_status AS ENUM ('open','fixed','graduated','retired');
CREATE TABLE learning.case (
  case_id        text PRIMARY KEY,
  signature      text NOT NULL REFERENCES learning.signature(signature),
  scope          learning.scope NOT NULL,
  arm_id         text REFERENCES world.arm(arm_id),       -- scope=arm
  arm_a          text REFERENCES world.arm(arm_id),       -- scope=pair
  arm_b          text REFERENCES world.arm(arm_id),
  workspace_id   text REFERENCES world.workspace(workspace_id),
  status         learning.case_status NOT NULL DEFAULT 'open',
  bundle_complete boolean NOT NULL DEFAULT false,         -- both arms' evidence for pair
  export_path    text,                                    -- export/<case-id>/
  graduated_path text,                                    -- tests/agent_cases/<id>/
  fix_commit     text,
  created_at     timestamptz NOT NULL DEFAULT now(),
  updated_at     timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX case_signature_idx ON learning.case(signature);
CREATE INDEX case_status_idx ON learning.case(status);

CREATE TYPE learning.case_event_kind AS ENUM
  ('opened','exported','fixed','graduated','retired','recurred');
CREATE TABLE learning.case_event (
  id        bigserial PRIMARY KEY,
  case_id   text NOT NULL REFERENCES learning.case(case_id) ON DELETE CASCADE,
  t         timestamptz NOT NULL DEFAULT now(),
  event     learning.case_event_kind NOT NULL,
  detail    jsonb
);

-- Triage verdict per task window (replaces labels.jsonl).
CREATE TYPE learning.verdict AS ENUM ('fail','ok','unknown');
CREATE TABLE learning.label (
  id          bigserial PRIMARY KEY,
  episode_id  text NOT NULL REFERENCES evidence.episode(episode_id),
  task_id     integer NOT NULL,
  verdict     learning.verdict NOT NULL,
  class       learning.failure_class,
  layer       text,
  problems    jsonb NOT NULL DEFAULT '[]',
  plan_ids    jsonb NOT NULL DEFAULT '[]',
  signature   text REFERENCES learning.signature(signature),
  created_at  timestamptz NOT NULL DEFAULT now(),
  UNIQUE (episode_id, task_id)
);

-- ===== section block 7 =====
CREATE TYPE coord.arm_mode AS ENUM ('idle','moving','holding','fault');
CREATE TABLE coord.arm_state (
  arm_id       text PRIMARY KEY REFERENCES world.arm(arm_id),
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  q            jsonb,                            -- last measured [7]
  tip_world    jsonb,                            -- FK(measured) in W
  mode         coord.arm_mode NOT NULL DEFAULT 'idle',
  updated_at   timestamptz NOT NULL DEFAULT now()
);

CREATE TYPE coord.schedule_status AS ENUM ('planned','executing','done','aborted');
CREATE TABLE coord.schedule (
  group_id     text PRIMARY KEY REFERENCES evidence.plan_group(group_id),
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  status       coord.schedule_status NOT NULL DEFAULT 'planned',
  updated_at   timestamptz NOT NULL DEFAULT now()
);

-- Rendezvous state machine. Transitions are SELECT ... FOR UPDATE + UPDATE in one
-- transaction; double-grasp / both-own are impossible by construction.
CREATE TYPE coord.rendezvous_state AS ENUM
  ('pending','a_at','b_at','confirmed','released','failed');
CREATE TABLE coord.rendezvous (
  id              bigserial PRIMARY KEY,
  group_id        text NOT NULL REFERENCES evidence.plan_group(group_id),
  arm_a           text NOT NULL REFERENCES world.arm(arm_id),
  arm_b           text NOT NULL REFERENCES world.arm(arm_id),
  exchange_pose_id bigint REFERENCES learning.exchange_pose(id),
  state           coord.rendezvous_state NOT NULL DEFAULT 'pending',
  updated_at      timestamptz NOT NULL DEFAULT now()
);

CREATE TYPE coord.object_state AS ENUM ('free','held','placed');
CREATE TABLE coord.object_track (
  object_id    text PRIMARY KEY,
  workspace_id text NOT NULL REFERENCES world.workspace(workspace_id),
  kind         text,
  pose_world   jsonb,
  held_by_arm  text REFERENCES world.arm(arm_id),
  state        coord.object_state NOT NULL DEFAULT 'free',
  updated_at   timestamptz NOT NULL DEFAULT now(),
  -- a held object must have an owner; a free object must not
  CHECK ((state = 'held') = (held_by_arm IS NOT NULL))
);

-- ===== section block 8 =====
CREATE TYPE orch.workflow_kind AS ENUM
  ('plan','coordinate','reembed','scrub','identify','coverage');
CREATE TYPE orch.workflow_status AS ENUM ('pending','running','done','failed','dead');
CREATE TABLE orch.workflow (
  workflow_id text PRIMARY KEY,
  df_instance text,                              -- df.instances.id (pg_durable runtime)
  kind        orch.workflow_kind NOT NULL,
  status      orch.workflow_status NOT NULL DEFAULT 'pending',
  input       jsonb NOT NULL,
  created_at  timestamptz NOT NULL DEFAULT now(),
  updated_at  timestamptz NOT NULL DEFAULT now()
);

-- Isolated native activities. A crash here fails the row (-> dead-letter),
-- never the cluster. attempts/max_attempts bound the retry loop.
CREATE TYPE orch.activity_kind AS ENUM ('kinematics','decode','embed','fit');
CREATE TYPE orch.activity_status AS ENUM ('pending','running','done','failed','dead');
CREATE TABLE orch.activity_run (
  id          bigserial PRIMARY KEY,
  workflow_id text NOT NULL REFERENCES orch.workflow(workflow_id) ON DELETE CASCADE,
  kind        orch.activity_kind NOT NULL,
  status      orch.activity_status NOT NULL DEFAULT 'pending',
  isolated    boolean NOT NULL DEFAULT true,     -- false for Rust-native kinematics and fitting
  attempts    integer NOT NULL DEFAULT 0,
  max_attempts integer NOT NULL DEFAULT 5,
  last_error  text,
  created_at  timestamptz NOT NULL DEFAULT now(),
  updated_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX activity_status_idx ON orch.activity_run(status) WHERE status IN ('pending','running');

-- ===== section block 9 =====
CREATE TABLE search.embedding_model (
  model_id   text PRIMARY KEY,                   -- name:version
  name       text NOT NULL,
  version    text NOT NULL,
  dim        integer NOT NULL,
  kind       text NOT NULL,                      -- frame | episode
  created_at timestamptz NOT NULL DEFAULT now(),
  UNIQUE (name, version)
);

CREATE TABLE search.frame_embedding (
  id         bigserial PRIMARY KEY,
  still_sha  text NOT NULL REFERENCES evidence.blob(sha256),
  model_id   text NOT NULL REFERENCES search.embedding_model(model_id),
  embedding  vector(768) NOT NULL,               -- fixed column dim; see note below
  created_at timestamptz NOT NULL DEFAULT now(),
  UNIQUE (still_sha, model_id)
);
CREATE INDEX frame_embedding_hnsw_idx
  ON search.frame_embedding USING hnsw (embedding vector_cosine_ops);

CREATE TABLE search.episode_embedding (
  id         bigserial PRIMARY KEY,
  episode_id text NOT NULL REFERENCES evidence.episode(episode_id),
  model_id   text NOT NULL REFERENCES search.embedding_model(model_id),
  embedding  vector(768) NOT NULL,               -- fixed column dim; see note below
  created_at timestamptz NOT NULL DEFAULT now(),
  UNIQUE (episode_id, model_id)
);
CREATE INDEX episode_embedding_hnsw_idx
  ON search.episode_embedding USING hnsw (embedding vector_cosine_ops);

-- ===== section block 10 =====
CREATE FUNCTION evidence.blob_ref_delta() RETURNS trigger AS $$
DECLARE col text := TG_ARGV[0]; sha text;
BEGIN
  IF TG_OP = 'INSERT' THEN
    sha := hstore(NEW) -> col;
    UPDATE evidence.blob SET refcount = refcount + 1 WHERE sha256 = sha;
  ELSIF TG_OP = 'DELETE' THEN
    sha := hstore(OLD) -> col;
    UPDATE evidence.blob SET refcount = refcount - 1 WHERE sha256 = sha;
  END IF;
  RETURN NULL;
END $$ LANGUAGE plpgsql;

-- One trigger per referencing table, bound to its sha column:
CREATE TRIGGER blob_ref_still AFTER INSERT OR DELETE ON evidence.still
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('sha256');
CREATE TRIGGER blob_ref_frag AFTER INSERT OR DELETE ON evidence.media_fragment
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('sha256');
CREATE TRIGGER blob_ref_plan AFTER INSERT OR DELETE ON evidence.plan
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('blob_sha');
CREATE TRIGGER blob_ref_telem AFTER INSERT OR DELETE ON evidence.telemetry
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('blob_sha');
CREATE TRIGGER blob_ref_hold AFTER INSERT OR DELETE ON evidence.oracle_hold
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');
CREATE TRIGGER blob_ref_femb AFTER INSERT OR DELETE ON search.frame_embedding
  FOR EACH ROW EXECUTE FUNCTION evidence.blob_ref_delta('still_sha');

-- ===== section block 11 =====
CREATE FUNCTION evidence.gc_orphan_blobs(grace interval DEFAULT interval '1 hour')
RETURNS bigint AS $$
  WITH gone AS (
    DELETE FROM evidence.blob
    WHERE refcount <= 0 AND created_at < now() - grace
    RETURNING 1)
  SELECT count(*) FROM gone;
$$ LANGUAGE sql;

-- ===== section block 12 =====
CREATE TABLE world.invalidation (
  id             bigserial PRIMARY KEY,
  source_kind    text NOT NULL,                  -- rig | zeros | kp
  source_id      text NOT NULL,
  target_kind    text NOT NULL,                  -- sag_map | accuracy_cell | clearance_cell
  target_id      bigint NOT NULL,
  invalidated_at timestamptz NOT NULL DEFAULT now()
);

CREATE FUNCTION world.invalidate_rig(p_rig text) RETURNS void AS $$
BEGIN
  UPDATE learning.sag_map SET status='superseded'
    WHERE rig_fingerprint = p_rig AND status='active';
  UPDATE learning.accuracy_cell SET validated_at=NULL, n_good=0
    WHERE rig_fingerprint = p_rig;
  -- clearance cells were measured through this rig's geometry: unvalidate all
  UPDATE learning.clearance_cell SET validated_at=NULL;
  INSERT INTO world.invalidation(source_kind, source_id, target_kind, target_id)
    SELECT 'rig', p_rig, 'sag_map', map_id FROM learning.sag_map
      WHERE rig_fingerprint = p_rig;
END $$ LANGUAGE plpgsql;
