# reBot B601 — Durable Postgres State Substrate

Implementation spec against `sweatybridge/reBot-B601-Agent-Guide`. Companion to
`rebot-trace-collector-spec.md` (the collector/oracle/replay design); this document
replaces that spec's *storage* layer (§3 state directory) and extends it to a
two-arm cooperating system. Line references to app code are to the current checkout.

## 1. Goals and non-goals

**Goals**

1. One authoritative substrate — Postgres — for world config, learning state,
   coordination state, durable orchestration, the evidence catalog, the pgvector
   similarity index, and the bounded media corpus.
2. Flatten `examples/host/state/` into `config/` (boot + secrets) and `export/`
   (derived handoff bundles). Nothing authoritative remains on the filesystem.
3. Permanent visual-record retention: human review may happen months later and
   improved methods must re-process old episodes. The spec's 30-day prune is
   overridden for the visual corpus.
4. Continuous learning over years of daily operation, with cross-session
   similarity search (pgvector) and a three-tier learning model (arm / pair / fleet).
5. Two-arm physical cooperation: one shared overhead camera, simultaneous motion
   planned offline, coordinator as a transactional state machine.
6. Crash isolation: unsafe native compute never runs in a postmaster child.

**Non-goals**

- Replacing the edge realtime control loop (Pi, Rust, CAN). It stays per-arm and
  unchanged; the rendezvous handshake is edge-to-edge, never via Postgres.
- Changing the execution gates (`REBOT_EXECUTE=1`, edge `--allow-execute`,
  deliberate confirm) or the shared `plan_to_target` motion path.
- 24/7 recording. Media is recorded only for executed trajectories (~30 min/day
  median, one shared camera), which is what makes media-in-Postgres viable.
- Force control, multi-view perception, temporal/dynamic perception. The stack is
  still-frame based; video is a forensic/re-extraction source, not a learning input.

**Design principles** (inherited from the guide): reported rather than assumed;
refused rather than guessed; safety centralized and mandatory; bit-identical replay;
content addressing as the integrity anchor.

## 2. Architecture

```
                 ┌─────────────────────────────────────────────────────────┐
                 │  Postgres (named volume rebot-pgdata)                     │
                 │  ┌───────────────┐  ┌──────────────────────────────────┐ │
   page/agent ───┼─▶│ evidence +    │  │ pg_durable 0.2.9 (duroxide runtime)│ │
   POST /trace   │  │ catalog +     │◀─│ microsoft/pg_durable, pgrx bgworker│ │
                 │  │ learning +    │  │  - coordinator state machine      │ │
                 │  │ coord state   │  │  - planning workflows             │ │
                 │  └───────────────┘  │  - re-embed / scrub / identify    │ │
                 │  ┌───────────────┐  └───────────────┬──────────────────┘ │
                 │  │ pgvector index│                  │ isolated activities │
                 │  └───────────────┘                  ▼                      │
                 └─────────────────────────────┬──────────────────────────────┘
                                               │ kinematics: openrr/k via rebot_kinematics (pgrx)
                                               │ decode / embed (isolated process)
   edge player A ◀── trajectory ──┐            │
   edge player B ◀── trajectory ──┴── gateway ─┘   rendezvous handshake: A ↔ B (edge-to-edge)

   filesystem:  config/ (tls, bootstrap)   export/ (materialized bundles)
   git:         tests/agent_cases/ (graduated regression cases)
```

Three compute placements, by safety:

| Compute | Placement | Why |
|---|---|---|
| Coordinator state transitions, catalog, queries | in Postgres (SQL) | pure relational, ACID |
| Kinematics / collision planning; sag / accuracy / clearance fitting | Rust-native, in the Duroxide worker | memory-safe numerics on structured internal data; no FFI crash island |
| Media decode, embedding inference (native ONNX/PyTorch), OpenCV perception | **isolated activity** (separate process) | unsafe parser / heavy native runtime; a crash fails the job, never resets the cluster |

The load-bearing rule: a deterministic crash in unsafe native code running in a
postmaster child resets the whole cluster, and durable retry replays the same
input → crash-loop. Isolating that compute (or making it memory-safe Rust) is the
one invariant this design will not trade away.

**Concrete artifacts (built and validated, §11):**
- **Orchestration:** [`microsoft/pg_durable`](https://github.com/microsoft/pg_durable)
  0.2.9 (PostgreSQL License, preview). A pgrx extension hosting the
  [`duroxide`](https://github.com/microsoft/duroxide) runtime + `duroxide-pg` state
  provider. SQL DSL (`df.start`, `df.sql`, `df.seq`, `df.join`, `df.http`, …);
  state in `df.*` (instances/nodes/vars) and `duroxide.*`. Prebuilt image
  `ghcr.io/microsoft/pg_durable:pg18` (PG17/18). Requires
  `shared_preload_libraries=pg_durable`; worker role must be superuser; app roles
  need `df.grant_usage()`.
- **Kinematics:** [`openrr/k`](https://github.com/openrr/k) 0.32 (Apache-2.0),
  pure-Rust FK/IK/URDF on nalgebra 0.30, wrapped as the `rebot_kinematics` pgrx
  extension (`rebot_fk`, `rebot_ik`, `rebot_dof`). Memory-safe: bad input → SQL
  ERROR (pgrx catches the panic), never a backend crash. `k` itself provides FK/IK
  only; the remaining numerics are **Rust ports** in the same extension —
  `rebot_path_safe` / `rebot_joint_points` (port of `sweep.path_safe`),
  `rebot_gravity_torques` (port of `rebot_ik.gravity_torques`), capsule self-collision
  (`rebot_self_collision_free` / `rebot_min_self_clearance` / `rebot_seg_seg_distance`),
  the traj time law (`rebot_ease_fraction` / `rebot_ease_velocity`),
  `rebot_seconds_for` / `rebot_within_limits`, projection (`rebot_project` /
  `rebot_project_dist`), and `rebot_ray_plane` (iterative undistortion + plane
  intersection, port of `extrinsics.ray_plane`) — all validated against the
  pure-numpy/OpenCV references (§9.1).
- **Similarity:** pgvector 0.8.7 (HNSW). The **isolated embed activity** is wired:
  `embed_service.py` (separate HTTP process, pluggable `Embedder`) reached from a
  durable workflow via `df.http()` (`embed.start_embed_workflow`); inference crashes
  fail the step, never the cluster. Validated end-to-end (§9.1).
- **Execute→oracle:** `0021`/`0022` add the durable graph and `workflow_service.py`, the
  isolated activity that merges the planner half (record + play + telemetry) and the
  capture half (decode + select hold stills + detect). `/execute` starts it; live
  hold grabs are retired. Dry validation with the arm parked (§9.1).

## 3. Filesystem layout (flattened)

```
examples/host/
  config/
    tls/                      # certs/keys (deploy secrets, read at startup)
    bootstrap.toml            # PG DSN, edge endpoints, gate flags
  export/                     # DERIVED materialized bundles (staging for handoff)
    <case-id>/
      case.json report.md events.jsonl tape.json
      stills/ calib/ rig/ trajectories/ telemetry/ oracle/
  state/                      # REMOVED as a source of truth (migration §8)

rebot-pgdata/                 # Docker named volume — ALL authoritative state
tests/agent_cases/            # git — graduated regression cases (code, not data)
```

`config/` is needed before Postgres is reachable (you cannot read the DSN from PG).
`export/` is a materialized view, not a source of truth; permanent cases graduate
into `tests/agent_cases/` under version control. The fragile Windows bind-mount now
holds only small, non-WAL-critical files; everything durability-sensitive is in the
named volume, which sidesteps the SQLite/PG WAL-on-bind-mount locking problem.

### 3.1 Migration map (old `state/` → substrate)

| Old path | New home |
|---|---|
| `calib/extrinsics-fixed.json`, candidates | `world.shared_rig`, `world.arm_base_transform` |
| `calib/sag-*.json` | `learning.sag_map` |
| `calib/accuracy-map.json` | `learning.accuracy_cell` |
| `calib/*intrinsics*` | `world.calibration` |
| `captures/<id>/` (meta + frame) | `evidence.episode`, `evidence.still`, `evidence.blob` |
| `out/<plan_id>.json` | `evidence.plan` + `evidence.blob` (trajectory bytes) |
| `telemetry/<id>.json` | `evidence.telemetry` + `evidence.blob` (samples) |
| `oracle/<id>.json`, `hold-N.jpg` | `evidence.oracle_verdict`, `evidence.oracle_hold`, `evidence.still` |
| `recordings/phone/*.mp4` | `evidence.media_fragment` + `evidence.blob` (HLS) |
| `traces/<sid>/events.jsonl` | `evidence.event` |
| `traces/<sid>/manifest.json` | `evidence.manifest` |
| `traces/<sid>/labels.jsonl` | `learning.label` |
| `traces/blobs/<sha>.jpg` | `evidence.blob` |
| `traces/export/<case>/` | `export/<case>/` (materialized) + `learning.case` |
| `tags/` | `world.tag` |
| `tls/` | `config/tls/` (stays files) |

### 3.2 `app/` code partition

The migration absorbs the numerical/relational modules into the substrate and leaves
only I/O shells and unsafe-native workers as processes. Of the 24 current `app/` scripts:

**Absorbed → PG functions (13):**

| Script | Form |
|---|---|
| `oracle.py` | SQL/PLpgSQL — already pure (no I/O); `attribute` over `oracle_hold`, `hold_windows`/`downsample` as window functions |
| `trace.py` | dissolves — recorder → INSERTs; `_redact` → recursive jsonb SQL function |
| `calib.py` | SQL (fingerprint/data) |
| `rebot_ik.py` | Rust pgrx — IK/FK/`gravity_torques` |
| `plan_to_target.py` | Rust pgrx — the planner; largest rewrite, prototype first |
| `plan_with_pinocchio.py` | Rust pgrx — merges into the planner |
| `traj.py` | Rust pgrx — `build`/`validate`/`ease_fraction`/velocity |
| `sweep.py` | Rust pgrx — `aim`/`gravity_sag`/`sag_correction`/`path_safe`/`zeros_key` |
| `identify.py` | Rust pgrx — `fit_sag_map`/accuracy/coverage |
| `extrinsics.py` | Rust pgrx (PnP fit) + SQL (fingerprint/provenance) |
| `base_marker.py` | Rust pgrx — `solve_marker`/verify given corners |
| `locate_target.py` | Rust pgrx/SQL — `project`/`ray_plane` |
| `witness.py` | Rust pgrx/SQL — drift/scale/reprojection |

**Left as thin service processes (6):** `capture_server.py`, `planner_server.py`,
`mediamtx.py`, `camera.py`, `healthcheck.py`, `make_cert.py` — HTTP/hardware/config
shells; state and logic move to PG, the shell stays a thin client.

**Left as isolated vision activities (5):** `auto_calibrate.py`,
`calibrate_intrinsics.py`, `calibrate_extrinsics.py`, `identify_cad.py`,
`sweep_video.py` — OpenCV/FFmpeg (C++ FFI crash island); run as isolated workers.

Several scripts **split**: the geometry/math half becomes a PG function, the
vision-detection half stays an isolated activity (`extrinsics`, `base_marker`,
`locate_target`, `witness`, `identify`, `calibrate_extrinsics`). The pure-core modules
(`oracle`, `traj`, `sweep`, `rebot_ik`) were already written as functions of their
arguments, which is what makes them portable into the worker.

## 4. Schema design

Conventions:
- Schemas: `world`, `evidence`, `learning`, `coord`, `orch`, `search`.
- Domain ids are `text` reusing the capture format `YYYYMMDDTHHMMSSZ-hex6`
  (`trace.py:36`) where one already exists (plan, episode, arm, rig fingerprint,
  signature). Purely internal append rows use `bigserial`.
- All timestamps `timestamptz`, default `now()`.
- Numeric arrays (q[7], T[4][4], tips[3]) stored `jsonb`; exact bytes (trajectory,
  telemetry samples, media) stored `bytea` with `STORAGE EXTERNAL` for bit-identical
  replay and to skip wasted TOAST compression on already-compressed media.
- Content addressing: `evidence.blob.sha256` is the integrity anchor; every media
  reference is a real FK to it (true ACID integrity, no cross-system protocol).
- Extensions: `pgvector`, `pgcrypto` (gen_random_uuid), `hstore` (generic refcount
  trigger), `pg_durable` (Duroxide).
- **Creation order** is by dependency: `world` → `evidence` → `learning` → `coord`
  → `orch` → `search`. The subsections below are grouped by domain, not creation
  order; a migration applies `world` first (evidence FKs reference it). The
  `episode.plan_id` FK is added after `evidence.plan` exists (shown inline).

```sql
CREATE EXTENSION IF NOT EXISTS vector;
CREATE EXTENSION IF NOT EXISTS pgcrypto;
CREATE EXTENSION IF NOT EXISTS hstore;
CREATE SCHEMA IF NOT EXISTS world;
CREATE SCHEMA IF NOT EXISTS evidence;
CREATE SCHEMA IF NOT EXISTS learning;
CREATE SCHEMA IF NOT EXISTS coord;
CREATE SCHEMA IF NOT EXISTS orch;
CREATE SCHEMA IF NOT EXISTS search;
```

### 4.1 Evidence catalog + content store

```sql
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
```

### 4.2 Plans, telemetry, oracle

```sql
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
```

### 4.3 World config

```sql
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
```

### 4.4 Learning — arm tier (never merges across arms)

```sql
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
```

### 4.5 Learning — pair tier (cooperation-specific)

```sql
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
```

### 4.6 Learning — fleet tier (arm-independent, aggregates)

```sql
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
```

### 4.7 Coordinator state machine (transactional)

```sql
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
```

### 4.8 Orchestration metadata (pg_durable owns the runtime)

**pg_durable provides the durable-execution state itself** — `df.instances`,
`df.nodes`, `df.vars` (the DSL graph + progress) and the `duroxide.*` schema
(runtime history/work queues, owned by `duroxide-pg`). The app does NOT reimplement
checkpointing; a workflow is `df.start(df.sql(...) ~> ...)` and its status/result
are read from `df.*`. The `orch` tables below are an **optional app-level metadata
layer** that links a durable instance to domain entities (plan/episode/case); they
reference `df.instances.id` rather than replacing it. Isolated native activities
(decode/embed/OpenCV) are invoked from a workflow via `df.http()` against a
sandboxed service, so a crash fails the step (→ retry/dead-letter), never the
cluster.

```sql
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
```

### 4.9 Similarity search (pgvector)

```sql
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
```

HNSW is chosen over IVFFlat: incremental insert needs no scheduled rebuild, and the
corpus grows steadily. Recall is monitored conditionally (sample ANN vs exact KNN);
rebuild only on parameter change or recall drop. The substantive refresh is
**re-embedding on model upgrade** (§5.4), version-driven, not calendar-driven.

**Embedding dimension is a deployment constant.** HNSW requires a fixed-dimension
column — a bare `vector` cannot be indexed (validated: PG rejects it with "column
does not have dimensions"). `768` here is illustrative (e.g. CLIP ViT-L/14); pick the
deployment's dimension at DDL time. The column typmod is the enforcement: inserting a
vector of any other dimension errors, so `embedding_model.dim` must agree with it. A
model upgrade that changes dimension is a migration — add a new column/table at the
new dimension and re-embed (§5.4), not an in-place change.

**Embeddings must be non-zero.** Cosine distance (`<=>`) is undefined for zero vectors
(returns NaN; the HNSW scan returns nothing — validated). L2-normalize embeddings in
the embed activity before insert (standard for CLIP-style models).

## 5. Key mechanisms

### 5.1 Referential integrity (true ACID, in-cluster)

Because media bytes live in `evidence.blob`, every reference is a real FK — no
cross-system protocol is needed. `refcount` is maintained by triggers on the
referencing tables (`still`, `media_fragment`, `plan`, `telemetry`, `oracle_hold`,
`frame_embedding`):

The sha column differs per referencing table (`sha256`, `blob_sha`, `still_sha`),
so the trigger takes the column name as an argument and reads it via `hstore`:

```sql
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
```

Ingest is blob-first: write the blob row (`pending`), the isolated decode/embed
activity verifies the hash and flips it to `verified`, then context rows reference
it. A crash leaves an orphan blob (reclaimable), never a dangling reference. On-read
hash verification detects corruption; a mismatch sets `status='missing'` and is
reported, never served silently.

### 5.2 Retention and GC

The visual corpus is **permanent** (no 30-day prune). GC reclaims only orphans:

```sql
CREATE FUNCTION evidence.gc_orphan_blobs(grace interval DEFAULT interval '1 hour')
RETURNS bigint AS $$
  WITH gone AS (
    DELETE FROM evidence.blob
    WHERE refcount <= 0 AND created_at < now() - grace
    RETURNING 1)
  SELECT count(*) FROM gone;
$$ LANGUAGE sql;
```

The grace period avoids racing an in-flight ingest. Export bundles are materialized
on demand from the catalog; they are derived, not a retention exemption.

### 5.3 Invalidation graph (shared-rig cascade)

A shared-rig refit invalidates **both arms'** accuracy cells and sag maps **and** the
pair clearance map, because all were measured through that rig. A zeros/kp change
cascades only to that arm.

```sql
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
```

A trigger on `world.shared_rig` status → `superseded` calls `invalidate_rig`. The old
rig stays in `shared_rig` (history) with `superseded_by`; nothing is deleted. This
replaces the file-era "backup + witness + restart capture" dance with one transaction.

### 5.4 Re-embedding on model upgrade

A durable `reembed` workflow walks the permanent blob store, runs the new model as
isolated `embed` activities, writes new-version vectors, and builds the new HNSW
index. Old and new embeddings are not comparable across models, so search is scoped
to a `model_id`; the corpus is re-embedded into the new space and the old index
retired. Permanent media retention is what makes this possible.

### 5.5 Bit-identical replay

Trajectory and telemetry bytes are stored exactly (`bytea EXTERNAL`); stills are the
exact delivered JPEG. A bundle materializes these bytes unchanged, so deterministic
replay (`agent_eval --trace --replay`) feeds bit-identical inputs. `jsonb` is used
only for queryable metadata, never for the replay-critical payload.

## 6. Two-arm cooperation specifics

- **Three scope tiers** (§4.4–4.6): arm (physical, never merges), pair (exchange
  poses, clearance, rendezvous), fleet (protocol/perception cases). `learning.scope`
  on signatures/cases drives what aggregates across benches.
- **Coordination failure class** (`learning.failure_class = 'coordination'`) with
  rendezvous-miss / coupled-collision / sync-fault / handoff-drop kinds, attributed
  by a coordination oracle (rendezvous error in W, minimum inter-arm distance from
  both telemetry streams + FK, handoff outcome from the post-exec still).
- **Rendezvous handshake is edge-to-edge**, never via Postgres. The durable
  `coord.rendezvous` row records the schedule and outcome; the millisecond
  "A confirms → B grasps" signal runs between the edge players.
- **Coupled clearance margin** ≥ measured tracking error (~23 mm/arm). The planner
  reads `learning.clearance_cell`; unvalidated relative poses are reported, never
  extrapolated (same identifiability discipline as sag/accuracy).
- **Coordination bundles need both arms' evidence** or `bundle_complete=false` and
  the case is not exported. Replay starts two `MockEdge` instances + the shared world
  config + tapes for both.

## 7. Safety and gates (unchanged)

- The three execution gates and the shared `plan_to_target` check chain
  (`command_ok`, `path_safe`, speed clamp, edge follow-error/stall abort) stay
  per-arm. The seated-hinge peel remains mandatory in the shared move path.
- New global gates: no motion into the other arm's occupied volume; rendezvous
  requires both arms confirmed before release; a fault in one arm safe-stops the
  cooperation (the other holds/retracts via a pre-planned reaction).
- The coordinator never moves an arm outside the existing gates; it schedules
  gated single-arm motions with synchronization points.

## 8. Migration from current `state/`

Phased, each step gated by the previous step's tests staying green:

1. **Stand up the substrate.** Create schemas/tables/extensions on the named volume;
   deploy Duroxide; bring up isolated activity workers. No behavior change yet.
2. **Dual-write.** Capture server writes events/stills/telemetry/oracle to both the
   files and PG; verify row counts and hash equality against the files.
3. **Backfill.** Ingest existing `state/` into PG (blobs by sha, episodes, plans,
   telemetry, oracle, calib, sag, accuracy). Verify referential integrity and that
   bit-identical replay still passes from PG-sourced bundles.
4. **Cut over reads.** Planner/gateway/triage/identify read from PG; files become
   shadow. Run the full offline suite + a bench dry-plan in the planner container.
5. **Flatten.** Remove `state/` as a source of truth; rename residuals to `config/`
   and `export/`. Keep a read-only archive of the old `state/` until cutover is proven.

Motion-affecting cutover steps require explicit user go-ahead with the arm parked.

**Execution status (done):** steps 2–3 were skipped by directive (non-production bench;
old file state discarded, no backfill). The cutover went straight to PG-only writes/reads
in PG mode, and the flatten is complete: `state/` removed; `config/` (tls + bootstrap)
and `export/` (derived bundles) remain; disposable `work/` scratch holds cv2
materializations, the MediaMTX spool (ingested into PG then deleted), and file-mode
artifacts for offline tests. The rig/witness/intrinsics/sag caches rematerialize from PG
at startup (`rig_doc`, `witness_sha` migration 0009, `calib_doc`, `learning.sag_map`), so
wiping `work/` loses nothing authoritative. File mode (no `REBOT_PG_DSN`) survives for the
offline selftest under `work/`. Validated: 646-check offline selftest (file mode) + live
PG-mode dry checks (synthetic-pose `/plan` → `evidence.plan`, capture upload/serving/list
from PG with no file writes, `/trace` → `evidence.event`, rig seeded into
`world.shared_rig`). The arm was never commanded; motor reads were unavailable (unpowered)
during the work, and no motion path was exercised.

**Execute→oracle wiring (0021):** the live hold-grab timer is retired. `/execute`
now starts the durable graph (prepare → claim+play activity → wait for the recording
→ extract stills offline → detect → `oracle_compute`/`oracle_learn`). The isolated
activity service merges the planner half (record, play, ingest telemetry) and the
capture half (decode, select, detect). Dry validation with the arm parked: the real
graph against the real activity service in `REBOT_WORKFLOW_DRY=1` (no gateway/edge
contact) produced a verified/learned execution, one extracted hold, one verdict, and
an idempotent replay (no second verdict/cell/still, no re-claim). The planner log
showed only health checks throughout. Full offline selftest: 682 checks, 0 failed.
Live parity still needs a powered arm.

## 9. Test plan

- **Schema/integrity (offline):** FK cascade, blob refcount triggers, orphan GC grace,
  blob-first ingest crash simulation (orphan, never dangling), on-read hash mismatch →
  `missing`.
- **Invalidation:** shared-rig refit unvalidates both arms' cells + pair map in one
  transaction; zeros change cascades to one arm only; old rig retained.
- **Coordinator:** concurrent rendezvous transitions cannot double-grasp / both-own
  (transactional test); object_track CHECK holds.
- **Crash isolation:** a fault injected in an isolated decode/embed activity fails the
  job and dead-letters after max_attempts; the cluster stays up. A Rust-native
  kinematics panic is caught at the boundary (no cluster reset).
- **Replay bit-identity:** a PG-sourced bundle replays bit-exact against unchanged
  code; a perturbed geometry constant produces a geometry-diff, not a pass.
- **Re-embed:** a model-upgrade workflow re-embeds a synthetic corpus into a new
  space; search is scoped per model_id; old index retired.
- **Similarity:** pgvector ANN recall vs exact KNN above threshold on a seeded corpus.
- **Bench (gated, manual):** dry CLI plan in the planner container before any motion;
  coordination handoff with consent, both arms parked/idle verified first.

### 9.1 Validation status (PostgreSQL 18.7, combined substrate image)

Validated end-to-end on `rebot-substrate:pg18` — the prebuilt
`ghcr.io/microsoft/pg_durable:pg18` (pg_durable 0.2.9) + pgvector 0.8.7 + the
`rebot_kinematics` pgrx extension built from `openrr/k` 0.32 (§11). PostgreSQL 18.7.

**Schema + mechanisms** (all 37 tables, enums, FKs, indexes, functions create in one
transaction; self-checking assertions pass):
- refcount triggers across all three sha-column names (`sha256`, `blob_sha`,
  `still_sha`); multi-table refcount (fragment + plan + telemetry + hold = 4) correct
  on insert and cascade delete.
- orphan GC removes unreferenced blobs past the grace period, keeps referenced ones.
- `invalidate_rig` supersedes the sag map, unvalidates accuracy + clearance cells,
  writes `world.invalidation` rows.
- `coord.object_track` CHECK rejects a held object without an owner.
- `oracle_hold` → `oracle_verdict` ON DELETE CASCADE.
- pgvector HNSW index builds and is used by the planner (`Index Scan using
  frame_embedding_hnsw_idx`); cosine search returns the nearest row.

**Kinematics (openrr/k → rebot_kinematics):**
- `rebot_dof(ReBot_Arm_RS.urdf)` = 8 (joint1..6 + gripper_joint1/2); URDF parses.
- `rebot_fk` at zero gives wrist `[0.1355, 0, 0.2177, …]` (tool-down).
- **FK/IK round-trip on the real 6-DOF arm passes**: IK from a zero seed recovered
  `[0.199, 0.299, 0.398, 0.199, 0.0996, 0.0004]` for the target pose (≈ the original
  `[0.2, 0.3, 0.4, 0.2, 0.1, 0]`, within 1e-3).
- Joint limits are enforced (an out-of-limit seed raises a SQL ERROR, not a crash).

**Gravity + collision (Rust ports of the numpy reference):**
- `rebot_gravity_torques` matches `rebot_ik.gravity_torques` to ~1e-6 Nm at
  `q=[0.2,0.3,0.4,0.2,0.1,0]` (ref `[0, 1.845377, 5.741341, 1.520166, -0.019008,
  -0.000782]`); uses the same 7-link inertial table and `g=9.80665`.
- `rebot_joint_points` reproduces the 7 joint/tool points (tool within 1e-3 m).
- `rebot_path_safe` reproduces `sweep.path_safe`: true for a high pose→itself, false
  for a fold-down that dips below the worktop (`CLEARANCE=0.012`, per-link `RADII`,
  `LINK_RADIUS_MAX=0.85`, 40 samples).
- `rebot_ray_plane` reproduces `extrinsics.ray_plane` (10-iteration fixed-point
  undistortion + ray/plane intersection): point/distance/grazing match to 1e-9 across
  six off-center pixels with radial+tangential distortion, the zero-distortion path,
  and the parallel-ray / away-ray rejections (SQL ERROR, matching `ExtrinsicsError`).

**Plan as a SQL function (`rebot_plan` / `rebot_plan_sequence` / `plan_one`, validated):**
- `rebot_plan(urdf, dump, target, options, snap)` is a pure Rust core (`plan.rs`)
  porting `plan_to_target.plan`'s hover branch — the same damped-LS CLIK IK as
  `reBotArm_control_py.solve_ik` (LOCAL Jacobian, log6 error, adaptive damping, active
  joint-limit constraints, backtracking line search), orientation/staging helpers,
  trapezoid legs, sag compensation, `traj.build`/`validate`.
- `rebot_plan_sequence(urdf, dump, targets, options, snap)` ports `plan_sequence`
  (multi-marker, gripper alternate): endpoint-first per marker, gripper open/close on
  odd/even, stage-and-slide on odd, joint-space return; shares the helpers.
- `plan_one` (PL/pgSQL, `0010`/`0011`) wraps both: consistent snapshot (URDF + active
  rig + keyed sag) → dispatch on target type (object→hover, array→sequence) → core →
  `evidence.plan` + blob, one transaction.
- Golden-trajectory parity vs the Pinocchio reference (`tests/plan_parity.py`): hover
  (3 seeds) + sequence (2 marker sets, 401/469 waypoints) — joint commands match to
  ~6e-15 rad, tau to ~2e-3 Nm (URDF RNEA gravity, not the numpy LINK_INERTIALS
  approximation the reference uses for the aim command), velocity/time at rounding
  precision; identical waypoint counts/durations/worst-IK.
- Scope: hover + sequence (both live gateway paths) done; the CLI-only Cartesian pick
  branch is a follow-up. `world.robot_model` must be seeded.

**Full compute switch (validated; the gateway is a thin HTTP→SQL adapter in PG mode):**
- `/move` builder: `rebot_move_build` + `move_one` (`0012`); `planner_server.move_one`
  switched. Parity exact (multi 2.8e-17), `tests/move_parity.py`.
- Oracle verdict: `oracle_compute` (`0013`) + Rust helpers `rebot_fk_tip`/`rebot_tip_point`/
  `rebot_sag_rad`; `capture_server._oracle_pg_compute` switched (file mode preserved).
  Parity 2.1e-17, `tests/oracle_parity.py`.
- Hand-eye fit: `rebot_calibrate_fit` (`calib.rs`) — DLT+SVD+RQ init, Huber LM, covariance,
  conditioning + predicted-error gates (port of `extrinsics.fit`). `extrinsics.fit`
  dispatches to it in PG mode (`_fit_pg`), so all calibration callers switch transparently.
  Parity exact 1.9e-15 / noisy 1.9e-12, identical rms; `tests/calib_parity.py` incl. dispatch.
- `/fk` display: `planner_server.fk_one` → `rebot_fk_tip` + `rebot_joint_points`. Parity
  2.2e-16 (7 points), `tests/fk_parity.py`.
- The Pinocchio `solve_ik` path is retired from the live motion path; residual `rebot_ik`
  (pure-numpy) calls are perception rendering, the oracle predicted-tip overlay, fit
  diagnostics, joint-limit constants, and file-mode fallbacks. Each switch keeps a Python
  fallback that re-validates (a substrate refusal still propagates). Full offline selftest
  646/0; SQL suites `test_plan_one`/`test_store`/`test_pg_trace`/`test_pg_evidence` pass.
  `/move` + oracle live executions need a powered arm + bench go-ahead (dry plans only here).

**Canonical arm config (`world.arm_config`, `0014`, validated):** the per-arm physical-model
constants formerly hardcoded and duplicated 2–3× across `rebot_ik`/`sweep`/`witness`/
`plan_to_target`/`plan_with_pinocchio` and the Rust cores now have one table-backed source
(`arm_id` + `config_doc` jsonb: gravity/limits/link_inertials/link_radii/tool/gripper/
collision/motion). The seed is bit-identical to the former constants (compute unchanged);
`link_inertials` is the simplified gravity model, deliberately not URDF-derived (~2e-3 Nm
vs raw RNEA). Python: `rebot_ik` is the single source (`sweep.RADII`/`witness.LINK_RADII`/planner
`MOTOR7_*` and the `traj`/`sweep` motion tuning alias it; default-arg functions resolve
at call time so table edits propagate); `arm_config.load_config()` reads the table in PG
mode at startup, defaults in file mode. Rust (option B, config REQUIRED): the cores stay
pure jsonb→jsonb and take a mandatory `config jsonb` arg parsed by `config.rs::Config::from_json`
(missing doc/field → hard refusal); `PlanCtx` owns the Config, helpers take &Config; the
hardcoded tuning consts are removed from the compute path (only structural ARM_JOINTS, the
4π MIT range, and out-of-scope learning/calib gates remain). The wrappers `plan_one`/`move_one`/
`oracle_compute` (`0015`) read `world.arm_config` and pass the doc in (missing row → refusal).
A required param cannot follow a defaulted one in PG, so config precedes the first optional arg.
Drift guard `tests/arm_config_parity.py`: table == Python bit-exact (whole doc incl. motion),
apply_config no-op on seed, and a core hard-refuses an empty config. Both physical-model and
motion groups consolidated; Rust fully table-driven. Parity machine-exact; selftest 646/0;
gravity/collision SQL test + SQL suites pass. The remaining gate groups are now consolidated
too (same required-config pattern): calibration fit gates (`0016`, `rebot_calibrate_fit`),
oracle attribution thresholds (`0017`, `oracle_attribute(doc, config)` fed `cfg->'oracle'` by
`oracle_compute`), and learning thresholds (`0018`, `rebot_fit_sag_map` + `accuracy_cell_update`
+ `identify`; `store.update_accuracy_cells` reads the learning group). `rebot_ik` is the Python
single source for all groups (CALIBRATION/ORACLE/LEARNING dicts; `apply_config` re-syncs the
aliasing modules). The via-chain + stream-rate tuning (`traj.DEFAULT_HZ`/`CORNER_RADIUS_RAD`/
`CHAIN_LINE_SAMPLES`/`CHAIN_FILLET_SAMPLES`, added by the S-ramp/via re-port) is consolidated
too (`0019`, motion group): the Rust cores read them from the required config, `rebot_joint_move`/
`rebot_joint_sequence` take `hz` as an optional arg resolved from `motion.default_hz` when omitted
(no baked-in 50.0 SQL default), and the sample counts are integer-validated at parse. Only
structural identifiers (ARM_JOINTS, MAX_VIAS) and pure numeric epsilons remain hardcoded.
Live /move + oracle executions need a powered arm + bench go-ahead (dry plans only here).

**Learning plane (Phase 1, ports validated against the Python reference):**
- `oracle_attribute(jsonb)` (PLpgSQL, `0002`) reproduces `oracle.attribute` across all
  seven residual branches (clean/sag/rig-mismatch/joint-zeros/abort/one-plane/tip-detector):
  layer, confidence, and problem kinds/layers match.
- `trace_redact(jsonb)` reproduces `trace._redact` (drops token/authorization/api_key
  recursively, keeps usage counters, reduces llm_url to host).
- `accuracy_cell_update` reproduces `identify.update_accuracy_map` (n/n_good/measured_mm/
  validated_at per cell).
- `rebot_fit_sag_map` (Rust) reproduces `identify.fit_sag_map` (Huber IRLS affine per
  joint, span + hold-out gates, FK sensitivity for validated_mm); per-motor coefficients
  match the reference (joint1's slope is unidentifiable — vertical axis, ~zero gravity
  torque — and both implementations correctly return ~0).
- `store.py` (psycopg3) is the PG-backed store seam: content-addressed blobs (hash-verified
  on read), redacted append-only events with per-episode advisory-lock seq assignment,
  episode/plan catalog. Round-trip tested.

**Offline hold-frame extraction (recording-as-source, wired into the workflow):**
- The oracle's hold stills are no longer grabbed live: the permanent executed-trajectory
  recording (`media_fragment.t_start/t_end`, episode-relative) + telemetry stillness let
  them be extracted after execution. `extract.py` (pure) aligns hold windows to fragments
  (`+t_offset`) and selects the stillest frame (min meas joint speed, midpoint tiebreak);
  `extract_service.py` is the isolated cv2-decode activity (`df.http`, crash-isolation §2).
  This dissolves the grab/playback interleaving so execute→oracle is a sequential durable
  workflow. Validated by `tests/test_extract.py` (19 checks) on a deterministic synthetic
  rig: window→fragment alignment, stillest-frame selection (incl. jitter), and tip-pixel
  parity with the old live midpoint grab (<0.1 px).
- **Wired end to end (`0021`):** `execute_workflow_start` builds the pg_durable graph
  prepare → play (isolated activity owns the claim guard) → wait for recording fragments
  → extract → detect → `oracle_compute`/`oracle_learn`. The activity service
  (`workflow_service.py`) merges the planner half (play/telemetry/record/ingest) and the
  capture half (decode/select/detect) in one isolated process. The live
  `_oracle_hold_grabs` timer is deleted; `/execute` no longer interleaves camera grabs
  with playback. Coverage/identify moves are recorded and use the same
  extract/detect/finalize functions via `execution_adopt`.
- **Logic placement (`0022`):** the remaining pure decisions moved from the
  activity/capture Python into SQL: `extraction_windows`, `extract_joint_speeds`,
  `extract_speed_at`, `select_hold_frame`, `episode_add_segment`/`execution_add_segment`,
  `execution_dry_telemetry`, `execution_receipt`, and the `coverage_workflow_start`
  graph. Python keeps only HTTP/edge, MediaMTX/file I/O, cv2/FFmpeg decode + JPEG
  encode, and the OpenCV detector. The large media payloads stay in the catalog; the
  activity reads them by execution id, so `df.nodes` only holds small JSON.
- **Play vs record timeouts:** `execute_workflow_start` has a separate
  `p_play_timeout_s` (default 900) for the in-flight `/execute` `df.http` step;
  `p_timeout_s` is only the recording wait. The activity→planner and planner→edge
  hops use `REBOT_PLAY_TIMEOUT` (default 900). A controlled 130 s `df.http`
  probe completed (`duration_ms=130003`, `timeout_seconds=180`), confirming
  duroxide has no shorter implicit step timeout. The one-shot claim (attempts=0)
  remains the motion guard; no pg_durable per-step `max_attempts` is needed.
- **Dry validation (arm parked):** `tests/test_execute_workflow.py` runs the real
  `df.start` graph against the real activity service in `REBOT_WORKFLOW_DRY=1` with a
  synthetic recording; verified/learned, one extracted hold, one verdict, attempts==1,
  and a replay that adds no verdict, folds no second cell, duplicates no still and does
  not re-claim the play. The planner log shows only health checks — no `/execute` reached
  it, so the arm never moved. Real-episode parity still needs a powered arm.

**Two-arm coordination (Phase 2, validated):**
- `rebot_self_collision_free` / `rebot_min_self_clearance` / `rebot_path_collision_free`
  (Rust): capsule-capsule check over non-adjacent links (sweep `RADII`), via an Ericson
  segment-distance primitive. Matches an independent Python capsule model to 1e-4 m.
  Deliberate contacts (parked j4-on-j2) must be excepted by the caller.
- `rebot_seg_seg_distance`: the segment-distance primitive, also used for inter-arm
  clearance (clearance = dist − radius_a − radius_b).
- `clearance_cell_update` (`0003`): folds inter-arm clearance into the pair-specific
  map, keyed on relative-tip-pose (position bins; extensible to full 6D). Validates at
  n≥3, tracks the worst gap.
- `coordination_oracle` (`0003`): attributes coordination residuals (clearance-violation,
  rendezvous-miss, sync-offset, handoff-drop, sync-fault).
- `rendezvous_transition` (`0003`): guarded state machine; release only from `confirmed`
  (both arms present), so premature release / double-grasp are impossible by construction.

**Media + similarity (Phase 3, validated):**
- `ingest.py`: HLS fragment ingester — MediaMTX spool (`playlist.m3u8` + segments) →
  `evidence.blob` + `evidence.media_fragment`, with `#EXTINF` timing. Safe byte-shuffle
  + hash + INSERT, **no decode** (FFmpeg stays an isolated activity). Content-addressed,
  idempotent per (episode, seq); blobs are bit-identical to the source segments.
- `embed.py`: pluggable `Embedder` interface + a deterministic `HistogramEmbedder`
  default for offline tests. Embeddings normalized (non-zero) before insert.
- **Isolated embed activity wired and validated end-to-end:** `embed_service.py`
  (separate HTTP process) is reached from a durable workflow via `df.http()`
  (`embed.start_embed_workflow`): read blob → base64 → POST → register model → upsert
  vector. The workflow-produced vector matches the reference (float4 storage precision),
  cosine search retrieves it, and durable replay is idempotent. A production CLIP/ONNX
  embedder implements `Embedder` and is selected in the service; the workflow and DB
  side do not change. Dev compose sets `pg_durable.http_security=unrestricted` to reach
  the host service; production keeps `restricted` + an approved HTTPS endpoint.
- `store.py` similarity methods populate `search.frame_embedding` (pgvector) and run
  `<=>` cosine search; an identical query returns distance ~0 and the correct nearest.
- The embedding-model choice remains a deployment fork (§10); the pipeline is model-
  agnostic.

**Durable orchestration (pg_durable):** a workflow `df.start(df.sql('SELECT
to_jsonb(rebot_fk(...))'))` ran to `status=completed` via the duroxide background
worker; `df.result` returned `{"rows":[{"pose":[0.1355,…]}],"row_count":1}`.
The execute→oracle graph (`execute_workflow_start`, `0021`) runs the same way:
`df.sql` prepare/ledger steps, a `df.http` activity step (claim+record+play),
a `df.loop(df.sleep)` recording-readiness loop, then `df.http` extract/detect and
`df.sql` finalize. Dry (`REBOT_WORKFLOW_DRY=1`) completed with the arm parked; the
activity returned `claimed:false` on replay, `execution_after_play` left the ledger
alone, and `oracle_learn` returned the recorded verdict.

**Gotchas found and fixed during validation:**
1. HNSW cannot index a bare `vector` column ("column does not have dimensions") —
   embedding columns are typed `vector(768)` (§4.9); dimension is a deployment constant.
2. Cosine distance is undefined for zero vectors (NaN; empty scan) — L2-normalize
   embeddings before insert.
3. pgrx 0.16 `cargo pgrx install` requires a `[[bin]]` named
   `pgrx_embed_<package>` (auto-discovered from `src/bin/pgrx_embed_rebot_kinematics.rs`);
   without it, schema generation fails ("Failed to find a pgrx_embed binary").
4. PG18 Docker images require the data volume mounted at `/var/lib/postgresql`
   (they create a major-version subdir); mounting `.../data` directly is rejected.
5. **pg_durable step results cannot be `FLOAT8[]`** (supported: scalars, text, jsonb,
   json, …). Kinematics returns `float8[]`, so wrap in `to_jsonb()` for a durable step.
6. nalgebra `UnitQuaternion::from_quaternion` takes a `Quaternion`, not a `Vector4`.
7. **pg_durable DSL graphs are built by SQL evaluation.** Use the functional form
   (`df.seq`/`df.as`) inline in `df.start(...)`; passing operator syntax (`~>`, `|=>`)
   as a bound parameter is treated as one literal SQL query ("syntax error at or near df").
8. **`df.http()` body substitution:** put the `$result.field` token inside a JSON string
   (`{"data_b64": "$req.b64"}`); a bare whole-body `$req.body` arrives SQL-quoted and is
   not valid JSON. Also strip `encode(bytea,'base64')` line wraps before strict decoding.
9. **pgvector stores float4** — vectors round-trip at single precision (~1e-8 absolute);
   compare against float64 references with a matching tolerance.

**Not yet built (gaps / later phases):**
- Isolated decode/OpenCV activities (FFmpeg frame re-extraction, calibration feature
  matching, detector) as `df.http()` services — the embed activity is the proven pattern.
- The production embedding model choice (§10 fork); the pipeline is model-agnostic.
- General obstacle collision beyond the worktop capsule model and deliberate-contact
  exceptions for two-arm self-collision.

**Flatten validation (added):** the full offline selftest passes (646 checks, 0 failed)
in file mode with the flattened `work/` root, run inside the capture container with the
workdir on container-local storage. Two environment traps found: (1) the selftest's
`O_APPEND` event log is unreliable on the Windows Docker bind mount (spurious empty
lines / partial appends) — run it with `--workdir /tmp/...` (container-local fs);
(2) the CAD-mesh and commissioned-rig tests need gitignored assets provisioned
(`app/cad/meshes/`, a fitted rig in the default calib dir). Live PG-mode dry checks
passed: synthetic-pose `/plan` persisted to `evidence.plan` (no file written), capture
upload/serving/list PG-only (bit-identical frame, no scratch file), `/trace` →
`evidence.event`, rig + witness schema seeded into `world.shared_rig`.

**Reproduction:** see §11 (build the image, `docker compose up -d`, apply migrations
`0001`–`0011`, run the SQL + Python suites under `examples/host/substrate/tests/`).

## 10. Open forks / decisions

1. **Clearance cell parameterization:** relative-tip-pose (6D, learnable in years of
   daily handoffs, ignores redundancy) vs full relative configuration (14D, exact,
   excitation-hungry). Recommended: 6D with explicit unlearned regions. Schema uses
   `cell_key` so the discretization is swappable without migration.
2. **Embedding model + versioning:** determines re-embed cost per upgrade. Decide
   before ingest so `model_id` is stable.
3. **bytea vs large objects:** bytea EXTERNAL for ~1 MB HLS fragments (recommended);
   `lo` only if fragments grow much larger.
4. **Trace spool:** optional local fire-and-forget buffer if chat-side tracing must
   survive PG blips. Recommended: none — PG is already a hard motion dependency.
5. **Cold tiering:** optional at multi-TB (≈5 yr, 1080p). Move cold fragments to a
   blob store with `blob.status`/location tracking; not required at current volume.

## 11. Substrate build & deployment (validated)

Reference implementation lives in `examples/host/substrate/`:

```
substrate/
  Dockerfile                 # multi-stage: build rebot_kinematics, ship in pg_durable:pg18
  docker-compose.yml         # PG service on named volume rebot-pgdata, port 5433
  rebot_kinematics/          # pgrx wrapper over openrr/k
    Cargo.toml               # pgrx =0.16.1, k =0.32, nalgebra =0.30, urdf-rs =0.9
    rebot_kinematics.control
    src/lib.rs               # rebot_fk / rebot_ik / rebot_dof
    src/bin/pgrx_embed_rebot_kinematics.rs   # REQUIRED for cargo pgrx schema gen
  migrations/0001_substrate.sql   # the §4+§5 DDL in dependency order
  migrations/0002_learning_functions.sql # oracle_attribute, trace_redact, accuracy_cell_update
  migrations/0003_coordination.sql  # clearance_cell_update, coordination_oracle, rendezvous_transition
  migrations/0020_execute_workflow.sql # execution ledger, claim guard, idempotent oracle_learn
  migrations/0021_execute_oracle_workflow.sql # durable graph, offline stills, finalize
  migrations/0022_workflow_logic.sql # extract planning/selection, timing, receipt, coverage graph
  store.py                        # PG-backed store seam (blobs/events/catalog), psycopg3
  ingest.py                       # HLS fragment ingester (no decode)
  embed.py                        # pluggable embedder + pgvector population
  embed_service.py                # isolated embed activity (df.http)
  extract.py                      # pure hold-window alignment + stillest-frame selection
  extract_service.py              # isolated cv2 decode activity (unit/legacy)
  workflow_service.py             # isolated execute + record + extract + detect activity
  execute_workflow.py             # start/await the durable graphs, receipt helpers
  ../app/oracle_vision.py         # green-tip detector shared by capture and activity
  tests/mechanism_tests.sql       # refcount/GC/invalidation/CHECK/cascade/HNSW
  tests/kinematics_tests.sql      # FK/IK round-trip on the real arm
  tests/gravity_collision_tests.sql # gravity + path_safe vs the numpy reference
  tests/oracle_attribute_tests.sql  # oracle_attribute + trace_redact vs Python
  tests/accuracy_cell_tests.sql   # accuracy_cell_update vs identify.update_accuracy_map
  tests/sag_fit_tests.sql         # rebot_fit_sag_map vs identify.fit_sag_map
  tests/self_collision_tests.sql  # capsule self-collision vs Python model
  tests/coordination_tests.sql    # clearance/oracle/rendezvous primitives
  tests/test_store.py             # store seam round-trip (needs psycopg)
  tests/test_ingest_embed.py      # HLS ingest + embedding + similarity (needs psycopg)
  tests/test_extract.py           # offline hold-frame extraction (synthetic rig)
  tests/execute_workflow_tests.sql # 0020 ledger/claim/idempotent learning
  tests/execute_oracle_workflow_tests.sql # 0021 prepare/ready/stills/finalize replay
  tests/workflow_logic_tests.sql  # 0022 SQL extract planning/selection/timing/receipt
  tests/test_execute_workflow.py  # dry end-to-end df.start graph (arm parked)
  tests/df_workflow.sql           # pg_durable workflow calling rebot_fk
```

**Image stack.** Base = prebuilt `ghcr.io/microsoft/pg_durable:pg18` (pg_durable
0.2.9 already preloaded + created in `postgres`). The Dockerfile's builder stage
adds Rust + `cargo-pgrx` 0.16.1 + `postgresql-server-dev-18`, compiles
`rebot_kinematics` against the image's own PG18 (`cargo pgrx init --pg18
/usr/lib/postgresql/18/bin/pg_config` then `cargo pgrx install --release`), and the
final stage copies the `.so` + extension SQL in and apt-installs `postgresql-18-pgvector`.
pg_durable itself is NOT compiled from source.

**Build & run:**
```
cd examples/host/substrate
docker build -t rebot-substrate:pg18 .
docker compose up -d                 # named volume; PG18 mount at /var/lib/postgresql
# connect: psql "postgresql://postgres@localhost:5433/postgres"
```

**Apply schema + extensions** (pg_durable is preinstalled; create the rest):
```
docker exec rebot-substrate psql -U postgres -c "CREATE EXTENSION IF NOT EXISTS rebot_kinematics;"
docker exec rebot-substrate psql -U postgres --single-transaction -v ON_ERROR_STOP=1 \
  -f /migrations/0001_substrate.sql      # creates vector/pgcrypto/hstore + all schemas
```

**Tests:**
```
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/mechanism_tests.sql
docker cp ../app/cad/ReBot_Arm_RS.urdf rebot-substrate:/tmp/rebot.urdf
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/kinematics_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/gravity_collision_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/df_workflow.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/execute_workflow_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/execute_oracle_workflow_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/workflow_logic_tests.sql
docker exec rebot-capture-capture-1 python /app/substrate/tests/test_execute_workflow.py
```

**Durable workflow shape** (kinematics as a checkpointed step):
```sql
SELECT df.start(df.sql(
  'SELECT to_jsonb(rebot_fk(pg_read_file(''/tmp/rebot.urdf''),
     ARRAY[0,0,0,0,0,0,0,0]::float8[], ''joint6'')) AS pose'), 'fk_demo') AS iid \gset
SELECT df.await_instance(:'iid', 30);
SELECT df.result(:'iid');     -- {"rows":[{"pose":[...]}],"row_count":1}
```

**Durable execute→oracle shape** (`0021`/`0022`; the activity owns the claim guard):
```sql
-- capture calls this through execute_workflow.start_execute_workflow():
SELECT execute_workflow_start(
  '20261124T000000Z-abc123',        -- exec_id
  '20261124T000000Z-abc123',        -- plan_id
  'b601', 'exec-20261124T000000Z-abc123',
  'http://activities:8766', 'default', 60, 900, 2, 'execute_oracle');
-- args: workspace, record_timeout_s, play_timeout_s, poll_s, label
-- graph: prepare -> /execute (claim+record+play) -> wait fragments
--        -> /extract -> store stills -> /detect -> finalize/learn
SELECT df.await_instance(:'iid', 1200);
SELECT df.status(:'iid'), df.result(:'iid');
```

Dry validation with the arm parked is `REBOT_WORKFLOW_DRY=1` on a **separate**
activity process (refused when `REBOT_EXECUTE=1`); `tests/test_execute_workflow.py`
seeds a synthetic recording and proves the graph, offline extraction, detection and
idempotent replay without contacting the edge.

**Deployment caveats:**
- The published `pg_durable` image is for **evaluation/learning, not production**
  (superuser durable instances, restricted HTTP). For production, install the
  released `.deb` on a hardened `postgres:18`, set `pg_durable.http_security`, and
  `df.grant_usage('app_role')` for non-superuser access. The worker role
  (`pg_durable.worker_role`) must be a superuser.
- `rebot_kinematics` parses the URDF per call (correctness-first); cache the loaded
  chain per session for hot IK loops.
- `k` covers FK/IK; collision (`rebot_path_safe`) and gravity (`rebot_gravity_torques`)
  are Rust ports of the numpy reference in the same extension (§9.1). `rebot_path_safe`
  is the worktop table-clearance gate, not full two-arm self-collision (still a gap).
- Data on a named volume, never a Windows bind mount (WAL locking). Disk headroom
  matters: the Rust builder layer is multi-GB; prune build cache if the host is tight.
