# reBot durable substrate

Single-substrate Postgres backend for the reBot arm stack: durable orchestration
(`microsoft/pg_durable` / duroxide), memory-safe kinematics (`openrr/k` as the
`rebot_kinematics` pgrx extension), similarity search (pgvector), and the full
state/learning/evidence schema. Design and rationale: `rebot-durable-state-spec.md`
(repo root), especially §11 (build & deployment) and §9.1 (validation status).

This is the **durable substrate** (spec §8). It stands up the database, proves the
extensions compose, and carries the app cutover: with `REBOT_PG_DSN` set, the
capture/planner path reads and writes PG-only and `state/` is flattened away
(`config/` + `export/` + disposable `work/` scratch). It moves no arm.

## Stack

| Component | Source | Version |
|---|---|---|
| Orchestration | prebuilt `ghcr.io/microsoft/pg_durable:pg18` | pg_durable 0.2.9 |
| Kinematics | `rebot_kinematics/` (pgrx wrapper over `openrr/k`) | k 0.32, pgrx 0.16.1 |
| Similarity | `postgresql-18-pgvector` (apt) | pgvector 0.8.7 |
| Server | PostgreSQL | 18.7 |

`k` provides FK/IK/URDF. Collision checking and gravity dynamics are added as Rust
ports in the same extension: `rebot_path_safe` / `rebot_joint_points` (from
`sweep.path_safe`) and `rebot_gravity_torques` (from `rebot_ik.gravity_torques`),
validated against the pure-numpy reference. `rebot_path_safe` is the worktop
table-clearance gate, not full two-arm self-collision (a later phase).

SQL functions: `rebot_fk`, `rebot_ik`, `rebot_dof`, `rebot_gravity_torques`,
`rebot_joint_points`, `rebot_path_safe`, `rebot_fit_sag_map` (extension);
`oracle_attribute`, `trace_redact`, `accuracy_cell_update` (migration `0002`).

Learning plane (Phase 1): `oracle_attribute` / `trace_redact` / `accuracy_cell_update`
are SQL ports of `oracle.attribute` / `trace._redact` / `identify.update_accuracy_map`;
`rebot_fit_sag_map` is a Rust port of `identify.fit_sag_map`. All validated against the
Python reference. `store.py` (psycopg3) is the PG-backed store seam replacing the
file-based trace writes: content-addressed blobs, redacted append-only events, catalog.

Media + similarity (Phase 3): `ingest.py` moves MediaMTX HLS fragments into the catalog
(safe byte-shuffle, no decode); `embed.py` provides a pluggable `Embedder` (deterministic
`HistogramEmbedder` default for offline tests; production CLIP/ONNX runs isolated) and
populates pgvector. The embedding model is a deployment fork; the pipeline is model-agnostic.
`embed_service.py` is the **isolated embed activity**: a separate HTTP process reached from
a pg_durable workflow via `df.http()` (`embed.start_embed_workflow`), so native inference
can never crash the cluster (see "Isolated embed activity" below).

## Quickstart

```bash
cd examples/host/substrate
docker build -t rebot-substrate:pg18 .
docker compose up -d                       # named volume; host port 5433
# psql "postgresql://postgres@localhost:5433/postgres"

docker exec rebot-substrate psql -U postgres \
  -c "CREATE EXTENSION IF NOT EXISTS rebot_kinematics;"
docker exec rebot-substrate psql -U postgres --single-transaction \
  -v ON_ERROR_STOP=1 -f /migrations/0001_substrate.sql
```

## Tests

```bash
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/mechanism_tests.sql
docker cp ../app/cad/ReBot_Arm_RS.urdf rebot-substrate:/tmp/rebot.urdf
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/kinematics_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/gravity_collision_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/oracle_attribute_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/accuracy_cell_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/sag_fit_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/coordination_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/judge_protocol_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/ray_plane_tests.sql
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/df_workflow.sql
python tests/test_store.py   # needs psycopg; container on localhost:5433
python tests/test_pg_trace.py
python tests/test_pg_evidence.py
python tests/test_ingest_embed.py tests/_spool/ep_ingest   # HLS ingest + similarity
python tests/test_embed_activity.py   # isolated embed activity via df.http()
# plan core/wrapper (run in the planner container; see "Plan as a SQL function"):
#   docker exec rebot-capture-planner-1 python /app/substrate/tests/plan_parity.py
#   docker exec rebot-capture-planner-1 python /app/substrate/tests/test_plan_one.py
```

Apply migrations `0001`–`0019` in order (each is idempotent).

Full offline app suite (file mode; 646 checks) — run inside the capture container with
the workdir on container-local storage (the Windows bind mount breaks `O_APPEND`
atomicity and produces spurious empty lines in the event log):

```bash
docker run --rm -v "<examples/host>:/app" -w /app rebot-capture:local \
  python tests/selftest.py --workdir /tmp/selftest-work
```

Needs the gitignored assets provisioned: `app/cad/meshes/` (app/cad/fetch.py) and a
fitted rig in the default calib dir for the environment-dependent Bench tests.

All pass on PG 18.7: schema mechanisms, FK/IK round-trip on the real 6-DOF arm,
gravity + collision matching the numpy reference, and a pg_durable workflow calling
`rebot_fk` to completion.

Note: after rebuilding the image with new functions, re-create the extension in the
running DB so it picks up the new SQL: `DROP EXTENSION rebot_kinematics CASCADE;
CREATE EXTENSION rebot_kinematics;` (the version is unchanged, so `CREATE EXTENSION
IF NOT EXISTS` alone will not refresh it).

## App cutover (§8) — status

The trace/evidence pipeline is cut over to the substrate. Run the capture server with
`--pg-dsn` (or `REBOT_PG_DSN`) and traces go to Postgres instead of the file log:

```bash
python app/capture_server.py --pg-dsn "postgresql://postgres@localhost:5433/postgres"
```

- `pg_trace.py:PgTraceRecorder` implements the same async interface as
  `app/trace.py:TraceRecorder` (event/events/blob/manifest/prune + read_events/read_manifest),
  backed by `store.py`. PG I/O runs in a thread; a PG failure costs trace evidence, never
  a chat turn (§8.4). Retention is permanent (`prune` is a no-op).
- `handle_trace_read` reads via the recorder, so it works for both backends; the file
  `TraceRecorder` gained matching `read_events`/`read_manifest`.
- `--pg-workspace` sets the workspace id for trace sessions (default `default`).
- Requires `psycopg` in the capture server's environment and the substrate schema applied.
- Validated: `tests/test_pg_trace.py` (events/blobs/manifest round-trip, redaction,
  bit-identical blobs, write-once manifest, seq continuation across restart).

**Deeper paths — PG-only in PG mode.** With `--pg-dsn` set, the substrate is the system
of record and the authoritative writes/reads are PG-only (no file fallback):
- **Capture** (`handle_capture`, `grab_one`): meta/frames → `evidence.episode`/`still`/`blob`
  + manifest; a capture that cannot be persisted is refused (503), not stranded on disk.
  `handle_captures` lists from PG; `handle_file` serves frame/meta from PG; `click_for`
  reads meta from PG and materializes the blob for the detector.
- **Oracle**: plan read from `evidence.plan`; verdict → `evidence.oracle_verdict` (the
  mirror IS the write); hold/pre/post frames cataloged as `evidence.still` rows (refcount-
  protected) with scratch materializations for cv2; accuracy cells via `update_accuracy_cells`.
- **Planner**: plans/telemetry written and read PG-only (`evidence.plan`/`telemetry`); plan
  age from the catalog `created_at`; `/health` reports PG counts. The pure planner paths
  (plan_to_target/sweep) keep their file contract via a rig+sag read-cache materialized
  from PG per plan/move (`refresh_calib_cache`).
- **Calib**: rig is PG-authoritative (`world.shared_rig.rig_doc`, lossless `Rig.from_dict`);
  the witness reference frame is a blob (`witness_sha`, migration `0009`); intrinsics
  (`calib_doc`) and the sag map are materialized into the scratch cache at startup, so a
  cache wipe loses nothing. Accuracy reads are PG-only.
- Validated: `tests/test_pg_evidence.py`, `tests/test_pg_trace.py`, and the full offline
  selftest (646 checks, file mode) + live PG-mode dry checks (plan/capture/trace round-trips).

### The flatten — complete

`state/` is gone. The layout is now:
- **PG (named volume)** — all authoritative state: traces, captures, oracle, plans,
  telemetry, rig/witness/intrinsics, accuracy, sag, tags, monitor/KV, orchestration.
- **`config/`** — tls + bootstrap (deploy secrets, read at startup).
- **`export/`** — derived handoff bundles (staging; graduated cases live in git).
- **`work/`** — disposable scratch (gitignored): cv2 materializations, the MediaMTX
  recording spool (ingested into PG then deleted), and file-mode artifacts when no
  substrate is configured (offline selftest). Wiping `work/` loses nothing authoritative;
  the calib/rig/witness/sag caches rematerialize from PG at startup.

File mode (no `REBOT_PG_DSN`) still works for offline development/tests, with its
artifacts under `work/` instead of `state/`.

### Deployment (arm parked)

The host compose includes the `substrate` service (reuses the `substrate_rebot-pgdata`
named volume). From `examples/host`:

1. `docker compose up -d --build` — substrate + mediamtx + capture + planner; capture/planner
   wait for substrate healthy. `REBOT_PG_DSN` defaults to the compose-network DSN.
2. Apply migrations `0001`–`0019` (idempotent) and `CREATE EXTENSION rebot_kinematics`.
   Seed `world.robot_model` with the URDF if using `plan_one` (`tests/test_plan_one.py`);
   `world.arm_config` is seeded for `b601` by `0014` (the cores require it).
3. Verify `/health` (capture + planner), a dry `/plan` (synthetic pose — no motion), a
   capture upload → PG, and `/trace` → PG. Re-verify the arm parked read-only.

**Rollback**: unset `REBOT_PG_DSN` and restart — the servers run in file mode under `work/`.
PG data is untouched; switching back resumes from the substrate.

### What lives in PG now (comprehensive move)

**State / evidence (mirrored or PG-primary):**
- Traces (events/blobs/manifests) — full switch (`PgTraceRecorder`).
- Captures (meta/frame/target) — catalog + serving PG-primary; `locate_target` mirrors target.json.
- Oracle verdicts + hold/pre/post frames — `oracle_verdict` + `evidence.still`/blobs (via `trace_blob`).
- Plans + telemetry — `evidence.plan`/`telemetry` (planner mirrors at plan time; PG-primary reads).
- Rig — `world.shared_rig.rig_doc` (lossless `Rig.from_dict`), PG-primary startup load.
- Intrinsics — `world.calibration.calib_doc` + corners blob (`calibrate_intrinsics` mirrors).
- Accuracy envelope — `learning.accuracy_cell` (PG-primary reads).
- Sag maps — `learning.sag_map` (mirrored at fit time).
- Identify/CAD session evidence — episodes + blobs (`pg_cli.mirror_session`).
- Sweep poses — `sweep_video` mirrors poses.json.
- Recordings — periodic ingest loop mirrors completed mp4s to blobs.
- Tags — `world.tag`; in-memory state (monitor EWMA, llm context) — `learning.monitor_state`/`substrate_kv`.
  The monitor EWMA/tag baseline/proposals are **restored at startup**, so a restart
  doesn't reset the drift trend.

**Logic ported to PG functions (all validated against the Python reference):**
- Rust pgrx (`rebot_kinematics`): FK/IK/dof, gravity torques, worktop `path_safe`, capsule
  self-collision + `seg_seg_distance`, traj time law (`ease_fraction`/`ease_velocity`),
  `seconds_for`/`within_limits`, `project` (no-dist + distortion), `ray_plane` (iterative
  undistortion + plane intersection, matches `extrinsics.ray_plane` to 1e-9 incl. the
  parallel/away-ray rejections).
- Rust motion cores (`plan.rs`): `rebot_plan`/`rebot_plan_sequence` (hover + multi-marker,
  faithful CLIK port of `reBotArm_control_py.solve_ik`), `rebot_joint_move`/`rebot_joint_sequence`,
  `rebot_traj_build`/`rebot_traj_validate`, `rebot_unpark_pose`, `rebot_move_build` (the `/move`
  builder). Parity: joint commands to ~6e-15 rad, move builders exact (multi 2.8e-17).
- Rust oracle helpers (`rebot_fk_tip`/`rebot_tip_point`/`rebot_sag_rad`) + `oracle_compute`
  (migration `0013`, the hold-window verdict): parity 2.1e-17.
- Rust hand-eye fit (`calib.rs`): `rebot_calibrate_fit` — DLT+SVD+RQ init, Huber LM,
  covariance, conditioning + predicted-error gates (port of `extrinsics.fit`). Parity:
  exact rig 1.9e-15, noisy 1.9e-12, identical rms; `extrinsics.fit` dispatches to it in PG mode.
- SQL: `oracle_attribute`, `trace_redact`, `accuracy_cell_update`, `accuracy_cell_key`/
  `accuracy_validated_fraction`/`accuracy_warnings`, `judge_protocol`, sag fit (`rebot_fit_sag_map` Rust).

**Residual (correctly NOT in-worker):**
- OpenCV core (calibrate_intrinsics/auto_calibrate feature matching, `witness.check` drift/scale
  matching, detector) and FFmpeg decode — native/unsafe, run as **isolated activities**
  (crash-isolation invariant), called via `df.http()`.
- Embedding inference — the isolated activity is **wired** (`embed_service.py` +
  `embed.start_embed_workflow`, validated end-to-end); the production model choice is open
  (the default `HistogramEmbedder` exercises the pipeline deterministically).
- Judge geometry (`boxes_of`/`_geometry_problems`) — stays in the Python judge.
- `coord.*` two-arm cooperation logic — schema ready, wired when multi-arm lands.
- tls/secrets (`config/`), temp dirs, git code (`tests/agent_cases/`) — correctly stay files.

## Isolated embed activity (df.http)

Native inference must never run in a postmaster child (crash → cluster reset →
durable-retry loop). The embed activity is a separate process; pg_durable orchestrates
and checkpoints, the inference runs outside the cluster:

```bash
python embed_service.py --port 8765        # isolated activity (host or sidecar)
# then, from Python:
#   start_embed_workflow(store, still_sha, "http://host.docker.internal:8765/embed")
```

The durable graph (`embed.embed_workflow_sql`): read blob bytes → base64 → `df.http()`
POST to the service → register the reported model → upsert the vector into
`search.frame_embedding`. Every step is idempotent, so durable replay is safe. A service
crash or 5xx fails the HTTP step (retry/dead-letter) — the cluster is untouched.
Validated by `tests/test_embed_activity.py` (vector matches the reference, similarity
search, idempotent replay).

Deployment notes:
- The dev compose sets `pg_durable.http_security=unrestricted` so `df.http()` can reach
  the host service over plain HTTP. **Dev-only**: production keeps `restricted` and
  serves the activity behind an approved HTTPS endpoint (`pg_durable.http_allowed_domains`).
- A production CLIP/ONNX embedder implements `embed.Embedder` and is selected in
  `embed_service.py`; the workflow and database side do not change.

## Plan as a SQL function (`rebot_plan` / `plan_one`)

The plan logic runs in the substrate as a synchronous SQL function — no `df.start`,
no separate planner round trip for the compute. Two layers:

- **`rebot_plan(urdf, dump, target, options, snap)`** — the pure Rust core
  (`rebot_kinematics/src/plan.rs`), `jsonb -> jsonb`, no database access. A faithful
  port of `plan_to_target.plan`'s **hover branch** (the gateway tap path): the same
  damped-LS CLIK IK as `reBotArm_control_py.solve_ik` (LOCAL Jacobian, `log6` error,
  adaptive damping, active joint-limit constraints, backtracking line search — *not*
  openrr/k's different solver), the orientation/staging helpers, trapezoid legs, sag
  compensation, and `traj.build`/`validate`. Refusals panic → SQL ERROR → rollback.
- **`rebot_plan_sequence(urdf, dump, targets, options, snap)`** — the multi-marker
  core (port of `plan_to_target.plan_sequence`, the gateway `gripper="alternate"`
  path): endpoint-first per marker, gripper open/close on odd/even, stage-and-slide
  on odd markers, joint-space return. Shares the CLIK/leg/sag/build helpers.
- **`plan_one(workspace, pose, target, options, arm_id)`** — thin PL/pgSQL wrapper
  (migrations `0010`/`0011`): gathers a consistent snapshot (URDF from
  `world.robot_model` + active rig + keyed sag map), **dispatches on the target type**
  (object → `rebot_plan`, array → `rebot_plan_sequence`), calls the core in **one
  transaction**, persists the trajectory to `evidence.plan` + blob, returns
  `{plan_id, summary}`. The gateway becomes a thin HTTP→SQL adapter; the CLI can call
  the same function.

**Parity (validated, `tests/plan_parity.py`, run in the planner container):** both
hover (3 seeds) and sequence (2 marker sets, 401/469 waypoints) match the Pinocchio
reference — joint commands to **~6e-15 rad** (machine precision), tau to **~2e-3 Nm**
(URDF RNEA gravity), velocity/time at rounding precision; identical waypoint counts,
durations, and worst-IK. Iterative IK is not literally byte-identical across solvers,
so parity is physical equivalence within the acceptance band.

**Scope / follow-ups:** hover single-target and multi-marker sequence are implemented
and validated (both live gateway paths). The CLI-only Cartesian **pick** branch is a
follow-up (the orientation-blend helper is staged). `world.robot_model` must be seeded
with the URDF (`tests/test_plan_one.py` does this).

### Full compute switch — complete

With `REBOT_PG_DSN` set, **every gateway compute flow runs in the substrate**; the
gateway is a thin HTTP→SQL adapter and the Pinocchio `solve_ik` path is retired from
the live motion path. Each switch keeps a Python fallback for file mode / substrate
unavailability (the fallback re-validates, so a refusal still propagates):

| Flow | Gateway entry | Substrate function | Parity |
|---|---|---|---|
| Hover plan / sequence | `planner_server.plan_targets` | `plan_one` → `rebot_plan`/`rebot_plan_sequence` | dq ~6e-15 |
| `/move` builder | `planner_server.move_one` | `move_one` → `rebot_move_build` | exact / 2.8e-17 |
| Oracle verdict | `capture_server._oracle_pg_compute` | `oracle_compute` (0013) | 2.1e-17 |
| Hand-eye fit | `extrinsics.fit` → `_fit_pg` | `rebot_calibrate_fit` | 1.9e-15 |
| `/fk` display | `planner_server.fk_one` | `rebot_fk_tip` + `rebot_joint_points` | 2.2e-16 |

`extrinsics.fit` dispatches transparently, so all calibration callers (`auto_calibrate`
coarse/final fits, `calibrate_extrinsics`) use the substrate without per-call wiring.
The residual `rebot_ik` (pure-numpy) calls are perception rendering (identify/identify_cad/
witness/sweep), the oracle predicted-tip overlay, fit diagnostics, joint-limit constants,
and file-mode fallbacks — not the motion/oracle/calibration solve.

Validated: `tests/plan_parity.py`, `move_parity.py`, `oracle_parity.py`, `calib_parity.py`
(incl. the live `ex.fit` dispatch), `fk_parity.py`; the full offline selftest (646 checks);
SQL suites `test_plan_one`/`test_store`/`test_pg_trace`/`test_pg_evidence`. The `/move` and
oracle live executions need a powered arm + bench go-ahead (motors unpowered here; compute
parity proven separately, dry plans only).

```bash
docker exec rebot-capture-planner-1 python /app/substrate/tests/plan_parity.py
docker exec rebot-capture-planner-1 python /app/substrate/tests/move_parity.py
docker exec rebot-capture-planner-1 python /app/substrate/tests/fk_parity.py
docker exec rebot-capture-planner-1 python /app/substrate/tests/test_plan_one.py
docker exec rebot-capture-capture-1 python /app/substrate/tests/oracle_parity.py
docker exec rebot-capture-capture-1 python /app/substrate/tests/calib_parity.py
```

## Canonical arm config (`world.arm_config`)

The per-arm **physical-model constants** that were hardcoded — and duplicated 2–3×
across `rebot_ik.py` / `sweep.py` / `witness.py` / `plan_to_target.py` /
`plan_with_pinocchio.py` and the Rust cores — now have one table-backed source of
truth: `world.arm_config` (migration `0014`, `arm_id` text PK + `config_doc` jsonb).
The seed is **bit-identical** to the former constants, so the compute path is
unchanged. It is deliberately *not* derived from the URDF: `link_inertials` is the
simplified gravity model (gripper fingers omitted, tool mass lumped into
`gripper_end`) that differs from raw URDF RNEA by ~2e-3 Nm — the table stores that
model, not the URDF inertials.

`config_doc` groups: `gravity`, `limits`, `link_inertials`, `link_radii`, `tool`
(xyz/rpy), `gripper` (motor7_open/closed, rad_per_mm), `collision` (clearance,
link_radius_max), `motion` (speeds/ramp/limits tuning + stream rate `default_hz`
and via-chain `corner_radius_rad`/`chain_line_samples`/`chain_fillet_samples`,
`0019`), `calibration` (min_poses/
max_camera_m/max_tip_m fit gates, `0016`), `oracle` (12 attribution thresholds, `0017`),
`learning` (21 identify thresholds — sag-fit gates, accuracy edges/min_n/sectors,
marker/coverage/rig-age tuning, `0018`).

- **Python single source + table-driven.** `rebot_ik` owns the constants
  (`LINK_RADII`, `GRIPPER`, `CLEARANCE`, `LINK_RADIUS_MAX`, `MOTION` added);
  `sweep.RADII` / `witness.LINK_RADII`, the planners' `MOTOR7_*`, and the motion
  tuning (`traj.MAX_JOINT_STEP`/`LIMIT_TOL`/`RAMP_S`/`DEFAULT_HZ`/`CORNER_RADIUS_RAD`/
  `CHAIN_LINE_SAMPLES`/`CHAIN_FILLET_SAMPLES`, `sweep.DEFAULT_MIN_Z`/
  `PEAK_SPEED`/`MIN_CRUISE_SPEED`/`MIN_SECONDS`/`MAX_SECONDS`/`GRIPPER_PEAK_SPEED`/
  `SAG_LIMIT_RAD`) now alias it — the duplicate literals are gone. Functions that
  took these as default args (`traj.ease_fraction`/`ease_velocity`/`joint_move`/
  `joint_sequence`/`chain_series`/`_chain_path`, `sweep.seconds_for`/
  `gripper_seconds`/`gravity_sag`, `plan_to_target.Options.hz`) resolve them at
  call time, so a table edit propagates. The chain sample counts are floats in the
  doc (motion-group convention) and ints at the use sites; the Rust parser refuses
  a non-integer count. `rebot_ik.default_config()` / `apply_config(cfg)` build/apply the doc;
  `arm_config.load_config()` reads the table in PG mode (called at capture/planner
  startup) and falls back to the defaults in file mode.
- **Rust cores: config REQUIRED, no hardcoded constants (option B).** The cores stay
  pure `jsonb -> jsonb` (never read the table via SPI). Each compute core
  (`rebot_plan`/`rebot_plan_sequence`/`rebot_move_build`/`rebot_joint_move`/
  `rebot_joint_sequence`/`rebot_traj_build`/`rebot_traj_validate`/`rebot_unpark_pose`/
  `rebot_sag_rad`, and `rebot_gravity_torques`/`rebot_path_safe`/`rebot_path_collision_free`/
  `rebot_within_limits`/`rebot_seconds_for`) takes a mandatory `config jsonb` arg,
  parsed by `config.rs::Config::from_json` — a missing doc/field is a hard refusal
  (panic → SQL ERROR), so the live path can never silently run on a baked-in value.
  `PlanCtx` owns the `Config`; helpers take `&Config`. The former hardcoded tuning
  consts are **removed** from the compute path (only structural `ARM_JOINTS`, the
  mathematical MIT range 4π, and the out-of-scope learning/calib gates remain). The
  SQL wrappers (`plan_one`/`move_one`/`oracle_compute`, migration `0015`) read
  `world.arm_config` and pass the doc in; a missing row is a hard refusal there too.
  Because a required param cannot follow a defaulted one in PG, `config` sits before
  the first optional arg in the signatures that have defaults.
- **Drift guard.** With no independent Rust constants left, `tests/arm_config_parity.py`
  asserts table == Python (`rebot_ik.default_config`) bit-exact across the whole doc,
  that `apply_config` is a no-op on the seed, and that a core **hard-refuses an empty
  config** (the required-config guarantee).

```bash
docker exec rebot-capture-capture-1 python /app/substrate/tests/arm_config_parity.py
```

**Scope:** all gate groups are consolidated and table-driven (option B, required
config): *physical-model*, *motion-tuning*, *calibration* (`rebot_calibrate_fit`),
*oracle* attribution (`oracle_attribute`), and *learning* (`rebot_fit_sag_map`,
`accuracy_cell_update`, `identify`). The table is the canonical source; Python is
table-driven in PG mode (defaults in file mode); the Rust/SQL compute consumes the doc
the wrapper passes. Parity is machine-exact (the seed equals the former constants).
Only structural identifiers (`ARM_JOINTS`, parameter `NAMES`) and pure numeric
epsilons (`CMD_TOL`, `HOLD_TIME_TOL`, the 4π MIT range) remain hardcoded — they are not
tuning gates. Live `/move` + oracle executions still need a powered arm + bench
go-ahead (dry plans only here).

## Notes / gotchas

- Data lives on a Docker **named volume**, never a Windows bind mount (WAL locking).
  PG18 images require the mount at `/var/lib/postgresql` (not `.../data`).
- The `rebot_kinematics` crate must include `src/bin/pgrx_embed_rebot_kinematics.rs`
  or `cargo pgrx install` fails at SQL generation.
- pg_durable step results cannot be `FLOAT8[]`; wrap kinematics output in `to_jsonb()`.
- pg_durable DSL graphs are built by SQL evaluation: use the functional form
  (`df.seq`/`df.as`) inline in `df.start(...)`. Passing operator syntax (`~>`, `|=>`)
  as a bound parameter is treated as one literal SQL query.
- `df.http()` body substitution: put the `$result.field` token **inside a JSON string**
  (`{"data_b64": "$req.b64"}`); a bare whole-body `$req.body` arrives SQL-quoted and is
  not valid JSON. Strip `encode(...,'base64')` line wraps before strict decoding.
- pgvector stores **float4**: vectors round-trip at single precision (~1e-8 absolute);
  compare against float64 references with a matching tolerance.
- The published pg_durable image is for evaluation, not production (superuser durable
  instances). Harden before production use (spec §11 caveats).

## FAQ — design tradeoffs and deciding factors

These capture the alternatives considered and why the design landed where it did.
Full reasoning is in the spec; this is the short version.

**Q: Why move to Postgres at all — weren't files fine?**
For the *evidence plane* (immutable captures, frames, traces), files are still the right
shape: keyed access, write-once immutability, self-contained bundles, bit-identical
replay, inspectable with ordinary tools. What changed the calculus was the roadmap —
an arm running daily for **years**, learning continuously from failures. The *learning
plane* (accuracy cells, sag maps, failure signatures, case lifecycle) is permanent,
incrementally updated, and queried across the whole history. Files give neither
transactional consistency nor efficient cross-session queries; the accuracy map's
read-modify-write per execution is a latent race. So: files for evidence, a database
for learning. The two-plane split is the core decision.

**Q: Why Postgres and not SQLite?**
SQLite was the initial recommendation: single machine, single writer, zero operational
surface, backup = copy one file. Three things tipped it to Postgres: (1) the coordinator
as a **transactional state machine** shared across separate processes (rendezvous,
ownership) needs concurrent writers + row locking; (2) `LISTEN/NOTIFY`-style event
signaling and `pg_durable`'s background-worker runtime; (3) **pgvector** for similarity
search. SQLite's single-writer model and lack of these features became the friction.
SQLite is still the right answer for a single-process, single-bench setup; Postgres is
the answer once coordination, durable orchestration, and vector search share a substrate.

**Q: Why not keep media in a blob store (or files) and only metadata in Postgres?**
That was the first position, on grounds of WAL amplification, backup bloat, cache
pollution, and the FFmpeg crash risk. The deciding factor was **actual volume**: we
record only executed trajectories (~30 min/day, one shared camera), not 24/7 — roughly
120–250 GB/year, sub-TB over the system's life. At that bounded volume, media-in-Postgres
is viable and buys **true FK/ACID integrity** between vectors and media (no cross-system
integrity protocol). HLS fragments (small, append-only, surgical time-range access) fit
Postgres far better than whole-video blobs. If volume were 24/7 multi-TB, the answer
would flip back to a blob store + content-addressed references.

**Q: Why permanent retention instead of the spec's 30-day prune?**
Human review is asynchronous — a failure may be investigated months later — and improved
methods must be able to **re-process old episodes** (e.g. re-embed with a better model).
A 30-day window assumes timely review and fixed methods; neither holds for years-long
continuous learning. So the visual record is permanent; the 30-day default is overridden.

**Q: Stills vs video — what does learning actually consume?**
Learning consumes **stills + telemetry + events**, never video. The oracle measures
hold-window stills; sag/clearance fit from telemetry; perception/eval cases are fed
bit-identical stills. Video is a *forensic / re-extraction* source (scrubbing inter-hold
dynamics, pulling a frame at a new timestamp later), not a learning input. This is why
stills are the kept evidence and recordings can be treated as a re-extractable source.

**Q: Why pgvector / HNSW, and how often is the index refreshed?**
pgvector gives similarity search over the permanent corpus ("have we seen this failure
before?"). HNSW is chosen over IVFFlat because it builds **incrementally on insert** — no
scheduled rebuild as the corpus grows. There is no refresh cron: new data appends
continuously; recall is monitored conditionally; the only substantial refresh is
**re-embedding on a model upgrade** (version-driven, rare). Two gotchas: HNSW needs a
fixed-dimension column (`vector(768)`), and cosine distance is undefined for zero vectors
(normalize embeddings before insert).

**Q: How is referential integrity between vectors and media handled?**
Because media bytes live in `evidence.blob`, every reference is a real **foreign key** —
true ACID integrity, cascade deletes, no orphan/dangling protocol. (Had media stayed in a
blob store, integrity would be eventual: content-addressed sha256, blob-first writes /
row-first deletes, grace-period GC, and hash-verification scrub — crash-safe but not a
single transaction.) In-Postgres media is what lets us use plain FKs.

**Q: Why openrr/k (Rust) instead of Pinocchio?**
Pinocchio is C++; calling it from Postgres means FFI, and a deterministic crash in a
postmaster child resets the whole cluster (see next question). `openrr/k` is pure Rust
(nalgebra) — memory-safe, so a bad input raises a SQL ERROR (pgrx catches the panic),
never a backend crash. The tradeoff: `k` provides FK/IK/URDF only, **no collision checking
or gravity dynamics**, so those were ported to Rust (`rebot_path_safe`, `rebot_gravity_torques`,
capsule self-collision) and validated against the numpy reference.

**Q: Why must native compute be isolated? (the crash-isolation invariant)**
A deterministic crash in unsafe native code (C++ Pinocchio via FFI, FFmpeg decode, native
ONNX/PyTorch) running in a postmaster child resets the entire cluster; durable retry then
replays the same input → **crash-loop**. This is the one invariant the design won't trade
away. So: memory-safe Rust numerics (kinematics, sag/accuracy fitting) run in-worker;
unsafe/heavy native compute (decode, embedding inference, OpenCV) runs as **isolated
activities** whose failure fails the job, never the cluster. FFmpeg is never in the hot
ingest path — ingest is safe byte-shuffling; decode is retrospective and isolated.

**Q: Why pg_durable instead of cron + workers or Temporal?**
`pg_durable` (duroxide runtime) gives durable execution **inside** Postgres: workflows
defined in SQL (`df.start`, `df.sql`, `df.seq`), checkpointed per step, resumable after
crashes/restarts, with state in `df.*`/`duroxide.*`. No extra service infrastructure
(Redis/Temporal/queues), same auth/backup model as the data. It replaces the
cron + jobs-table + polling-worker pattern. The published image is evaluation-only;
production needs hardening (superuser worker, HTTP policy, `df.grant_usage`).

**Q: Is the coordinator realtime? Can it plan offline?**
The coordinator is a **planner**, and planning is offline — exactly like the existing
single-arm planner (compute a trajectory, hand it to the edge to play back). Only the
**edge control loop** is realtime. Simultaneous two-arm motion makes planning harder
(coupled, time-parameterized collision checking) but not realtime. The execution-time
rendezvous handshake ("A confirms → B grasps") is **edge-to-edge**, never via Postgres;
the durable `coord.rendezvous` row records the schedule/outcome, not the millisecond signal.

**Q: How does multi-arm cooperation change the design?**
The target is physical cooperation: one shared overhead camera, simultaneous motion,
offline planning, no live object tracking. The world model is **static config + immutable
plans** — no hot shared mutable state — so it doesn't force a realtime store. Learning
splits into three tiers: **arm** (sag/accuracy/zeros — physical, never merges across arms),
**pair** (exchange poses, clearance, rendezvous — specific to an arm pair + workspace),
**fleet** (protocol/perception cases — arm-independent, aggregates). A shared-rig refit
cascades invalidation to both arms' cells and the pair map. Clearance cells key on
relative-tip-pose (position bins implemented; full 6D orientation extensible).

**Q: Which embedding model?**
Left as a **deployment fork** — the pipeline is model-agnostic (`embed.py`'s `Embedder`
interface). A deterministic `HistogramEmbedder` is the offline default so the pipeline is
testable without a model; a production CLIP/ONNX embedder implements the interface and
runs isolated. The choice determines re-embed cost per upgrade, so decide before ingest.

**Q: What stayed out of scope?**
Hardware-dependent steps (the arm never moves in this work); the production embedding
model choice; full 6D clearance orientation; deliberate-contact exceptions for
self-collision (the parked j4-on-j2 hinge contact); and general obstacle collision
beyond the worktop capsule gate (`rebot_path_safe`). The §8 app migration itself is
done: PG-only reads/writes in PG mode, `state/` flattened into `config/` + `export/`
+ disposable `work/` scratch, MediaMTX spool ingested-then-deleted.
