# reBot durable substrate

Single-substrate Postgres backend for the reBot arm stack: durable orchestration
(`microsoft/pg_durable` / duroxide), memory-safe kinematics (`openrr/k` as the
`rebot_kinematics` pgrx extension), similarity search (pgvector), and the full
state/learning/evidence schema. Design and rationale: `rebot-durable-state-spec.md`
(repo root), especially §11 (build & deployment) and §9.1 (validation status).

This is the **substrate foundation** (spec §8 step 1). It stands up the database and
proves the extensions compose; it does not yet wire the live capture/planner path
(dual-write/backfill/cutover are later phases) and does not move the arm.

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
docker exec rebot-substrate psql -U postgres -v ON_ERROR_STOP=1 -f /tests/df_workflow.sql
python tests/test_store.py   # needs psycopg; container on localhost:5433
python tests/test_ingest_embed.py tests/_spool/ep_ingest   # HLS ingest + similarity
```

Apply `0002_learning_functions.sql` and `0003_coordination.sql` after `0001`.

All pass on PG 18.7: schema mechanisms, FK/IK round-trip on the real 6-DOF arm,
gravity + collision matching the numpy reference, and a pg_durable workflow calling
`rebot_fk` to completion.

Note: after rebuilding the image with new functions, re-create the extension in the
running DB so it picks up the new SQL: `DROP EXTENSION rebot_kinematics CASCADE;
CREATE EXTENSION rebot_kinematics;` (the version is unchanged, so `CREATE EXTENSION
IF NOT EXISTS` alone will not refresh it).

## App cutover (§8) — status

The trace/evidence pipeline is cut over to the substrate. Run the capture server with
`--pg-dsn` (or `REBOT_PG_DSN`) and traces go to Postgres instead of `state/traces/`:

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

**Deeper paths — additive evidence mirror.** The capture/calibration/serving flow is
deeply file-coupled (`click_for` passes `frame.jpg` paths to the detector; image URLs
serve files; the rig hot-reload reads calib files), so a PG-only replacement would break
it. Instead, `pg_evidence.py:PgEvidenceMirror` mirrors evidence into the substrate as it
is produced — PG is the learning system of record, files stay the realtime serving cache.
Wired so far (additive, wrapped so a PG failure never breaks the arm/chat, §8.4):
- **Capture** (`save_capture`): meta/frames → `evidence.episode`/`still`/`blob` + manifest.
- **Oracle** (verdict write): plan trajectory → `evidence.plan`, verdict →
  `evidence.oracle_verdict`, holds → `accuracy_cell_update`. The plan is mirrored first
  to satisfy the `oracle_verdict.plan_id` FK.
- Validated: `tests/test_pg_evidence.py` (capture/plan/oracle/telemetry round-trip in FK
  order, accuracy cells validate). `mirror_telemetry` is implemented but not yet wired to
  the live telemetry write.

**Remaining migration scope:** calib (rig/intrinsics/sag/accuracy) → `world.*`/`learning.*`;
planner `out/` + `telemetry/` live wiring; and the `state/` → `config/` + `export/` flatten
(gated on those). Old file state can be discarded (non-production); no backfill needed.

## Notes / gotchas

- Data lives on a Docker **named volume**, never a Windows bind mount (WAL locking).
  PG18 images require the mount at `/var/lib/postgresql` (not `.../data`).
- The `rebot_kinematics` crate must include `src/bin/pgrx_embed_rebot_kinematics.rs`
  or `cargo pgrx install` fails at SQL generation.
- pg_durable step results cannot be `FLOAT8[]`; wrap kinematics output in `to_jsonb()`.
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
The §8 app migration (dual-write → backfill → cutover → flatten `state/` into
`config/` + `export/`); hardware-dependent steps; full 6D clearance orientation; and
deliberate-contact exceptions for self-collision (the parked j4-on-j2 hinge contact).
`rebot_path_safe` is the worktop table-clearance gate, not general obstacle collision.
The substrate moves no arm and does not yet wire the live capture/planner path.