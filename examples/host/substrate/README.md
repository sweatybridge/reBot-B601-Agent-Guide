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

## Notes / gotchas

- Data lives on a Docker **named volume**, never a Windows bind mount (WAL locking).
  PG18 images require the mount at `/var/lib/postgresql` (not `.../data`).
- The `rebot_kinematics` crate must include `src/bin/pgrx_embed_rebot_kinematics.rs`
  or `cargo pgrx install` fails at SQL generation.
- pg_durable step results cannot be `FLOAT8[]`; wrap kinematics output in `to_jsonb()`.
- The published pg_durable image is for evaluation, not production (superuser durable
  instances). Harden before production use (spec §11 caveats).