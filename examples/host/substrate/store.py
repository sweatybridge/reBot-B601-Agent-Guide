"""PG-backed store seam (spec §3.2 / §5) — replaces the file-based trace.py writes.

Content-addressed blobs (write-once, hash-verified on read), append-only events
(redacted at write via the SQL trace_redact function), and the episode/plan catalog.
The database is the system of record; no authoritative state lives on the filesystem.

Synchronous psycopg3 implementation. The capture server can wrap these calls in its
async loop (psycopg async or run_in_executor); the per-episode advisory lock keeps
seq assignment correct under concurrent writers.

    store = SubstrateStore("postgresql://postgres@localhost:5433/postgres")
    sha = store.put_blob(jpeg_bytes, kind="still")
    store.create_episode(sid, workspace_id, [arm_id])
    seq = store.record_event(sid, "page", "user_message", {"text": "hi", "api_key": "SECRET"})
"""
from __future__ import annotations

import hashlib
import json

import psycopg
from psycopg.rows import dict_row


class SubstrateStore:
    def __init__(self, dsn: str):
        self.dsn = dsn

    def _conn(self):
        return psycopg.connect(self.dsn, row_factory=dict_row)

    # -- blobs ---------------------------------------------------------- #
    def put_blob(self, data: bytes, kind: str = "still") -> str:
        """Store bytes once under their sha256; returns the hex digest."""
        sha = hashlib.sha256(data).hexdigest()
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status) "
                "VALUES (%s, %s, %s, %s, 'verified') ON CONFLICT (sha256) DO NOTHING",
                (sha, kind, data, len(data)),
            )
        return sha

    def get_blob(self, sha: str) -> bytes:
        with self._conn() as c:
            row = c.execute("SELECT bytes FROM evidence.blob WHERE sha256=%s", (sha,)).fetchone()
        if row is None:
            raise KeyError(sha)
        data = bytes(row["bytes"])
        if hashlib.sha256(data).hexdigest() != sha:
            # reported, never served silently (spec §5.1)
            raise ValueError("blob hash mismatch for %s" % sha)
        return data

    def blob_exists(self, sha: str) -> bool:
        with self._conn() as c:
            return c.execute("SELECT 1 FROM evidence.blob WHERE sha256=%s", (sha,)).fetchone() is not None

    # -- episodes / events ---------------------------------------------- #
    def ensure_workspace(self, workspace_id: str) -> None:
        """Create a minimal workspace if absent (trace sessions need a workspace FK)."""
        with self._conn() as c:
            c.execute(
                "INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m) "
                "VALUES (%s, %s, 0, 0.04, 0.05) ON CONFLICT (workspace_id) DO NOTHING",
                (workspace_id, workspace_id),
            )

    def ensure_arm(self, arm_id: str) -> None:
        """Create a minimal arm row if absent (evidence FKs need world.arm)."""
        with self._conn() as c:
            c.execute(
                "INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key) "
                "VALUES (%s, %s, 'unknown', 'none', 'default') ON CONFLICT (arm_id) DO NOTHING",
                (arm_id, arm_id),
            )

    def create_episode(self, episode_id: str, workspace_id: str, arm_ids, kind: str = "execution") -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind) "
                "VALUES (%s, %s, %s, %s) ON CONFLICT (episode_id) DO NOTHING",
                (episode_id, workspace_id, json.dumps(list(arm_ids)), kind),
            )

    def record_event(self, episode_id: str, src: str, type: str, payload=None,
                     arm_id=None, blob_sha=None) -> int:
        """Append one redacted event; returns its per-episode monotonic seq."""
        with self._conn() as c:
            # serialize seq assignment per episode within this transaction
            c.execute("SELECT pg_advisory_xact_lock(hashtext(%s))", (episode_id,))
            red = c.execute("SELECT trace_redact(%s::jsonb) AS r", (json.dumps(payload or {}),)).fetchone()["r"]
            seq = c.execute(
                "SELECT coalesce(max(seq), 0) + 1 AS s FROM evidence.event WHERE episode_id=%s",
                (episode_id,),
            ).fetchone()["s"]
            c.execute(
                "INSERT INTO evidence.event(episode_id, seq, t, src, type, arm_id, payload, blob_sha) "
                "VALUES (%s, %s, now(), %s, %s, %s, %s, %s)",
                (episode_id, seq, src, type, arm_id, json.dumps(red), blob_sha),
            )
        return seq

    def get_events(self, episode_id: str):
        with self._conn() as c:
            return c.execute(
                "SELECT seq, src, type, arm_id, payload, blob_sha FROM evidence.event "
                "WHERE episode_id=%s ORDER BY seq", (episode_id,),
            ).fetchall()

    # -- manifests / episode listing ------------------------------------ #
    def write_manifest(self, episode_id: str, snapshot: dict) -> None:
        """Write-once environment snapshot (a later call cannot change it), redacted."""
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.manifest(episode_id, snapshot) VALUES (%s, trace_redact(%s::jsonb)) "
                "ON CONFLICT (episode_id) DO NOTHING",
                (episode_id, json.dumps(snapshot)),
            )

    def read_manifest(self, episode_id: str):
        with self._conn() as c:
            row = c.execute("SELECT snapshot FROM evidence.manifest WHERE episode_id=%s",
                            (episode_id,)).fetchone()
        return row["snapshot"] if row else None

    def episode_exists(self, episode_id: str) -> bool:
        with self._conn() as c:
            return c.execute("SELECT 1 FROM evidence.episode WHERE episode_id=%s",
                             (episode_id,)).fetchone() is not None

    def list_episodes(self):
        with self._conn() as c:
            return [r["episode_id"] for r in c.execute(
                "SELECT episode_id FROM evidence.episode ORDER BY episode_id").fetchall()]

    # -- capture catalog reads (PG-primary serving) --------------------- #
    def list_captures(self, limit: int = 20):
        with self._conn() as c:
            return c.execute(
                "SELECT e.episode_id, e.kind, e.started_at, m.snapshot, "
                "s.sha256, s.width, s.height, b.size_bytes "
                "FROM evidence.episode e "
                "LEFT JOIN evidence.manifest m ON m.episode_id=e.episode_id "
                "LEFT JOIN LATERAL (SELECT sha256, width, height FROM evidence.still "
                "  WHERE episode_id=e.episode_id AND purpose='witness' ORDER BY id DESC LIMIT 1) s ON true "
                "LEFT JOIN evidence.blob b ON b.sha256=s.sha256 "
                "WHERE e.kind IN ('calibration','execution') "
                "ORDER BY e.started_at DESC LIMIT %s", (limit,)).fetchall()

    def read_capture_meta(self, capture_id: str):
        return self.read_manifest(capture_id)

    def read_capture_frame(self, capture_id: str):
        with self._conn() as c:
            row = c.execute(
                "SELECT b.bytes FROM evidence.still s JOIN evidence.blob b ON b.sha256=s.sha256 "
                "WHERE s.episode_id=%s AND s.purpose='witness' LIMIT 1", (capture_id,)).fetchone()
        return bytes(row["bytes"]) if row else None

    # -- plan catalog --------------------------------------------------- #
    def register_plan(self, plan_id: str, episode_id: str, arm_id: str, blob_sha: str,
                      metadata=None, snapshot=None, group_id=None) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.plan(plan_id, episode_id, group_id, arm_id, blob_sha, metadata, snapshot) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s) ON CONFLICT (plan_id) DO NOTHING",
                (plan_id, episode_id, group_id, arm_id, blob_sha,
                 json.dumps(metadata or {}), json.dumps(snapshot or {})),
            )

    # -- media fragments ------------------------------------------------ #
    def add_still(self, episode_id: str, sha: str, purpose: str = "hold", arm_id=None,
                  t=None, width=None, height=None) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.still(episode_id, sha256, arm_id, purpose, t, width, height) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s)",
                (episode_id, sha, arm_id, purpose, t, width, height),
            )

    def add_fragment(self, episode_id: str, sha: str, seq: int, t_start: float, t_end: float,
                     codec=None, width=None, height=None) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end, codec, width, height) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT (episode_id, seq) DO NOTHING",
                (episode_id, sha, seq, t_start, t_end, codec, width, height),
            )

    # -- calibration catalog (world / learning) ------------------------- #
    def ensure_calibration(self, calib_fingerprint: str, K=None, dist=None, image_size=None) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size) "
                "VALUES (%s, %s, %s, %s) ON CONFLICT (calib_fingerprint) DO NOTHING",
                (calib_fingerprint, json.dumps(K or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                 json.dumps(dist or [0, 0, 0, 0, 0]), json.dumps(image_size or [0, 0])),
            )

    def upsert_rig(self, rig_fingerprint: str, workspace_id: str, calib_fingerprint: str,
                   T_base_cam, image_size, tip_offset, method="base-marker",
                   rms_px=None, rms_mm=None, witness=None, provenance=None) -> None:
        """Mirror a rig as the active shared rig; supersede prior active rigs."""
        self.ensure_calibration(calib_fingerprint)
        with self._conn() as c:
            c.execute(
                "UPDATE world.shared_rig SET status='superseded' "
                "WHERE workspace_id=%s AND status='active' AND rig_fingerprint<>%s",
                (workspace_id, rig_fingerprint),
            )
            c.execute(
                "INSERT INTO world.shared_rig(rig_fingerprint, workspace_id, calib_fingerprint, "
                "T_base_cam, image_size, tip_offset, method, rms_px, rms_mm, witness, provenance, status) "
                "VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,'active') "
                "ON CONFLICT (rig_fingerprint) DO UPDATE SET status='active', "
                "T_base_cam=EXCLUDED.T_base_cam, provenance=EXCLUDED.provenance",
                (rig_fingerprint, workspace_id, calib_fingerprint, json.dumps(T_base_cam),
                 json.dumps(image_size), json.dumps(tip_offset), method, rms_px, rms_mm,
                 json.dumps(witness or {}), json.dumps(provenance or {})),
            )

    def upsert_sag_map(self, arm_id: str, map_doc: dict) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO learning.sag_map(arm_id, rig_fingerprint, kp_key, zeros_key, "
                "per_motor, samples, rms_mrad, validated_mm, envelope, status) "
                "VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,'active')",
                (arm_id, map_doc.get("rig_fingerprint", ""), map_doc.get("kp_key", "default"),
                 map_doc.get("zeros_key", "none"), json.dumps(map_doc.get("per_motor", {})),
                 map_doc.get("samples", 0), map_doc.get("rms_mrad"), map_doc.get("validated_mm"),
                 json.dumps(map_doc.get("envelope", {}))),
            )

    # -- oracle / telemetry catalog ------------------------------------- #
    def add_oracle_verdict(self, plan_id: str, arm_id: str, verdict: dict, doc: dict, exec_id=None) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s)",
                (plan_id, exec_id, arm_id, verdict.get("layer", "none"),
                 verdict.get("confidence", "low"), json.dumps(verdict.get("problems", [])),
                 json.dumps(doc)),
            )

    def update_accuracy_cells(self, arm_id: str, rig_fingerprint: str, doc: dict, threshold_mm: float = 10.0) -> None:
        with self._conn() as c:
            c.execute("SELECT accuracy_cell_update(%s, %s, %s::jsonb, %s)",
                      (arm_id, rig_fingerprint, json.dumps(doc), threshold_mm))

    def add_telemetry(self, exec_id: str, plan_id: str, arm_id: str, blob_sha: str,
                      kp_key: str, summary: dict) -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary) "
                "VALUES (%s, %s, %s, %s, %s, %s) ON CONFLICT (exec_id) DO NOTHING",
                (exec_id, plan_id, arm_id, blob_sha, kp_key, json.dumps(summary)),
            )

    # -- similarity (pgvector) ------------------------------------------ #
    def register_embedding_model(self, model_id: str, name: str, version: str, dim: int, kind: str = "frame") -> None:
        with self._conn() as c:
            c.execute(
                "INSERT INTO search.embedding_model(model_id, name, version, dim, kind) "
                "VALUES (%s, %s, %s, %s, %s) ON CONFLICT (model_id) DO NOTHING",
                (model_id, name, version, dim, kind),
            )

    def put_frame_embedding(self, still_sha: str, model_id: str, embedding) -> None:
        vec = "[" + ",".join(repr(float(x)) for x in embedding) + "]"
        with self._conn() as c:
            c.execute(
                "INSERT INTO search.frame_embedding(still_sha, model_id, embedding) "
                "VALUES (%s, %s, %s::vector) ON CONFLICT (still_sha, model_id) DO NOTHING",
                (still_sha, model_id, vec),
            )

    def search_similar(self, query_embedding, model_id: str, k: int = 5):
        vec = "[" + ",".join(repr(float(x)) for x in query_embedding) + "]"
        with self._conn() as c:
            return c.execute(
                "SELECT still_sha, embedding <=> %s::vector AS distance "
                "FROM search.frame_embedding WHERE model_id=%s "
                "ORDER BY embedding <=> %s::vector LIMIT %s",
                (vec, model_id, vec, k),
            ).fetchall()