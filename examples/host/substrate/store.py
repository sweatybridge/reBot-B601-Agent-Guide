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