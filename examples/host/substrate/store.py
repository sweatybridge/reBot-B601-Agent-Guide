"""PG-backed store seam (spec §3.2 / §5) — pooled connections + single-round-trip RPCs.

Content-addressed blobs (write-once, hash-verified on read), append-only redacted
events, and atomic single-round-trip SQL RPCs for trace batches, capture ingest,
rig activation, planning, calibration, sag fitting, and durable execute->oracle.

Uses `psycopg_pool.ConnectionPool` when installed, falling back to a lightweight
thread-safe bounded `LifoQueue` pool with connect timeout and TCP keepalives so
pooling works with zero extra runtime dependencies.
"""
from __future__ import annotations

import base64
import contextlib
import hashlib
import json
import math
import queue
from pathlib import Path

import psycopg
from psycopg.rows import dict_row

try:
    from psycopg_pool import ConnectionPool as _PsycopgPool
except ImportError:  # pragma: no cover - fallback when psycopg-pool is absent
    _PsycopgPool = None


def _connect_kwargs(dsn: str) -> dict:
    return {
        "conninfo": dsn,
        "row_factory": dict_row,
        "connect_timeout": 10,
        "keepalives": 1,
        "keepalives_idle": 30,
        "keepalives_interval": 10,
        "keepalives_count": 5,
    }


class _SimplePool:
    """Tiny thread-safe bounded psycopg connection pool with liveness checks."""

    def __init__(self, dsn: str, max_size: int = 8):
        self.dsn = dsn
        self._q: queue.LifoQueue = queue.LifoQueue(maxsize=max_size)

    def _open(self):
        return psycopg.connect(**_connect_kwargs(self.dsn))

    def _acquire(self):
        while True:
            try:
                conn = self._q.get_nowait()
            except queue.Empty:
                return self._open()
            if conn.closed or conn.broken:
                continue
            try:
                if conn.info.transaction_status != psycopg.pq.TransactionStatus.IDLE:
                    conn.rollback()
                return conn
            except Exception:  # noqa: BLE001
                try:
                    conn.close()
                except Exception:  # noqa: BLE001
                    pass

    def _release(self, conn) -> None:
        if conn.closed or conn.broken:
            return
        try:
            if conn.info.transaction_status != psycopg.pq.TransactionStatus.IDLE:
                conn.rollback()
            self._q.put_nowait(conn)
        except queue.Full:
            conn.close()
        except Exception:  # noqa: BLE001
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass

    @contextlib.contextmanager
    def connection(self):
        conn = self._acquire()
        try:
            yield conn
            if not conn.closed and conn.info.transaction_status == psycopg.pq.TransactionStatus.INTRANS:
                conn.commit()
        except BaseException:
            if not conn.closed:
                try:
                    conn.rollback()
                except Exception:  # noqa: BLE001
                    conn.close()
            raise
        finally:
            self._release(conn)

    def close(self) -> None:
        while True:
            try:
                conn = self._q.get_nowait()
                conn.close()
            except queue.Empty:
                break
            except Exception:  # noqa: BLE001
                pass


class SubstrateStore:
    def __init__(self, dsn: str, pool_size: int = 8):
        self.dsn = dsn
        if _PsycopgPool is not None:
            self._pool = _PsycopgPool(
                dsn, min_size=1, max_size=pool_size,
                kwargs={k: v for k, v in _connect_kwargs(dsn).items() if k != "conninfo"},
                open=True,
            )
            self._native_pool = True
        else:
            self._pool = _SimplePool(dsn, max_size=pool_size)
            self._native_pool = False

    def close(self) -> None:
        self._pool.close()

    @contextlib.contextmanager
    def conn(self):
        """Borrow a pooled connection; commits on clean exit, rolls back on error."""
        if self._native_pool:
            with self._pool.connection() as c:
                with c.transaction():
                    yield c
        else:
            with self._pool.connection() as c:
                yield c

    # Backwards-compatible alias used across substrate modules & tests.
    _conn = conn

    def scalar(self, sql: str, args=()):
        """Execute a single-row, single-column query and return its value."""
        with self.conn() as c:
            row = c.execute(sql, args).fetchone()
        return next(iter(row.values())) if row else None

    def _verify_blob(self, sha: str, data: bytes) -> bytes:
        if hashlib.sha256(data).hexdigest() != sha:
            # Mark corrupted blob missing before reporting (spec §5.1).
            try:
                with self.conn() as c:
                    c.execute("UPDATE evidence.blob SET status='missing' WHERE sha256=%s", (sha,))
            except Exception:  # noqa: BLE001
                pass
            raise ValueError("blob hash mismatch for %s" % sha)
        return data

    # -- blobs ---------------------------------------------------------- #
    def put_blob(self, data: bytes, kind: str = "still") -> str:
        """Store bytes once under their sha256; refreshes created_at on conflict."""
        sha = hashlib.sha256(data).hexdigest()
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status) "
                "VALUES (%s, %s, %s, %s, 'verified') "
                "ON CONFLICT (sha256) DO UPDATE SET created_at = now()",
                (sha, kind, data, len(data)),
            )
        return sha

    def get_blob(self, sha: str) -> bytes:
        with self.conn() as c:
            row = c.execute("SELECT bytes FROM evidence.blob WHERE sha256=%s", (sha,)).fetchone()
        if row is None:
            raise KeyError(sha)
        return self._verify_blob(sha, bytes(row["bytes"]))

    def blob_exists(self, sha: str) -> bool:
        with self.conn() as c:
            return c.execute("SELECT 1 FROM evidence.blob WHERE sha256=%s", (sha,)).fetchone() is not None

    # -- episodes / events / traces ------------------------------------- #
    def ensure_workspace(self, workspace_id: str) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m) "
                "VALUES (%s, %s, 0, 0.04, 0.05) ON CONFLICT (workspace_id) DO NOTHING",
                (workspace_id, workspace_id),
            )

    def ensure_arm(self, arm_id: str) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key) "
                "VALUES (%s, %s, 'unknown', 'none', 'default') ON CONFLICT (arm_id) DO NOTHING",
                (arm_id, arm_id),
            )

    def create_episode(self, episode_id: str, workspace_id: str, arm_ids, kind: str = "execution") -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.episode(episode_id, workspace_id, arm_ids, kind) "
                "VALUES (%s, %s, %s, %s) ON CONFLICT (episode_id) DO NOTHING",
                (episode_id, workspace_id, json.dumps(list(arm_ids)), kind),
            )

    def record_event(self, episode_id: str, src: str, type: str, payload=None,
                     arm_id=None, blob_sha=None) -> int:
        """Append one redacted event inline in a single transaction."""
        with self.conn() as c:
            c.execute("SELECT pg_advisory_xact_lock(hashtext(%s))", (episode_id,))
            seq = c.execute(
                "SELECT coalesce(max(seq), 0) + 1 AS s FROM evidence.event WHERE episode_id=%s",
                (episode_id,),
            ).fetchone()["s"]
            c.execute(
                "INSERT INTO evidence.event(episode_id, seq, t, src, type, arm_id, payload, blob_sha) "
                "VALUES (%s, %s, now(), %s, %s, %s, trace_redact(%s::jsonb), %s)",
                (episode_id, seq, src, type, arm_id, json.dumps(payload or {}), blob_sha),
            )
        return seq

    def record_trace_batch(self, episode_id: str, workspace_id: str, snapshot,
                           blobs: list[dict], events: list[dict]) -> list[int]:
        """Item 1: atomic episode + manifest + blobs + events in 1 SQL round trip."""
        res = self.scalar(
            "SELECT trace_record_batch(%s, %s, %s::jsonb, %s::jsonb, %s::jsonb)",
            (episode_id, workspace_id,
             json.dumps(snapshot) if snapshot is not None else None,
             json.dumps(blobs), json.dumps(events)),
        )
        return list(res or [])

    def get_events(self, episode_id: str):
        with self.conn() as c:
            return c.execute(
                "SELECT seq, src, type, arm_id, payload, blob_sha FROM evidence.event "
                "WHERE episode_id=%s ORDER BY seq", (episode_id,),
            ).fetchall()

    # -- manifests / captures ------------------------------------------- #
    def write_manifest(self, episode_id: str, snapshot: dict) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.manifest(episode_id, snapshot) VALUES (%s, trace_redact(%s::jsonb)) "
                "ON CONFLICT (episode_id) DO NOTHING",
                (episode_id, json.dumps(snapshot)),
            )

    def read_manifest(self, episode_id: str):
        with self.conn() as c:
            row = c.execute("SELECT snapshot FROM evidence.manifest WHERE episode_id=%s",
                            (episode_id,)).fetchone()
        return row["snapshot"] if row else None

    def episode_exists(self, episode_id: str) -> bool:
        with self.conn() as c:
            return c.execute("SELECT 1 FROM evidence.episode WHERE episode_id=%s",
                             (episode_id,)).fetchone() is not None

    def list_episodes(self):
        with self.conn() as c:
            return [r["episode_id"] for r in c.execute(
                "SELECT episode_id FROM evidence.episode ORDER BY episode_id").fetchall()]

    def record_capture(self, capture_id: str, workspace_id: str, meta: dict, frame_bytes: bytes,
                       arm_id=None, kind: str = "calibration", image_size=None) -> str:
        """Item 6: atomic episode + blob + still + manifest in 1 SQL round trip."""
        w, h = (image_size or [None, None])[:2]
        return self.scalar(
            "SELECT capture_record(%s, %s, %s::jsonb, %s, %s, %s, %s, %s)",
            (capture_id, workspace_id, json.dumps(meta or {}), frame_bytes, arm_id, kind, w, h),
        )

    def list_captures(self, limit: int = 20):
        with self.conn() as c:
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
        with self.conn() as c:
            row = c.execute(
                "SELECT s.sha256, b.bytes FROM evidence.still s JOIN evidence.blob b ON b.sha256=s.sha256 "
                "WHERE s.episode_id=%s AND s.purpose='witness' ORDER BY s.id DESC LIMIT 1",
                (capture_id,)).fetchone()
        return self._verify_blob(row["sha256"], bytes(row["bytes"])) if row else None

    # -- plans, gateway & calibration single-call RPCs ------------------ #
    def register_plan(self, plan_id: str, episode_id: str, arm_id: str, blob_sha: str,
                      metadata=None, snapshot=None, group_id=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.plan(plan_id, episode_id, group_id, arm_id, blob_sha, metadata, snapshot) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s) ON CONFLICT (plan_id) DO NOTHING",
                (plan_id, episode_id, group_id, arm_id, blob_sha,
                 json.dumps(metadata or {}), json.dumps(snapshot or {})),
            )

    def plan_one(self, workspace_id: str, pose: dict, target, options: dict, arm_id: str = "b601") -> dict:
        return self.scalar(
            "SELECT plan_one(%s, %s::jsonb, %s::jsonb, %s::jsonb, %s)",
            (workspace_id, json.dumps(pose), json.dumps(target), json.dumps(options), arm_id),
        )

    def move_one(self, workspace_id: str, pose: dict, targets: list, options: dict,
                 arm_id: str = "b601", plan_id: str | None = None) -> dict:
        """Item 5: returns {plan_id, summary, payload} in 1 SQL round trip."""
        return self.scalar(
            "SELECT move_one(%s, %s::jsonb, %s::jsonb, %s::jsonb, %s, %s)",
            (workspace_id, json.dumps(pose), json.dumps(targets), json.dumps(options),
             arm_id, plan_id or None),
        )

    def fk_one(self, q, arm_id: str = "b601") -> dict:
        """Item 5: single-call FK (tip_m + joint_points)."""
        return self.scalar("SELECT fk_one(%s::float8[], %s)", ([float(v) for v in q], arm_id))

    def calibrate_fit_one(self, observations: list, K: list, dist: list, options: dict,
                          arm_id: str = "b601") -> dict:
        """Item 5: single-call hand-eye calibration fit."""
        return self.scalar(
            "SELECT calibrate_fit_one(%s::jsonb, %s::jsonb, %s::jsonb, %s::jsonb, %s)",
            (json.dumps(observations), json.dumps(K), json.dumps(dist), json.dumps(options), arm_id),
        )

    def plan_fetch(self, plan_id: str, max_age_s: float | None = None):
        """Item 5: single-call plan blob + created_at + age/expiry check."""
        with self.conn() as c:
            row = c.execute(
                "SELECT p.blob_sha, f.bytes, f.created_at, f.age_s, f.expired "
                "FROM plan_fetch(%s, %s) f "
                "JOIN evidence.plan p ON p.plan_id = %s",
                (plan_id, max_age_s, plan_id),
            ).fetchone()
        if row is None:
            return None
        row["bytes"] = self._verify_blob(row["blob_sha"], bytes(row["bytes"]))
        return row

    def record_telemetry(self, exec_id: str, plan_id: str, arm_id: str, data: bytes,
                         kp_key: str, summary=None) -> str:
        """Item 5: atomic blob + telemetry row in 1 SQL call."""
        return self.scalar(
            "SELECT telemetry_record(%s, %s, %s, %s, %s, %s::jsonb)",
            (exec_id, plan_id, arm_id, data, kp_key, json.dumps(summary or {})),
        )

    def read_robot_urdf(self) -> str | None:
        return self.scalar("SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1")

    def read_plan_blob(self, plan_id: str):
        with self.conn() as c:
            row = c.execute(
                "SELECT p.blob_sha, b.bytes FROM evidence.plan p JOIN evidence.blob b ON b.sha256=p.blob_sha "
                "WHERE p.plan_id=%s", (plan_id,)).fetchone()
        return self._verify_blob(row["blob_sha"], bytes(row["bytes"])) if row else None

    def read_plan_created_at(self, plan_id: str):
        return self.scalar("SELECT created_at FROM evidence.plan WHERE plan_id=%s", (plan_id,))

    def count_plans(self) -> int:
        return int(self.scalar("SELECT count(*) FROM evidence.plan") or 0)

    def read_telemetry_doc(self, exec_id: str):
        with self.conn() as c:
            row = c.execute(
                "SELECT t.blob_sha, b.bytes FROM evidence.telemetry t JOIN evidence.blob b ON b.sha256=t.blob_sha "
                "WHERE t.exec_id=%s", (exec_id,)).fetchone()
        if not row:
            return None
        return json.loads(self._verify_blob(row["blob_sha"], bytes(row["bytes"])))

    # -- media fragments & workflow extraction -------------------------- #
    def add_still(self, episode_id: str, sha: str, purpose: str = "hold", arm_id=None,
                  t=None, width=None, height=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.still(episode_id, sha256, arm_id, purpose, t, width, height) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s)",
                (episode_id, sha, arm_id, purpose, t, width, height),
            )

    def add_fragment(self, episode_id: str, sha: str, seq: int, t_start: float, t_end: float,
                     codec=None, width=None, height=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end, codec, width, height) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT (episode_id, seq) DO NOTHING",
                (episode_id, sha, seq, t_start, t_end, codec, width, height),
            )

    def ingest_execution_segments(self, exec_id: str, blobs: list[bytes], durations: list[float],
                                  t_offset: float, codec: str = "mp4") -> dict:
        """Item 6: atomic multi-segment recording ingest + t_offset in 1 SQL call."""
        return self.scalar(
            "SELECT execution_ingest_segments(%s, %s::bytea[], %s::float8[], %s, %s)",
            (exec_id, blobs, [float(d) for d in durations], float(t_offset), codec),
        )

    def ingest_episode_segments(self, episode_id: str, workspace_id: str, arm_id: str | None,
                                blobs: list[bytes], durations: list[float], codec: str = "mp4") -> dict:
        """Item 6: atomic multi-segment episode ingest in 1 SQL call."""
        return self.scalar(
            "SELECT episode_ingest_segments(%s, %s, %s, %s::bytea[], %s::float8[], %s)",
            (episode_id, workspace_id, arm_id, blobs, [float(d) for d in durations], codec),
        )

    def extraction_plan_with_fragments(self, exec_id: str) -> tuple[dict, dict[int, dict]]:
        """Item 2: single connection fetch of extraction_plan + only needed fragment bytes."""
        with self.conn() as c:
            plan = c.execute("SELECT extraction_plan(%s) AS p", (exec_id,)).fetchone()["p"] or {}
            ep = plan.get("episode_id")
            needed = plan.get("needed_seqs") or []
            by_seq: dict[int, dict] = {}
            if ep and needed:
                rows = c.execute(
                    "SELECT f.seq, f.t_start, f.t_end, f.codec, b.bytes "
                    "FROM evidence.media_fragment f JOIN evidence.blob b ON b.sha256 = f.sha256 "
                    "WHERE f.episode_id = %s AND f.seq = ANY(%s::int[]) ORDER BY f.seq",
                    (ep, [int(s) for s in needed]),
                ).fetchall()
                for r in rows:
                    by_seq[int(r["seq"])] = {
                        "seq": int(r["seq"]), "t_start": float(r["t_start"]),
                        "t_end": float(r["t_end"]), "codec": r["codec"],
                        "data": bytes(r["bytes"]),
                    }
        return plan, by_seq

    def store_execution_stills(self, exec_id: str, holds_meta: list[dict],
                               blobs: list[bytes]) -> dict:
        """Store extracted hold JPEGs directly as binary bytea[] (no base64 in HTTP/df.history)."""
        return self.scalar(
            "SELECT execution_store_stills_bin(%s, %s::jsonb, %s::bytea[])",
            (exec_id, json.dumps(holds_meta), blobs),
        ) or {"stored": 0, "holds": []}

    def detect_request_with_stills(self, exec_id: str) -> tuple[dict, dict[str, bytes]]:
        """Single-connection fetch of detect metadata + binary still bytes (no base64)."""
        with self.conn() as c:
            req = c.execute("SELECT execution_detect_request(%s) AS r", (exec_id,)).fetchone()["r"] or {}
            shas = list({h["frame"] for h in (req.get("holds") or []) if h.get("frame")})
            by_sha: dict[str, bytes] = {}
            if shas:
                rows = c.execute(
                    "SELECT sha256, bytes FROM evidence.blob WHERE sha256 = ANY(%s::text[])",
                    (shas,),
                ).fetchall()
                for r in rows:
                    by_sha[r["sha256"]] = bytes(r["bytes"])
        return req, by_sha

    def await_execution_receipt(self, iid: str, exec_id: str, wait_s: int) -> dict:
        """Item 5: await durable instance + fetch receipt in 1 SQL call."""
        return self.scalar(
            "SELECT execution_await_receipt(%s, %s, %s)", (iid, exec_id, int(wait_s))
        )

    # -- calibration & rig catalog (world / learning) ------------------- #
    def ensure_calibration(self, calib_fingerprint: str, K=None, dist=None,
                           image_size=None, calib_doc=None) -> None:
        """Upsert calibration; real K/dist/image_size replace identity/zero placeholders."""
        with self.conn() as c:
            c.execute(
                "INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size, calib_doc) "
                "VALUES (%s, %s, %s, %s, %s) ON CONFLICT (calib_fingerprint) DO UPDATE SET "
                "K = CASE WHEN %s THEN EXCLUDED.K ELSE world.calibration.K END, "
                "dist = CASE WHEN %s THEN EXCLUDED.dist ELSE world.calibration.dist END, "
                "image_size = CASE WHEN %s THEN EXCLUDED.image_size ELSE world.calibration.image_size END, "
                "calib_doc = COALESCE(EXCLUDED.calib_doc, world.calibration.calib_doc)",
                (calib_fingerprint,
                 json.dumps(K if K is not None else [[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                 json.dumps(dist if dist is not None else [0, 0, 0, 0, 0]),
                 json.dumps(image_size if image_size is not None else [0, 0]),
                 json.dumps(calib_doc) if calib_doc is not None else None,
                 K is not None, dist is not None, image_size is not None),
            )

    def upsert_rig(self, rig_fingerprint: str, workspace_id: str, calib_fingerprint: str,
                   T_base_cam, image_size, tip_offset, method="base-marker",
                   rms_px=None, rms_mm=None, witness=None, provenance=None, rig_doc=None,
                   witness_sha=None, calib_K=None, calib_dist=None,
                   calib_image_size=None, calib_doc=None) -> None:
        """Item 6: atomic rig activation + prior-rig invalidation in 1 SQL call."""
        with self.conn() as c:
            c.execute(
                "SELECT rig_activate(%s,%s,%s,%s::jsonb,%s::jsonb,%s::jsonb,%s,%s,%s,"
                "%s::jsonb,%s::jsonb,%s::jsonb,%s,%s::jsonb,%s::jsonb,%s::jsonb,%s::jsonb)",
                (rig_fingerprint, workspace_id, calib_fingerprint,
                 json.dumps(T_base_cam), json.dumps(image_size), json.dumps(tip_offset),
                 method, rms_px, rms_mm,
                 json.dumps(witness or {}), json.dumps(provenance or {}),
                 json.dumps(rig_doc) if rig_doc is not None else None, witness_sha,
                 json.dumps(calib_K) if calib_K is not None else None,
                 json.dumps(calib_dist) if calib_dist is not None else None,
                 json.dumps(calib_image_size) if calib_image_size is not None else None,
                 json.dumps(calib_doc) if calib_doc is not None else None),
            )

    def read_active_rig_doc(self, workspace_id: str):
        return self.scalar(
            "SELECT rig_doc FROM world.shared_rig WHERE workspace_id=%s AND status='active' "
            "ORDER BY fitted DESC NULLS LAST LIMIT 1", (workspace_id,))

    def read_active_rig_row(self, workspace_id: str):
        with self.conn() as c:
            return c.execute(
                "SELECT rig_doc, witness_sha FROM world.shared_rig "
                "WHERE workspace_id=%s AND status='active' "
                "ORDER BY fitted DESC NULLS LAST LIMIT 1", (workspace_id,)).fetchone()

    def read_latest_calib_doc(self):
        with self.conn() as c:
            return c.execute(
                "SELECT calib_fingerprint, calib_doc FROM world.calibration "
                "WHERE calib_doc IS NOT NULL "
                "ORDER BY is_default DESC, created_at DESC LIMIT 1").fetchone()

    # -- durable app state / bootstrap / health ------------------------- #
    def app_bootstrap(self, workspace_id: str, arm_id: str = "b601") -> dict:
        """Item 6: single-query capture_server startup state load."""
        return self.scalar("SELECT app_bootstrap(%s, %s)", (workspace_id, arm_id)) or {}

    def health_summary(self, workspace_id: str, arm_id: str = "b601") -> dict:
        """Item 6: single-query /health summary."""
        return self.scalar("SELECT health_summary(%s, %s)", (workspace_id, arm_id)) or {}

    def set_kv(self, key: str, value) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO substrate_kv(key, value, updated_at) VALUES (%s, %s, now()) "
                "ON CONFLICT (key) DO UPDATE SET value=EXCLUDED.value, updated_at=now()",
                (key, json.dumps(value)),
            )

    def get_kv(self, key: str, default=None):
        with self.conn() as c:
            row = c.execute("SELECT value FROM substrate_kv WHERE key=%s", (key,)).fetchone()
        return row["value"] if row else default

    def set_monitor_state(self, arm_id: str, ewma_pixel_error_mm=None, ewma_tag_rms_px=None, last_doc=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO learning.monitor_state(arm_id, ewma_pixel_error_mm, ewma_tag_rms_px, last_doc, updated_at) "
                "VALUES (%s,%s,%s,%s,now()) ON CONFLICT (arm_id) DO UPDATE SET "
                "ewma_pixel_error_mm=EXCLUDED.ewma_pixel_error_mm, ewma_tag_rms_px=EXCLUDED.ewma_tag_rms_px, "
                "last_doc=EXCLUDED.last_doc, updated_at=now()",
                (arm_id, ewma_pixel_error_mm, ewma_tag_rms_px,
                 json.dumps(last_doc) if last_doc is not None else None),
            )

    def get_monitor_state(self, arm_id: str):
        with self.conn() as c:
            return c.execute("SELECT * FROM learning.monitor_state WHERE arm_id=%s", (arm_id,)).fetchone()

    def upsert_tag(self, tag_id: str, workspace_id: str, kind: str, side_mm: float, pose_world=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO world.tag(tag_id, workspace_id, kind, side_mm, pose_world) "
                "VALUES (%s,%s,%s,%s,%s) ON CONFLICT (tag_id) DO UPDATE SET "
                "kind=EXCLUDED.kind, side_mm=EXCLUDED.side_mm, pose_world=EXCLUDED.pose_world",
                (tag_id, workspace_id, kind, side_mm,
                 json.dumps(pose_world) if pose_world is not None else None),
            )

    def upsert_sag_map(self, arm_id: str, map_doc: dict) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO learning.sag_map(arm_id, rig_fingerprint, kp_key, zeros_key, "
                "per_motor, samples, rms_mrad, validated_mm, envelope, status) "
                "VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,'active') "
                "ON CONFLICT (arm_id, rig_fingerprint, kp_key, zeros_key) WHERE status = 'active' "
                "DO UPDATE SET per_motor=EXCLUDED.per_motor, samples=EXCLUDED.samples, "
                "rms_mrad=EXCLUDED.rms_mrad, validated_mm=EXCLUDED.validated_mm, "
                "envelope=EXCLUDED.envelope, created_at=now()",
                (arm_id, map_doc.get("rig_fingerprint", ""), map_doc.get("kp_key", "default"),
                 map_doc.get("zeros_key", "none"), json.dumps(map_doc.get("per_motor", {})),
                 map_doc.get("samples", 0), map_doc.get("rms_mrad"), map_doc.get("validated_mm"),
                 json.dumps(map_doc.get("envelope", {}))),
            )

    def fit_sag_map_from_corpus(self, arm_id: str, rig_fingerprint: str,
                                kp_spec: str = "50,2:150,3:150",
                                zeros_key: str = "none", min_holds: int | None = None) -> dict:
        """Item 3: aggregate holds + Rust fit + upsert sag_map in 1 SQL round trip."""
        return self.scalar(
            "SELECT sag_fit_and_save(%s, %s, %s, %s, %s)",
            (arm_id, rig_fingerprint, kp_spec, zeros_key, min_holds),
        )

    # -- oracle / telemetry catalog ------------------------------------- #
    def add_oracle_verdict(self, plan_id: str, arm_id: str, verdict: dict, doc: dict, exec_id=None) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.oracle_verdict(plan_id, exec_id, arm_id, layer, confidence, problems, doc) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s)",
                (plan_id, exec_id, arm_id, verdict.get("layer", "none"),
                 verdict.get("confidence", "low"), json.dumps(verdict.get("problems", [])),
                 json.dumps(doc)),
            )

    def record_oracle(self, plan_id: str, arm_id: str, doc: dict,
                      rig_fingerprint=None, exec_id=None) -> int:
        """Item 6: atomic oracle verdict + accuracy fold + monitor_observe in 1 SQL call."""
        return int(self.scalar(
            "SELECT oracle_record(%s, %s, %s::jsonb, %s, %s)",
            (plan_id, arm_id, json.dumps(doc), rig_fingerprint, exec_id),
        ))

    def update_accuracy_cells(self, arm_id: str, rig_fingerprint: str, doc: dict,
                              threshold_mm: float | None = None) -> None:
        with self.conn() as c:
            row = c.execute("SELECT config_doc->'learning' AS learning FROM world.arm_config WHERE arm_id=%s",
                            (arm_id,)).fetchone()
            cfg = row["learning"] if row else None
            c.execute("SELECT accuracy_cell_update(%s, %s, %s::jsonb, %s::jsonb, %s)",
                      (arm_id, rig_fingerprint, json.dumps(doc),
                       json.dumps(cfg) if cfg is not None else None, threshold_mm))

    def read_accuracy_map(self, arm_id: str, rig_fingerprint: str) -> dict:
        with self.conn() as c:
            rows = c.execute(
                "SELECT cell_key, predicted_mm, measured_mm, n, n_good, validated_at "
                "FROM learning.accuracy_cell WHERE arm_id=%s AND rig_fingerprint=%s",
                (arm_id, rig_fingerprint)).fetchall()
        cells = {}
        for r in rows:
            cells[r["cell_key"]] = {
                "predicted_mm": r["predicted_mm"], "measured_mm": r["measured_mm"],
                "n": r["n"], "n_good": r["n_good"],
                "validated_at": r["validated_at"].isoformat() if r["validated_at"] else None,
            }
        return {"version": 1, "cells": cells}

    def add_telemetry(self, exec_id: str, plan_id: str, arm_id: str, blob_sha: str,
                      kp_key: str, summary: dict) -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO evidence.telemetry(exec_id, plan_id, arm_id, blob_sha, kp_key, summary) "
                "VALUES (%s, %s, %s, %s, %s, %s) ON CONFLICT (exec_id) DO NOTHING",
                (exec_id, plan_id, arm_id, blob_sha, kp_key, json.dumps(summary)),
            )

    def read_sag_map_doc(self, arm_id: str, rig_fingerprint: str | None = None):
        with self.conn() as c:
            if rig_fingerprint:
                row = c.execute(
                    "SELECT rig_fingerprint, kp_key, zeros_key, per_motor, samples, "
                    "rms_mrad, validated_mm, envelope FROM learning.sag_map "
                    "WHERE arm_id=%s AND rig_fingerprint=%s AND status='active' "
                    "ORDER BY created_at DESC LIMIT 1",
                    (arm_id, rig_fingerprint)).fetchone()
            else:
                row = c.execute(
                    "SELECT rig_fingerprint, kp_key, zeros_key, per_motor, samples, "
                    "rms_mrad, validated_mm, envelope FROM learning.sag_map "
                    "WHERE arm_id=%s AND status='active' ORDER BY created_at DESC LIMIT 1",
                    (arm_id,)).fetchone()
        if row is None:
            return None
        return {"rig_fingerprint": row["rig_fingerprint"], "kp_key": row["kp_key"],
                "zeros_key": row["zeros_key"], "per_motor": row["per_motor"],
                "samples": row["samples"], "rms_mrad": row["rms_mrad"],
                "validated_mm": row["validated_mm"], "envelope": row["envelope"]}

    # -- similarity (pgvector) ------------------------------------------ #
    def register_embedding_model(self, model_id: str, name: str, version: str, dim: int, kind: str = "frame") -> None:
        with self.conn() as c:
            c.execute(
                "INSERT INTO search.embedding_model(model_id, name, version, dim, kind) "
                "VALUES (%s, %s, %s, %s, %s) ON CONFLICT (model_id) DO NOTHING",
                (model_id, name, version, dim, kind),
            )

    def _validate_vector(self, embedding) -> str:
        vals = [float(x) for x in embedding]
        if len(vals) != 768:
            raise ValueError("embedding must have 768 dimensions, got %d" % len(vals))
        if not all(math.isfinite(v) for v in vals):
            raise ValueError("embedding values must be finite")
        if sum(v * v for v in vals) <= 1e-24:
            raise ValueError("embedding must be non-zero for cosine distance")
        return "[" + ",".join(repr(v) for v in vals) + "]"

    def put_frame_embedding(self, still_sha: str, model_id: str, embedding) -> None:
        vec = self._validate_vector(embedding)
        with self.conn() as c:
            c.execute(
                "INSERT INTO search.frame_embedding(still_sha, model_id, embedding) "
                "VALUES (%s, %s, %s::vector) ON CONFLICT (still_sha, model_id) DO NOTHING",
                (still_sha, model_id, vec),
            )

    def search_similar(self, query_embedding, model_id: str, k: int = 5):
        vec = self._validate_vector(query_embedding)
        with self.conn() as c:
            return c.execute(
                "SELECT still_sha, embedding <=> %s::vector AS distance "
                "FROM search.frame_embedding WHERE model_id=%s "
                "ORDER BY embedding <=> %s::vector LIMIT %s",
                (vec, model_id, vec, k),
            ).fetchall()
