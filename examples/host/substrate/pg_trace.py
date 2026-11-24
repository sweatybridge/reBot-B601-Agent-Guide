"""PG-backed trace recorder (spec §8) — single-round-trip batch ingest.

Replaces ~70-130 per-event/per-blob connections per batch with a single atomic
`trace_record_batch(...)` SQL call (Item 1). Never writes files to CWD.
"""
from __future__ import annotations

import asyncio
import base64
import binascii
import hashlib
import json
import re
from pathlib import Path

DATA_URL = re.compile(r"^data:image/[a-zA-Z0-9.+-]+;base64,")
BLOB_REF = re.compile(r'"\$blob":\s*"([0-9a-f]{64})"')
MAX_BATCH = 64
MAX_LOCKS = 256


class PgTraceRecorder:
    def __init__(self, store, workspace_id: str = "default", snapshot=None):
        self.store = store
        self.workspace_id = workspace_id
        self._snapshot = snapshot
        self._locks: dict[str, asyncio.Lock] = {}
        self._seen_episodes: set[str] = set()
        store.ensure_workspace(workspace_id)

    # Path compat (no filesystem writes in PG mode; callers check is_pg).
    is_pg = True

    def session_dir(self, sid: str) -> Path:
        return Path(sid)

    def blob_path(self, sha: str) -> Path:
        return Path(sha + ".jpg")

    def session_ids(self) -> list:
        return self.store.list_episodes()

    def blob(self, data: bytes) -> str:
        return self.store.put_blob(data, "still")

    def _extract(self, value, blobs_out: list[dict], seen_shas: set[str]):
        if isinstance(value, dict):
            return {k: self._extract(v, blobs_out, seen_shas) for k, v in value.items()}
        if isinstance(value, list):
            return [self._extract(v, blobs_out, seen_shas) for v in value]
        if isinstance(value, str) and DATA_URL.match(value):
            try:
                raw = base64.b64decode(value.split(",", 1)[1], validate=False)
            except (binascii.Error, ValueError):
                return value
            sha = hashlib.sha256(raw).hexdigest()
            if sha not in seen_shas:
                seen_shas.add(sha)
                blobs_out.append({
                    "sha256": sha,
                    "b64": base64.b64encode(raw).decode("ascii"),
                })
            return {"$blob": sha}
        return value

    def manifest(self, sid: str, snapshot: dict) -> None:
        self.store.write_manifest(sid, snapshot)

    def _lock(self, sid: str) -> asyncio.Lock:
        lock = self._locks.get(sid)
        if lock is None:
            if len(self._locks) >= MAX_LOCKS:
                for key in list(self._locks):
                    if not self._locks[key].locked():
                        del self._locks[key]
                        if len(self._locks) < MAX_LOCKS:
                            break
            lock = asyncio.Lock()
            self._locks[sid] = lock
        return lock

    async def event(self, sid: str, src: str, type: str, **fields) -> int:
        return (await self.events(sid, [dict(fields, src=src, type=type)]))[0]

    async def events(self, sid: str, items: list) -> list:
        if len(items) > MAX_BATCH:
            raise ValueError("a trace batch is limited to %d events" % MAX_BATCH)
        # Capture app snapshot on the event loop thread (safe against live aiohttp state).
        snap = None
        if sid not in self._seen_episodes and self._snapshot is not None:
            snap = dict(self._snapshot(sid) or {}, session=sid)
        async with self._lock(sid):
            seqs = await asyncio.to_thread(self._events_sync, sid, items, snap)
            self._seen_episodes.add(sid)
            return seqs

    def _events_sync(self, sid: str, items: list, snap: dict | None) -> list:
        blobs: list[dict] = []
        seen_shas: set[str] = set()
        events_payload: list[dict] = []
        for item in items:
            if not isinstance(item, dict) or not item.get("type"):
                raise ValueError("every trace event needs a 'type'")
            record = self._extract(item, blobs, seen_shas)
            m = BLOB_REF.search(json.dumps(record))
            blob_sha = m.group(1) if m else None
            events_payload.append({
                "src": record.get("src", "page"),
                "type": record["type"],
                "arm_id": record.get("arm_id"),
                "payload": record,
                "blob_sha": blob_sha,
            })
        return self.store.record_trace_batch(sid, self.workspace_id, snap, blobs, events_payload)

    def prune(self) -> None:
        pass

    async def read_events(self, sid: str) -> list:
        rows = await asyncio.to_thread(self.store.get_events, sid)
        out = []
        for r in rows:
            rec = dict(r["payload"] or {})
            rec.update(seq=r["seq"], src=r["src"], type=r["type"])
            if r["arm_id"]:
                rec["arm_id"] = r["arm_id"]
            out.append(rec)
        return out

    async def read_manifest(self, sid: str):
        return await asyncio.to_thread(self.store.read_manifest, sid)
