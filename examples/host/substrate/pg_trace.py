"""PG-backed trace recorder — the cutover from the file-based TraceRecorder (spec §8).

Same async interface as app/trace.py:TraceRecorder, but events/blobs/manifests live in
the substrate (SubstrateStore) instead of state/traces/. Retention is permanent (prune
is a no-op). Reads go through read_events/read_manifest, so handle_trace_read does not
touch the filesystem. PG I/O is offloaded to a thread so it never blocks the event loop;
a PG failure costs trace evidence, never a chat turn (§8.4) — the caller wraps the call.

    recorder = PgTraceRecorder(SubstrateStore(dsn), workspace_id="default",
                               snapshot=lambda sid: trace_manifest(app, sid))
    seq = await recorder.event(sid, "page", "user_message", text="hello")
"""
from __future__ import annotations

import asyncio
import base64
import binascii
import json
import re
from pathlib import Path

DATA_URL = re.compile(r"^data:image/[a-zA-Z0-9.+-]+;base64,")
BLOB_REF = re.compile(r'"\$blob":\s*"([0-9a-f]{64})"')
MAX_BATCH = 64


class PgTraceRecorder:
    def __init__(self, store, workspace_id: str = "default", snapshot=None):
        self.store = store
        self.workspace_id = workspace_id
        self._snapshot = snapshot
        self._locks: dict[str, asyncio.Lock] = {}
        store.ensure_workspace(workspace_id)

    # -- path compat (reads go through read_* methods, not the filesystem) --
    def session_dir(self, sid: str) -> Path:
        return Path(sid)

    def blob_path(self, sha: str) -> Path:
        return Path(sha + ".jpg")

    def session_ids(self) -> list:
        return self.store.list_episodes()

    # -- writing -------------------------------------------------------- #
    def blob(self, data: bytes) -> str:
        return self.store.put_blob(data, "still")

    def _extract(self, value):
        if isinstance(value, dict):
            return {k: self._extract(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._extract(v) for v in value]
        if isinstance(value, str) and DATA_URL.match(value):
            try:
                data = base64.b64decode(value.split(",", 1)[1], validate=False)
            except (binascii.Error, ValueError):
                return value
            return {"$blob": self.blob(data)}
        return value

    def manifest(self, sid: str, snapshot: dict) -> None:
        self.store.write_manifest(sid, snapshot)

    def _lock(self, sid: str) -> asyncio.Lock:
        if sid not in self._locks:
            self._locks[sid] = asyncio.Lock()
        return self._locks[sid]

    async def event(self, sid: str, src: str, type: str, **fields) -> int:
        return (await self.events(sid, [dict(fields, src=src, type=type)]))[0]

    async def events(self, sid: str, items: list) -> list:
        if len(items) > MAX_BATCH:
            raise ValueError("a trace batch is limited to %d events" % MAX_BATCH)
        async with self._lock(sid):
            return await asyncio.to_thread(self._events_sync, sid, items)

    def _events_sync(self, sid: str, items: list) -> list:
        if not self.store.episode_exists(sid):
            self.store.create_episode(sid, self.workspace_id, [], kind="chat")
            snap = self._snapshot(sid) if self._snapshot else {}
            self.manifest(sid, dict(snap or {}, session=sid))
        seqs = []
        for item in items:
            if not isinstance(item, dict) or not item.get("type"):
                raise ValueError("every trace event needs a 'type'")
            record = self._extract(item)
            m = BLOB_REF.search(json.dumps(record))
            blob_sha = m.group(1) if m else None
            # store.record_event redacts (trace_redact) and assigns seq under an
            # advisory lock; the payload keeps the $blob refs.
            seq = self.store.record_event(
                sid, record.get("src", "page"), record["type"],
                payload=record, arm_id=record.get("arm_id"), blob_sha=blob_sha)
            seqs.append(seq)
        return seqs

    # -- retention (permanent; the substrate GC reclaims only orphan blobs) --
    def prune(self) -> None:
        pass

    # -- reading -------------------------------------------------------- #
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