#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Append-only session traces for the /live chat sidebar (spec Phase 1).

One directory per conversation under `state/traces/`, one JSONL line per event,
and content-addressed frames under `blobs/`. The log is evidence: it is never
rewritten, and context compaction on the page is recorded as an event, not
applied here. Everything a replay needs is either in the log or referenced by
sha256, so a later refit of the live rig cannot change what a session saw.

    recorder = TraceRecorder(state / "traces")
    seq = await recorder.event(sid, "page", "user_message", text="hello")

Privacy (§8.2): keys matching token|authorization|api_key are dropped at write
time, `llm_url` is reduced to its host, and data-URL images become blob refs.
Nothing here may raise into a chat or arm path -- the caller wraps every call.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import hashlib
import json
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlparse

# Same scheme as capture_server.CAPTURE_ID / new_capture_id(): one id format
# serves captures, sessions and plans. Kept as a copy because importing
# capture_server from here would be circular (it imports this module).
SESSION_ID = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{6}$")
SECRET_KEY = re.compile(r"token|authorization|api_key", re.IGNORECASE)
# Usage counters are not secrets: "total_tokens" contains "token" but is a
# count, and model_turn must keep its usage block intact (§4.2).
TOKEN_COUNT = re.compile(r"(?:prompt_|completion_|total_|max_|context_)?tokens",
                         re.IGNORECASE)
DATA_URL = re.compile(r"^data:image/[a-zA-Z0-9.+-]+;base64,")
BLOB_REF = re.compile(r'"\$blob":\s*"([0-9a-f]{64})"')
# A batch is one page flush; the cap keeps a runaway page from filling the disk
# in a single request (the handler also caps the body at 8 MiB).
MAX_BATCH = 64


def _stamp() -> str:
    now = datetime.now(timezone.utc)
    return now.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (now.microsecond // 1000)


def _redact(value):
    """Drop secret keys and reduce llm_url to its host, recursively.

    Substring match on purpose: `planner_token`, `X-Rebot-Token` lowercased and
    `api_key` all contain one of the three stems, and a nested config object
    must not leak a key the top-level redaction never sees.
    """
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            if SECRET_KEY.search(str(key)) and not TOKEN_COUNT.fullmatch(str(key)):
                continue
            if str(key).lower() == "llm_url":
                out[key] = urlparse(str(item or "")).hostname or ""
            else:
                out[key] = _redact(item)
        return out
    if isinstance(value, list):
        return [_redact(item) for item in value]
    return value


class TraceRecorder:
    """The only writer of events.jsonl; blobs are write-once by sha256."""

    def __init__(self, root: Path, retention_days: int = 30, snapshot=None):
        self.root = Path(root)
        self.retention_days = int(retention_days)
        # Optional callable(sid) -> dict: the environment snapshot the server
        # wants on first sight of a session. Without one a minimal stamp is
        # written, so a manifest always exists even for direct recorder use.
        self._snapshot = snapshot
        self._locks: dict[str, asyncio.Lock] = {}
        self._seqs: dict[str, int] = {}

    # -- paths ----------------------------------------------------------

    def session_dir(self, sid: str) -> Path:
        if not SESSION_ID.match(sid or ""):
            raise ValueError("not a session id: %r" % (sid,))
        return self.root / sid

    def session_ids(self) -> list:
        if not self.root.is_dir():
            return []
        return sorted(p.name for p in self.root.iterdir()
                      if p.is_dir() and SESSION_ID.match(p.name))

    def blob_path(self, sha: str) -> Path:
        return self.root / "blobs" / (sha + ".jpg")

    # -- writing --------------------------------------------------------

    def blob(self, data: bytes) -> str:
        """Store bytes once under their sha256; returns the hex digest.

        The bytes are kept exactly as delivered (the page's quality-0.82 JPEG)
        so a replay feeds the model bit-identical images. The `.jpg` suffix is
        the layout's convention; the content is whatever was delivered.
        """
        sha = hashlib.sha256(data).hexdigest()
        path = self.blob_path(sha)
        if path.is_file():
            return sha
        path.parent.mkdir(parents=True, exist_ok=True)
        # Write-then-rename: a reader never sees a half-written frame, and two
        # concurrent writers of the same frame cannot corrupt each other.
        tmp = path.with_name(sha + ".part-%d" % os.getpid())
        with open(tmp, "wb") as handle:
            handle.write(data)
        os.replace(tmp, path)
        return sha

    def _extract(self, value):
        """Replace data-URL images with {"$blob": sha} anywhere in a field."""
        if isinstance(value, dict):
            return {key: self._extract(item) for key, item in value.items()}
        if isinstance(value, list):
            return [self._extract(item) for item in value]
        if isinstance(value, str) and DATA_URL.match(value):
            try:
                data = base64.b64decode(value.split(",", 1)[1], validate=False)
            except (binascii.Error, ValueError):
                return value
            return {"$blob": self.blob(data)}
        return value

    def manifest(self, sid: str, snapshot: dict) -> None:
        """Write the environment snapshot once; a later call cannot change it.

        The live rig and calib files may be refitted after this session, so the
        snapshot (and the file copies the server puts beside it) is frozen at
        first sight -- that is what makes an old trace replayable.
        """
        directory = self.session_dir(sid)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "manifest.json"
        if path.exists():
            return
        payload = json.dumps(_redact(snapshot), ensure_ascii=False, indent=2) + "\n"
        try:
            fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            return
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(payload)

    def _lock(self, sid: str) -> asyncio.Lock:
        lock = self._locks.get(sid)
        if lock is None:
            lock = asyncio.Lock()
            self._locks[sid] = lock
        return lock

    def _next_seq(self, sid: str, directory: Path) -> int:
        """Continue the seq across a restart: count what is already on disk."""
        if sid not in self._seqs:
            path = directory / "events.jsonl"
            count = 0
            if path.is_file():
                with open(path, "rb") as handle:
                    count = sum(1 for _ in handle)
            self._seqs[sid] = count
        self._seqs[sid] += 1
        return self._seqs[sid]

    def _append(self, directory: Path, record: dict) -> None:
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        data = line.encode("utf-8")
        fd = os.open(str(directory / "events.jsonl"),
                     os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        try:
            # One os.write per line: O_APPEND makes concurrent handlers
            # interleave whole lines, never halves of two events.
            written = os.write(fd, data)
            if written != len(data):
                raise OSError("short append to events.jsonl (%d of %d)" % (written, len(data)))
        finally:
            os.close(fd)

    async def event(self, sid: str, src: str, type: str, **fields) -> int:
        """Append one event; returns its server-assigned monotonic seq."""
        return (await self.events(sid, [dict(fields, src=src, type=type)]))[0]

    async def events(self, sid: str, items: list) -> list:
        """Append a batch in order under one lock; returns the seqs.

        Page events carry their own `pseq` so offline merge can order them even
        when a batch arrives late: sort by (t, src, pseq).
        """
        directory = self.session_dir(sid)
        if len(items) > MAX_BATCH:
            raise ValueError("a trace batch is limited to %d events" % MAX_BATCH)
        async with self._lock(sid):
            directory.mkdir(parents=True, exist_ok=True)
            if not (directory / "manifest.json").exists():
                snapshot = self._snapshot(sid) if self._snapshot else {}
                self.manifest(sid, dict(snapshot or {}, session=sid, created=_stamp()))
            seqs = []
            for item in items:
                if not isinstance(item, dict) or not item.get("type"):
                    raise ValueError("every trace event needs a 'type'")
                record = _redact(item)
                record = self._extract(record)
                seq = self._next_seq(sid, directory)
                record["seq"] = seq
                record.setdefault("src", "page")
                record.setdefault("t", _stamp())
                self._append(directory, record)
                seqs.append(seq)
            return seqs

    # -- retention ------------------------------------------------------

    def prune(self) -> None:
        """Drop session dirs older than retention_days, then unreferenced blobs.

        `export/` is exempt: handoff packages are the one artifact meant to
        outlive the retention window, and blobs they reference stay too.
        """
        if not self.root.is_dir():
            return
        cutoff = time.time() - self.retention_days * 86400
        for sid in self.session_ids():
            directory = self.root / sid
            # Last activity, not creation: a long-running conversation must not
            # be pruned out from under its own session id's birth date.
            newest = directory.stat().st_mtime
            for path in directory.iterdir():
                try:
                    newest = max(newest, path.stat().st_mtime)
                except OSError:
                    pass
            if newest < cutoff:
                for path in sorted(directory.iterdir()):
                    if path.is_file():
                        path.unlink(missing_ok=True)
                directory.rmdir()
        referenced = set()
        for path in self.root.rglob("*"):
            if not path.is_file() or path.suffix not in (".jsonl", ".json", ".md"):
                continue
            if path.parent.name == "blobs" or "blobs" in path.parts:
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            referenced.update(BLOB_REF.findall(text))
        blobs = self.root / "blobs"
        if blobs.is_dir():
            for path in blobs.iterdir():
                if path.is_file() and path.stem.split(".")[0] not in referenced:
                    path.unlink(missing_ok=True)

    # -- reading (uniform interface with the PG-backed recorder) -------- #
    async def read_events(self, sid: str) -> list:
        path = self.session_dir(sid) / "events.jsonl"
        if not path.is_file():
            raise FileNotFoundError(sid)
        events = []
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # a torn line must not hide the rest of the log
        return events

    async def read_manifest(self, sid: str):
        path = self.session_dir(sid) / "manifest.json"
        if not path.is_file():
            raise FileNotFoundError(sid)
        return json.loads(path.read_text(encoding="utf-8"))