"""CLI mirror helper: one-shot tools (identify_cad, sweep_video, locate_target)
mirror their session evidence into the substrate via REBOT_PG_DSN.
No-op when REBOT_PG_DSN is unset; never raises into the tool (§8.4)."""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _store():
    dsn = os.environ.get("REBOT_PG_DSN", "")
    if not dsn:
        return None
    sub = str(Path(__file__).resolve().parent)
    if sub not in sys.path:
        sys.path.insert(0, sub)
    from store import SubstrateStore
    return SubstrateStore(dsn)


def mirror_session(session_id, workspace_id="default", arm_id="b601", meta=None,
                   files=None, kind="calibration"):
    """Mirror a session: episode + manifest + image blobs + other file blobs."""
    store = _store()
    if store is None:
        return
    try:
        from pg_evidence import PgEvidenceMirror
        mirror = PgEvidenceMirror(store, workspace_id=workspace_id)
        store.ensure_arm(arm_id)
        store.create_episode(session_id, workspace_id, [arm_id], kind=kind)
        if meta is not None:
            store.write_manifest(session_id, meta)
        for path in files or []:
            p = Path(path)
            if not p.is_file():
                continue
            if p.suffix.lower() in (".jpg", ".jpeg", ".png"):
                mirror.mirror_still(session_id, p.read_bytes(), purpose="witness", arm_id=arm_id)
            else:
                store.put_blob(p.read_bytes(), "other")
    except Exception as exc:  # noqa: BLE001
        print("  warning: session PG mirror failed: %s" % exc)