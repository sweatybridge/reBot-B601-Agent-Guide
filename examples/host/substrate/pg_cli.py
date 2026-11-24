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
    return SubstrateStore(dsn, pool_size=2)


def mirror_session(session_id, workspace_id=None, arm_id=None, meta=None,
                   files=None, kind="calibration"):
    """Mirror a session: episode + manifest + image stills + referenced artifact events."""
    store = _store()
    if store is None:
        return
    workspace_id = workspace_id or os.environ.get("REBOT_PG_WORKSPACE", "default")
    arm_id = arm_id or os.environ.get("REBOT_PG_ARM", "b601")
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
                sha = store.put_blob(p.read_bytes(), "other")
                # Reference the blob from an event so refcount >= 1 protects it from orphan GC.
                store.record_event(session_id, "server", "cli_artifact",
                                   payload={"name": p.name}, arm_id=arm_id, blob_sha=sha)
    except Exception as exc:  # noqa: BLE001
        print("  warning: session PG mirror failed: %s" % exc)
    finally:
        store.close()
