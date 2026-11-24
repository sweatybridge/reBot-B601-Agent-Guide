"""HLS fragment ingester (spec §5 / §11).

Hardened against path traversal in `playlist.m3u8` (every segment path must
resolve inside `spool_dir`), uses natural numeric sorting when no playlist is
present, ensures workspace exists, and ingests segments atomically in one call.
"""
from __future__ import annotations

import os
import re
from pathlib import Path


def _nat_key(name: str):
    return [int(p) if p.isdigit() else p.lower() for p in re.split(r"(\d+)", name)]


def _safe_segment_path(root: Path, entry: str) -> Path:
    candidate = (root / entry).resolve()
    if not candidate.is_relative_to(root):
        raise ValueError("playlist segment escapes spool_dir: %r" % entry)
    return candidate


def ingest_hls(store, episode_id: str, workspace_id: str, arm_ids, spool_dir: str,
               codec: str = "h264") -> int:
    root = Path(spool_dir).resolve()
    store.ensure_workspace(workspace_id)
    store.create_episode(episode_id, workspace_id, arm_ids)
    playlist = root / "playlist.m3u8"
    segments: list[tuple[str, float]] = []
    if playlist.is_file():
        dur = None
        with playlist.open(encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                m = re.match(r"#EXTINF:([0-9.]+)", line)
                if m:
                    dur = float(m.group(1))
                elif line and not line.startswith("#"):
                    segments.append((line, dur or 0.0))
                    dur = None
    else:
        for fn in sorted(os.listdir(root), key=_nat_key):
            if fn.endswith((".m4s", ".ts", ".mp4")):
                segments.append((fn, 0.0))

    if not segments:
        return 0
    blobs: list[bytes] = []
    durations: list[float] = []
    for fn, dur in segments:
        seg_path = _safe_segment_path(root, fn)
        blobs.append(seg_path.read_bytes())
        durations.append(dur)

    arm_id = list(arm_ids)[0] if arm_ids else None
    store.ingest_episode_segments(episode_id, workspace_id, arm_id, blobs, durations, codec=codec)
    return len(blobs)
