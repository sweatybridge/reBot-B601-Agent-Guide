"""HLS fragment ingester (spec §5 / §11).

MediaMTX records HLS segments to a spool dir; this moves completed segments into
evidence.blob + evidence.media_fragment. It is safe byte-shuffling + hashing + INSERT
— NO decode. FFmpeg is an isolated activity, used only for retrospective re-extraction
or transcoding, never in this hot path. Content-addressed and idempotent.

Spool layout: <spool>/<episode_id>/playlist.m3u8 + segN.m4s (or .ts). The playlist's
#EXTINF durations give fragment timing; without a playlist, segments are globbed in
sorted order with zero durations.

    from store import SubstrateStore
    from ingest import ingest_hls
    n = ingest_hls(SubstrateStore(dsn), sid, workspace_id, [arm_id], spool_dir)
"""
from __future__ import annotations

import os
import re


def ingest_hls(store, episode_id: str, workspace_id: str, arm_ids, spool_dir: str, codec: str = "h264") -> int:
    store.create_episode(episode_id, workspace_id, arm_ids)
    playlist = os.path.join(spool_dir, "playlist.m3u8")
    segments = []  # (filename, duration_s)
    if os.path.exists(playlist):
        dur = None
        with open(playlist, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                m = re.match(r"#EXTINF:([0-9.]+)", line)
                if m:
                    dur = float(m.group(1))
                elif line and not line.startswith("#"):
                    segments.append((line, dur or 0.0))
                    dur = None
    else:
        for fn in sorted(os.listdir(spool_dir)):
            if fn.endswith((".m4s", ".ts")):
                segments.append((fn, 0.0))

    t = 0.0
    n = 0
    for seq, (fn, dur) in enumerate(segments):
        with open(os.path.join(spool_dir, fn), "rb") as fh:
            data = fh.read()
        sha = store.put_blob(data, kind="fragment")
        store.add_fragment(episode_id, sha, seq, t, t + dur, codec=codec)
        t += dur
        n += 1
    return n