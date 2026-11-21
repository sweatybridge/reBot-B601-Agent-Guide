"""Ingest + embedding + similarity round-trip against the running substrate.
Run: python tests/test_ingest_embed.py <spool_dir>
(spool_dir holds playlist.m3u8 + seg0.m4s + seg1.m4s; create it externally).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import psycopg
from psycopg.rows import dict_row
from store import SubstrateStore
from ingest import ingest_hls
from embed import HistogramEmbedder, populate_frame_embedding

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres@localhost:5433/postgres")
spool = sys.argv[1]
ep = "ep_ingest"
store = SubstrateStore(DSN)

with psycopg.connect(DSN) as c:
    c.execute("INSERT INTO world.workspace(workspace_id,name,plane_z,min_z,hover_m) "
              "VALUES ('ws_ingest','ing',0,0.04,0.05) ON CONFLICT DO NOTHING")
    c.execute("INSERT INTO world.arm(arm_id,name,urdf_fingerprint,zeros_key,kp_key) "
              "VALUES ('arm_ingest','I','u','z','kp') ON CONFLICT DO NOTHING")

# ingest HLS fragments (safe byte-shuffle, no decode)
n = ingest_hls(store, ep, "ws_ingest", ["arm_ingest"], spool)
assert n == 2, "expected 2 fragments, got %s" % n
with psycopg.connect(DSN, row_factory=dict_row) as c:
    frags = c.execute("SELECT seq, sha256, t_start, t_end FROM evidence.media_fragment "
                      "WHERE episode_id=%s ORDER BY seq", (ep,)).fetchall()
assert len(frags) == 2
assert frags[0]["t_start"] == 0.0 and frags[0]["t_end"] == 6.0, "playlist timing wrong"
assert frags[1]["t_start"] == 6.0, "fragment 1 start wrong"

# blob bytes match the source segment (content-addressed, bit-identical)
data0 = open(os.path.join(spool, "seg0.m4s"), "rb").read()
assert store.get_blob(frags[0]["sha256"]) == data0, "fragment blob mismatch"

# embed + pgvector similarity: a query identical to seg0 must retrieve seg0 first
emb = HistogramEmbedder()
for f in frags:
    populate_frame_embedding(store, f["sha256"], emb)
hits = store.search_similar(emb.embed(data0), emb.model_id, k=2)
assert hits[0]["still_sha"] == frags[0]["sha256"], "nearest should be seg0"
assert hits[0]["distance"] < 1e-9, "identical vector distance should be ~0"

print("INGEST + EMBED + SIMILARITY OK")