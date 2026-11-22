"""Isolated embed activity end-to-end: embed_service.py (separate process) reached
from a pg_durable workflow via df.http(), vector upserted into pgvector.

Run: python tests/test_embed_activity.py
Prereq: substrate up with pg_durable.http_security=unrestricted (dev compose).
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import psycopg
from psycopg.rows import dict_row
from store import SubstrateStore
from embed import HistogramEmbedder, start_embed_workflow

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres@localhost:5433/postgres")
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# free port for the isolated activity
s = socket.socket()
s.bind(("127.0.0.1", 0))
port = s.getsockname()[1]
s.close()

proc = subprocess.Popen([sys.executable, os.path.join(HERE, "embed_service.py"),
                         "--host", "127.0.0.1", "--port", str(port)],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
try:
    # wait for health
    url = "http://127.0.0.1:%d" % port
    for _ in range(50):
        try:
            with urllib.request.urlopen(url + "/health", timeout=1) as r:
                assert json.loads(r.read())["ok"]
            break
        except Exception:
            time.sleep(0.2)
    else:
        raise RuntimeError("embed service did not start")

    store = SubstrateStore(DSN)
    data = bytes((i * 7 + 3) % 256 for i in range(4096))
    sha = store.put_blob(data, kind="still")

    # the container reaches the host service via host.docker.internal
    iid = start_embed_workflow(store, sha, "http://host.docker.internal:%d/embed" % port)
    assert iid, "no instance id"

    emb = HistogramEmbedder()
    expected = emb.embed(data)
    with psycopg.connect(DSN, row_factory=dict_row) as c:
        row = c.execute("SELECT model_id, embedding::text AS v FROM search.frame_embedding "
                        "WHERE still_sha=%s", (sha,)).fetchone()
        assert row is not None, "workflow did not upsert the vector"
        assert row["model_id"] == emb.model_id
        got = json.loads(row["v"])
        assert len(got) == emb.dim
        # pgvector stores float4: allow single-precision rounding vs the float64 reference
        assert max(abs(a - b) for a, b in zip(got, expected)) < 1e-7, "vector mismatch"
        model = c.execute("SELECT name, version, dim FROM search.embedding_model "
                          "WHERE model_id=%s", (emb.model_id,)).fetchone()
        assert model and model["dim"] == emb.dim

    # similarity over the workflow-produced vector
    hits = store.search_similar(expected, emb.model_id, k=1)
    assert hits[0]["still_sha"] == sha and hits[0]["distance"] < 1e-9

    # replay is idempotent (durable re-run must not duplicate)
    start_embed_workflow(store, sha, "http://host.docker.internal:%d/embed" % port)
    with psycopg.connect(DSN, row_factory=dict_row) as c:
        n = c.execute("SELECT count(*) AS n FROM search.frame_embedding "
                      "WHERE still_sha=%s AND model_id=%s", (sha, emb.model_id)).fetchone()["n"]
        assert n == 1, "replay duplicated the vector"

    print("ISOLATED EMBED ACTIVITY (df.http) OK")
finally:
    proc.terminate()
    proc.wait(timeout=10)