"""Pluggable frame embedder + pgvector population (spec §4.9 / §5.4).

The embedding MODEL is a deployment fork (§10). This module provides the interface and
a deterministic default (HistogramEmbedder) for offline testing. A production CLIP/ONNX
embedder implements `Embedder` and runs as an ISOLATED activity — native inference is
unsafe in a postmaster child (crash -> cluster reset -> durable-retry loop). It calls
`populate_frame_embedding` to write vectors.

Embeddings must be NON-ZERO (cosine distance is undefined for zero vectors); normalize
before insert. The column dimension is a deployment constant (vector(768) here).
"""
from __future__ import annotations

import math

DIM = 768


class Embedder:
    dim = DIM
    model_id = "default"
    name = "default"
    version = "1"

    def embed(self, data: bytes):
        raise NotImplementedError


class HistogramEmbedder(Embedder):
    """Deterministic 768-dim normalized positional byte histogram (offline default).
    Identical bytes -> identical vector; similar content -> similar vector. Not a
    semantic model — it exists to exercise the pipeline deterministically offline."""
    model_id = "histogram:1"
    name = "histogram"
    version = "1"

    def embed(self, data: bytes):
        v = [0.0] * DIM
        for i, b in enumerate(data):
            v[i % DIM] += b
        norm = math.sqrt(sum(x * x for x in v))
        if norm < 1e-12:
            return [1.0] + [0.0] * (DIM - 1)  # never a zero vector
        return [x / norm for x in v]


def populate_frame_embedding(store, still_sha: str, embedder: Embedder):
    """Embed a stored blob and upsert its vector. Returns the vector.

    In-process path for offline tests with the safe default embedder. Production
    native inference goes through `start_embed_workflow` (isolated activity)."""
    data = store.get_blob(still_sha)
    vec = embedder.embed(data)
    store.register_embedding_model(embedder.model_id, embedder.name, embedder.version, embedder.dim)
    store.put_frame_embedding(still_sha, embedder.model_id, vec)
    return vec


# --------------------------------------------------------------------------- #
# Isolated embed activity via pg_durable df.http() (spec §2 crash isolation).
# The inference runs in embed_service.py (a separate process); the workflow only
# checkpoints the request/response and the vector upsert. A native crash fails
# the HTTP step (durable retry/dead-letter) — it can never reset the cluster.
# --------------------------------------------------------------------------- #

def embed_workflow_sql(still_sha: str, service_url: str) -> str:
    """SQL expression building the durable graph: read blob -> POST to the
    isolated embedder -> upsert vector. Uses the functional DSL (df.seq/df.as),
    which must be evaluated inline by SQL — df.start() takes the resulting graph
    JSON, not a string of operator syntax."""
    sha = still_sha.replace("'", "''")
    url = service_url.replace("'", "''")
    step_req = (
        # encode() wraps base64 at 76 cols; strip newlines for strict decoding.
        f"df.as(df.sql($wf$SELECT replace(encode(bytes, 'base64'), chr(10), '') AS b64 "
        f"FROM evidence.blob WHERE sha256 = '{sha}'$wf$), 'req')"
    )
    # token inside a JSON string: whole-value substitution inserts the raw value
    # (pg_durable's documented multipart pattern); a bare '$req.body' whole-body
    # substitution arrives SQL-quoted and is not valid JSON.
    step_http = f"df.as(df.http('{url}', 'POST', '{{\"data_b64\": \"$req.b64\"}}'), 'resp')"
    step_model = (
        "df.sql($wf$INSERT INTO search.embedding_model(model_id, name, version, dim, kind) "
        "SELECT j->>'model_id', j->>'name', j->>'version', (j->>'dim')::int, 'frame' "
        "FROM (SELECT ($resp.body)::jsonb AS j) r WHERE ($resp.ok)::boolean "
        "ON CONFLICT (model_id) DO NOTHING$wf$)"
    )
    step_vec = (
        f"df.sql($wf$INSERT INTO search.frame_embedding(still_sha, model_id, embedding) "
        f"SELECT '{sha}', j->>'model_id', (j->'vector')::text::vector "
        "FROM (SELECT ($resp.body)::jsonb AS j) r "
        "ON CONFLICT (still_sha, model_id) DO NOTHING RETURNING still_sha, model_id$wf$)"
    )
    return f"df.seq(df.seq(df.seq({step_req}, {step_http}), {step_model}), {step_vec})"


def start_embed_workflow(store, still_sha: str, service_url: str, timeout: int = 60) -> str:
    """Run the isolated embed workflow to completion; returns the instance id.

    The start commits before awaiting (pg_durable 'caller' mode: the worker only
    sees the instance after commit). Raises if the workflow did not complete."""
    graph = embed_workflow_sql(still_sha, service_url)
    with store._conn() as c:
        iid = c.execute(f"SELECT df.start({graph}, 'embed_still') AS iid").fetchone()["iid"]
    with store._conn() as c:
        c.execute("SELECT df.await_instance(%s, %s)", (iid, timeout))
        status = c.execute("SELECT df.status(%s) AS s", (iid,)).fetchone()["s"]
        result = c.execute("SELECT df.result(%s) AS r", (iid,)).fetchone()["r"]
    if status != "completed":
        raise RuntimeError("embed workflow %s ended %s: %s" % (iid, status, result))
    return iid