"""Pluggable frame embedder + pgvector population (spec §4.9 / §5.4).

Hardened against SQL injection in `embed_workflow_sql` (strict hex SHA-256 and
URL validation + `WHERE ($resp.ok)::boolean` on both SQL write steps) and faster
vectorized byte histogram in `HistogramEmbedder`.
"""
from __future__ import annotations

import math
import re

DIM = 768
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_URL_RE = re.compile(r"^https?://[^\s'\"$]+$")


class Embedder:
    dim = DIM
    model_id = "default"
    name = "default"
    version = "1"

    def embed(self, data: bytes):
        raise NotImplementedError


class HistogramEmbedder(Embedder):
    """Deterministic 768-dim normalized positional byte histogram (offline default)."""
    model_id = "histogram:1"
    name = "histogram"
    version = "1"

    def embed(self, data: bytes):
        v = [0.0] * DIM
        mv = memoryview(data)
        for offset in range(DIM):
            v[offset] = float(sum(mv[offset::DIM]))
        norm = math.sqrt(sum(x * x for x in v))
        if norm < 1e-12:
            return [1.0] + [0.0] * (DIM - 1)
        return [x / norm for x in v]


def populate_frame_embedding(store, still_sha: str, embedder: Embedder):
    data = store.get_blob(still_sha)
    vec = embedder.embed(data)
    store.register_embedding_model(embedder.model_id, embedder.name, embedder.version, embedder.dim)
    store.put_frame_embedding(still_sha, embedder.model_id, vec)
    return vec


def embed_workflow_sql(still_sha: str, service_url: str) -> str:
    """SQL expression building the durable graph: read blob -> POST to embedder -> upsert vector."""
    if not isinstance(still_sha, str) or not _SHA256_RE.match(still_sha):
        raise ValueError("still_sha must be a 64-char lowercase hex sha256")
    if not isinstance(service_url, str) or not _URL_RE.match(service_url):
        raise ValueError("service_url must be a valid http(s) URL without quotes or '$'")
    sha = still_sha
    url = service_url
    step_req = (
        f"df.as(df.sql($wf$SELECT replace(encode(bytes, 'base64'), chr(10), '') AS b64 "
        f"FROM evidence.blob WHERE sha256 = '{sha}'$wf$), 'req')"
    )
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
        "FROM (SELECT ($resp.body)::jsonb AS j) r WHERE ($resp.ok)::boolean "
        "ON CONFLICT (still_sha, model_id) DO NOTHING RETURNING still_sha, model_id$wf$)"
    )
    return f"df.seq(df.seq(df.seq({step_req}, {step_http}), {step_model}), {step_vec})"


def start_embed_workflow(store, still_sha: str, service_url: str, timeout: int = 60) -> str:
    graph = embed_workflow_sql(still_sha, service_url)
    with store.conn() as c:
        iid = c.execute(f"SELECT df.start({graph}, 'embed_still') AS iid").fetchone()["iid"]
    with store.conn() as c:
        c.execute("SELECT df.await_instance(%s, %s)", (iid, timeout))
        status = c.execute("SELECT df.status(%s) AS s", (iid,)).fetchone()["s"]
        result = c.execute("SELECT df.result(%s) AS r", (iid,)).fetchone()["r"]
    if status != "completed":
        raise RuntimeError("embed workflow %s ended %s: %s" % (iid, status, result))
    return iid
