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
    """Embed a stored blob and upsert its vector. Returns the vector."""
    data = store.get_blob(still_sha)
    vec = embedder.embed(data)
    store.register_embedding_model(embedder.model_id, embedder.name, embedder.version, embedder.dim)
    store.put_frame_embedding(still_sha, embedder.model_id, vec)
    return vec