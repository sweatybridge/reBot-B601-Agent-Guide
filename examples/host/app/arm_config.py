"""Loader for `world.arm_config` — the canonical per-arm physical-model config.

In PG mode (`REBOT_PG_DSN` set) the table is the system of record: `load_config()`
reads it and applies it to `rebot_ik` (the single Python source for the physical
model). In file mode it returns `rebot_ik.default_config()` unchanged. The seed is
bit-identical to the former hardcoded constants, so loading is a no-op until the
table is edited; `tests/arm_config_parity.py` guards table == Python == Rust against
drift. The `motion` tuning group (traj/sweep) is a separate consolidation phase.
"""
from __future__ import annotations

import json
import os

import rebot_ik


def load_config(dsn=None, arm_id=None, apply=True):
    """Return the arm config doc; in PG mode read it from world.arm_config and
    (by default) apply it to rebot_ik. Falls back to the defaults if no row exists."""
    dsn = dsn if dsn is not None else os.environ.get("REBOT_PG_DSN", "")
    arm_id = arm_id or os.environ.get("REBOT_PG_ARM", "b601")
    cfg = rebot_ik.default_config()
    if dsn:
        import psycopg
        with psycopg.connect(dsn) as c:
            row = c.execute("SELECT config_doc FROM world.arm_config WHERE arm_id=%s",
                            (arm_id,)).fetchone()
        if row is not None:
            doc = row[0]
            if isinstance(doc, str):
                doc = json.loads(doc)
            cfg = doc
    if apply:
        rebot_ik.apply_config(cfg)
    return cfg