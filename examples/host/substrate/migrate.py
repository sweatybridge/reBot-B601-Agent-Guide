"""Idempotent substrate migrator & URDF seeder.

Creates `rebot_kinematics` extension, applies `migrations/00*.sql` in lexical
order, records applied checksums in `substrate_migrations`, and seeds
`world.robot_model` from `examples/host/app/cad/ReBot_Arm_RS.urdf` if empty.

Safe to re-run at any time (fresh database OR existing cluster).
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path

import psycopg

HERE = Path(__file__).resolve().parent
MIGRATIONS_DIR = HERE / "migrations"
DEFAULT_URDF = HERE.parent / "app" / "cad" / "ReBot_Arm_RS.urdf"


def migrate(dsn: str, urdf_path: Path | None = DEFAULT_URDF, force: bool = False) -> list[str]:
    applied: list[str] = []
    with psycopg.connect(dsn, autocommit=True) as conn:
        conn.execute("CREATE EXTENSION IF NOT EXISTS rebot_kinematics")
        conn.execute(
            "CREATE TABLE IF NOT EXISTS substrate_migrations ("
            "  name       text PRIMARY KEY,"
            "  sha256     text NOT NULL,"
            "  applied_at timestamptz NOT NULL DEFAULT now()"
            ")"
        )
        known = dict(conn.execute("SELECT name, sha256 FROM substrate_migrations").fetchall())

        for path in sorted(MIGRATIONS_DIR.glob("00*.sql")):
            sql = path.read_text(encoding="utf-8")
            sha = hashlib.sha256(sql.encode("utf-8")).hexdigest()
            if not force and known.get(path.name) == sha:
                continue
            with conn.transaction():
                conn.execute(sql)
                conn.execute(
                    "INSERT INTO substrate_migrations(name, sha256, applied_at) "
                    "VALUES (%s, %s, now()) "
                    "ON CONFLICT (name) DO UPDATE SET sha256 = EXCLUDED.sha256, applied_at = now()",
                    (path.name, sha),
                )
            applied.append(path.name)

        if urdf_path and urdf_path.is_file():
            urdf_text = urdf_path.read_text(encoding="utf-8")
            model_id = "rebot-arm-rs-" + hashlib.sha256(urdf_text.encode("utf-8")).hexdigest()[:12]
            with conn.transaction():
                exists = conn.execute("SELECT 1 FROM world.robot_model LIMIT 1").fetchone()
                if not exists:
                    conn.execute(
                        "INSERT INTO world.robot_model(model_id, urdf) VALUES (%s, %s) "
                        "ON CONFLICT (model_id) DO NOTHING",
                        (model_id, urdf_text),
                    )
    return applied


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dsn", default=os.environ.get("REBOT_PG_DSN", ""))
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--force", action="store_true", help="Re-apply all migrations even if unchanged")
    args = ap.parse_args()
    if not args.dsn:
        raise SystemExit("REBOT_PG_DSN or --dsn is required")
    urdf = Path(args.urdf) if args.urdf else None
    applied = migrate(args.dsn, urdf_path=urdf, force=args.force)
    print("substrate migrate OK (%d applied: %s)" % (len(applied), ", ".join(applied) or "up-to-date"))


if __name__ == "__main__":
    main()
