#!/usr/bin/env python3
"""Unified test runner for the reBot durable substrate and host stack.

Usage (inside the capture/tools container or any host with psycopg):
    python substrate/tests/run_all.py --fresh-db --suite sql substrate rpcs
    python substrate/tests/run_all.py --suite planner          # inside planner container
    python substrate/tests/run_all.py --suite workflow         # against live postgres (pg_durable)
    python substrate/tests/run_all.py --suite selftest         # full 682-check file-mode selftest
    python substrate/tests/run_all.py --fresh-db --suite all   # sql + substrate + rpcs + workflow + selftest
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from urllib.parse import urlparse, urlunparse

TESTS_DIR = Path(__file__).resolve().parent
SUBSTRATE_DIR = TESTS_DIR.parent
HOST_DIR = SUBSTRATE_DIR.parent

if str(SUBSTRATE_DIR) not in sys.path:
    sys.path.insert(0, str(SUBSTRATE_DIR))

SQL_TESTS = [
    "kinematics_tests.sql",
    "gravity_collision_tests.sql",
    "sag_fit_tests.sql",
    "self_collision_tests.sql",
    "ray_plane_tests.sql",
    "accuracy_cell_tests.sql",
    "coordination_tests.sql",
    "judge_protocol_tests.sql",
    "default_intrinsics_tests.sql",
    "oracle_attribute_tests.sql",
    "execute_workflow_tests.sql",
    "execute_oracle_workflow_tests.sql",
    "workflow_logic_tests.sql",
    "mechanism_tests.sql",
]

SUBSTRATE_PY_TESTS = [
    "arm_config_parity.py",
    "via_config_propagation.py",
    "fk_parity.py",
    "oracle_parity.py",
    "test_store.py",
    "test_pg_trace.py",
    "test_pg_evidence.py",
    "test_extract.py",
]

PLANNER_PY_TESTS = [
    "plan_parity.py",
    "move_parity.py",
    "calib_parity.py",
    "test_plan_one.py",
]


def _replace_db(dsn: str, dbname: str) -> str:
    p = urlparse(dsn)
    return urlunparse(p._replace(path="/" + dbname))


def _create_fresh_db(base_dsn: str, dbname: str) -> str:
    import psycopg
    import migrate

    admin_dsn = _replace_db(base_dsn, "postgres")
    with psycopg.connect(admin_dsn, autocommit=True) as c:
        c.execute(f'DROP DATABASE IF EXISTS "{dbname}" WITH (FORCE)')
        c.execute(f'CREATE DATABASE "{dbname}"')
    fresh_dsn = _replace_db(base_dsn, dbname)
    urdf = HOST_DIR / "app" / "cad" / "ReBot_Arm_RS.urdf"
    # Fresh boot + forced idempotent re-run proof.
    migrate.migrate(fresh_dsn, urdf_path=urdf, force=False)
    migrate.migrate(fresh_dsn, urdf_path=urdf, force=True)
    with psycopg.connect(fresh_dsn, autocommit=True) as c:
        c.execute(
            "INSERT INTO world.workspace(workspace_id, name, plane_z, min_z, hover_m) "
            "VALUES ('default', 'default', 0.0, 0.04, 0.05) ON CONFLICT DO NOTHING"
        )
        c.execute(
            "INSERT INTO world.arm(arm_id, name, urdf_fingerprint, zeros_key, kp_key) "
            "VALUES ('b601', 'b601', 'default', 'default', 'default') ON CONFLICT DO NOTHING"
        )
    return fresh_dsn


def _drop_fresh_db(base_dsn: str, dbname: str) -> None:
    import psycopg

    admin_dsn = _replace_db(base_dsn, "postgres")
    with psycopg.connect(admin_dsn, autocommit=True) as c:
        c.execute(f'DROP DATABASE IF EXISTS "{dbname}" WITH (FORCE)')


def _strip_psql_meta(sql: str) -> str:
    lines = [ln for ln in sql.splitlines() if not ln.lstrip().startswith("\\")]
    text = "\n".join(lines)
    # Allow SQL kinematics tests to run without copying URDF into /tmp/rebot.urdf.
    return text.replace(
        "pg_read_file('/tmp/rebot.urdf')",
        "(SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1)",
    )


def run_sql_suite(dsn: str) -> list[tuple[str, bool, float, str]]:
    import psycopg

    results = []
    for name in SQL_TESTS:
        t0 = time.monotonic()
        path = TESTS_DIR / name
        sql = _strip_psql_meta(path.read_text(encoding="utf-8-sig"))
        try:
            if name == "mechanism_tests.sql":
                # Isolate mechanism fixtures in a rolled-back transaction.
                with psycopg.connect(dsn, autocommit=False) as c:
                    c.execute(sql)
                    c.rollback()
            else:
                with psycopg.connect(dsn, autocommit=True) as c:
                    c.execute(sql)
            results.append((f"sql/{name}", True, time.monotonic() - t0, ""))
        except Exception as exc:  # noqa: BLE001
            results.append((f"sql/{name}", False, time.monotonic() - t0, str(exc)))
    return results


def _run_py_script(path: Path, env: dict[str, str], extra_args: list[str] | None = None,
                   label: str | None = None) -> tuple[str, bool, float, str]:
    t0 = time.monotonic()
    cmd = [sys.executable, str(path)] + (extra_args or [])
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    dt = time.monotonic() - t0
    tag = label or path.name
    if proc.returncode == 0:
        return (tag, True, dt, "")
    tail = (proc.stderr or proc.stdout or "").strip().splitlines()[-12:]
    return (tag, False, dt, "\n".join(tail))


def run_substrate_suite(dsn: str) -> list[tuple[str, bool, float, str]]:
    env = dict(os.environ, REBOT_PG_DSN=dsn, PYTHONPYCACHEPREFIX="/tmp/pyc")
    results = []
    for name in SUBSTRATE_PY_TESTS:
        results.append(_run_py_script(TESTS_DIR / name, env, label=f"substrate/{name}"))
    # Run test_ingest_embed.py with a synthetic HLS spool directory.
    with tempfile.TemporaryDirectory(prefix="rebot-spool-") as td:
        p = Path(td)
        (p / "seg0.m4s").write_bytes(bytes((i * 13 + 7) % 256 for i in range(4096)))
        (p / "seg1.m4s").write_bytes(bytes((i * 29 + 3) % 256 for i in range(4096)))
        (p / "playlist.m3u8").write_text(
            "#EXTM3U\n#EXTINF:6.0,\nseg0.m4s\n#EXTINF:6.0,\nseg1.m4s\n", encoding="utf-8"
        )
        results.append(
            _run_py_script(
                TESTS_DIR / "test_ingest_embed.py", env, extra_args=[td],
                label="substrate/test_ingest_embed.py",
            )
        )
    return results


def run_planner_suite(dsn: str) -> list[tuple[str, bool, float, str]]:
    env = dict(os.environ, REBOT_PG_DSN=dsn, PYTHONPYCACHEPREFIX="/tmp/pyc")
    return [_run_py_script(TESTS_DIR / n, env, label=f"planner/{n}") for n in PLANNER_PY_TESTS]


def run_rpcs_suite(dsn: str) -> list[tuple[str, bool, float, str]]:
    """Regression suite for P0/P1 fixes and Items 1-6 single-round-trip RPCs."""
    import psycopg
    from embed import embed_workflow_sql
    from ingest import ingest_hls
    from store import SubstrateStore

    t0 = time.monotonic()
    store = SubstrateStore(dsn)
    try:
        with store.conn() as c:
            urdf = c.execute("SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1").fetchone()["urdf"]
            cfg = c.execute("SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()["config_doc"]

            # Real encoder sag j2 = -0.01 must not panic
            tip = c.execute("SELECT rebot_fk_tip(%s, %s::float8[]) AS v",
                            (urdf, [0.0, -0.01, 0.3, 0.0, 0.0, 0.0])).fetchone()["v"]
            assert len(tip) == 3 and all(math.isfinite(x) for x in tip)

            # Negative samples must error cleanly
            try:
                c.execute("SELECT rebot_path_safe(%s, %s::float8[], %s::float8[], %s::jsonb, -1)",
                          (urdf, [0.0, 0.2, 0.3, 0.0, 0.0, 0.0],
                           [0.1, 0.2, 0.3, 0.0, 0.0, 0.0], json.dumps(cfg))).fetchone()
                raise AssertionError("negative samples not rejected")
            except psycopg.Error:
                c.rollback()

            # Excursion > 20 mrad refused; single-target gripper ramps smoothly
            try:
                c.execute("SELECT rebot_move_build(%s, %s::jsonb, %s::jsonb, %s::jsonb, %s::jsonb, %s::jsonb)",
                          (urdf, json.dumps({"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, -0.1, 0.3, 0.0, 0.0, 0.0, 0.0]}),
                           json.dumps([[0.1, 0.3, 0.4, 0.0, 0.0, 0.0]]),
                           json.dumps({"seconds": 2.0}), json.dumps({}), json.dumps(cfg))).fetchone()
                raise AssertionError("excursion > 20 mrad not rejected")
            except psycopg.Error as exc:
                assert "20 mrad" in str(exc)
                c.rollback()

            mv = c.execute("SELECT rebot_move_build(%s, %s::jsonb, %s::jsonb, %s::jsonb, %s::jsonb, %s::jsonb) AS v",
                           (urdf, json.dumps({"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, 0.3, 0.4, 0.0, 0.0, 0.0, 0.0]}),
                            json.dumps([[0.1, 0.3, 0.4, 0.0, 0.0, 0.0]]),
                            json.dumps({"seconds": 2.0, "gripper": 0.4}), json.dumps({}), json.dumps(cfg))).fetchone()["v"]
            wps = mv["payload"]["waypoints"]
            assert abs(wps[0]["q"][6] - 0.0) < 1e-6 and abs(wps[-1]["q"][6] - 0.4) < 1e-6

            # Dead Rust pg_externs pruned
            dead = c.execute(
                "SELECT proname FROM pg_proc WHERE proname = ANY(%s)",
                (["rebot_path_collision_free", "rebot_ease_fraction", "rebot_ease_velocity",
                  "rebot_seconds_for", "rebot_project", "rebot_unpark_pose",
                  "rebot_traj_validate", "rebot_traj_build"],),
            ).fetchall()
            assert not dead, f"dead Rust functions remain: {dead}"

        # Item 5: move_one inline payload + p_plan_id, plan_fetch, fk_one, telemetry_record
        pid = f"20261124T010203Z-{os.getpid() & 0xffffff:06x}"
        res_mv = store.move_one(
            "default", {"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, 0.3, 0.4, 0.0, 0.0, 0.0, 0.0]},
            [[0.1, 0.3, 0.4, 0.0, 0.0, 0.0]],
            {"seconds": 2.0, "hold_s": 1.0}, arm_id="b601", plan_id=pid,
        )
        assert res_mv["plan_id"] == pid and "payload" in res_mv
        pf = store.plan_fetch(pid, 600.0)
        assert pf is not None and not pf["expired"] and pf["bytes"]
        fk = store.fk_one([0.0, 0.3, 0.4, 0.0, 0.0, 0.0], arm_id="b601")
        assert len(fk["tip_m"]) == 3 and len(fk["joint_points"]) == 7

        # KP parser + select_hold_frame t_offset alignment + Item 2 extraction_plan
        with store.conn() as c:
            dry = c.execute("SELECT execution_dry_telemetry(%s, '50,2:150,3:150') AS v", (pid,)).fetchone()["v"]
            assert dry["kp"] == {"1": 50.0, "2": 150.0, "3": 150.0, "4": 50.0, "5": 50.0, "6": 50.0}
            c.execute("INSERT INTO evidence.episode(episode_id, workspace_id, kind) "
                      "VALUES ('rpc_ep', 'default', 'execution') ON CONFLICT DO NOTHING")
            tel = {"version": 2, "kp": {"2": 150}, "samples": [
                {"t": 2.0, "cmd": [0, 0.3, 0.4, 0, 0, 0, 0], "meas": [0.0, 0.3, 0.4, 0, 0, 0, 0]},
                {"t": 2.1, "cmd": [0, 0.3, 0.4, 0, 0, 0, 0], "meas": [0.5, 0.3, 0.4, 0, 0, 0, 0]},
                {"t": 2.4, "cmd": [0, 0.3, 0.4, 0, 0, 0, 0], "meas": [0.5, 0.3, 0.4, 0, 0, 0, 0]},
                {"t": 2.6, "cmd": [0, 0.3, 0.4, 0, 0, 0, 0], "meas": [0.5, 0.3, 0.4, 0, 0, 0, 0]},
                {"t": 2.9, "cmd": [0, 0.3, 0.4, 0, 0, 0, 0], "meas": [0.9, 0.3, 0.4, 0, 0, 0, 0]},
            ]}
            store.record_telemetry("rpc_exec", pid, "b601", json.dumps(tel).encode(), "50,2:150", summary={"n": 5})
            c.execute(
                "INSERT INTO evidence.execution(exec_id, plan_id, arm_id, episode_id, status, t_offset, hold_windows) "
                "VALUES ('rpc_exec', %s, 'b601', 'rpc_ep', 'played', 1.5, "
                "'[{\"index\":1,\"t_start\":2.0,\"t_end\":3.0}]'::jsonb) "
                "ON CONFLICT (exec_id) DO UPDATE SET t_offset=1.5, hold_windows=EXCLUDED.hold_windows",
                (pid,),
            )
            sel = c.execute("SELECT select_hold_frame('rpc_exec', 1, ARRAY[3.6, 4.0, 4.4]::float8[]) AS v").fetchone()["v"]
            assert sel is not None and abs(sel["t"] - 4.0) < 1e-6 and sel["score"] == 0.0
            assert c.execute("SELECT evidence.reconcile_blob_refcounts() AS v").fetchone()["v"] == 0

        # Items 1, 2, 3, 4, 6
        seqs = store.record_trace_batch(
            f"20261124T010204Z-{os.getpid() & 0xffffff:06x}", "default",
            {"version": "0.2.0", "llm": {"api_key": "secret"}},
            [{"sha256": "e" * 64, "b64": base64.b64encode(b"fakejpeg").decode()}],
            [{"type": "user_message", "text": "ping"}],
        )
        assert seqs == [1]
        ep, _frags = store.extraction_plan_with_fragments("rpc_exec")
        assert ep["windows"] and ep["windows"][0]["target_t"] is not None
        sag_r = store.fit_sag_map_from_corpus("b601", "nonexistent_rig", "50,2:150,3:150")
        assert isinstance(sag_r, dict) and "ok" in sag_r
        mon_r = store.scalar(
            "SELECT monitor_observe(%s, %s, %s::jsonb)",
            ("b601", "wf_rig", json.dumps({"holds": [{"pixel_error_mm": 4.2}], "verdict": {"layer": "none"}})),
        )
        assert isinstance(mon_r, dict) and mon_r.get("ewma_pixel_error_mm") is not None
        boot = store.app_bootstrap("default", "b601")
        assert "arm_config" in boot and "calibration" in boot
        hs = store.health_summary("default", "b601")
        assert {"captures", "trace_sessions", "accuracy_map"}.issubset(hs.keys())

        # Security guards: HLS path traversal & embed workflow SQL injection
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)
            (p / "playlist.m3u8").write_text("#EXTM3U\n#EXTINF:6.0,\n../../etc/passwd\n", encoding="utf-8")
            try:
                ingest_hls(store, "ep_bad_trav", "ws_ing", ["b601"], td)
                raise AssertionError("path traversal not blocked")
            except ValueError:
                pass
        try:
            embed_workflow_sql("not-a-sha", "http://activities:8766/embed")
            raise AssertionError("invalid sha not blocked")
        except ValueError:
            pass
        wf_sql = embed_workflow_sql("a" * 64, "http://activities:8766/embed")
        assert "WHERE ($resp.ok)::boolean" in wf_sql
        assert "still_sha" in wf_sql and "data_b64" not in wf_sql
        # Binary still storage + metadata-only detect request (zero frame_b64)
        st = store.store_execution_stills(
            "rpc_exec",
            [{"index": 1, "t": 4.0, "score": 0.0, "width": 16, "height": 16}],
            [b"\xff\xd8\xff\xd9"],
        )
        assert st["stored"] >= 0 and len(st["holds"]) == 1 and st["holds"][0]["still_sha"]
        det_req, det_blobs = store.detect_request_with_stills("rpc_exec")
        assert "frame_b64" not in json.dumps(det_req)
        for h in det_req.get("holds") or []:
            assert h["frame"] in det_blobs

        # Negative t_offset (H.264 keyframe lag) must not fail wait_fragments when [0, w1] is covered
        with store.conn() as c:
            c.execute("INSERT INTO evidence.episode(episode_id, workspace_id, kind) "
                      "VALUES ('neg_off_ep', 'default', 'execution') ON CONFLICT DO NOTHING")
            c.execute("INSERT INTO evidence.blob(sha256, kind, bytes, size_bytes, status) "
                      "VALUES ('neg_off_frag', 'fragment', %s, 1, 'verified') ON CONFLICT DO NOTHING", (b"\x00",))
            c.execute("INSERT INTO evidence.media_fragment(episode_id, sha256, seq, t_start, t_end) "
                      "VALUES ('neg_off_ep', 'neg_off_frag', 0, 0.0, 20.0) ON CONFLICT DO NOTHING")
            ws = json.dumps([{"index": 1, "t_start": 0.02, "t_end": 2.58},
                             {"index": 2, "t_start": 9.96, "t_end": 11.96}])
            assert c.execute("SELECT wait_fragments('neg_off_ep', %s::jsonb, -0.6) AS v", (ws,)).fetchone()["v"] is True
            ws_uncov = json.dumps([{"index": 1, "t_start": 0.02, "t_end": 2.58},
                                   {"index": 2, "t_start": 22.0, "t_end": 24.0}])
            assert c.execute("SELECT wait_fragments('neg_off_ep', %s::jsonb, -0.6) AS v", (ws_uncov,)).fetchone()["v"] is False

            # Completed play with no_recording=true (e.g. phone Live preview off) degrades
            # immediately to telemetry-only oracle verification instead of timing out 60 s -> dead.
            c.execute("INSERT INTO world.calibration(calib_fingerprint, K, dist, image_size) "
                      "VALUES ('norec_cal', '[[600,0,320],[0,600,240],[0,0,1]]', '[0,0,0,0,0]', '[640,480]') "
                      "ON CONFLICT DO NOTHING")
            c.execute("INSERT INTO world.shared_rig(rig_fingerprint, workspace_id, calib_fingerprint, "
                      "T_base_cam, image_size, tip_offset, method, provenance, status, rig_doc) "
                      "VALUES ('norec_rig', 'default', 'norec_cal', "
                      "'[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]', '[640,480]', '[0,0,-0.035]', "
                      "'base-marker', '{}', 'active', '{\"fingerprint\":\"norec_rig\"}') "
                      "ON CONFLICT DO NOTHING")
            c.execute("SELECT execution_prepare('norec_exec', %s, 'b601', 'norec_ep', 'default', 60)", (pid,))
            assert c.execute("SELECT execution_claim_play('norec_exec') AS v").fetchone()["v"] is True
            resp_norec = json.dumps({
                "ok": True, "claimed": True, "executed": True,
                "no_recording": True, "telemetry": dry,
            })
            c.execute("SELECT execution_after_play('norec_exec', %s::jsonb)", (resp_norec,))
            assert c.execute("SELECT execution_waiting('norec_exec') AS v").fetchone()["v"] is False
            assert c.execute("SELECT execution_ready('norec_exec') AS v").fetchone()["v"] is True
            fin = c.execute("SELECT execution_finalize('norec_exec', '{\"detections\":{}}'::jsonb) AS v").fetchone()["v"]
            assert fin.get("verdict") is not None
            rcpt = c.execute("SELECT execution_receipt('norec_exec') AS v").fetchone()["v"]
            assert rcpt["ok"] is True and rcpt["executed"] is True and rcpt["ledger"]["status"] == "verified"

        return [("rpcs/p0_and_items_1_to_6", True, time.monotonic() - t0, "")]
    except Exception as exc:  # noqa: BLE001
        return [("rpcs/p0_and_items_1_to_6", False, time.monotonic() - t0, repr(exc))]
    finally:
        store.close()


def run_workflow_suite(dsn: str) -> list[tuple[str, bool, float, str]]:
    """End-to-end pg_durable tests (must run against the DB with pg_durable workers)."""
    env = dict(
        os.environ,
        REBOT_PG_DSN=dsn,
        REBOT_WORKFLOW_DRY="1",
        REBOT_EMBED_ACTIVITY_URL=os.environ.get("REBOT_EMBED_ACTIVITY_URL", "http://activities:8766/embed"),
        PYTHONPYCACHEPREFIX="/tmp/pyc",
    )
    results = [
        _run_py_script(TESTS_DIR / "test_execute_workflow.py", env, label="workflow/test_execute_workflow.py"),
        _run_py_script(TESTS_DIR / "test_embed_activity.py", env, label="workflow/test_embed_activity.py"),
    ]
    # Also run df_workflow.sql (pg_durable + rebot_kinematics).
    import psycopg
    t0 = time.monotonic()
    try:
        with psycopg.connect(dsn, autocommit=True) as c:
            iid = c.execute(
                "SELECT df.start("
                "df.sql('SELECT to_jsonb(rebot_fk((SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1), "
                "ARRAY[0,0,0,0,0,0,0,0]::float8[], ''joint6'')) AS pose'), 'rebot_fk_demo') AS iid"
            ).fetchone()[0]
            c.execute("SELECT df.await_instance(%s, 30)", (iid,))
            status = c.execute("SELECT df.status(%s)", (iid,)).fetchone()[0]
            res = c.execute("SELECT df.result(%s)", (iid,)).fetchone()[0]
            assert status == "completed" and res and "0.1355" in res, f"df_workflow failed: {status} {res}"
        results.append(("workflow/df_workflow", True, time.monotonic() - t0, ""))
    except Exception as exc:  # noqa: BLE001
        results.append(("workflow/df_workflow", False, time.monotonic() - t0, str(exc)))
    return results


def run_selftest_suite() -> list[tuple[str, bool, float, str]]:
    clean_env = dict(
        os.environ,
        PYTHONPYCACHEPREFIX="/tmp/pyc",
        REBOT_PG_DSN="",
        REBOT_CERT="",
        REBOT_KEY="",
        REBOT_CALIB="",
        REBOT_MARKER_AT="",
        REBOT_LLM_URL="",
        REBOT_LLM_MODEL="",
    )
    return [_run_py_script(HOST_DIR / "tests" / "selftest.py", clean_env, label="host/selftest.py")]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Unified reBot test runner")
    ap.add_argument(
        "--dsn",
        default=os.environ.get("REBOT_PG_DSN", "postgresql://postgres:rebot@substrate:5432/postgres"),
        help="Postgres DSN (default: REBOT_PG_DSN or compose substrate)",
    )
    ap.add_argument(
        "--fresh-db",
        action="store_true",
        help="Create a disposable DB, run migrate.py twice, run sql/substrate/rpcs against it, then drop it",
    )
    ap.add_argument(
        "--suite",
        nargs="+",
        default=["sql", "substrate", "rpcs"],
        choices=["sql", "substrate", "rpcs", "planner", "workflow", "selftest", "all"],
        help="Which test suites to run (default: sql substrate rpcs)",
    )
    args = ap.parse_args(argv)
    suites = (
        ["sql", "substrate", "rpcs", "workflow", "selftest"]
        if "all" in args.suite
        else list(dict.fromkeys(args.suite))
    )

    fresh_name = None
    target_dsn = args.dsn
    results: list[tuple[str, bool, float, str]] = []
    try:
        if args.fresh_db and any(s in suites for s in ("sql", "substrate", "rpcs", "planner")):
            fresh_name = f"rebot_test_{os.getpid()}"
            t0 = time.monotonic()
            target_dsn = _create_fresh_db(args.dsn, fresh_name)
            results.append(("migrate/fresh_plus_idempotent", True, time.monotonic() - t0, ""))

        if "sql" in suites:
            results.extend(run_sql_suite(target_dsn))
        if "substrate" in suites:
            results.extend(run_substrate_suite(target_dsn))
        if "rpcs" in suites:
            results.extend(run_rpcs_suite(target_dsn))
        if "planner" in suites:
            results.extend(run_planner_suite(target_dsn))
        if "workflow" in suites:
            results.extend(run_workflow_suite(args.dsn))
        if "selftest" in suites:
            results.extend(run_selftest_suite())
    finally:
        if fresh_name is not None:
            try:
                _drop_fresh_db(args.dsn, fresh_name)
            except Exception as exc:  # noqa: BLE001
                print(f"warning: failed to drop {fresh_name}: {exc}", file=sys.stderr)

    passed = sum(1 for _, ok, _, _ in results if ok)
    failed = [r for r in results if not r[1]]
    print("\n=== reBot test summary ===")
    for name, ok, dt, err in results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name:<36} ({dt:.2f}s)")
        if err:
            for ln in err.splitlines():
                print(f"         {ln}")
    print(f"\n{passed}/{len(results)} passed, {len(failed)} failed")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
