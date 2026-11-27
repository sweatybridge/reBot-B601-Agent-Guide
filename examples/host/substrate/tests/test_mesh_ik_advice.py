#!/usr/bin/env python3
"""Regression tests for:
  1. Exact parry3d CAD mesh self-collision (`rebot_mesh_self_clearance`) and
     moving-link lowest-Z (`rebot_mesh_lowest_z`).
  2. Recovery-safe table surface guard (`rebot_path_safe` & `sweep.path_safe`):
     starting below clearance allows upward escape while blocking further descent
     or staying below floor.
  3. Self-verified IK refusal advice in `rebot_plan` / `rebot_plan_sequence`:
     every suggested alternative (`tool_axis`, `yaw`, or closest rounded tip)
     actually solves when resubmitted.
"""
from __future__ import annotations

import json
import os
import re
import sys

sys.path.insert(0, "/app/app")
import psycopg
import sweep

DSN = os.environ.get("REBOT_PG_DSN", "postgresql://postgres:rebot@substrate:5432/postgres")


def main() -> int:
    conn = psycopg.connect(DSN, autocommit=True)
    urdf = conn.execute("SELECT urdf FROM world.robot_model ORDER BY created_at LIMIT 1").fetchone()[0]
    cfg = json.dumps(
        conn.execute("SELECT config_doc FROM world.arm_config WHERE arm_id='b601'").fetchone()[0]
    )

    # 1. Exact mesh self-clearance & lowest-Z
    cases = [
        ("HOME", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("unpark", [0.0, 0.0, 0.14, 0.0, 0.0, 0.0]),
        ("b602e4_bind", [0.0, 0.0, 0.057, -0.55, 0.0, 0.0]),
        ("deep_fold", [0.0, 2.5, 0.5, 0.0, 0.0, 0.0]),
    ] + [(f"sweep_{i + 1}", list(p)) for i, p in enumerate(sweep.POSES)]

    metrics = {}
    for name, q in cases:
        clr, lz = conn.execute(
            "SELECT rebot_mesh_self_clearance(%s,%s::float8[],0.0::float8), "
            "rebot_mesh_lowest_z(%s,%s::float8[],0.0::float8)",
            (urdf, q, urdf, q),
        ).fetchone()
        metrics[name] = (clr, lz)
        print(f"  {name:14s}: mesh_clr={clr*1000:+7.2f} mm  mesh_low_z={lz*1000:+7.2f} mm")

    # All 8 calibration sweep poses must be collision-free and above the worktop.
    for i in range(len(sweep.POSES)):
        clr, lz = metrics[f"sweep_{i + 1}"]
        assert clr > -0.008, f"sweep_{i+1} unexpectedly colliding: {clr*1000:.2f} mm"
        assert lz > 0.0, f"sweep_{i+1} pierces worktop: {lz*1000:.2f} mm"

    # The known bench crash pose (b602e4: j3=0.057, j4=-0.55) must show real penetration,
    # and deep_fold must pierce below worktop z=0.
    assert metrics["b602e4_bind"][0] < 0.0, (
        f"b602e4_bind should show mesh penetration, got {metrics['b602e4_bind'][0]*1000:.2f} mm"
    )
    assert metrics["deep_fold"][1] < 0.0, (
        f"deep_fold should pierce below z=0, got {metrics['deep_fold'][1]*1000:.2f} mm"
    )

    # 2. Recovery-safe table guard (both Rust PG and Python sweep.path_safe)
    # Find a low pose slightly below clearance and a higher clear pose above it.
    low = [0.0, 2.15, 0.5, 0.0, 0.0, 0.0]
    lower = [0.0, 2.45, 0.5, 0.0, 0.0, 0.0]
    high = [0.0, 1.2, 0.5, 0.0, 0.0, 0.0]
    low_ok = conn.execute(
        "SELECT rebot_path_safe(%s,%s::float8[],%s::float8[],%s::jsonb,40)",
        (urdf, low, high, cfg),
    ).fetchone()[0]
    sink_ok = conn.execute(
        "SELECT rebot_path_safe(%s,%s::float8[],%s::float8[],%s::jsonb,40)",
        (urdf, low, lower, cfg),
    ).fetchone()[0]
    stay_ok = conn.execute(
        "SELECT rebot_path_safe(%s,%s::float8[],%s::float8[],%s::jsonb,40)",
        (urdf, low, low, cfg),
    ).fetchone()[0]
    print(f"  recovery_guard (Rust): low->high={low_ok}, low->lower={sink_ok}, low->low={stay_ok}")
    assert low_ok is True, "upward recovery from below floor should be allowed"
    assert sink_ok is False, "further descent below floor must be refused"
    assert stay_ok is False, "staying flat below floor must be refused"

    # Matching Python sweep.path_safe recovery check
    assert sweep.path_safe(low, high) is True, "Python sweep upward recovery failed"
    assert sweep.path_safe(low, lower) is False, "Python sweep sink check failed"
    assert sweep.path_safe(low, low) is False, "Python sweep stay-low check failed"

    # 3. Self-verified IK refusal advice
    dump = json.dumps({"ids": [1, 2, 3, 4, 5, 6, 7], "q": [0.0, 0.5, 0.8, 0.0, 0.3, 0.0, 4.9]})
    opts_base = {
        "hover": True, "hover_m": 0.0, "tip_m": 0.035, "hz": 30.0,
        "approach": 0.06, "hold_s": 0.6, "no_return": False,
        "unpark_j2": 0.0, "unpark_j3": 8.0, "unpark_s": 2.0,
        "ik_pos_tol": 0.010, "ik_angle_tol": 0.5,
        "min_z": 0.04, "max_z": 0.95, "max_reach": 0.75,
        "tool_axis": "x+", "yaw": 0.0, "kp_spec": "50,2:150,3:150",
    }

    # Case A: target beyond top-down (x+) reach but reachable in horizontal (z-)
    tgt_far_td = {"format": "rebot-target", "point": [0.58, 0.0, 0.12], "point_frame": "tip", "tip_offset_m": 0.035}
    err_a = None
    try:
        conn.execute(
            "SELECT rebot_plan(%s,%s,%s,%s,%s,%s::jsonb)",
            (urdf, dump, json.dumps(tgt_far_td), json.dumps(opts_base), "{}", cfg),
        ).fetchone()
    except psycopg.Error as exc:
        err_a = str(exc).splitlines()[0]
    print(f"  IK advice (far top-down): {err_a}")
    assert err_a and 'Verified alternative: try tool_axis="z-"' in err_a, (
        f"expected verified z- alternative, got: {err_a}"
    )
    # Resubmit with the exact suggested tool_axis="z-" -> MUST succeed!
    res_a = conn.execute(
        "SELECT rebot_plan(%s,%s,%s,%s,%s,%s::jsonb)",
        (urdf, dump, json.dumps(tgt_far_td), json.dumps(dict(opts_base, tool_axis="z-")), "{}", cfg),
    ).fetchone()[0]
    assert res_a["summary"]["waypoints"] > 0

    # Case B: target beyond horizontal reach too (inside max_reach=0.75 cylinder
    # but outside spherical arm reach) -> must give closest verified reachable tip!
    tgt_too_far = {"format": "rebot-target", "point": [0.74, 0.0, 0.35], "point_frame": "tip", "tip_offset_m": 0.035}
    err_b = None
    try:
        conn.execute(
            "SELECT rebot_plan(%s,%s,%s,%s,%s,%s::jsonb)",
            (urdf, dump, json.dumps(tgt_too_far), json.dumps(dict(opts_base, tool_axis="z-")), "{}", cfg),
        ).fetchone()
    except psycopg.Error as exc:
        err_b = str(exc).splitlines()[0]
    print(f"  IK advice (out of reach): {err_b}")
    m = re.search(
        r'Closest verified reachable tip: \[([0-9.+-]+), ([0-9.+-]+), ([0-9.+-]+)\] m .* tool_axis="([^"]+)"',
        err_b or "",
    )
    assert m, f"expected closest verified reachable tip advice, got: {err_b}"
    sug_pt = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
    sug_ax = m.group(4)
    # Resubmit with the exact suggested tip & axis -> MUST succeed!
    tgt_sug = {"format": "rebot-target", "point": sug_pt, "point_frame": "tip", "tip_offset_m": 0.035}
    res_b = conn.execute(
        "SELECT rebot_plan(%s,%s,%s,%s,%s,%s::jsonb)",
        (urdf, dump, json.dumps(tgt_sug), json.dumps(dict(opts_base, tool_axis=sug_ax)), "{}", cfg),
    ).fetchone()[0]
    assert res_b["summary"]["waypoints"] > 0

    # 4. Structured advice parsing in planner_server.parse_ik_advice
    import planner_server
    adv_a = planner_server.parse_ik_advice(err_a)
    assert adv_a == {"kind": "tool_axis", "marker": 1, "tool_axis": "z-"}, f"bad adv_a: {adv_a}"
    adv_b = planner_server.parse_ik_advice(err_b)
    assert adv_b and adv_b["kind"] == "tip" and adv_b["marker"] == 1
    assert adv_b["tip"] == sug_pt and adv_b["tool_axis"] == sug_ax
    adv_m2 = planner_server.parse_ik_advice(
        'IK failed at [0.740 0.000 0.385] m (marker 2): no solution. '
        'Closest verified reachable tip: [0.666, 0.000, 0.350] m (74 mm away, tool_axis="z-").'
    )
    assert adv_m2 == {
        "kind": "tip", "marker": 2, "tip": [0.666, 0.0, 0.35],
        "distance_mm": 74.0, "tool_axis": "z-",
    }, f"bad adv_m2: {adv_m2}"

    print("MESH COLLISION + RECOVERY GUARD + IK ADVICE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
