#!/usr/bin/env python3
"""Extract recordings, phase-aligned storyboards, oracle hold grid, and repro metadata zip
for a batch of B601-RS executions stored in Postgres.

Run inside rebot-capture-capture-1:
    docker exec -i rebot-capture-capture-1 python - /app/work/batch_manifest.json \
        < skills/batch-validation/scripts/extract_and_bundle.py
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, "/app/app")
sys.path.insert(0, "/app/substrate")
from store import SubstrateStore  # noqa: E402


def default_json(obj):
    if isinstance(obj, datetime):
        return obj.isoformat()
    if isinstance(obj, (bytes, bytearray)):
        return f"<bytes:{len(obj)}>"
    return str(obj)


def decode_frame_sequential(frags_paths: list[tuple[float, float, str]], vt: float) -> np.ndarray | None:
    """Seek 0.5 s before target timestamp and step forward sequentially to avoid H.264 keyframe drift."""
    for t0, t1, path in frags_paths:
        if t0 - 0.15 <= vt <= t1 + 0.15:
            cap = cv2.VideoCapture(path)
            try:
                if not cap.isOpened():
                    continue
                fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
                rel = max(0.0, vt - t0)
                if rel > 0.8:
                    cap.set(cv2.CAP_PROP_POS_MSEC, max(0.0, rel - 0.5) * 1000.0)
                best = None
                idx = 0
                while True:
                    pos_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        break
                    cur_t = t0 + (pos_ms / 1000.0 if pos_ms > 0 else idx / fps)
                    dist = abs(cur_t - vt)
                    if best is None or dist < best[0]:
                        best = (dist, frame)
                    if cur_t > vt + 0.15:
                        break
                    idx += 1
                if best is not None:
                    return best[1]
            finally:
                cap.release()
    return None


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: extract_and_bundle.py <manifest.json> [--zip-only]", file=sys.stderr)
        return 2
    manifest_path = Path(sys.argv[1])
    zip_only = "--zip-only" in sys.argv[2:]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    out_dir = Path(manifest.get("out_dir") or "/app/work/export_batch")
    raw_dir = out_dir / "raw_frags"
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_dir.mkdir(parents=True, exist_ok=True)
    bundle_name = manifest.get("bundle_name") or (
        "repro-" + datetime.now(timezone.utc).strftime("%Y%m%d") + "-batch-validation.zip"
    )
    runs = manifest.get("runs") or []
    if not runs:
        print("Manifest has no 'runs' entries", file=sys.stderr)
        return 1

    store = SubstrateStore(os.environ["REBOT_PG_DSN"])

    with store.conn() as c:
        rig_row = c.execute(
            "SELECT r.*, c.k AS calib_k, c.dist AS calib_dist, c.image_size AS calib_image_size, c.calib_doc "
            "FROM world.shared_rig r JOIN world.calibration c ON c.calib_fingerprint = r.calib_fingerprint "
            "WHERE r.status = 'active' ORDER BY r.fitted DESC NULLS LAST LIMIT 1"
        ).fetchone()
        arm_cfg = c.execute("SELECT * FROM world.arm_config WHERE arm_id='b601'").fetchone()
        try:
            sag_maps = c.execute("SELECT * FROM learning.sag_model ORDER BY id DESC LIMIT 10").fetchall()
        except Exception:
            sag_maps = []

    holds_rows_img = []
    run_payloads = []

    for run in runs:
        key = run["key"]
        eid = run["exec_id"]
        slug = run.get("slug") or f"{key}-{eid}"
        ep = "exec-" + eid

        with store.conn() as c:
            ex_row = c.execute("SELECT * FROM evidence.execution WHERE exec_id=%s", (eid,)).fetchone()
            plan_row = c.execute(
                "SELECT plan_id, episode_id, group_id, arm_id, blob_sha, metadata, snapshot, created_at "
                "FROM evidence.plan WHERE plan_id=%s",
                (eid,),
            ).fetchone()
            verdict_row = c.execute("SELECT * FROM evidence.oracle_verdict WHERE exec_id=%s", (eid,)).fetchone()
            stills = c.execute(
                "SELECT id, purpose, arm_id, t, width, height, sha256, created_at "
                "FROM evidence.still WHERE episode_id=%s ORDER BY id",
                (ep,),
            ).fetchall()
            frags_meta = c.execute(
                "SELECT f.seq, f.t_start, f.t_end, f.codec, f.width, f.height, f.sha256, "
                "octet_length(b.bytes) AS size_bytes "
                "FROM evidence.media_fragment f JOIN evidence.blob b ON b.sha256 = f.sha256 "
                "WHERE f.episode_id=%s ORDER BY f.seq",
                (ep,),
            ).fetchall()
            frags_bytes = c.execute(
                "SELECT f.seq, f.t_start, f.t_end, b.bytes "
                "FROM evidence.media_fragment f JOIN evidence.blob b ON b.sha256 = f.sha256 "
                "WHERE f.episode_id=%s ORDER BY f.seq",
                (ep,),
            ).fetchall()
            detect_req = c.execute("SELECT execution_detect_request(%s) AS r", (eid,)).fetchone()["r"]

        plan_doc = json.loads(store.read_plan_blob(eid))
        tel_doc = store.read_telemetry_doc(eid)
        verdict_doc = (verdict_row or {}).get("doc") or {}
        t_off = float((ex_row or {}).get("t_offset") or 0.0)
        wps = plan_doc.get("waypoints") or []
        dur = float(wps[-1]["t"]) if wps else 0.0
        targets = ((plan_doc.get("capture") or {}).get("targets")) or []

        # Write raw MP4 fragments + generate storyboard unless --zip-only
        t_raw_dir = raw_dir / key
        t_raw_dir.mkdir(parents=True, exist_ok=True)
        storyboard_path = out_dir / f"storyboard_{key}.jpg"

        if not zip_only and frags_bytes:
            seg_paths = []
            res_summary = []
            for fb in frags_bytes:
                sp = t_raw_dir / f"seg_{fb['seq']:02d}.mp4"
                sp.write_bytes(bytes(fb["bytes"]))
                cap_v = cv2.VideoCapture(str(sp))
                w = int(cap_v.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap_v.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap_v.get(cv2.CAP_PROP_FPS)
                cap_v.release()
                res_summary.append(f"seq{fb['seq']}:{w}x{h}@{fps:.1f}fps[{fb['t_start']:.1f}-{fb['t_end']:.1f}s]")
                seg_paths.append((float(fb["t_start"]), float(fb["t_end"]), str(sp)))

            total_vid_t = float(frags_bytes[-1]["t_end"])
            if len(targets) >= 2:
                p1s, p1e = targets[0]["hold_t_start"], targets[0]["hold_t_end"]
                p2s, p2e = targets[1]["hold_t_start"], targets[1]["hold_t_end"]
                sample_ts = [
                    ("01 start", 0.5),
                    ("02 stage1", max(1.0, p1s - 2.2)),
                    ("03 enter1", max(1.5, p1s - 0.5)),
                    ("04 grasp_mid", (p1s + p1e) / 2.0),
                    ("05 grasp_end", p1e),
                    ("06 post_pick", p1e + 1.2),
                    ("07 transit_mid", (p1e + p2s) / 2.0),
                    ("08 enter2", max(p1e + 1.5, p2s - 0.6)),
                    ("09 place_start", p2s + 0.2),
                    ("10 release_end", p2e),
                    ("11 retract/ret", min(dur - 0.5, p2e + 2.0)),
                    ("12 final", min(total_vid_t - t_off - 0.3, dur + 1.5)),
                ]
            else:
                sample_ts = [(f"{i + 1:02d}", dur * i / 11.0) for i in range(12)]

            panels = []
            for lbl, pt in sample_ts:
                vt = max(0.1, min(total_vid_t - 0.2, pt + t_off))
                fr = decode_frame_sequential(seg_paths, vt)
                if fr is None:
                    fr = np.zeros((360, 640, 3), np.uint8)
                    cv2.putText(fr, "MISSING", (200, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                else:
                    fr = cv2.resize(fr, (640, 360), interpolation=cv2.INTER_AREA)
                cv2.rectangle(fr, (0, 0), (420, 34), (0, 0, 0), -1)
                cv2.putText(fr, f"{lbl}  t={pt:.1f}s (v={vt:.1f}s)", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
                panels.append(fr)
            sheet = np.vstack([np.hstack(panels[r * 4:(r + 1) * 4]) for r in range(3)])
            cv2.imwrite(str(storyboard_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])

            print(f"=== {key} ({eid}) ===")
            print(f"  axis={plan_doc.get('tool_axis')} entry={plan_doc.get('entry')} pitch={plan_doc.get('marker_pitch_deg')} dur={dur:.2f}s t_off={t_off:.2f}s")
            print(f"  frags: {', '.join(res_summary)}")
            print(f"  verdict={verdict_doc.get('verdict')} grasp={verdict_doc.get('grasp_verification')}")
            for h in verdict_doc.get("holds") or []:
                print(
                    f"    H{h['index']}: scale={h.get('grid_scale')} tip_err={h.get('tip_error_mm')}mm "
                    f"place_err={h.get('placement_mm')}mm px_err={h.get('pixel_error_px')}px/{h.get('pixel_error_mm')}mm"
                )

        # Build oracle hold panel row
        h_panels = []
        for h in (verdict_doc.get("holds") or [])[:3]:
            sha = h.get("frame")
            img = np.zeros((360, 640, 3), np.uint8)
            if sha:
                raw = store.get_blob(sha)
                dec = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                if dec is not None:
                    img = cv2.resize(dec, (640, 360), interpolation=cv2.INTER_AREA)
            else:
                cv2.putText(img, "NO FRAME IN DOC", (140, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            info = (
                f"{key} H{h['index']} [{h['t_start']:.1f}-{h['t_end']:.1f}s] "
                f"err={h.get('pixel_error_px')}px/{h.get('pixel_error_mm')}mm sc={h.get('grid_scale')}"
            )
            cv2.rectangle(img, (0, 0), (640, 32), (0, 0, 0), -1)
            cv2.putText(img, info, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 1)
            h_panels.append(img)
        while len(h_panels) < 3:
            h_panels.append(np.zeros((360, 640, 3), np.uint8))
        holds_rows_img.append(np.hstack(h_panels[:3]))

        run_payloads.append(
            (run, slug, ex_row, plan_row, plan_doc, tel_doc, verdict_doc, stills, frags_meta, detect_req, storyboard_path)
        )

    holds_sheet_path = out_dir / "oracle_holds_all.jpg"
    if holds_rows_img:
        cv2.imwrite(str(holds_sheet_path), np.vstack(holds_rows_img), [cv2.IMWRITE_JPEG_QUALITY, 90])

    # Package enriched repro zip
    zip_path = out_dir / bundle_name
    summary_index = {"generated_at": datetime.now(timezone.utc).isoformat(), "runs": []}

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        if rig_row:
            zf.writestr("env/rig_and_calibration.json", json.dumps(rig_row, indent=2, default=default_json) + "\n")
        if arm_cfg:
            zf.writestr("env/arm_config.json", json.dumps(arm_cfg, indent=2, default=default_json) + "\n")
        if sag_maps:
            zf.writestr("env/sag_maps.json", json.dumps(sag_maps, indent=2, default=default_json) + "\n")
        if holds_sheet_path.is_file():
            zf.write(holds_sheet_path, "oracle_holds_all.jpg")

        for run, slug, ex_row, plan_row, plan_doc, tel_doc, verdict_doc, stills, frags_meta, detect_req, sb_path in run_payloads:
            eid = run["exec_id"]
            prefix = f"runs/{eid}/"
            sha_to_local = {}
            for idx, st in enumerate(stills, 1):
                local_rel = f"stills/still_{st['purpose']}_{idx}.jpg"
                sha_to_local[st["sha256"]] = local_rel
                try:
                    zf.writestr(prefix + local_rel, store.get_blob(st["sha256"]))
                except Exception:
                    pass
            if isinstance(detect_req, dict):
                for h in detect_req.get("holds") or []:
                    if h.get("frame") in sha_to_local:
                        h["local_file"] = sha_to_local[h["frame"]]

            wps = plan_doc.get("waypoints") or []
            plan_meta = {k: v for k, v in plan_doc.items() if k != "waypoints"}
            plan_meta["waypoint_count"] = len(wps)
            plan_meta["duration_s"] = wps[-1]["t"] if wps else 0.0
            tel_summary = {k: v for k, v in (tel_doc or {}).items() if k != "samples"}
            tel_summary["sample_count"] = len((tel_doc or {}).get("samples") or [])

            meta = {
                "key": run["key"],
                "exec_id": eid,
                "recording_filename": slug + ".mp4",
                "strategy": run.get("strategy", ""),
                "grade": run.get("grade", ""),
                "plan_record": plan_row,
                "plan_metadata": plan_meta,
                "execution": ex_row,
                "telemetry_summary": tel_summary,
                "stills": stills,
                "recording_fragments": frags_meta,
            }
            summary_index["runs"].append({
                "key": run["key"],
                "exec_id": eid,
                "recording_filename": slug + ".mp4",
                "strategy": run.get("strategy", ""),
                "grade": run.get("grade", ""),
                "tool_axis": plan_doc.get("tool_axis"),
                "entry": plan_doc.get("entry"),
                "marker_pitch_deg": plan_doc.get("marker_pitch_deg"),
                "duration_s": plan_meta["duration_s"],
                "waypoint_count": plan_meta["waypoint_count"],
                "verdict": verdict_doc.get("verdict"),
                "grasp_verification": verdict_doc.get("grasp_verification"),
            })

            zf.writestr(prefix + "metadata.json", json.dumps(meta, indent=2, default=default_json) + "\n")
            zf.writestr(prefix + "execution.json", json.dumps(ex_row, indent=2, default=default_json) + "\n")
            zf.writestr(prefix + "plan.json", json.dumps(plan_doc, indent=2, default=default_json) + "\n")
            zf.writestr(prefix + "telemetry.json", json.dumps(tel_doc, indent=2, default=default_json) + "\n")
            zf.writestr(prefix + "verdict.json", json.dumps(verdict_doc, indent=2, default=default_json) + "\n")
            zf.writestr(prefix + "detect_request.json", json.dumps(detect_req, indent=2, default=default_json) + "\n")
            if sb_path.is_file():
                zf.write(sb_path, prefix + "storyboard.jpg")
            after_img = Path(run["after_image"]) if run.get("after_image") else None
            if after_img and after_img.is_file():
                zf.write(after_img, prefix + "after.jpg")

        zf.writestr("summary.json", json.dumps(summary_index, indent=2, default=default_json) + "\n")

    print(f"Wrote repro bundle: {zip_path} ({zip_path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
