#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Freeze one labeled task window into a replayable bundle (spec §6.2, §6.4, §6.5).

    python tests/trace_bundle.py --session <sid> [--task N] [--traces-dir DIR]
    python tests/trace_bundle.py --session <sid> --export [--case-id ID]
    python tests/trace_bundle.py --dedup [--traces-dir DIR]
    python tests/trace_bundle.py --bundle DIR --minimize

The bundle carries everything a replay needs -- the window's events, the frozen
manifest, the calib/rig *file bytes*, every referenced frame, the gateway tape,
trajectories, telemetry and oracle docs -- so a later refit of the live rig
cannot change what the replay sees. The completeness gate refuses to ship a
half-bundle: a failure without its evidence is unreproducible by construction,
and saying so is the deliverable, not a guessed reconstruction.

Export (§6.4) turns a complete bundle into an agent_cases-compatible handoff
package; dedup (§6.5) groups labels by signature and keeps the shortest window;
minimize delta-debugs geometry cases while the replay still reproduces the
failure. Nothing here touches hardware or the model.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
HOST = HERE.parent
APP = HOST / "app"
STATE = HOST / "state"
sys.path.insert(0, str(APP))
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import extrinsics  # noqa: E402
import locate_target  # noqa: E402
import oracle as oracle_mod  # noqa: E402
import rebot_ik  # noqa: E402
import trace_triage  # noqa: E402

SESSION_ID = trace_triage.SESSION_ID


def read_events(path: Path) -> list:
    return trace_triage._read_events(Path(path))


def read_labels(session_dir: Path) -> list:
    path = Path(session_dir) / "labels.jsonl"
    labels = []
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                labels.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return labels


def blob_refs(value) -> set:
    """Every {"$blob": sha} anywhere in an event, recursively."""
    found = set()
    if isinstance(value, dict):
        if set(value) == {"$blob"} and isinstance(value["$blob"], str):
            found.add(value["$blob"])
        for item in value.values():
            found |= blob_refs(item)
    elif isinstance(value, list):
        for item in value:
            found |= blob_refs(item)
    return found


def tape_entries(events) -> list:
    """The gateway_call events as a mock_arm tape, in recorded order."""
    entries = []
    for event in events:
        if event.get("type") != "gateway_call":
            continue
        entry = {"key": event.get("key"), "method": event.get("method", "POST"),
                 "path": event.get("path", ""), "request": event.get("request") or {},
                 "status": event.get("status", 200), "response": event.get("response") or {}}
        if not entry["key"]:
            # Same computation as capture_server.planner_call / mock_arm.tape_key.
            canonical = json.dumps(entry["request"], sort_keys=True, ensure_ascii=False)
            entry["key"] = hashlib.sha256(
                ("%s|%s|%s" % (entry["method"], entry["path"], canonical))
                .encode("utf-8")).hexdigest()
        entries.append(entry)
    return entries


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def _copy_if(source: Path, target: Path) -> bool:
    if source.is_file():
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        return True
    return False


def _move_arm_coverage(events, entries):
    """Every move_arm must meet a tape entry: a plan id for the successes, a
    recorded refusal for the failures. Without it the replay cannot reproduce
    what the arm side answered."""
    reasons = []
    plan_entries = [e for e in entries if e["path"] == "/plan"]
    by_id = {}
    refusals = 0
    for entry in plan_entries:
        plan_id = (entry["response"] or {}).get("id")
        if entry["status"] == 200 and plan_id:
            by_id.setdefault(plan_id, entry)
        elif entry["status"] != 200:
            refusals += 1
    results = [e for e in events if e.get("type") == "tool_result"
               and e.get("name") == "move_arm"]
    calls = sum(1 for e in events if e.get("type") == "tool_call"
                and e.get("name") == "move_arm")
    if calls and not results:
        reasons.append("move_arm calls have no recorded tool_result")
    failed = 0
    for result in results:
        payload = result.get("result") or {}
        plan_id = payload.get("plan_id")
        if payload.get("ok") and plan_id:
            if plan_id not in by_id:
                reasons.append("no tape entry for plan %s" % plan_id)
        else:
            failed += 1
    if failed > refusals:
        reasons.append("%d refused move_arm call(s) but only %d recorded refusal(s)"
                       % (failed, refusals))
    return reasons


def bundle_session(traces_dir, sid: str, task=None, state_dir=None) -> dict:
    """Freeze one task window into <sid>/bundle/. Returns dir + completeness.

    `task` is 1-based; the default is the first failing window (the collector's
    purpose), else the first window. An incomplete bundle is still written --
    with its reasons -- so the operator sees what evidence is missing; it is
    simply never exported.
    """
    traces_dir = Path(traces_dir)
    state = Path(state_dir) if state_dir else traces_dir.parent
    session_dir = traces_dir / sid
    events = read_events(session_dir / "events.jsonl")
    labels = read_labels(session_dir)
    windows = trace_triage.task_windows(events)
    reasons = []
    if not SESSION_ID.match(sid or ""):
        raise ValueError("not a session id: %r" % (sid,))
    if not windows:
        reasons.append("no task window (no user_message) in the log")
        index = 0
        window = []
        label = None
    else:
        if task is not None:
            index = int(task) - 1
            if not 0 <= index < len(windows):
                raise ValueError("task %s out of range (1..%d)" % (task, len(windows)))
        else:
            index = next((i for i, item in enumerate(labels)
                          if item.get("verdict") == "fail"), 0)
            index = min(index, len(windows) - 1)
        window = windows[index]["events"]
        label = labels[index] if index < len(labels) else None
    if label is None:
        reasons.append("no label for this window (run tests/trace_triage.py first)")

    bundle = session_dir / "bundle"
    bundle.mkdir(parents=True, exist_ok=True)
    (bundle / "events.jsonl").write_text(
        "".join(json.dumps(e, ensure_ascii=False) + "\n" for e in window),
        encoding="utf-8")
    _copy_if(session_dir / "manifest.json", bundle / "manifest.json")
    if not (bundle / "manifest.json").is_file():
        reasons.append("no manifest.json in the session dir")

    # The frozen geometry: file bytes, not fingerprints (§8.3).
    has_calib = _copy_if(session_dir / "calib.json", bundle / "calib.json")
    has_rig = _copy_if(session_dir / "rig.json", bundle / "rig.json")
    if not has_calib:
        reasons.append("no calib.json copy in the session dir")
    if not has_rig:
        reasons.append("no rig.json copy in the session dir")

    # Every referenced frame, under its sha256 name.
    refs = set()
    for event in window:
        refs |= blob_refs(event)
    stills = bundle / "stills"
    stills.mkdir(parents=True, exist_ok=True)
    for sha in sorted(refs):
        if not _copy_if(traces_dir / "blobs" / (sha + ".jpg"), stills / (sha + ".jpg")):
            reasons.append("blob %s does not resolve" % sha)

    entries = tape_entries(window)
    _write_json(bundle / "tape.json", {"entries": entries})
    reasons += _move_arm_coverage(window, entries)

    plan_ids = list((label or {}).get("plan_ids") or [])
    for event in window:
        for candidate in (event.get("plan_id"), (event.get("result") or {}).get("plan_id")
                          if isinstance(event.get("result"), dict) else None):
            if candidate and candidate not in plan_ids:
                plan_ids.append(candidate)
    oracle_docs = {}
    for plan_id in plan_ids:
        _copy_if(state / "out" / (plan_id + ".json"),
                 bundle / "trajectories" / (plan_id + ".json"))
        _copy_if(state / "telemetry" / (plan_id + ".json"),
                 bundle / "telemetry" / (plan_id + ".json"))
        if _copy_if(state / "oracle" / (plan_id + ".json"),
                    bundle / "oracle" / (plan_id + ".json")):
            oracle_docs[plan_id] = json.loads(
                (bundle / "oracle" / (plan_id + ".json")).read_text(encoding="utf-8"))
    if (label or {}).get("class") == "geometry" and not oracle_docs:
        reasons.append("a geometry failure needs at least one oracle doc")

    if label is not None:
        _write_json(bundle / "labels.json", label)
    meta = {"session": sid, "task": index + 1,
            "task_id": (label or {}).get("task_id", "%s-task-%d" % (sid, index + 1)),
            "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "complete": not reasons, "reasons": reasons}
    _write_json(bundle / "bundle.json", meta)
    return {"dir": bundle, "complete": not reasons, "reasons": reasons,
            "label": label, "events": window, "oracle_docs": oracle_docs}


# --------------------------------------------------------------------------- #
# expect{} derivation (§6.3b) -- pure, so the offline tests can drive it
# --------------------------------------------------------------------------- #

def derive_expect(label: dict, events: list, oracle_docs: dict) -> dict:
    """The triage verdict translated into the judge's expect{} vocabulary."""
    label = label or {}
    cls = label.get("class")
    kinds = {p.get("kind") for p in label.get("problems") or []}
    if cls == "geometry":
        # Not pixel boxes: the oracle recomputation with the frozen rig.
        return {"geometry_check": True, "honest": True,
                "markers_min": 0, "markers_max": 10}
    if cls == "perception":
        # The --scaffold growth flow: zero boxes for a human to annotate.
        return {"markers_min": 1, "markers_max": 3, "must_mark": ["object"],
                "plan_first": True, "honest": True,
                "final_regex": "(?i)plan|marker|done"}
    if cls == "protocol":
        expect = {"honest": True, "plan_first": True,
                  "markers_min": 0, "markers_max": 10}
        refused = any((e.get("result") or {}).get("ok") is False
                      for e in events if e.get("type") == "tool_result"
                      and e.get("name") == "move_arm")
        # Gave up after a refusal: the retry loop is the behavior under test.
        if "repeated-tool-error" in kinds or "user-abort" in kinds \
                or ("user-fail" in kinds and refused):
            expect["plan_attempts_min"] = 2
        return expect
    return {"honest": True, "markers_min": 0, "markers_max": 10}


def geometry_problems(K, dist, T_base_cam, tip_m, rig_tip_offset, oracle_docs) -> list:
    """Recompute the oracle's pixel residual with the *current* host geometry.

    Frozen evidence (the detected tip pixel, the held joints) meets the code
    under test: FK(q) + tip offset -> project -> compare against the recorded
    detection, in mm at the tip plane. A changed constant moves the residual;
    that is the regression signal, and it needs no arm and no model.
    """
    K = np.asarray(K, dtype=float)
    dist = np.asarray(dist, dtype=float).reshape(-1)
    T = np.asarray(T_base_cam, dtype=float)
    offset = np.asarray(rig_tip_offset, dtype=float)
    if np.linalg.norm(offset) <= 1e-6:
        offset = np.array([0.0, 0.0, -float(tip_m)])
    problems = []
    for plan_id, doc in sorted((oracle_docs or {}).items()):
        for hold in doc.get("holds") or []:
            q = hold.get("q_meas") or hold.get("q_cmd")
            detected = hold.get("detected_tip_px")
            if not q or not detected:
                continue
            tool = rebot_ik.forward(np.asarray(q[:6], dtype=float))
            tip = tool[:3, :3] @ offset + tool[:3, 3]
            predicted = locate_target.project(K, T, tip, dist)
            if predicted is None:
                continue
            # The detected pixel back-projected to the tip's own plane: the
            # residual in metres, independent of the recorded mm scale.
            point, _distance, _grazing = extrinsics.ray_plane(
                K, dist, T, float(detected[0]), float(detected[1]), float(tip[2]))
            error_mm = 1000.0 * float(np.linalg.norm(np.asarray(point) - tip))
            if error_mm > oracle_mod.PIXEL_ERROR_MM:
                recorded = hold.get("pixel_error_mm")
                problems.append(
                    "geometry residual reproduced: plan %s hold %s error %.1f mm%s"
                    % (plan_id, hold.get("index", "?"), error_mm,
                       " (recorded %.1f mm)" % recorded if recorded is not None else ""))
    return problems


# --------------------------------------------------------------------------- #
# Export (§6.4)
# --------------------------------------------------------------------------- #

def _first_user(events):
    for event in events:
        if event.get("type") == "user_message":
            frame = event.get("frame") or {}
            sha = frame.get("$blob") if isinstance(frame, dict) else None
            return str(event.get("text") or ""), sha
    return "", None


def _render_report(case_id, label, manifest, oracle_docs, replay_report,
                   expect, occurrences, duplicates) -> str:
    label = label or {}
    manifest = manifest or {}
    rig = manifest.get("rig") or {}
    llm = manifest.get("llm") or {}
    lines = ["# Failure case %s" % case_id, ""]
    lines.append("## Symptom")
    lines.append("")
    problems = label.get("problems") or []
    if problems:
        for problem in problems:
            lines.append("- **%s** (%s): %s" % (problem.get("kind"),
                                                problem.get("class"),
                                                problem.get("detail")))
    else:
        lines.append("- verdict %s, class %s" % (label.get("verdict"), label.get("class")))
    lines += ["", "## Attribution", "",
              "- layer: `%s`" % (label.get("layer") or "none"),
              "- class: `%s`" % (label.get("class") or "unknown"),
              "- signature: `%s`" % label.get("signature", ""),
              "- occurrences: %d%s" % (occurrences,
                                       (" (duplicates: %s)" % ", ".join(duplicates))
                                       if duplicates else ""), ""]
    lines += ["## Expected vs actual", ""]
    for plan_id, doc in sorted((oracle_docs or {}).items()):
        for hold in doc.get("holds") or []:
            if hold.get("pixel_error_mm") is None and hold.get("tip_error_mm") is None:
                continue
            lines.append("- plan `%s` hold %s: pixel error %s mm, tip error %s mm, "
                         "hold error max %s rad"
                         % (plan_id, hold.get("index", "?"),
                            hold.get("pixel_error_mm"), hold.get("tip_error_mm"),
                            hold.get("hold_error_max_rad")))
    if replay_report:
        for diff in (replay_report.get("diffs") or [])[:20]:
            lines.append("- replay %s `%s.%s`: recorded %s, replayed %s"
                         % (diff.get("class"), diff.get("tool"), diff.get("field"),
                            json.dumps(diff.get("recorded")), json.dumps(diff.get("replayed"))))
    if not any("pixel error" in line or line.startswith("- replay") for line in lines):
        lines.append("- see trace.jsonl and the oracle/ docs for the recorded numbers")
    lines += ["", "## Expect (auto-derived)", "",
              "```json", json.dumps(expect, indent=2), "```", ""]
    lines += ["## Versions", "",
              "- git commit: `%s`" % trace_triage.code_version(),
              "- live.html sha256: `%s`" % manifest.get("live_html_sha256"),
              "- model: `%s`" % llm.get("model"),
              "- rig fingerprint: `%s`" % rig.get("fingerprint"),
              "- calib fingerprint: `%s`" % manifest.get("calib_fingerprint"),
              "- capture server version: `%s`" % manifest.get("version"), ""]
    lines += ["## Reproduce", "",
              "```",
              "python tests/agent_eval.py --trace export/%s --replay" % case_id,
              "python tests/agent_eval.py --trace export/%s --mock-edge --runs 3" % case_id,
              "```", "",
              "The deterministic replay needs no model endpoint. A fix retires this",
              "case: the signature is keyed by the code version, and the replay",
              "passes once the numbers agree again.", ""]
    return "\n".join(lines)


def export_bundle(bundle_dir, export_root=None, case_id=None, expect=None,
                  replay_report=None, occurrences=1, duplicates=()) -> Path | None:
    """A complete bundle -> state/traces/export/<case-id>/. None if incomplete."""
    bundle = Path(bundle_dir)
    meta = json.loads((bundle / "bundle.json").read_text(encoding="utf-8"))
    if not meta.get("complete"):
        print("not exported: bundle incomplete (%s)" % "; ".join(meta.get("reasons") or []))
        return None
    label = json.loads((bundle / "labels.json").read_text(encoding="utf-8")) \
        if (bundle / "labels.json").is_file() else {}
    events = read_events(bundle / "events.jsonl")
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8")) \
        if (bundle / "manifest.json").is_file() else {}
    oracle_docs = {}
    if (bundle / "oracle").is_dir():
        for path in sorted((bundle / "oracle").glob("*.json")):
            oracle_docs[path.stem] = json.loads(path.read_text(encoding="utf-8"))
    user, sha = _first_user(events)
    if not sha or not (bundle / "stills" / (sha + ".jpg")).is_file():
        print("not exported: the task's first frame is missing")
        return None
    if export_root is None:
        # <traces>/<sid>/bundle -> <traces>/export; a moved bundle falls back
        # to the repo's state root.
        sid_dir = bundle.parent
        export_root = (sid_dir.parent / "export") if SESSION_ID.match(sid_dir.name or "") \
            else STATE / "traces" / "export"
    case_id = case_id or "%s-%s" % (label.get("class") or "trace",
                                    (label.get("signature") or meta["session"])[:12])
    target = Path(export_root) / case_id
    target.mkdir(parents=True, exist_ok=True)
    for name in ("stills", "trajectories", "telemetry", "oracle"):
        if (bundle / name).is_dir():
            shutil.copytree(bundle / name, target / name, dirs_exist_ok=True)
    calib = target / "calib"
    calib.mkdir(parents=True, exist_ok=True)
    for name in ("calib.json", "rig.json"):
        _copy_if(bundle / name, calib / name)
    _copy_if(bundle / "tape.json", target / "tape.json")
    _copy_if(bundle / "events.jsonl", target / "trace.jsonl")
    _copy_if(bundle / "labels.json", target / "labels.json")
    _write_json(target / "oracle.json", oracle_docs)

    expect = expect if expect is not None else derive_expect(label, events, oracle_docs)
    case = {
        "id": case_id,
        "user": user,
        "still": "stills/%s.jpg" % sha,
        "expect": expect,
        "needs_mock_edge": True,
        "tape": "tape.json",
        "oracle": "oracle.json",
        "calibration": "calib/calib.json",
        "extrinsics": "calib/rig.json",
        "plane_z": manifest.get("plane_z", 0.0),
        "tip_mm": 1000.0 * float(manifest.get("tip_m", 0.035)),
        "min_z": manifest.get("min_z", 0.10),
        "hover_m": manifest.get("hover_m", 0.10),
        "signature": label.get("signature"),
        "occurrences": occurrences,
        "duplicates": list(duplicates),
        "code_version": trace_triage.code_version(),
    }
    if label.get("class") == "perception":
        case["objects"] = {"object": [0, 0, 0, 0]}
        case["_note"] = ("Perception-suspect: replace the zero box with what the "
                         "object covers in the still, then --dry, then --runs 3.")
    _write_json(target / "case.json", case)
    (target / "report.md").write_text(
        _render_report(case_id, label, manifest, oracle_docs, replay_report,
                       expect, occurrences, duplicates), encoding="utf-8")
    return target


# --------------------------------------------------------------------------- #
# Dedup and minimization (§6.5)
# --------------------------------------------------------------------------- #

def dedup(traces_dir) -> dict:
    """Group every session's labels by signature; keep the shortest window.

    The signature already embeds the code version (§8.3), so a fix that changes
    behavior lands in a new group and the old case retires instead of silently
    re-matching. Writes dedup.json beside the sessions.
    """
    traces_dir = Path(traces_dir)
    groups = {}
    for sid in sorted(p.name for p in traces_dir.iterdir()
                      if p.is_dir() and SESSION_ID.match(p.name)):
        events = read_events(traces_dir / sid / "events.jsonl")
        windows = trace_triage.task_windows(events)
        for number, label in enumerate(read_labels(traces_dir / sid), start=1):
            window = windows[number - 1]["events"] if number <= len(windows) else []
            frames = len(set().union(*(blob_refs(e) for e in window))) if window else 0
            entry = {"task_id": label.get("task_id"), "session": sid, "task": number,
                     "class": label.get("class"), "verdict": label.get("verdict"),
                     "window_events": len(window), "frames": frames}
            groups.setdefault(label.get("signature"), []).append(entry)
    out = {}
    for signature, entries in groups.items():
        canonical = min(entries, key=lambda e: (e["window_events"], e["frames"], e["task_id"]))
        out[signature] = {
            "canonical": canonical["task_id"], "session": canonical["session"],
            "class": canonical["class"], "occurrences": len(entries),
            "duplicates": [e["task_id"] for e in entries if e is not canonical],
            "members": entries,
        }
    _write_json(traces_dir / "dedup.json", out)
    return out


def _ddmin(items, test):
    """Classic delta debugging: a 1-minimal subset that still passes `test`."""
    items = list(items)
    if not test(items):
        return items
    n = 2
    while len(items) >= 2:
        size = (len(items) + n - 1) // n
        chunks = [items[i:i + size] for i in range(0, len(items), size)]
        reduced = False
        for chunk in chunks:
            rest = [item for item in items if item not in chunk]
            if test(rest):
                items, n, reduced = rest, max(n - 1, 2), True
                break
        if not reduced:
            if n >= len(items):
                break
            n = min(n * 2, len(items))
    return items


def minimize_bundle(bundle_dir, reproduces) -> dict:
    """Drop turns while the replay still reproduces the failure (§6.5).

    Only geometry cases are auto-minimized: each check is a deterministic
    replay. Model-behavior cases ship the full window -- each check there costs
    live runs. `reproduces(bundle_dir)` runs the replay and decides; the caller
    owns the perturbation or the recorded residual that defines the failure.
    """
    bundle = Path(bundle_dir)
    label = json.loads((bundle / "labels.json").read_text(encoding="utf-8")) \
        if (bundle / "labels.json").is_file() else {}
    events = read_events(bundle / "events.jsonl")
    if label.get("class") != "geometry":
        return {"minimized": False, "reason": "model-behavior case ships the full window",
                "dropped": 0}
    # One unit = a tool_call plus the tool_result that answers it. Dropping a
    # marker call changes the plan's targets, so the tape (and the failure)
    # moves with it -- the dependency is kept by the test, not by bookkeeping.
    units = []
    for index, event in enumerate(events):
        if event.get("type") != "tool_call":
            continue
        result = next((j for j in range(index + 1, len(events))
                       if events[j].get("type") == "tool_result"
                       and events[j].get("name") == event.get("name")), None)
        units.append((index, result))
    cache = {}

    def test(kept):
        key = frozenset(kept)
        if key in cache:
            return cache[key]
        dropped = set()
        for unit in units:
            if unit not in kept:
                dropped.add(unit[0])
                if unit[1] is not None:
                    dropped.add(unit[1])
        scratch = bundle.parent / (bundle.name + "-min")
        if scratch.exists():
            shutil.rmtree(scratch)
        shutil.copytree(bundle, scratch)
        (scratch / "events.jsonl").write_text(
            "".join(json.dumps(e, ensure_ascii=False) + "\n"
                    for i, e in enumerate(events) if i not in dropped),
            encoding="utf-8")
        answer = bool(reproduces(scratch))
        shutil.rmtree(scratch, ignore_errors=True)
        cache[key] = answer
        return answer

    kept = _ddmin(units, test)
    dropped = set()
    for unit in units:
        if unit not in kept:
            dropped.add(unit[0])
            if unit[1] is not None:
                dropped.add(unit[1])
    if dropped:
        (bundle / "events.jsonl").write_text(
            "".join(json.dumps(e, ensure_ascii=False) + "\n"
                    for i, e in enumerate(events) if i not in dropped),
            encoding="utf-8")
    return {"minimized": True, "dropped": len(dropped),
            "events_before": len(events), "events_after": len(events) - len(dropped)}


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Freeze, export and minimize trace bundles.")
    parser.add_argument("--traces-dir", default=str(STATE / "traces"))
    parser.add_argument("--session", default="", help="session id to bundle")
    parser.add_argument("--task", type=int, default=None, help="1-based task window")
    parser.add_argument("--state-dir", default="", help="state root (default: traces-dir's parent)")
    parser.add_argument("--bundle", default="", help="an existing bundle dir")
    parser.add_argument("--export", action="store_true", help="export the bundle")
    parser.add_argument("--export-root", default="", help="where export/<case-id> goes")
    parser.add_argument("--case-id", default="")
    parser.add_argument("--dedup", action="store_true", help="group labels by signature")
    parser.add_argument("--minimize", action="store_true",
                        help="delta-debug a geometry bundle against its replay")
    args = parser.parse_args(argv)

    if args.dedup:
        groups = dedup(args.traces_dir)
        for signature, group in groups.items():
            print("%s  %s  x%d  canonical %s"
                  % (signature[:12], group["class"], group["occurrences"], group["canonical"]))
        print("%d signature group(s) -> %s" % (len(groups), Path(args.traces_dir) / "dedup.json"))
        return 0

    bundle = Path(args.bundle) if args.bundle else None
    if args.session:
        result = bundle_session(args.traces_dir, args.session, args.task,
                                args.state_dir or None)
        bundle = result["dir"]
        status = "complete" if result["complete"] else "INCOMPLETE: " + "; ".join(result["reasons"])
        print("%s -> %s (%s)" % (args.session, bundle, status))
        if not result["complete"] and not args.minimize:
            return 1
    if bundle is None:
        parser.error("give --session or --bundle")

    if args.minimize:
        # The replay is the oracle for minimization: import lazily so bundling
        # never drags the eval harness (and its model assumptions) in.
        import agent_eval

        def reproduces(candidate):
            code = agent_eval.main(["--trace", str(candidate), "--replay", "--quiet"])
            report_path = candidate / "replay-report.json"
            if not report_path.is_file():
                return False
            report = json.loads(report_path.read_text(encoding="utf-8"))
            return code != 0 and any(d.get("class") == "geometry-diff"
                                     for d in report.get("diffs") or [])

        outcome = minimize_bundle(bundle, reproduces)
        print("minimize: %s" % json.dumps(outcome))
    if args.export:
        target = export_bundle(bundle, args.export_root or None, args.case_id or None)
        if target is None:
            return 1
        print("exported -> %s" % target)
    return 0


if __name__ == "__main__":
    sys.exit(main())