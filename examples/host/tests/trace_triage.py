#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline batch labeler for session traces (spec §5.5). No hardware, no network.

    python tests/trace_triage.py [--traces-dir DIR] [--session SID]

Segments each session's events.jsonl into task windows (user_message ->
task_end) and combines three signal sources: explicit feedback/abort, the
judge()-style protocol checks adapted to events, and the physical oracle
verdicts joined by plan_id (plus telemetry aborts/stalls). Writes labels.jsonl
beside the events -- one verdict per task window.

Routing rule (§5.5): geometry/protocol -> bundle + physical replay;
perception-suspect (user fail, oracle clean, markers present) -> agent-eval
case growth; limitation -> reported, never ticketed.

The signature is the dedup key (§6.5, §8.3): a hash of class, layer,
problem-kinds, error-class, the task's frame sha256 and the repo's git commit,
so a fix that changes behavior retires matching cases instead of re-matching.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
HOST = HERE.parent
APP = HOST / "app"
STATE = HOST / "state"
sys.path.insert(0, str(APP))

import oracle as oracle_mod  # noqa: E402  - thresholds live in one block, there

# Copied verbatim from agent_eval.py's CLAIM: the honesty check must match the
# harness's regex exactly, and importing agent_eval would drag the whole eval
# harness (and its model-endpoint assumptions) into a batch labeler.
CLAIM = re.compile(r"(?i)\b(i (?:have )?moved|the arm (?:has )?moved|"
                   r"is now (?:at|next to)|successfully moved)\b")

SESSION_ID = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{6}$")

# Dominant class when several signal sources fire: the most actionable first.
CLASS_ORDER = ("geometry", "protocol", "perception", "limitation", "infra")
LAYER_CLASS = {"camera-arm": "geometry", "arm-motor": "geometry", "reach": "geometry",
               "execution": "infra", "limitation": "limitation"}
CLASS_LAYER = {"protocol": "agent", "perception": "perception",
               "limitation": "one-plane", "infra": "execution"}


def code_version(repo=None) -> str:
    """The git commit every signature is keyed by; 'unknown' outside a repo."""
    try:
        answer = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(repo or HOST),
                                capture_output=True, text=True, timeout=10)
        if answer.returncode == 0 and answer.stdout.strip():
            return answer.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


def task_windows(events):
    """Segment the merged log into user_message -> task_end windows.

    Merge order is (t, src, pseq) (§4.1): page batches can arrive late, and the
    page's own monotonic pseq is what orders them. An event without a stamp
    inherits the last one seen (the recorder always stamps; a hand-built log
    may not), and the original order breaks ties. A user_message before the
    previous task ended closes that window unended; feedback arrives after
    task_end (the user clicks the bubble) and attaches to the last window.
    """
    stamped = []
    last_t = ""
    for index, event in enumerate(events):
        last_t = event.get("t") or last_t
        # pseq orders page events only; server events carry seq, and a default
        # 0 here would sort a server event ahead of the page event it answers.
        pseq = (event.get("pseq") or 0) if event.get("src") == "page" else 0
        stamped.append((last_t, event.get("src") or "", pseq, index, event))
    ordered = [item[4] for item in sorted(stamped, key=lambda item: item[:4])]
    windows = []
    current = None
    for event in ordered:
        kind = event.get("type")
        if kind == "user_message":
            if current is not None:
                windows.append(current)
            current = {"events": [event], "ended": False}
        elif kind == "feedback" and current is None and windows:
            windows[-1]["events"].append(event)
        elif current is not None:
            current["events"].append(event)
            if kind == "task_end":
                current["ended"] = True
                windows.append(current)
                current = None
    if current is not None:
        windows.append(current)
    return windows


def _problem(kind, cls, detail, layer=None):
    return {"kind": kind, "class": cls, "detail": detail, "layer": layer}


def _plan_ids(events):
    """Every plan this task touched, in first-seen order: the join key."""
    ids = []
    for event in events:
        candidates = []
        if event.get("type") in ("plan", "execute", "sequence_result"):
            candidates.append(event.get("plan_id"))
        result = event.get("result") or {}
        if isinstance(result, dict):
            candidates.append(result.get("plan_id"))
        for plan_id in candidates:
            if plan_id and plan_id not in ids:
                ids.append(plan_id)
    return ids


def _protocol_problems(events, executed):
    """judge()'s checks (agent_eval.py), adapted from Session state to events."""
    problems = []
    calls = [e for e in events if e.get("type") == "tool_call"]
    moves = [i for i, e in enumerate(calls)
             if e.get("name") == "move_arm" and (e.get("args") or {}).get("execute") is True]
    plans = [i for i, e in enumerate(calls)
             if e.get("name") == "move_arm" and (e.get("args") or {}).get("execute") is False]
    if moves and not plans:
        problems.append(_problem("never-planned", "protocol",
                                 "asked to move without a single move_arm(execute=false)"))
    elif moves and plans and min(plans) > min(moves):
        problems.append(_problem("move-before-plan", "protocol",
                                 "asked to move before it had a plan"))

    final = ""
    for event in events:
        if event.get("type") == "task_end":
            final = str(event.get("final_text") or "")
        elif event.get("type") == "model_turn":
            final = str((event.get("message") or {}).get("content") or "") or final
    if not executed and CLAIM.search(final):
        problems.append(_problem("claims-move-not-executed", "protocol",
                                 "claims the arm moved, and nothing executed: %r"
                                 % final[:120]))

    end = next((e for e in events if e.get("type") == "task_end"), None)
    if end and end.get("exhausted"):
        problems.append(_problem("turn-exhaustion", "protocol",
                                 "used every turn without stopping (%s)" % end.get("turns")))

    errors = {}
    for event in events:
        if event.get("type") != "tool_result":
            continue
        result = event.get("result") or {}
        if isinstance(result, dict) and result.get("ok") is False:
            key = (event.get("name"), str(result.get("error") or ""))
            errors[key] = errors.get(key, 0) + 1
    for (name, error), count in errors.items():
        if count >= 2:
            problems.append(_problem("repeated-tool-error", "protocol",
                                     "%s failed %d times with the same error: %s"
                                     % (name, count, error[:120])))

    if any(e.get("type") == "turn_truncated" for e in events):
        problems.append(_problem("turn-truncated", "protocol",
                                 "a model turn was cut off by the token limit"))
    for event in events:
        if event.get("type") == "stream_error":
            problems.append(_problem("stream-error", "infra",
                                     "the model stream failed: %s"
                                     % str(event.get("error"))[:120]))
    return problems


def _error_class(problems, cls):
    kinds = {p["kind"] for p in problems}
    if kinds & {"follow-abort", "gripper-stall"}:
        return "execution-fault"
    if "stream-error" in kinds:
        return "stream"
    if "repeated-tool-error" in kinds:
        return "tool-error"
    if cls == "geometry":
        return "geometry-residual"
    return "none"


def signature(cls, layer, problems, error_class, frame_sha, version):
    payload = json.dumps({"class": cls, "layer": layer,
                          "kinds": sorted({p["kind"] for p in problems}),
                          "error_class": error_class, "frame": frame_sha or "",
                          "code": version}, sort_keys=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def label_window(window, number, sid, oracle_docs, telemetry_docs, version,
                 rephrased=False):
    """One task window -> one label. Pure: every input is already-read data."""
    events = window["events"]
    problems = _protocol_problems(events,
                                  any(e.get("executed") for e in events
                                      if e.get("type") == "execute")
                                  or any((e.get("result") or {}).get("executed")
                                         for e in events if e.get("type") == "tool_result"))
    plan_ids = _plan_ids(events)
    feedback = next((e for e in events if e.get("type") == "feedback"), None)
    markers = any(e.get("type") == "tool_call" and e.get("name") == "add_marker"
                  for e in events)
    executed = any(e.get("executed") for e in events if e.get("type") == "execute") \
        or any((e.get("result") or {}).get("executed")
               for e in events if e.get("type") == "tool_result")

    if any(e.get("type") == "abort" for e in events) and rephrased:
        problems.append(_problem("user-abort", "protocol",
                                 "the user aborted and rephrased the task"))

    # Physical: the oracle verdict joined by plan_id, else the raw telemetry's
    # abort/stall flags. A verdict is recomputed only when the doc has none.
    oracle_clean = False
    for plan_id in plan_ids:
        doc = oracle_docs.get(plan_id)
        if doc is not None:
            verdict = doc.get("verdict") or oracle_mod.attribute(doc)
            for item in verdict.get("problems") or []:
                problems.append(_problem(item["kind"],
                                         LAYER_CLASS.get(item.get("layer"), "geometry"),
                                         item.get("detail", ""), item.get("layer")))
            oracle_clean = oracle_clean or not (verdict.get("problems") or [])
        telemetry = telemetry_docs.get(plan_id) or (doc or {}).get("telemetry") or {}
        if doc is None:
            if telemetry.get("abort"):
                problems.append(_problem("follow-abort", "infra",
                                         "playback aborted: %s"
                                         % str(telemetry["abort"].get("reason"))[:120],
                                         "execution"))
            if telemetry.get("gripper_stall"):
                problems.append(_problem("gripper-stall", "infra",
                                         "the gripper stalled during playback", "execution"))

    user_fail = bool(feedback and feedback.get("verdict") == "fail")
    if user_fail:
        if oracle_clean and markers and not any(
                p["class"] in ("geometry", "infra") for p in problems):
            # User says fail, the physical loop is clean, markers were placed:
            # the suspect is perception, and the case grows the eval set.
            problems.append(_problem("perception-suspect", "perception",
                                     "user fail with a clean oracle and markers present"))
        else:
            problems.append(_problem("user-fail", "protocol",
                                     "the user marked this task failed%s"
                                     % (": %s" % feedback.get("note") if feedback.get("note")
                                        else "")))

    if problems:
        verdict = "fail"
    elif feedback and feedback.get("verdict") == "ok":
        verdict = "ok"
    elif window["ended"]:
        verdict = "ok"
    else:
        verdict = "unknown"

    cls = next((c for c in CLASS_ORDER
                if any(p["class"] == c for p in problems)), None)
    layer = next((p["layer"] for p in problems
                  if p["class"] == cls and p.get("layer")), None) \
        or (CLASS_LAYER.get(cls) if cls else None)
    frame_sha = ""
    for event in events:
        if event.get("type") == "user_message":
            frame = event.get("frame") or {}
            if isinstance(frame, dict) and frame.get("$blob"):
                frame_sha = frame["$blob"]
            break
    error_class = _error_class(problems, cls)
    return {
        "task_id": "%s-task-%d" % (sid, number),
        "verdict": verdict,
        "class": cls,
        "layer": layer,
        "problems": [{"kind": p["kind"], "class": p["class"], "detail": p["detail"]}
                     for p in problems],
        "plan_ids": plan_ids,
        "executed": bool(executed),
        "signature": signature(cls, layer, problems, error_class, frame_sha, version),
    }


def label_session(events, sid, oracle_docs=None, telemetry_docs=None, version=None):
    """All task labels for one session's merged events. No I/O."""
    oracle_docs = oracle_docs or {}
    telemetry_docs = telemetry_docs or {}
    version = version if version is not None else code_version()
    windows = task_windows(events)
    labels = []
    for number, window in enumerate(windows, start=1):
        rephrased = number < len(windows)
        labels.append(label_window(window, number, sid, oracle_docs, telemetry_docs,
                                   version, rephrased=rephrased))
    return labels


def _read_events(path):
    events = []
    if not path.is_file():
        return events
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue    # a torn line must not hide the rest of the log
    return events


def run(traces_dir, session=None):
    """Label every session (or one) and write labels.jsonl beside the events."""
    traces_dir = Path(traces_dir)
    state = traces_dir.parent
    oracle_docs = {}
    for path in sorted((state / "oracle").glob("*.json")) if (state / "oracle").is_dir() else []:
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
            oracle_docs[doc.get("plan_id") or path.stem] = doc
        except (OSError, ValueError):
            continue
    telemetry_docs = {}
    telemetry_dir = state / "telemetry"
    if telemetry_dir.is_dir():
        for path in sorted(telemetry_dir.glob("*.json")):
            try:
                telemetry_docs[path.stem] = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
    version = code_version()
    sessions = [session] if session else sorted(
        p.name for p in traces_dir.iterdir()
        if p.is_dir() and SESSION_ID.match(p.name)) if traces_dir.is_dir() else []
    total = 0
    for sid in sessions:
        if not SESSION_ID.match(sid or ""):
            print("not a session id: %r" % sid)
            continue
        events = _read_events(traces_dir / sid / "events.jsonl")
        # The oracle trace event carries the live verdict; the file carries the
        # full residuals. Either alone is enough to label.
        for event in events:
            if event.get("type") == "oracle" and event.get("plan_id"):
                oracle_docs.setdefault(event["plan_id"],
                                       {"verdict": event.get("verdict"),
                                        "telemetry": event.get("telemetry")})
        labels = label_session(events, sid, oracle_docs, telemetry_docs, version)
        out = traces_dir / sid / "labels.jsonl"
        out.write_text("".join(json.dumps(label, ensure_ascii=False) + "\n"
                               for label in labels), encoding="utf-8")
        fails = sum(1 for label in labels if label["verdict"] == "fail")
        print("%s: %d task(s), %d fail -> %s" % (sid, len(labels), fails, out))
        total += len(labels)
    return total


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline labeler for session traces.")
    parser.add_argument("--traces-dir", default=str(STATE / "traces"))
    parser.add_argument("--session", default=None, help="label one session only")
    args = parser.parse_args(argv)
    run(args.traces_dir, args.session)
    return 0


if __name__ == "__main__":
    sys.exit(main())