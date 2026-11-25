# reBot B601 — Failure-Trace Collector, Closed-Loop Oracle & Offline Replay

Implementation spec against `sweatybridge/reBot-B601-Agent-Guide` (main, cloned 2026-11-18).
All line numbers refer to that checkout. Paths are relative to the repo root unless absolute.

## 1. Goals and non-goals

**Goals**

1. Automatically collect traces of chat tasks (`/live` agent sidebar) that failed, into `state/`.
2. Every collected failure is replayable end-to-end offline: no browser, no board, no CAN, no arm motion.
3. Failures are deduplicated, minimized, and exported as `agent_cases/`-compatible cases a coding agent can fix and verify with one command.
4. Detect **silent geometry failures** (camera→arm, arm→motor correspondence) via closed-loop oracles, not just protocol violations — this is the dominant error class.
5. Use the oracle residuals to hillclimb the physical correspondence parameters in a previously unknown environment, within measured identifiability and safety limits.

**Non-goals**

- Replacing `browser_check.py` (page plumbing) or the bench validation; those stay.
- Guaranteeing "any object, accurately". The deliverable is a *measured accuracy envelope* with explicit refusal outside it (see §7.4).
- Force control, multi-view perception, or grasp planning — out of scope of the guide's stack.
- Changing the execution gates. Nothing in the collector, replay, or hillclimb may move the arm without the existing three gates (`REBOT_EXECUTE=1`, edge `--allow-execute`, deliberate confirm).

**Design principle**: reuse the repo's existing primitives — immutable captures, rig/calib fingerprints, `traj.validate`, the `/move` safety checks, the tag-rig verification gates, `MockEdge`, `judge()` — and add only what is missing. The repo's philosophy ("reported rather than assumed", "refused rather than guessed") applies to every new component.

## 2. Architecture

```
/live sidebar (live.html)
   │  POST /trace  (session events: user msg, tool calls+results, frames, compaction, abort, feedback)
   │  POST /agent  (existing; server now also logs model turns)
   ▼
capture_server.py ──► app/trace.py (TraceRecorder) ──► state/traces/<session>/events.jsonl
   │                                                    state/traces/blobs/<sha256>.jpg
   │  planner_call() choke point logs every gateway request/response (the "tape")
   ▼
planner_server.py ──► edge /play ──► NEW: structured telemetry (cmd/meas per waypoint)
   │                     state/telemetry/<plan_id>.json
   │  NEW /fk route (FK of measured joints → base-frame tip)
   ▼
capture_server.py post-execution verification (NEW)
   │  grab_one() still + gripper-tip detector + locate_target.project(FK(measured q))
   ▼
state/oracle/<plan_id>.json   (predicted-vs-observed pixel, per-joint hold error, witness, verdict)
   │
   ▼  offline batch (no hardware)
tests/trace_triage.py   labeler: judge rules + oracle verdicts + user feedback → labels.jsonl
tests/trace_bundle.py   bundler: freeze frames/calib/rig/env/tape → bundle/
tests/agent_eval.py     NEW --trace mode: deterministic replay (no LLM) + live-model replay ×N
   │
   ▼
state/traces/export/<case>/  ──► coding agent (case.json + report.md + repro command)
   │
   ▼  physical hillclimb (gated, on the bench)
app/identify.py   residual attribution → identifiability gate → active excursions →
                  candidate rig / sag map → rig verify gates → hot-load
```

Two planes, mirroring the repo's own split:

- **Evidence plane** (live, on the bench): record everything needed to reconstruct a failure — conversation, frames, geometry provenance, gateway responses, joint telemetry, post-move verification. Once recorded, it is immutable.
- **Analysis plane** (offline, anywhere): label, bundle, replay, dedup, export. Never touches hardware. The only component that moves the arm is the hillclimb (§7), and it does so through the existing `/move` safety path.

## 3. State directory layout (all under `examples/host/state/`, already gitignored)

```
state/
  traces/
    <session-id>/                 # one per sidebar conversation, id = UTCstamp-hex6 (same format as captures)
      events.jsonl                # append-only, both page and server events, merged by seq
      manifest.json               # env snapshot at session start (rig/calib fingerprints, gates, versions)
      labels.jsonl                # written by triage; one verdict per task window
    blobs/<sha256>.jpg            # content-addressed frames, shared across sessions
    export/<case-id>/             # handoff packages (§6.4)
  telemetry/<plan_id>.json        # structured cmd/meas samples per execution (§5.2)
  oracle/<plan_id>.json           # closed-loop verification verdict (§5.3)
  calib/
    extrinsics-fixed.json         # existing; hillclimb writes candidates beside it
    extrinsics-candidate-<ts>.json# NEW: pending verification, never hot-loaded until it passes
    sag-<rig-fingerprint>.json    # NEW: learned residual-torque map (§7.3)
    accuracy-map.json             # NEW: validated envelope (§7.4)
  captures/  out/  recordings/    # unchanged
```

Rules:

- `events.jsonl` is append-only; writers use `O_APPEND` single-line writes (aiohttp handlers are single-process; the page batches over HTTP). No rewriting, no compaction of the *log* — compaction of the *model context* is recorded as an event, never applied to the log.
- Blobs are written once by sha256; a frame referenced by N sessions costs one file. JPEG bytes are stored exactly as delivered to the model (the page's `grabFrame()` quality-0.82 data-URL payload, decoded), so replay feeds the model bit-identical images.
- Retention: `--trace-retention-days` (default 30) prunes whole session dirs and unreferenced blobs; `export/` is exempt. Tokens and keys are never written (§8.2).

## 4. Phase 1 — Session trace collector

### 4.1 New module: `examples/host/app/trace.py`

```python
class TraceRecorder:
    def __init__(self, root: Path, retention_days: int = 30): ...
    def event(self, sid: str, src: str, type: str, **fields) -> int   # append one JSONL line, returns seq
    def events(self, sid: str, items: list[dict]) -> list[int]       # batch, preserves order
    def blob(self, data: bytes) -> str                               # sha256 hex; write-once under blobs/
    def manifest(self, sid: str, snapshot: dict) -> None             # written once, at first event
    def prune(self) -> None                                          # retention pass, called daily
```

- Session ids reuse the capture format (`new_capture_id()`, capture_server.py:75; regex `CAPTURE_ID`, :62) so one id scheme serves captures, sessions, plans.
- `event()` is the only writer; it assigns a monotonic per-session `seq` under an `asyncio.Lock`, stamps server `t`, and appends with a single `os.write` on an `O_APPEND` fd. Page events additionally carry `pseq` (page-monotonic) so offline merge can order them even if batches arrive late: sort by `(t, src, pseq)`.
- Data-URL images inside any event field are extracted to blobs and replaced by `{"$blob": "<sha256>"}` before writing. This keeps `events.jsonl` small and makes every frame reference content-addressed.
- Redaction at write time: drop keys matching `token|authorization|api_key` recursively; store only the *host* of `llm_url`, never the key.

### 4.2 `capture_server.py` changes

| Where | Change |
|---|---|
| `build_app` (:1849) | `app["traces"] = TraceRecorder(...)`; new args `--traces-dir` (default `STATE/"traces"`), `--trace-retention-days` (30), `--no-trace` |
| routes (:1956–1990) | `POST /trace` → `handle_trace`; `GET /trace/{sid}/events` and `/manifest` → `handle_trace_read`. **Note:** `token_middleware` (:124) exempts GET, so `handle_trace_read` must check `X-Rebot-Token` explicitly |
| `handle_trace` (new) | body `{session, events:[...]}`; validates sid format, caps body at 8 MiB and batch at 64 events; extracts blobs; writes manifest on first sight of a session (snapshot below); returns `{ok, seqs}` |
| `handle_agent` (:1231) | read `X-Rebot-Session` header; on the SSE `done` path (:1193–1211) and the JSON path (:1316–1338) record a `model_turn` event; on the 502/`error` paths (:1201–1204, :1289–1314) record `stream_error`. The event carries the **full upstream `messages` array** with data-URLs replaced by blob refs, the assembled assistant message, `finish_reason`, `complete`, `usage`, and the model config (`model, temperature, max_tokens, tools_sha256`). Storing the exact prompt per turn makes the log immune to the page's context compaction — no reconstruction by diffing |
| `_relay_sse` (:1096) | pass a callback so the assembled final message reaches `handle_agent`'s recorder (it already builds it at :1193–1198); no behavior change to the stream |
| `planner_call` (:783) | add optional `sid` parameter; when set, record a `gateway_call` event `{path, request, response, status, ms}` — this is the **response tape**. Key = sha256 of canonical-JSON request. Every gateway interaction (`/plan`, `/execute`, `/pose`, `/move`) flows through this one function, so one hook tapes them all |
| `handle_sequence` (:804), `handle_execute` (:1341) | forward `X-Rebot-Session` into `planner_call`; record `plan` / `execute` events linking `plan_id ↔ session` (the page also sends the header on these fetches) |
| `handle_health` (:347) | add `traces: {enabled, sessions, retention_days}` |

**Manifest snapshot** (written at first event of a session, from `app` state): `VERSION` (:56), sha256 of the served `live.html` bytes, `rig.provenance()` + `rig.fingerprint` (extrinsics.py:821, :878), full rig and calib **file bytes copied into the session dir** (`calib.json`, `rig.json` — not just fingerprints; the live files may be refitted later), `calib.fingerprint` (calib.py:124), `plane_z, tip_m, min_z, hover_m`, `execute` gate, planner URL host, `stream_turn` + source, marker placement, LLM config minus secrets, `tool_schemas_sha256`.

### 4.3 `live.html` + new `web/trace.js`

`web/trace.js`: a tiny queue — `trace(type, fields)`, batched flush (≤32 events or 1 s), `navigator.sendBeacon` on `pagehide`, sha256 via `crypto.subtle.digest`, and a frame-dedup cache so a frame shown twice uploads once. Loaded like the other modules (:176–186); served by a new `handle_trace_js` alongside the existing module routes.

Session id: `CHAT.sessionId` created at page load (stamp-hex6). Every `api()` call to `/agent`, `/sequence`, `/execute`, `/trace` carries `X-Rebot-Session`.

Hooks (exact locations):

| Hook | Event(s) |
|---|---|
| `sendChat` (:1313–1323) | `user_message {text, frame: $blob}` — the frame from `grabFrame()` (:840) is uploaded here, **before** any pruning |
| `runTool` wrapper (:1038–1053) | `tool_call {name, args}` and `tool_result {name, args, result, ms}` around the dispatch — one wrapper covers all seven tools |
| frame injection (:1281–1292) | `frame_delivered {$blob, width, height, reason: "capture_view"}` |
| `toolAddMarker` (:986–1013) | `marker_solved {u, v, height_m, tip_m, turns, rig_fingerprint, inside_band, above_floor}` — the client-side geometry provenance, since agent markers never touch the witness-checked capture path (comment :393–397) |
| `runSequence` (:457–527) | `sequence_result {plan_id, targets, summary, executed, error?}` — carries the in-page-solved targets exactly as posted |
| `compactHistory` (:685), `trimHistory` (:648) | `compaction {dropped_messages, dropped_frames, reason}` |
| `stopAgent` (:1158), abort paths (:1295, :1302) | `abort {}` |
| `runAgent` catch (:1297–1303), `finish_reason==="length"` (:1212–1219) | `stream_error {error}` / `turn_truncated {finish_reason}` |
| `runAgent` return without tool calls (:1253), loop exhaustion (:1296) | `task_end {final_text, turns, exhausted: bool}` |
| `pinStream` / `streamMismatch` (:1363–1390) | `stream_geometry {width, height, event: "pin"|"repin"|"mismatch"}` |
| NEW thumbs-up/down buttons per task bubble | `feedback {verdict: "ok"|"fail", note?}` — the explicit signal; placed next to the final agent reply of each task |

### 4.4 Event schema (one JSONL line each)

```json
{"seq": 41, "pseq": 17, "t": "2026-11-18T10:23:21.813Z", "src": "page",
 "type": "tool_result", "name": "move_arm", "args": {"execute": false},
 "result": {"ok": true, "planned": true, "executed": false, "plan_id": "20261118T102320Z-9f3a2c",
            "summary": {"waypoints": 660, "duration_s": 22.0, "worst_ik_m": 0.004, "peak_tau_nm": 11.2}},
 "ms": 2413}
```

Common fields: `seq` (server-assigned), `pseq` (page-assigned, page events only), `t`, `src` ∈ `page|server`, `type`. Types: `session_start, user_message, model_turn, tool_call, tool_result, frame_delivered, marker_solved, sequence_result, plan, execute, gateway_call, compaction, abort, stream_error, turn_truncated, task_end, feedback, stream_geometry, oracle` (Phase 2).

**Why both ends record**: tools execute in the browser, so the server never sees tool results directly; and `compactHistory`/`trimHistory` delete history from subsequent `/agent` payloads, so server-side diffing would lose turns. Page events give execution truth; server `model_turn` events give the exact prompt and model output. The merge is redundant by design — either side alone is lossy.

## 5. Phase 2 — Closed-loop oracle (silent geometry failures)

The judge-style checks catch protocol violations; they cannot catch a wrong `T_base_cam` or a sag-sign error, because every tool returns `ok: true`. The oracle measures the physical loop `pixel → tip → joints → motors → held pose → pixel` and attributes the residual to a layer. All evidence is recorded **live**; offline analysis only reads it.

### 5.1 Edge: structured playback telemetry (`examples/edge-rust/src/main.rs`)

Today the structured log exists but is CLI-only: `LogFile`/`write_log` (:2163–2213) produce `rebot-trajectory-log` v1 with `samples:[{t, cmd, meas}]`, but serve-mode `run_child` (:3025–3123) never forwards `--log`, so `/play` returns only stdout text (:3108–3113), truncated to 4000 chars by the gateway.

Changes:

1. `run_child`: pass `--log <tempfile>` to the child (alongside the existing `--track`, :3051–3074); after exit, parse the JSON log and include it in the `/play` response as `telemetry`. Downsample to ≤2000 samples for the wire, but **always keep every sample inside hold windows** (where `cmd` is constant) — those are the measurement points.
2. `write_log` (:2184–2213): bump to `version: 2`, add optional fields `abort: {t, reason}` (from the abort path :2501), `gripper_stall: {t, commanded, measured}` (:2435–2443), `tick: {mean_ms, p95_ms, max_ms}` (already computed :2512–2532), `follow_error_max: {motor, rad}` (:2464–2476). v1 readers ignore unknown fields; `--check` mode is unaffected.
3. No change to gates, timing, or the CAN path. The log write happens after playback, off the realtime loop (as today, `run()` :3287–3293).

### 5.2 Gateway: persist and relay telemetry (`planner_server.py`)

- `execute_one` (:375–404): write the full telemetry to `state/telemetry/<plan_id>.json` (trajectory ref, kp/kd, samples, abort/stall/tick), relay the downsampled block in the response next to `executed` and `log`. The 4000-char stdout tail stays for humans.
- `move_one` (:242–372): same, keyed by a generated move id; this tapes calibration moves too.
- New `GET /telemetry/{id}` (token-gated like other routes) so the capture server can fetch full samples after execution.
- New `POST /fk` `{q: [7 floats]}` → `{ok, tip_m: [x,y,z], joint_points: [[x,y,z]×7]}` via `rebot_ik` (Pinocchio lives in this image; the capture server has no FK). This is the only new compute route; it is read-only and offline-safe.

### 5.3 Capture server: verification during and after execution (`capture_server.py`)

Key insight: **the arm holds at every marker** (`plan_sequence` builds holds). A frame taken while the arm is still shows the held pose regardless of capture latency — the stillness-window principle. Piggyback the camera↔arm measurement on those holds — no extra arm motion:

1. `handle_execute` (:1341), when the plan will execute: grab one **pre-execution still** (template source and pre/post diff baseline), read `state/out/<plan_id>.json` (shared bind mount), find hold windows (≥0.8 s of constant `cmd`), and spawn an `asyncio` task that calls `grab_one` (:1707) at each window's midpoint (two grabs 300 ms apart, keep the later; skip windows <0.8 s — tracking-only oracle there). Frames go to blobs + `state/oracle/<plan_id>/hold-N.jpg`.
2. After `/execute` returns: fetch telemetry (§5.2), and per hold compute:
   - `q_meas` = mean measured joints in the window; `hold_error_rad[7] = q_meas − q_cmd`.
   - `tip_fk_cmd`, `tip_fk_meas` via planner `/fk`; `tip_error_mm = |tip_fk_meas − tip_fk_cmd|`.
   - `predicted_tip_px = locate_target.project(K, rig.T_base_cam, tip_fk_meas, dist)` (locate_target.py:196).
   - `detected_tip_px` via the gripper-tip detectors: `detect_tip_green` (auto_calibrate.py:554) when the green gripper is fitted, else `match_tip` (:447) with templates cut from the pre-execution frame at the predicted pixel (`tip_templates`, :414). Detector name + score recorded; a failed detection is `detected: null`, not a guessed pixel.
   - `pixel_error_px`, and `pixel_error_mm` via `witness.millimetres_per_pixel` (:240).
3. Post-execution still (arm back at park, out of the way): pre/post image difference around each destination marker, stored as a secondary placement signal. Optional VLM check through the existing `REBOT_LLM_*` endpoint ("is the object at the marked spot?") — recorded as opinion, never as ground truth.
4. Write `state/oracle/<plan_id>.json` and an `oracle` trace event. Also persist the already-computed tag reprojection error from `measure_stream_turn` (:1549–1618) and the witness report wherever a capture path ran.

### 5.4 Attribution: residual shape → layer (`app/oracle.py`, new)

Pure function `attribute(oracle_doc) -> {layer, problems, confidence}`, used live for the verdict and offline by triage. Signatures (from the guide's own measurements):

| Residual shape | Layer / cause |
|---|---|
| `pixel_error` large, `tip_error_mm` small (arm held what it was told, camera disagrees) | **camera→arm**: rig / K / turns |
| pixel error present only when `stream_turn.source == "shape"`, or only in landscape | turns correction |
| witness `drift_px` over limit, or `scale` off | stale rig — camera moved (already a refusal on capture paths; markers bypass it, §4.3) |
| `hold_error_rad` per-joint ∝ `tau_g(q)/kp`, load-dependent | **arm→motor**: sag model / gains (the 61→23.6 mm story) |
| per-joint constant bias, pose- and load-independent | joint zeros (5 mrad ≈ 3 mm at 0.6 m) |
| `tip_error_mm` grows near reach boundary, IK refusals in tape | reach model / `--max-reach` mismatch |
| placement error z-only ≈ object height | **limitation** (one-plane assumption) — label, never a bug ticket |
| follow-error abort / gripper stall in telemetry | execution fault — correlate, but not a correspondence bug |

Thresholds start conservative (`pixel_error_mm > 10`, `tip_error_mm > 8`, `hold_error > 30 mrad` — inside the arm's 23 mm repeatability but above the fit's ~5 mm normal) and live in one constants block, tunable from measured noise floors.

### 5.5 Labeler: `tests/trace_triage.py` (offline, no hardware)

Batch over `state/traces/`: segment each session into task windows (`user_message` → `task_end`), then combine three signal sources:

1. **Explicit**: `feedback` verdicts; `abort` followed by a rephrased `user_message`.
2. **Protocol**: reuse `judge()` logic (agent_eval.py:678–741) adapted to events — plan-first ordering, honesty (`CLAIM` regex :674 vs any `executed: true`), turn exhaustion, repeated identical tool errors, `turn_truncated`, `stream_error`.
3. **Physical**: oracle verdicts joined by `plan_id`; telemetry aborts/stalls.

Output `labels.jsonl` per session: `{task_id, verdict: fail|ok|unknown, class: protocol|geometry|perception|limitation|infra, layer, problems[], plan_ids[], signature}`. Routing rule: geometry/protocol → bundle + physical replay (§6); perception-suspect (user fail, oracle clean, markers off-object) → agent-eval case growth; limitation → reported, never ticketed. `signature` = hash of `(class, layer, problem-kind, error-class, frame-sha256, code-version)` — the dedup key (§6.5).

## 6. Phase 3 — Bundle, offline replay, export

### 6.1 Tape mode for `tests/mock_arm.py`

`serve(port, host, **options)` (:169–177) gains `tape=<path>`. The tape is the recorded `gateway_call` events:

```json
{"entries": [
  {"key": "<sha256(method|path|canonical-json(body))>", "method": "POST", "path": "/plan",
   "request": {...}, "status": 200, "response": {...}},
  {"key": "...", "path": "/pose", "response": {"ok": true, "pose": {"ids": [1,2,3,4,5,6,7], "q": [...]}}}
]}
```

- Hit → return the recorded status + body verbatim. **Same-key entries form a FIFO queue** (`/pose` is read repeatedly and legitimately changes between calls).
- Miss → fall back to today's synthetic behavior, and count the miss in `State` (new field `tape_misses`, exposed via `/mock/state`). A replay with tape misses is reported as *partial fidelity*, never silently passed.
- `fail_first` injection still works and takes precedence — existing retry tests are unchanged.

This mocks at exactly the boundary `MockEdge` already uses (the gateway), so no Pinocchio, board, or CAN is needed; and because responses are *recorded*, even a real IK refusal reproduces offline.

### 6.2 Bundler: `tests/trace_bundle.py` (offline)

`--session <sid> [--task <n>]` freezes one labeled task window into `state/traces/<sid>/bundle/`:

- `events.jsonl` (the window), `manifest.json`, copied `calib.json` / `rig.json`
- `stills/` — every referenced blob, under its sha256 name
- `tape.json` — from `gateway_call` events
- `trajectories/<plan_id>.json` — copied from `state/out/`
- `telemetry/`, `oracle/` — the physical evidence
- `labels.json` — triage verdict + signature

**Completeness gate**: every `$blob` resolves, every `move_arm` has a tape entry, rig + calib present, and (for geometry failures) at least one oracle doc. Missing evidence → the bundle is marked `incomplete` with the reason and is **not exported**. A failure without post-move evidence is unreproducible by construction; say so instead of shipping a half-bundle.

### 6.3 Two replay modes in `tests/agent_eval.py`

New flag `--trace <bundle-dir>`, with two sub-modes:

**a) Deterministic replay — `--replay` (no LLM, bit-exact ambition).**
Reconstruct `Session` from the bundle manifest (its `--calibration/--extrinsics/--plane-z/--tip-mm/--min-z/--hover-m` args map 1:1 to manifest fields), start `MockEdge` with the tape, then feed the **recorded** `tool_call` sequence through `session.call()` in order. Diff each result against the recorded `tool_result`:

- rounded fields (`round3`) exact; other floats `atol=1e-3`; plan summaries exact (they come from the tape);
- compare the **shared numeric fields** of page and harness results — prose fields (`note`) differ by construction and are ignored;
- the comparator diffs only fields the harness reproduces; page-only fields (the reach band, computed in `web/arm-model.js`) are excluded and listed as `uncovered` in the report, so they cannot raise false diffs;
- mismatch classes: `geometry-diff` (host code computes different numbers — the bug, or the fix), `tape-miss` (environment not fully recorded), `order-diff`.

Output `replay-report.json`. This mode proves the environment reconstructs, and after a code change it shows *exactly which numbers moved*. It is the reproduction command for geometry bugs and needs no model endpoint at all.

**b) Live-model replay — `--trace <bundle> --mock-edge --runs N`.**
Seed the conversation from the bundle's first `user_message` + frame blob; use the **recorded system prompt and tool schemas** (from the first `model_turn`), with the existing drift check against `live.html` (:1005–1009) still enforced. `capture_view` answers with the bundle's subsequent `frame_delivered` blobs in order; post-execution notes as in `run_once` (:661–667). Judge with auto-derived `expect{}`:

- protocol failures → direct translations (`plan_attempts_min: 2` for gave-up-after-refusal, `honest: true`, `plan_first: true`);
- geometry failures → the oracle recomputation (frozen rig → tips → residuals vs recorded), not pixel boxes;
- perception failures → `--scaffold`-style case with zero boxes for a human to annotate (the existing growth flow, docs "Growing the set").

Classification: **stable fail** (N/N, same signature) → export; **flaky** → separate bucket, not a ticket. Temperature 0 and `enable_thinking: false` as today (:998 comment) for speed and repeatability.

### 6.4 Export: `state/traces/export/<case-id>/`

```
case.json          # agent_cases-compatible: id, user, still, expect{}, needs_mock_edge: true,
                   # plus bundle-relative "tape": "tape.json", "oracle": "oracle.json"
stills/ calib/ tape.json trajectories/ trace.jsonl
report.md          # symptom, triage problems, layer attribution, expected-vs-actual numbers,
                   # versions (git commit, live.html sha, model, rig/calib fingerprints), repro:
                   #   python tests/agent_eval.py --trace export/<id> --replay
                   #   python tests/agent_eval.py --trace export/<id> --mock-edge --runs 3
```

`load_case`/`still_for` (:199–223) extend to resolve bundle-relative paths. The coding agent's loop: read `report.md` → fix → repro command passes → the case graduates into `tests/agent_cases/` as a permanent regression test. Every production failure grows the eval set — the repo's stated intent.

### 6.5 Dedup and minimization

- Group `labels.jsonl` entries by `signature`; keep one canonical case (shortest window, fewest frames), record `occurrences: N` and link the duplicates.
- **Minimize deterministic/geometry cases automatically**: delta-debug the turn sequence — drop turns/markers while `--replay` still reproduces the failure signature (residual over threshold). Model-behavior cases are not auto-minimized (each check costs live runs); they ship the full window.
- Re-export on code-version change: a signature includes the code version, so a fix that changes behavior retires the case instead of silently re-matching it.

## 7. Phase 4 — Physical hillclimb (gated, on the bench)

New module `app/identify.py` + routes, consuming the oracle/telemetry corpus. The repo already contains the building blocks (tag-based rig capture → verify → commit, the gravity-sag fixed point `aim`, the witness trigger); this phase makes calibration continuous, active, and identifiability-aware.

### 7.1 Triggers (propose, never auto-run)

A monitor (periodic task in `capture_server`, or `tools` CLI) watches: witness `drift_px` over limit; EWMA of oracle `pixel_error_mm` over threshold; tag reprojection error growth; missing/stale rig. A trigger **proposes** an identification session; running it requires `POST /identify/run {confirm: true}` with `app["execute"]` on, single-flight via the existing `app["calibrating"]` flag, progress via `GET /identify` (same shape as `GET /calibrate`, :1530).

### 7.2 Identifiability gate (the hard constraint)

The camera rig is fitted from the base tag, which is **arm-independent** — so camera extrinsics are never coupled to joint zeros or the URDF (the *"model becomes part of the calibration"* trap is avoided by construction). Rules:

- **Camera rig:** trust the tag PnP only when its geometry is good — `verify_marker` gates (rms ≤ 2 px, tag side ≥ 16 px, grazing ≤ 75°). A weak solve (small or grazing tag) is **refused, not averaged in**; report reprojection rms and grazing so a bad rig is visible rather than silent.
- **Joint zeros** are fixed at deployment (zero-calibration) and are *not* climbed from camera data — the tag cannot observe them. Zero drift is detected by the oracle (§5.4) and flagged for re-zeroing, never silently absorbed into the rig.
- **The sag map and accuracy map** are the learned quantities, and they need pose/load excitation: a sag direction with too few holds, or an unvalidated map cell, is reported as *unlearned*, never extrapolated.
- Every update reports what was identifiable and what wasn't; a partial result is reported as partial, never as success.

### 7.3 What gets climbed

**(a) Camera rig + tip offset** — tag-based, no calibration motion:

- Extrinsics from the base-tag single-capture PnP (`POST /capture/pose` → `solve_marker`, `method: "base-marker"`): one still that sees the printed tag on the base, **no arm motion**, arm-independent. Repeat captures and average the PnP estimates to cut noise; each must pass `verify_marker`.
- Tip offset is a **known tool parameter** (`REBOT_TIP_MM`), not fitted; joint zeros come from deployment zero-calibration (§7.2).
- Write `extrinsics-candidate-<ts>.json` → **verify before hot-load**: `verify_marker` gates (rms ≤ 2 px, side ≥ 16 px, grazing ≤ 75°) **plus a held-out capture's tag reprojection**. Pass → `commit_marker` + `attach_witness` + the existing hot-load path (:1791–1809). Fail → refuse with reasons, old rig stays.
- The tag prior is coarser than a multi-pose fit (~1% depth at 1 m for a 4 cm tag); the **measured** accuracy layer (§7.4) does the heavy lifting, and the oracle catches rig errors regardless of the prior.

**(b) Sag / gain residual map** — learned from execution telemetry, no extra motion:

- Per hold window: `sag_meas[j] = cmd[j] − meas[j]`; residual `r[j] = sag_meas[j] − gravity_sag(q)[j]` (the gravity-sag model).
- Fit per-joint affine correction `r[j] ≈ a_j·tau_g_j(q) + b_j` (huber, robust to stalls/outliers). Gate: ≥30 holds spanning the workspace; hold-out validation; report rms in mrad and mm at the tool.
- Store `state/calib/sag-<rig-fingerprint>.json` `{version, kp_key, per_motor:{a,b}, samples, rms_mrad, validated_mm, envelope}`. Apply inside the gravity-sag fixed point `aim` as an additive term, still clipped by `SAG_LIMIT_RAD`. Invalidate on rig-fingerprint, kp, or zero change (keyed).
- This attacks the guide's measured 30–40 % model gap at j2/j3; it interpolates within the explored region and is **not trusted outside it** (envelope below).

**(c) `plane_z` / surface** — deferred, optional: touch-down probing via `/move` with the edge's follow-error/stall signature as contact detection. Spec'd last because it is the only component that intentionally contacts the environment; it needs its own clearance review.

### 7.4 The accuracy envelope (`state/calib/accuracy-map.json`)

A grid over radius × height × angular sector. Per cell: `predicted_mm` (tag-rig prior), `measured_mm` (oracle holds), `n`, `validated_at`. A cell is *validated* only with `n ≥ 3` measured holds under threshold.

**Populating the measured layer:**

- *Passive:* every task execution contributes oracle holds to the cells it visits; frequently-used regions validate first ("grows with the bench").
- *Active (coverage mode):* `POST /identify/run {confirm: true, mode: "coverage"}` moves the arm through a safe pose grid targeting unvalidated cells — one `/move` trajectory with holds (`move_sequence`), poses filtered by `pose_ok` / `path_safe`, speed capped by `seconds_for` / `PEAK_SPEED` (0.35 rad/s; the gateway refuses faster requests, :325–334). The oracle measures each hold and validates cells. This is controlled motion to *gather measurements*, not a rig-fit procedure — coverage of the grid, not excitation of a fit.

`/sequence` consults the map: targets in unvalidated cells get a warning in the plan summary (visible to the agent and the panel); hard refusal stays where it is today (reach band, min_z, IK). `/health` reports the validated fraction. The guarantee is a **measured map with explicit uncertainty**, not "any object, accurately".

### 7.5 Limits stated up front

- Repeatability floor (~23.6 mm with vendor gains; friction on j1 is not gravity and no map predicts it) bounds what climbing can achieve.
- Perception errors are a *different* hillclimb (eval-set driven, §6); attribution (§5.4) gates this loop so geometry updates never chase a wrong pixel.
- Model-form errors outside the residual map's feature span, occlusion, grazing views, transparent/deformable objects: reported as envelope boundaries, not chased.
- Every excursion is a gated arm motion: consent, single-flight, speed caps, `path_safe`, edge follow-error/stall aborts. Candidates never hot-load before verification.

## 8. Cross-cutting concerns

### 8.1 Safety
- The collector and replay are **read-only / mock-only**. Replay always goes through `MockEdge`; `move_arm(execute=true)` in the harness never moves a real arm (existing guarantee, agent_eval.py:513–516). No new path to `/execute` is added outside the existing gates.
- The hillclimb is the only arm-moving addition, and it reuses `/move`'s full check chain (`command_ok`, `path_safe`, speed clamp, edge follow-error/stall abort). It adds consent + single-flight + verify-before-hotload on top.
- Blob/frame writes and telemetry writes are off the realtime path; the edge log write happens after playback (as today).

### 8.2 Privacy / secrets
- `trace.py` redacts `token|authorization|api_key` recursively at write time; only the LLM URL *host* is stored. `X-Rebot-Token` never enters an event.
- Frames are kitchen scenes: retention default 30 days, `export/` exempt, blobs pruned when unreferenced. A `--no-trace` flag disables collection entirely for sensitive deployments.
- `state/` is already gitignored; nothing here changes that. Export packages are the only artifacts meant to leave the machine, and they carry frames — treat them as sensitive.

### 8.3 Drift guards
- Tool-schema drift: existing startup check (agent_eval.py:1005–1009) stays; `manifest.tool_schemas_sha256` pins the contract per session so a replay against a changed page is flagged, not silently run.
- Rig/calib drift: bundles copy the **file bytes**, not just fingerprints, so a later refit cannot retroactively change a replay.
- Code-version drift: `signature` includes the git commit; a fix retires matching cases rather than re-matching them.

### 8.4 Failure of the collector itself
- Tracing must never break the chat or the arm path. All recorder calls are wrapped: on any exception, log a warning and continue (the request proceeds untraced). A full disk or a bad blob write degrades to "no trace for this session", never to a failed `/sequence` or `/execute`.
- `/trace` is fire-and-forget from the page (batched, `sendBeacon` on unload); a slow or down collector drops events, it does not block `runAgent`.

## 9. Test plan and rollout order

Each phase is independently shippable and testable offline. Build in this order; each step's gate is the previous step's tests staying green.

**Phase 1 (collector)** — no hardware, no model.
- Extend `tests/selftest.py`: `TraceRecorder` append/order/blob-dedup/redaction; manifest snapshot; retention prune. Pure unit, no server.
- Extend `tests/browser_check.py`: drive the sidebar, assert `events.jsonl` contains `user_message`, `tool_call`/`tool_result`, `frame_delivered`, and that a compaction event is logged without losing prior turns.
- Mock-only end-to-end: `MockEdge` + a scripted conversation → assert a complete, replayable event log.

**Phase 2 (oracle)** — edge change needs a bench once; everything else offline.
- Edge: `--check` still passes; a recorded `rebot-trajectory-log` v2 parses; hold-window samples preserved on downsampling. Verify on the reference arm that `/play` now returns `telemetry`.
- `app/oracle.attribute`: table-driven unit tests, one per residual signature in §5.4, using synthetic oracle docs (no arm).
- `trace_triage`: feed canned event logs (protocol fail, geometry fail, limitation, perception-suspect) → assert class/layer/signature.

**Phase 3 (replay/export)** — fully offline.
- `mock_arm` tape mode: hit/miss/FIFO-queue semantics; `fail_first` precedence.
- `agent_eval --replay`: a bundle replays bit-exact against unchanged code; a deliberately perturbed geometry constant produces a `geometry-diff`, not a pass. **This is the core regression: the replay must fail when the code is wrong.**
- The replay is also the mechanism that surfaces latent harness/host defects (a tool that errors on every run, a wrong projection): they appear as diffs or tool-errors in `replay-report.json` and are handed to a coding agent like any other failure — no pre-enumeration needed.
- Export → `load_case` round-trips; a graduated case runs under plain `--mock-edge`.

**Phase 4 (hillclimb)** — gated bench work, last.
- `identify` identifiability gate: unit tests — a weak tag solve (small/grazing) is refused, not averaged in; sag directions with too few holds are flagged unlearned, not extrapolated.
- Sag-map fit: synthetic telemetry with a known injected residual → recover `a_j, b_j` within tolerance; hold-out validation rejects an overfit map.
- Verify-before-hotload: a candidate that fails the rig verify gates must leave the running rig untouched.
- Only then run a real identification session on the bench, with consent, and confirm the accuracy map populates from oracle holds.

**CI**: Phases 1–3 run in the existing offline suite (`selftest.py`, `agent_eval.py --mock-edge`). Phase 4's bench steps are manual/gated, never CI.