---
name: batch-validation
description: >-
  Run multi-strategy physical pick-and-place batch validation on the reBot B601-RS bench,
  grade completion from Postgres-ingested MediaMTX MP4 recordings and oracle hold stills,
  bundle trajectory metadata into a repro zip, stitch browser-ready MP4 recordings, and
  publish a GitHub issue with the zip and individual video attachments via Chrome remote debugging.
  Use when the user asks for batch validation, multi-run physical pick-and-place evaluation,
  grading from recordings, or filing a bench evaluation GitHub issue with repro bundles and videos.
version: 1.1.0
---

# Batch Validation (B601-RS Physical Pick-and-Place)

End-to-end procedure for executing a batch of distinct physical pick-and-place runs on the
B601-RS bench, grading each run visually from its recorded video + telemetry + oracle verdict,
packaging trajectory metadata into one `.zip`, stitching individual `.mp4` recordings, and
filing a GitHub issue with native attachments (modeled on Issues `#10`, `#12`, and `#13`).

> **Review-only mode (already-executed DB runs):** If the user asks to grade runs that are
> already in Postgres, skip Steps 1–2, query `evidence.execution` for the target `exec_id`s
> (verifying each has rows in `evidence.media_fragment`), omit `after_image` in
> `batch_manifest.json`, and jump straight to Step 3.

## Environment & Known Host Quirks

- **Containers**: `rebot-capture-capture-1` (`:8443`), `rebot-capture-activities-1`,
  `rebot-capture-planner-1`, `rebot-substrate` (Postgres `:5433`), `rebot-capture-mediamtx-1` (`:8554`).
- **Python CV/DB work**: Always run inside `rebot-capture-capture-1` via `docker exec`.
  The Windows `.rebot-venv` has broken native DLLs for `cv2` and `aiohttp`.
- **Video stitching**: Use host `ffmpeg` (`scripts/stitch_recordings.ps1`) — the capture container
  does not ship the `ffmpeg` CLI.
- **GitHub uploads**: Use host **Node.js 24+** (`scripts/upload_issue_attachments.mjs`) over
  Chrome Remote Debugging (`%LOCALAPPDATA%\Google\Chrome\User Data\DevToolsActivePort`).
  Always pass `--repo sweatybridge/reBot-B601-Agent-Guide` to `gh` (the repo also has an
  `upstream` remote).
- **Plan expiry**: Plans expire after `600 s`. Always call `POST /sequence` and `POST /execute`
  **atomically in the same script/command** so user approval delays never stale a plan.

---

## Step 1: Pre-Flight & Live Bench Localization

1. Verify containers, arm pose, and capture health:
   ```powershell
   docker ps --format "table {{.Names}}\t{{.Status}}"
   curl.exe -sk https://127.0.0.1:8443/health
   curl.exe -sk https://127.0.0.1:8443/pose
   ```
   Confirm `execute: true`, `mode: "fixed"`, and arm joints `1..7` parked near `0`.
2. Grab a live frame from `rtsp://mediamtx:8554/phone` inside `rebot-capture-capture-1` into
   `/app/work/live_now.jpg`, inspect it with `read_file`, and verify base ArUco tag projection
   error (`< 5 px`; note `/health`'s cached `stream_turn.error_px` can be stale after a rig refit).
3. Project candidate object centers at `z = 0.00`, `0.04`, `0.08 m` onto the image to verify
   vertical-column alignment and reach radii:
   - **Top-down (`tool_axis="x+"`, `entry="vertical"`)**: optimal around `r = 0.35–0.52 m`,
     `vertical_min_z = 0.0`. Because `REBOT_TIP_MM = 35` lifts `flange_point.z` by `+35 mm`,
     commanding `tip.z = 0.005–0.010 m` places fingertips `40–45 mm` above the worktop.
   - **Horizontal (`tool_axis="z-"`, `entry="front"`)**: reaches `r = 0.45–0.74 m`, `min_z = 0.04 m`.
     Watch for lightweight/round objects near table edges on flat `0°` slide-ins; prefer
     elevated `via` waypoints (`z = 0.14–0.18 m`) on Marker 2 to avoid table drag and collisions.

---

## Step 2: Execute Distinct Pick-and-Place Runs

Design 3+ distinct strategies (e.g., top-down vertical, horizontal direct default, horizontal +
elevated `via` arc + tight `close_width_mm`, or pitch-fallback near reach boundaries).

For **each** run:
1. Locate the target from the **most recent** live frame (`/app/work/live_now.jpg` or previous
   `/app/work/after_taskN.jpg`) — never assume prior objects stayed put.
2. In **one atomic `docker exec rebot-capture-capture-1 python ...` call**:
   - `POST https://127.0.0.1:8443/sequence` with `X-Rebot-Token: $REBOT_TOKEN`.
   - Check `status == 200`, record `plan_id`, `duration_s`, `worst_ik_m`.
   - Immediately `POST https://127.0.0.1:8443/execute` with `{"id": plan_id}` (timeout `240 s`).
   - Grab a fresh post-execution frame to `/app/work/after_<key>.jpg`.
3. Inspect `/app/work/after_<key>.jpg` with `read_file` before planning the next run.

---

## Step 3: Extract Recordings, Storyboards, & Repro Metadata Bundle

1. Write a JSON manifest describing the batch runs (example at `/app/work/batch_manifest.json`):
   ```json
   {
     "bundle_name": "repro-20261126-batch-validation.zip",
     "git_commit": "<git rev-parse HEAD>",
     "out_dir": "/app/work/export_batch",
     "runs": [
       {
         "key": "task1",
         "slug": "task1-topdown-vertical-20261126T133700Z-a4aa4d",
         "exec_id": "20261126T133700Z-a4aa4d",
         "strategy": "Top-down vertical (x+, vertical, approach=0.09m, close=20mm)",
         "summary_label": "Top-down vertical, red fish — clean pick/place",
         "grade": "A- (90/100) PASS",
         "after_image": "/app/work/after_task1.jpg"
       }
     ]
   }
   ```
2. Copy or run [`scripts/extract_and_bundle.py`](./scripts/extract_and_bundle.py) inside
   `rebot-capture-capture-1`:
   ```powershell
   Get-Content -Raw skills\batch-validation\scripts\extract_and_bundle.py | docker exec -i rebot-capture-capture-1 python - /app/work/batch_manifest.json
   ```
   This produces in `examples/host/work/export_batch/`:
   - `storyboard_<key>.jpg` (12-panel phase-aligned recording contact sheet per run, sequentially
     decoded to avoid H.264 keyframe seek drift)
   - `oracle_holds_all.jpg` (all oracle-extracted hold stills side-by-side)
   - `<bundle_name>` (single `.zip` containing `summary.json` with `git_commit` and per-run
     `motion_metrics`, `env/{rig_and_calibration,arm_config,sag_maps}.json`,
     `oracle_holds_all.jpg`, and per-run `runs/<exec_id>/{metadata,execution,plan,telemetry,verdict,detect_request}.json`,
     extracted hold stills, storyboard, and optional post-exec still)
   - `raw_frags/<key>/seg_*.mp4` (raw MediaMTX fragments ready for stitching)
3. Visually inspect `storyboard_<key>.jpg`, `oracle_holds_all.jpg`, and `after_<key>.jpg` with
   `read_file` to grade every run honestly:
   - Do **not** trust `grasp_verification` blindly: verify in the recording whether the object was
     actually grasped & placed (vs knocked off the table = false positive, or occluded by the arm
     at Hold 3 = false negative).
   - Check `grid_scale` on every hold: mid-stream WebRTC ramps (`1356x762` -> `1920x1080`) can
     trigger false ~250 mm pixel errors if `oracle_compute` does not rescale `detected_tip_px`.
   - Separate servo tracking error (`tip_error_mm` = meas vs cmd) from net placement error
     (`placement_mm` = meas vs requested `flange_point`) to diagnose sag over-/under-compensation
     and catch inverted oracle verdicts (e.g., `placement_mm < 5 mm` falsely blamed as `sag`, or
     `placement_mm > 15 mm` missed because `tip_error_mm < 8 mm`).
   - **Check 50 Hz smoothness & tracking spikes (`motion_metrics`)**: inspect `jerk_cmd_max_rads3`,
     `acc_meas_max_rads2`, `jerk_meas_max_rads3`, and `track_err_per_joint_max_rad` printed by
     `extract_and_bundle.py`. Flag commanded jerk spikes (`> 150 rad/s³`, e.g., unblended V-corners
     or joint-limit clamps) and measured acceleration/tracking spikes (`acc_meas_max > 25 rad/s²`
     or `track_err_max > 0.10 rad`, e.g., mechanical binds or release slams) — a visually clean
     pick can still hide a severe mid-carry jerk spike (`f95aff` in `#13`).
   - Check motor-7 (`q[6]`) hold compliance (`meas - cmd` at grasp end) and edge tick jitter.
4. Update `grade` / `summary_label` in `batch_manifest.json` after visual inspection and re-run
   `extract_and_bundle.py --zip-only` if grades changed.

---

## Step 4: Stitch Individual Recordings (<10 MB MP4s)

Run [`scripts/stitch_recordings.ps1`](./scripts/stitch_recordings.ps1) on the Windows host
(use `powershell -File` on Windows PowerShell 5.1, or `pwsh -File` if PowerShell 7 is installed):

```powershell
powershell -File skills\batch-validation\scripts\stitch_recordings.ps1 `
  -ExportDir "d:\work\reBot-B601-Agent-Guide\examples\host\work\export_batch" `
  -Manifest "d:\work\reBot-B601-Agent-Guide\examples\host\work\batch_manifest.json"
```

This normalizes mixed-resolution segments (`scale=1280:720:flags=lanczos,setsar=1,fps=30`) into
browser-playable H.264 `yuv420p` `+faststart` `.mp4` files (typically 3–5 MB each, safely below
GitHub's 10 MB attachment cap).

---

## Step 5: Create GitHub Issue & Upload Attachments via Chrome CDP

1. Write the graded report + prioritized areas of improvement (with exact `exec_id`s, metrics,
   and file/line citations) to a temporary markdown file and create the issue:
   ```powershell
   gh issue create --repo sweatybridge/reBot-B601-Agent-Guide `
     --title "Pick-and-place batch validation (YYYY-MM-DD): <key findings>" `
     --body-file "<path-to-body.md>"
   ```
2. Upload the single metadata `.zip` and each individual `.mp4` recording directly to the new
   issue using [`scripts/upload_issue_attachments.mjs`](./scripts/upload_issue_attachments.mjs)
   over Chrome Remote Debugging (`chrome://inspect/#remote-debugging`, `DevToolsActivePort`):
   ```powershell
   node skills\batch-validation\scripts\upload_issue_attachments.mjs `
     --issue <ISSUE_NUMBER> `
     --manifest "d:\work\reBot-B601-Agent-Guide\examples\host\work\batch_manifest.json" `
     --export-dir "d:\work\reBot-B601-Agent-Guide\examples\host\work\export_batch"
   ```
   - If Chrome shows an **"Allow remote debugging?"** prompt, tell the user to click **Allow**.
   - The script opens the issue description editor (`...` kebab -> `Edit`), intercepts the file
     chooser via `Page.setInterceptFileChooserDialog` + trusted `Input.dispatchMouseEvent`,
     uploads the `.zip` and every `.mp4` one by one until `https://github.com/user-attachments/...`
     URLs resolve, appends formatted `### Repro Data Bundle` and `### Source Recordings` sections
     matching Issues `#10` & `#12`, and clicks **Save**.
3. Verify the persisted issue body:
   ```powershell
   gh issue view <ISSUE_NUMBER> --repo sweatybridge/reBot-B601-Agent-Guide --json body -q ".body" | Select-Object -Last 30
   ```
   Confirm zero `Uploading ...` placeholders remain and every attachment URL is present.
