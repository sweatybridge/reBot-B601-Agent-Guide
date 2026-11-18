# RebotCapture — native iOS client for the B601 phone capture page

A native iOS client for [`examples/host/web/phone.html`](https://github.com/sweatybridge/reBot-B601-Agent-Guide/blob/main/examples/host/web/phone.html)
from the reBot B601 Agent Guide: the phone is a calibrated sensor, taps on the
preview become tool-tip points in the arm base frame, and **Move arm** plans and
runs the numbered marker sequence through the host's capture server.

The camera and pose pipeline runs on **ARKit**; the arm geometry and the host
protocol are a line-for-line Swift port of the page's JavaScript, cross-validated
against it in CI. No WebView.

## ARKit architecture

ARKit replaces the page's per-frame ArUco pose loop, its optional 15-state
IMU/EKF, the per-phone checkerboard calibration, and the quarter-turn
bookkeeping:

- **Pose**: `ARSession` world tracking (visual-inertial odometry) gives a 6-DoF
  camera pose at frame rate in an arbitrary world frame. One ArUco solve ties
  that world to the arm base:

  ```
  T_base_world = T_base_cam(tag solve) × inverse(T_world_cam(same ARFrame))
  T_base_cam   = T_base_world × T_world_cam(live)      ← every frame
  ```

  The tag may leave the view after anchoring; tracking carries the pose. A
  re-anchor runs every 2 s while the tag is visible (and on demand via the
  **Re-anchor** chip) to correct VIO drift. Taps are refused while tracking is
  limited or before the first anchor.
- **Intrinsics**: `ARCamera.intrinsics` is factory-calibrated per frame — no
  checkerboard ritual. Trade-off: pure pinhole, no Brown-Conrady distortion
  terms, so edge-of-frame accuracy can be a few pixels worse than a fitted
  calibration on a wide lens. Calibration mode is kept for host-side workflows.
- **Orientation**: `ARFrame.displayTransform` maps image pixels to the view for
  any interface orientation; the page's pinned-geometry / `stream_turn`
  machinery is not needed on the phone. `/orientation` is still published for
  host compatibility, and captures carry the interface angle.
- **Preview**: an `ARSCNView` sized to the oriented image's aspect ratio, so its
  aspect-fill is exactly the page's letterboxed `object-fit: contain`.
- **WHIP**: ARKit's YUV frames feed the WebRTC video source directly, with an
  upright rotation for the capture orientation.
- **Depth anchor (LiDAR, no tag)**: when the tag is not visible, the arm itself
  is the landmark. Tap the arm base (unprojected through scene depth), hold the
  arm still while the host's joint angles are read twice, and the scene mesh is
  fitted to the FK capsule model (`ArmDepthFit`, the host's `sweep.RADII`
  table). Gravity alignment reduces the unknown to (x, y, z, yaw); a robust
  Gauss-Newton fit with multi-start yaw solves it. Refusal gates: residual
  ≤ 2 cm, ≥ 60 inliers across ≥ 3 links, a yaw-observability probe (perturb
  heading, re-optimize translation, require the cost to rise), and a leash to
  the tapped point. Coarse by design (~1–3 cm); a tag re-anchor supersedes it.

Requirements: A9-or-later device (ARKit world tracking), iOS 16+, a textured,
reasonably lit scene. The focus/exposure-lock chip is gone — ARKit owns the
camera.

## What is implemented

| phone.html | RebotCapture |
|---|---|
| `getUserMedia` preview, letterboxed | ARKit `ARSCNView`, letterboxed to the oriented frame |
| Per-frame `aruco.js` pose + 3 s loss grace | ARKit VIO at frame rate; ArUco anchors/re-anchors the base frame |
| (no equivalent — tag required) | LiDAR depth anchor: fit the arm's capsule model to the scene mesh, with refusal gates |
| Optional `?filter=ekf` 15-state EKF | Subsumed by ARKit visual-inertial tracking |
| Checkerboard intrinsics per phone/zoom | `ARCamera.intrinsics` (pinhole); calibration mode kept as an option |
| Pinned geometry + quarter turns + `/orientation` | `displayTransform`; orientation still published for the host |
| `locate.js` ray ∩ (plane_z + height) tip solve | `RebotCore.Locate.tipTarget` with ARKit intrinsics, per-marker frozen height |
| `arm-model.js` FK / DLS IK / reach band | `RebotCore.ArmModel`, same chain, seeds and gates |
| `reach.js` band overlay | SwiftUI Canvas overlay following the live ARKit pose |
| `markers.js` numbered pins, live reprojection | Amber odd / green even pins, red outline beyond 0.75 m, 10 max |
| `height.js` tap-vs-drag, 5 mm steps, floor | Same slop (10 px), gain (1.5 mm/px), range from `/health` `min_z` |
| `/health` 5 s loop, mode seed, rig fingerprint | Same, including config invalidation on a new rig |
| Camera chip handheld ⇄ fixed rig | Same; fixed uses `T_base_cam` from `/reach.json`, no anchor needed |
| Calibration mode → `POST /capture` | Multipart meta + JPEG (0.95) from the ARKit frame |
| Move arm → `/sequence` → `/execute` | Same flow, plan summary in the log, execution gate respected |
| WHIP live preview (`/whip`, DELETE on stop) | WebRTC (stasel/WebRTC SPM), ARKit frames with upright rotation |
| Token in URL/localStorage | Settings sheet, persisted; `X-Rebot-Token` on every request |
| Install `tls/ca.pem` as a profile | Import the CA in Settings; TLS evaluated against it in-app |
| Screen wake lock | `isIdleTimerDisabled` while the camera runs |

The agent sidebar and Rig fitting belong to the workstation's `/live` page and
are intentionally not part of the phone client.

## Build

Requirements: macOS with Xcode 15+ and an iOS 16+ device with an A9+ chip (the
camera, ARKit and WebRTC do not work in the simulator).

From a fresh clone, generate the Xcode project and the test fixtures once:

```bash
brew install xcodegen node
xcodegen generate
git clone --depth 1 https://github.com/sweatybridge/reBot-B601-Agent-Guide /tmp/guide
node tools/gen_fixtures.js RebotCore/Tests/RebotCoreTests/Fixtures /tmp/guide
open RebotCapture.xcodeproj
```

(The release zip ships with the project and fixtures already generated.)

1. Select your signing team for the `RebotCapture` target.
2. Let Swift Package Manager resolve `WebRTC` (first fetch downloads a ~100 MB
   binary framework; `branch: main` is used — pin it to a release tag in
   `project.yml` if you prefer reproducibility).
3. Run on the iPhone.

The project was generated from `project.yml` with [XcodeGen](https://github.com/yonaskolb/XcodeGen);
after editing the spec, regenerate with `xcodegen generate`.

## Connect to the host

Start the host stack exactly as the guide describes (`examples/host/compose.yaml`,
`make_cert.py`, base tag or fitted rig). Then in the app's Settings:

- **Host**: `https://<REBOT_HOST_IP>:8443`
- **Token**: the capture server's `--token`
- **TLS**: import the host's `tls/ca.pem` (AirDrop it to the phone, then pick it
  in Settings). No configuration profile or certificate-trust toggle needed —
  the app anchors trust to that CA itself.

The dot turns green when `/health` answers. From there: Start camera → point at
the base tag once to anchor → tap markers (drag for height) → Move arm.
`REBOT_EXECUTE=1` on the host and `--allow-execute` on the edge are still
required for the arm to actually move; otherwise plans are written to `out/`
only and the log says so.

## Layout

```
RebotCore/            pure-Swift port of the page's math (no UIKit), unit-tested
  Sources/RebotCore/
    Matrix.swift          geometry.js + aruco.js vector/matrix/rodrigues helpers
    CameraGeometry.swift  geometry.js: K scaling, Brown-Conrady, ray/plane, turns
    ArucoDetector.swift   aruco.js: threshold → components → hull → decode → subpixel
    ArucoPose.swift       aruco.js: homography/physical seeds, Gauss-Newton, mirror gate
    ArmModel.swift        arm-model.js: URDF chain, FK, DLS IK, reach band scan
    ArmDepthFit.swift     LiDAR base anchor: capsules, segmentation, robust 4-DoF fit
    Locate.swift          locate.js: tipTarget / solveTag
    APIModels.swift       /health, /reach.json, /sequence, /execute wire types
  Tests/                cross-validation against the real JS (see below)
RebotCapture/         the SwiftUI + ARKit app
  AppModel.swift        host, camera, anchor state, markers, height, log
  AppModel+Reach.swift  anchoring/re-anchor loop, tap locate, reach band, projection
  AppModel+Depth.swift  depth-anchor flow (tap base, read pose, fit mesh)
  CameraController.swift  ARSession, intrinsics, Y-plane gray, JPEG, displayTransform
  APIClient.swift       api.js + CA-anchored TLS
  WhipPublisher.swift   WebRTC WHIP publish of ARKit frames
  ContentView / StageView / SettingsView / Theme
tools/gen_fixtures.js  regenerates test fixtures from the guide repo's web JS
```

## Continuous Integration (GitHub Actions)

[`.github/workflows/mobile.yml`](../../.github/workflows/mobile.yml) defines two
jobs, run on pushes and pull requests that touch `examples/mobile`, the web JS
it is validated against (`examples/host/web`), or the workflow itself:

| job | runner | what it proves |
|---|---|---|
| `core-tests` | Linux, `swift:6.0` container | the ported math matches the page's JS (fixtures are regenerated from the checked-out commit each run, so they never go stale) |
| `ios-sdk-tests` | GitHub-hosted `macos-15` (Xcode 16) | the **full app compiles against the iOS SDK** (simulator) and the same suite passes as an Xcode test bundle (`RebotCaptureTests`) |

The iOS job is `continue-on-error: true` (the GitLab original used
`allow_failure`) so the macOS layer never blocks the pipeline — `core-tests`
is the required gate. SPM checkouts (including the WebRTC binary framework) are
cached between runs, and the `.xcresult` bundle is uploaded as an artifact when
the job fails.

Run the iOS layer locally the same way CI does:

```bash
xcodebuild test -project RebotCapture.xcodeproj -scheme RebotCapture \
  -destination 'platform=iOS Simulator,name=iPhone 15' CODE_SIGNING_ALLOWED=NO
```

## Verification

The vision/geometry core was cross-validated against the **actual JavaScript**
the page runs: `tools/gen_fixtures.js` renders synthetic tag scenes, runs the
real `aruco.js` detector and pose solve, `geometry.js`, `arm-model.js` and
`locate.js` in Node, and writes fixtures. The Swift tests then require the port
to agree:

- projection / undistort / ray-plane / pixel turns: ≤ 1e-9
- FK: ≤ 1e-9; reach bands: identical inner/outer radii on all tested heights
- tag detection corners, `T_base_cam`, rms, and the full detect→refine→pose
  pipeline on three scenes (clean, merged-with-base, oblique)
- `tipTarget` for all four quarter turns
- depth anchor: synthetic LiDAR scenes sampled on the capsules from the real
  JS FK — the fit recovers the known transform (≤ 1.5 cm / 0.04 rad required,
  ~0.3 mm achieved on the clean scene) and refuses the weak-heading and sparse
  scenes

Run them (macOS or Linux, no Xcode needed; generate fixtures first if absent):

```bash
node tools/gen_fixtures.js RebotCore/Tests/RebotCoreTests/Fixtures /path/to/reBot-B601-Agent-Guide
cd RebotCore && swift test
```

The app layer (ARKit, SwiftUI, WebRTC) targets the iOS SDK and needs Xcode to
build; the `ios-sdk-tests` CI job on the GitHub-hosted macOS runner is the
compile gate for that layer.

## Fidelity notes

- **Anchoring, not per-frame tags**: the page re-solves the tag for every
  marker; here one solve anchors ARKit's world and tracking carries the pose.
  Re-anchors every 2 s bound the VIO drift (~1% of distance traveled).
- **Pinhole intrinsics**: ARKit reports no distortion coefficients. If a wide
  lens shows edge bias, run the host's checkerboard calibration and compare;
  the ArUco anchor solve uses the same pinhole model, so anchor and taps stay
  mutually consistent.
- **Depth anchor is a fallback**: ~1–3 cm class accuracy, LiDAR-only, arm must
  hold still; the fit refuses rather than guesses (same philosophy as the rig
  sweep). The tag and the fitted rig remain the precision paths.
- **Safety**: nothing moves without Move arm; the host's `min_z` floor clamps
  the height badge; tips below the floor are refused by the server; the reach
  band clips to the planner's 0.75 m gate; taps are refused when tracking is
  limited or unanchored.
