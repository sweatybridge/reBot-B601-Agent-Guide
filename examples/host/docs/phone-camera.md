# Phone camera -> host trajectory

A phone in the browser is a camera the kitchen already owns. This turns one
still from it into a point in the arm's base frame, then into a joint trajectory
the Pi replays -- the same `rebot-joint-trajectory` file as
`plan_with_pinocchio.py`, with the same safety gates.

The design is two planes, not one video pipe:

| | preview (aiming) | capture (planning) |
|---|---|---|
| what | 640x360 - 1280x720, 15-30 fps | one full-resolution JPEG |
| how | WebRTC (WHIP) through MediaMTX | `POST /capture` |
| needs | best effort | calibration + tag + frozen arm |
| kept | no | yes, next to the trajectory |

Video is for the person holding the phone. The planner only ever sees an
immutable frame whose intrinsics, extrinsics and arm pose are all known, which
is what makes a plan reproducible and a failure debuggable.

```
phone page (tap) ─┐
                  ├─ grab the frame under the tap ─► /capture
/live page  (tap) ─┘                                      │
                                                          ▼
                                   capture server: locate_target.py
                                   calib + ArUco PnP + table plane
                                                          │
                                                          ▼
                                   arm gateway (planner container)
                                   edge /pose ─► Pinocchio ─► out/<id>.json
                                                          │
                             [Move the arm] ──────────────┴─► POST /play ─► rebot-edge
```

## First, prove it on your machine

```bash
# in the compose stack (nothing else needed)
docker compose -f examples/host/compose.yaml --profile tools run --rm tools tests/selftest.py

# or from a venv, see "Or run it on the host directly" below
python examples/host/tests/selftest.py
```

43 checks, no phone and no arm: a synthetic camera calibrates itself, a rendered
tag recovers a known 3D point to a fraction of a millimetre, a capture round-trips
through the real HTTP server, and the trajectory payload is checked against the
player's own rules. If this fails, the pipeline is wrong, not your measurement.

The page is the one part that needs a browser. `browser_check.py` runs it in
headless Chrome: it substitutes a canvas for the camera, taps a known pixel,
captures once, and checks the stored tap and image size:

```bash
# with the capture server already running
chrome --headless=new --remote-debugging-port=9333 --ignore-certificate-errors \
    --user-data-dir=/tmp/rebot-chrome about:blank
python examples/host/tests/browser_check.py --connect 9333 \
    --url "https://127.0.0.1:8443/?token=test-token"
```

It checks the page, not the lens: focus, exposure and intrinsics are still the
phone's problem.

## Run it in containers

The whole host side is one compose file: the capture server, MediaMTX, the arm
gateway (Pinocchio, talking HTTP to `rebot-edge serve`) and a one-shot `tools`
container (certificates, calibration, tags).

```bash
cd examples/host
cp .env.example .env            # REBOT_HOST_IP, tag placement, REBOT_EDGE_URL
docker compose up -d --build
docker compose --profile tools run --rm tools app/make_cert.py --host rebot-host
docker compose restart capture
```

| service | what | published |
|---|---|---|
| `capture` | the HTTPS endpoint the phone opens; tap -> capture -> plan | `8443/tcp` |
| `mediamtx` | WebRTC relay: phone in, `/live` out | `8189/udp` |
| `planner` | the arm gateway: edge `/pose`, Pinocchio planning, optional execute | nothing |
| `tools` | `calibrate_intrinsics.py`, `make_tag.py`, `make_cert.py`, `selftest.py` | nothing |

Everything after that is a tap or a command:

```bash
docker compose --profile tools run --rm tools app/calibrate_intrinsics.py
docker compose --profile tools run --rm tools app/make_tag.py --id 0 --size 80
docker compose --profile tools run --rm tools tests/selftest.py
```

Four things that matter:

- **The phone only ever talks to `:8443`.** MediaMTX's signaling is reached
  through the relay inside `capture_server.py`, so `8889` is not published; only
  the media port `8189/udp` is, because after the handshake the phone sends RTP
  straight to `<REBOT_HOST_IP>:8189`.
- **`REBOT_HOST_IP` must be the host's LAN address**, not the container's. It is
  the ICE candidate MediaMTX advertises, and it is what the certificate is
  issued for. A container address works for nothing but itself.
- **`captures/`, `calib/`, `tls/`, `work/` and `out/` are bind mounts**, so
  `docker compose down` does not lose them.
- **Without a certificate the stack still starts**, on plain http, and says so.
  The phone will refuse the camera until `make_cert.py` has run. That is
  deliberate: `compose up` should not crash-loop before the CA is installed.

The planner image is built from `examples/host/Dockerfile.pinocchio` (Pinocchio
has no Windows wheels) by `docker compose build`, about two minutes once. The
gateway needs `REBOT_EDGE_URL` to point at the board's `rebot-edge serve`;
without a reachable edge the stack still plans from taps but every plan fails at
the pose, which is what the panel will say.

In the commands that follow, `python examples/host/app/<script>.py` is either
the venv form or, in containers,
`docker compose --profile tools run --rm tools app/<script>.py`.

## Or run it on the host directly

The same code runs from a host venv; this is the shorter loop while editing the
page or the geometry, because it skips a rebuild.

```bash
uv pip install -r examples/host/requirements.txt
```

`opencv-contrib` is required: `cv2.aruco` is not in the plain build.
`plan_to_target.py` is not in that list; it runs in the Pinocchio container.

Download a MediaMTX binary from <https://github.com/bluenviron/mediamtx/releases>
and run it with the config next to this file:

```bash
mediamtx examples/host/deploy/mediamtx.yml
```

## TLS: the phone will not open the camera without it

`getUserMedia` only exists in a secure context, and `http://<lan-ip>` is not one.

```bash
# compose: REBOT_HOST_IP from .env becomes the certificate's IP
docker compose --profile tools run --rm tools app/make_cert.py --host rebot-host
docker compose restart capture

# host venv
python examples/host/app/make_cert.py --host rebot-host --ip 192.168.1.23
```

Install `tls/ca.pem` on the phone once:

- **iOS**: AirDrop or open the file -> Settings > General > VPN & Device
  Management > install the profile -> Settings > General > About > Certificate
  Trust Settings > enable it.
- **Android**: Settings > Security > Encryption & credentials > Install a
  certificate > CA certificate.

`docker compose up -d` already started the server with those files. From a venv
it is:

```bash
python examples/host/app/capture_server.py \
    --cert examples/host/state/tls/cert.pem \
    --key  examples/host/state/tls/key.pem \
    --token pick-something
```

Open `https://192.168.1.23:8443/?token=pick-something` on the phone. The URL
token is remembered in `localStorage` and stripped from the address bar. If the
host's IP changes, `make_cert.py --reuse --ip <new-ip>` mints a new leaf; the CA
stays installed.

## Print the tag and tape it down

```bash
python examples/host/app/make_tag.py --id 0 --size 80
```

Print at 100 % (no "fit to page"), then **measure the black square** and pass the
measured value as `--tag-size`. `--tag-size` is the side of the black square,
border included -- not the sheet and not the white margin.

Tape it flat on the worktop where the phone sees it **in the same frame as the
target**, and note where it sits relative to the arm base: `--tag-x`, `--tag-y`
(from the base's origin, metres) and `--tag-yaw` (degrees). Measurements are
easier than they sound: the base origin is the centre of the J1 axis, and the
tape measure is the calibration.

The tag supplies the extrinsics on every frame, so a handheld phone is fine.
Which means the rule that matters: **the tag must be visible in the same frame
as the target**. If the phone moves after the pose was solved, the target moves
with it.

## Calibrate once per phone camera

Take 15-20 shots of a printed checkerboard with **Calibration mode** on: tilt it,
put it in the corners, fill the frame, then pull back. Then:

```bash
python examples/host/app/state/calibrate_intrinsics.py \
    --captures examples/host/state/captures --cols 9 --rows 6 --square 0.025
```

It writes `calib/<device>.json`. The RMS gate is 1 px; drop views above 2 px
with `--drops`. A fit at 0.5 px on 12 good views beats 1.5 px on 30 sloppy ones,
and a bad `K` shows up later as a target that is "always 30 mm to the left".

Calibration is per camera, per resolution, **and per zoom**. The page reports
`zoom` when the browser exposes it; if it is not 1.00, recalibrate at that zoom
or don't zoom. `calib.load()` rescales `K` between resolutions of the same aspect
ratio and refuses to cross a crop.

## Tap the object

Tap the object in either viewport -- the phone page or the workstation's `/live`
page. The tap is a complete transaction:

```
tap (u,v)
  -> the page grabs that exact video frame to a JPEG     (/capture)
  -> capture_server runs locate_target.py on it          (/plan)
  -> arm gateway reads GET /pose, plans, writes           (planner:8000)
  -> the panel shows target, waypoints, worst IK, peak tau
  -> [Move the arm] -> two-step confirm -> gateway scp + rebot-edge --execute
```

Nothing is typed and nothing is reordered by hand. The frame that was tapped is
saved next to its `target.json` and its trajectory, so a bad move can be
replayed and understood afterwards.

- **The tag must be visible in the same frame as the object.** That is the rule
  the whole design rests on: the tag supplies the extrinsics on every tap.
- **Execution is off by default.** `REBOT_EXECUTE=0` means taps produce plans in
  `out/` and nothing moves. Set it to 1 in `.env` and start the edge with
  `--allow-execute` to get the button; it still asks twice.
- **Every plan is also a file.** `out/<id>.json` is the same trajectory the CLI
  writes, so `rebot-edge --check` works on it without the browser.

Two viewers, one module (`viewer.js`): the phone uses its own camera stream, the
workstation uses the WebRTC stream from MediaMTX, and both post the frame they
were looking at.

If you would rather drive it from a shell, the same three steps exist as
commands:

```bash
# 1. capture -> base-frame point, with a picture of what it believed
docker compose --profile tools run --rm tools app/locate_target.py \
    --capture /app/state/captures/<id> --calib /app/state/calib/<device>.json \
    --tag-size 0.080 --tag-x 0.35 --tag-y -0.20 --tag-yaw 15 \
    --annotate /app/state/captures/<id>/annotated.jpg

# 2. point -> trajectory, through the same code the gateway runs
docker compose run --rm --entrypoint python planner /app/app/plan_to_target.py \
    --joints /app/state/work/current.json \
    --target /app/state/captures/<id>/target.json \
    --out /app/state/out/seq.json --tool-axis z-

# 3. the gate as always
./rebot-edge seq.json --check
./rebot-edge seq.json --execute --track
```

### Which axis points down

`--tool-axis` holds the dumped pose's orientation by default, which is the one
the arm is known to reach from. A top-down grasp needs the local axis that points
out of the gripper. Ask the model instead of guessing:

```bash
docker compose run --rm --entrypoint python planner /app/app/plan_to_target.py \
    --joints /app/state/work/current.json --inspect
```

It prints each local axis of `gripper_end` in the base frame and its angle from
straight down; pick the one near 0 deg. `--yaw` then spins the gripper about the
vertical. (On the reference arm at the parked pose, `z-` is 0 deg and everything
else is 90 or 180.) Put both in `.env` as `REBOT_TOOL_AXIS` / `REBOT_YAW`.

The plan starts with a joint-space "unpark": 8 deg on j3 by default, the same
peel `plan_with_pinocchio.py` uses, because the parked pose rests on the j2/j3
lower stops and a Cartesian IK that begins exactly on a stop often refuses to
move. `--unpark-j3 0` disables it; the leg is mirrored at the end of the plan.

## What it can and cannot do

- **In-plane accuracy** is a few millimetres at 0.6-1.5 m with a good `K`; the
  synthetic test measures 0.15 mm, the real phone will be worse.
- **Out of plane** comes from the table-plane assumption (`--plane-z`). It is
  exact for things lying on the worktop and wrong for anything with height: a
  40 mm object is 40 mm off. Approach vertically and let the existing 6 cm close
  pose absorb it, or add a second view later.
- **Grazing views are the failure mode.** A tap whose ray hits the table at 20
  deg turns every pixel into centimetres; `--max-grazing` (80 deg) refuses the
  worst, but aim further down.
- **The phone is not a metrology camera.** Your kitchen manuals say the same
  about the fixed RGB-D; use this for "wipe here", "pick that", not for
  sub-millimetre placement.
- **IK is accepted by the pose it reached**, not by the solver's success flag: a
  result within 2 mm and 0.5 deg is kept (`--ik-pos-tol`), which is well inside
  the arm's own 23 mm repeatability. A miss beyond that is refused with the
  distance it got to, usually because the target is near a reach limit from the
  pose that was dumped.
- **Nothing here is a safety function.** The arm still only moves on
  `--execute`, after `--check`, with the limits re-verified on the edge. The
  gateway's own gate (`REBOT_EXECUTE=1`) and the two-step button are there to
  make an accidental tap harmless, not to make the arm safe.
- **The frame is the one you saw**, decoded in the browser, not a fresh one from
  the server: a tap and its pixels cannot drift apart. The cost is that a tap
  taken while the phone is moving is a few hundred milliseconds old.

## Troubleshooting

| symptom | cause |
|---|---|
| page loads, camera button does nothing | not https, or the CA is not trusted on the phone |
| `getUserMedia` NotAllowedError on iOS | permission denied; Safari > Website Settings > Camera |
| capture works, `locate_target.py` says "no ArUco marker" | tag out of frame, or too small/blurred at that distance |
| "tag reprojection 4 px is above --max-reproj" | phone moved during the exposure, or `--tag-size` is the sheet size, not the black square |
| live preview never connects | phone and host on different VLANs / AP isolation; UDP 8189 blocked |
| "aspect ratio changed" | calibrated at 4:3, captured 16:9; recalibrate at the capture size |
| live preview works, `/live` does not | MediaMTX up but the path has no publisher yet |
| IK failed at the target | target outside the arm's reach, or too close to the base; loosen `--max-reach`, move the phone view |
| compose: page loads but the camera is blocked | no certificate yet (`make_cert.py`), or the CA is not installed on the phone |
| compose: live preview never connects, `/health` is fine | `REBOT_HOST_IP` is empty or is the container's address; set the host's LAN IP in `.env` and restart |
| compose: `pull access denied for rebot-pino` | the planner image is not built; `docker compose build planner` |
| tap: "no ArUco marker in the frame" | the tag is outside the frame the tap came from; aim so the tag and the object are both in view |
| tap: "arm gateway unreachable" | the `planner` service is down; `docker compose ps` |
| tap: "cannot reach the edge" | the board is not running `rebot-edge serve`, or `REBOT_EDGE_URL` is wrong |
| tap: the edge answers 401 | `REBOT_EDGE_TOKEN` does not match `--token` on the board |
| tap: the edge answers 503 on /pose | `can0` is down on the board; see the edge README |
| tap: no "Move the arm" button | `REBOT_EXECUTE=0`; set it to 1 and `docker compose up -d` |

## Files

| file | job |
|---|---|
| `compose.yaml` | the host stack: `capture`, `mediamtx`, `tools`, `planner` |
| `Dockerfile` | the capture server and every one-shot tool, in one image |
| `.env.example` | `REBOT_HOST_IP`, `REBOT_TOKEN`, `REBOT_PORT` |
| `healthcheck.py` | compose healthcheck for the capture service |
| `phone.html` | the page on the phone: preview, tap-to-plan, calibration mode, WHIP |
| `live.html` | workstation viewer: WHEP playback and the same tap-to-plan |
| `viewer.js` | shared tap -> capture -> plan -> execute panel |
| `capture_server.py` | TLS endpoint, `/capture`, `/plan`, `/execute`, WHIP/WHEP relay |
| `planner_server.py` | the arm gateway: edge `/pose`, planning, gated execution |
| `make_cert.py` | local CA + leaf certificate for the phone |
| `make_tag.py` | printable ArUco tag at a measured size |
| `calibrate_intrinsics.py` | checkerboard fit -> `calib/<device>.json` |
| `locate_target.py` | capture -> `target.json` (PnP + table plane), annotated JPEG |
| `plan_to_target.py` | the planner: target + pose -> trajectory JSON, CLI and gateway |
| `traj.py` | payload assembly + the player's schema checks, no Pinocchio |
| `selftest.py` | offline proof of all of the above |
| `browser_check.py` | drives the page in headless Chrome: tap, capture, check the file |
| `mediamtx.yml` | minimal WHIP/WHEP/RTSP config |

`captures/`, `calib/`, `tls/`, `out/` and `work/` are local state; only the code
is committed.
