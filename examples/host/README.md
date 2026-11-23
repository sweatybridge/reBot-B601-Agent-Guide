# Host side

Everything that runs off the board: the phone camera, the planner, and the two
pipelines that turn a pose into a trajectory.

```
phone / workstation  --tap-->  capture_server (opencv)  -->  planner_server (Pinocchio)
                                      |                              |
                                      |  durable execute->oracle     |
                                      v                              v
                              activities (decode/extract/detect)   rebot-edge serve
                              <-- df.http -- Postgres -- HTTP -->  (board, :8081)
```

## What runs where

| service | image | listens on | job |
|---|---|---|---|
| `capture` | `Dockerfile` | `:8443` (LAN) | HTTPS for the phone, `/capture`, `/plan`, `/execute`, WHIP/WHEP relay |
| `mediamtx` | `bluenviron/mediamtx` | `:8189/udp`, `:8554` (RTSP), `127.0.0.1:9997` (API, host loopback) | WebRTC relay: phone publishes, `/live` reads |
| `planner` | `Dockerfile.pinocchio` | nothing | the arm gateway: edge `/pose`, planning, gated `/execute` |
| `activities` | `rebot-capture:local` | `:8766` (compose network) | isolated execute + recording ingest + extract + detect for the durable workflow |
| `tools` | `Dockerfile` | nothing | one-shot: calibration, tag, certificate, selftest |

`docker compose up -d --build` starts the first four; `tools` is a profile.
Paths inside and outside the containers are the same: the project root is `/app`,
the code lives in `app/`, the pages in `web/`, and authoritative state lives in
the substrate (Postgres); `work/` is disposable scratch.

## Two pipelines, one trajectory format

Both end in `rebot-joint-trajectory` JSON, validated by the player on the board.

**Measured pipeline** — the one behind the numbers in `examples/README.md`:

```bash
docker build -t rebot-pino -f examples/host/Dockerfile.pinocchio .
pwsh -File examples/host/deploy/repeat_pino.ps1     # dump -> plan -> save
# or by hand, from the repository root:
docker run --rm -v "$PWD:/work" rebot-pino python \
    /work/examples/host/app/plan_with_pinocchio.py \
    --joints /work/current.json --out /work/seq.json
```

**Tap pipeline** — phone or `/live`, no shell in the loop:

```bash
docker compose up -d --build
# tap an object in either viewport; the panel plans, then offers "Move the arm"
# switch either page to markers to collect points and run them in one pass
# or say what you want in /live's sidebar and let the agent place the markers
```

The `/live` sidebar is off until the host is told which model to use: see
`REBOT_LLM_URL` and `REBOT_LLM_MODEL` in `.env.example` and
[`docs/phone-camera.md`](docs/phone-camera.md#talking-to-it-the-agent-sidebar).
Its conversations are replayable without a browser:

```bash
python examples/host/tests/agent_eval.py --list    # cases and uncovered captures
python examples/host/tests/agent_eval.py --runs 3  # same set, three times each
```

The tap pipeline is documented end to end in
[`docs/phone-camera.md`](docs/phone-camera.md); the board side is
[`../edge-rust/README.md`](../edge-rust/README.md#http-server). Once a rig has
been fitted, `state/calib/extrinsics-fixed.json` is picked up at startup: the
camera is fixed and no tag is placed anywhere. Until then, the handheld phone
path uses the ArUco tag. The rig comes either from one still that sees the base
tag (`POST /capture/pose` — no arm motion) or from one tap:
`POST /calibrate/extrinsics` drives the arm through eight poses, takes the
stills, and writes the rig or refuses with the reason.

## Layout

```
compose.yaml              the host stack
Dockerfile                host image: aiohttp + opencv  (capture, tools, tests)
Dockerfile.pinocchio      planner image: Pinocchio + reBotArm_control_py
requirements.txt          the host image only; the planner installs its own
.env.example              REBOT_HOST_IP, REBOT_EDGE_URL, camera, tag, gates

app/
  capture_server.py       HTTPS endpoint, /capture, /plan, /execute, WHIP relay
  planner_server.py       arm gateway: edge /pose, plan(), gated execute
  locate_target.py        capture -> base-frame point (rig or tag PnP + plane)
  calib.py                intrinsics: load, rescale, fingerprint
  calibrate_intrinsics.py checkerboard fit -> work/calib/<device>.json
  calibrate_extrinsics.py capture -> base-tag rig or tool-tip sweep -> extrinsics-<mount>.json
  extrinsics.py           the rig: DLT + LM, covariance, predicted click error
  witness.py              did the camera move? background patches, arm masked
  sweep.py                the fixed eight-pose calibration sweep + clearance check
  auto_calibrate.py       drives the sweep, grabs the stills, finds the tip
  camera.py               RTSP / HTTP snapshot / device frames for that sweep
  mediamtx.py             MediaMTX control API: record a sweep, find its segments
  sweep_video.py          the sweep's pose log -> the recorded frame for each pose
  plan_to_target.py       plan(): target + pose -> trajectory (CLI and gateway)
  traj.py                 payload assembly + the player's schema checks
  plan_with_pinocchio.py  the measured demo trajectory (peel / lift / grip)
  rebot_ik.py             the URDF transcription, model reference for the docs
  base_marker.py          the base ArUco tag: printable sheet, placement spec, PnP pose
  make_cert.py            local CA + leaf certificate for the phone
  healthcheck.py          compose healthcheck for the capture service

web/                      phone page, shared api/stream/video/geometry modules, reach, locate
tests/                    selftest.py (the whole pipeline, no hardware), browser_check.py (Chromium),
                          agent_eval.py (chat trajectories against the model)
deploy/                   mediamtx.yml, repeat_pino.ps1
docs/                     phone-camera.md, arm_model.png
config/                   tls + bootstrap + calib/ (bench-default intrinsics, durable)
work/                     captures, calib cache, tags, out, recordings (disposable)
```

## Notes

- The planner image mounts `app/` read-only instead of copying it, so editing a
  planner does not need a rebuild: `docker compose restart planner`.
- `capture` and `tools` share one image; `tools` overrides the entry point.
- The arm gateway is the only process that talks to the board, and it publishes
  no port. The capture server is the only one exposed to the LAN.
- Execution is off twice: `REBOT_EXECUTE=0` on the host and, on the board,
  `rebot-edge serve` without `--allow-execute`.
