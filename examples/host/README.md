# Host side

Everything that runs off the board: the phone camera, the planner, and the two
pipelines that turn a pose into a trajectory.

```
phone / workstation  --tap-->  capture_server (opencv)  -->  planner_server (Pinocchio)
                                                                    |
                                        rebot-edge serve  <-- HTTP --+
                                        (board, :8081)
```

## What runs where

| service | image | listens on | job |
|---|---|---|---|
| `capture` | `Dockerfile` | `:8443` (LAN) | HTTPS for the phone, `/capture`, `/plan`, `/execute`, WHIP/WHEP relay |
| `mediamtx` | `bluenviron/mediamtx` | `:8189/udp` | WebRTC relay: phone publishes, `/live` reads |
| `planner` | `Dockerfile.pinocchio` | nothing | the arm gateway: edge `/pose`, planning, gated `/execute` |
| `tools` | `Dockerfile` | nothing | one-shot: calibration, tag, certificate, selftest |

`docker compose up -d --build` starts the first three; `tools` is a profile.
Paths inside and outside the containers are the same: the project root is `/app`,
the code lives in `app/`, the pages in `web/`, and everything that persists in
`state/`.

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
```

The tap pipeline is documented end to end in
[`docs/phone-camera.md`](docs/phone-camera.md); the board side is
[`../edge-rust/README.md`](../edge-rust/README.md#http-server).

## Layout

```
compose.yaml              the host stack
Dockerfile                host image: aiohttp + opencv  (capture, tools, tests)
Dockerfile.pinocchio      planner image: Pinocchio + reBotArm_control_py
requirements.txt          the host image only; the planner installs its own
.env.example              REBOT_HOST_IP, REBOT_EDGE_URL, tag placement, gates

app/
  capture_server.py       HTTPS endpoint, /capture, /plan, /execute, WHIP relay
  planner_server.py       arm gateway: edge /pose, plan(), gated execute
  locate_target.py        capture -> base-frame point (tag PnP + table plane)
  calib.py                intrinsics: load, rescale, fingerprint
  calibrate_intrinsics.py checkerboard fit -> state/calib/<device>.json
  plan_to_target.py       plan(): target + pose -> trajectory (CLI and gateway)
  traj.py                 payload assembly + the player's schema checks
  plan_with_pinocchio.py  the measured demo trajectory (peel / lift / grip)
  rebot_ik.py             the URDF transcription, model reference for the docs
  make_tag.py             printable ArUco tag at a measured size
  make_cert.py            local CA + leaf certificate for the phone
  healthcheck.py          compose healthcheck for the capture service

web/                      phone.html, live.html, viewer.js (shared tap module)
tests/                    selftest.py (43 checks) and browser_check.py (Chromium)
deploy/                   mediamtx.yml, repeat_pino.ps1
docs/                     phone-camera.md, arm_model.png
state/                    captures, calib, tls, tags, out, work  (gitignored)
```

## Notes

- The planner image mounts `app/` read-only instead of copying it, so editing a
  planner does not need a rebuild: `docker compose restart planner`.
- `capture` and `tools` share one image; `tools` overrides the entry point.
- The arm gateway is the only process that talks to the board, and it publishes
  no port. The capture server is the only one exposed to the LAN.
- Execution is off twice: `REBOT_EXECUTE=0` on the host and, on the board,
  `rebot-edge serve` without `--allow-execute`.
