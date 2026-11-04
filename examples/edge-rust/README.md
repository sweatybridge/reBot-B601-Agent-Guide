# rebot-edge — the edge player as one Rust binary

The edge half of the B601-RS pipeline, in one static executable. Input is the
trajectory JSON that `host/plan_with_pinocchio.py` writes:

```
seq_pino_peel.json ──► rebot-edge ──► SocketCAN can0 ──► motors 1-7
```

It replaces the former Python edge player (`play_trajectory.py`,
`gradual_motion_demo.py`, `mit_frames.py` and `dump_joints.py`, now removed)
with `src/main.rs` (serde + libc only). The wire format, the re-checked limits,
the eased alignment, dry-run default and the shutdown ramp are the same; the
only thing that changes on the board is that there is no Python, numpy or
`RobStride_Control` checkout to install.

## Build with Docker

The default target is **ARMv6 hard-float, statically linked musl** — the
Raspberry Pi Zero W (`armv6l`) running 32-bit Raspberry Pi OS:

```bash
# from examples/edge-rust
docker build --target bin --output dist .
file dist/rebot-edge
# ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, stripped
```

`dist/rebot-edge` is the whole deliverable (~650 KB); copy it to the board. Other
boards are one build argument away:

| board | `--build-arg TARGET=` |
|---|---|
| Pi Zero W / Pi 1, 32-bit OS (default) | `arm-unknown-linux-musleabihf` |
| Pi 2 / 3 / 4, 32-bit OS | `armv7-unknown-linux-musleabihf` |
| Pi Zero 2 W / Pi 4 / 5, 64-bit OS | `aarch64-unknown-linux-musl` |
| workstation / CI | `x86_64-unknown-linux-musl` |

```bash
docker build --target bin --output dist \
    --build-arg TARGET=aarch64-unknown-linux-musl .
```

The musl targets are self-contained: rustc links its bundled libc with
`rust-lld`, so the image needs no cross C toolchain and the result has no glibc
version to match on the board.

Run the unit tests (host architecture, inside the build image):

```bash
docker build --target test .
```

The tests include byte-for-byte golden vectors generated from the Python
`MitSender`, so a change to the frame scaling fails the build. A second test
runs bring-up and the whole playback loop against an in-memory motor simulator,
so the control loop is covered without hardware.

`--check` on a workstation, without a CAN interface:

```bash
docker build --target hostrun -t rebot-edge-host .
docker run --rm -v "$PWD/../trajectories:/data:ro" rebot-edge-host \
    --check /data/seq_pino_peel.json
```

(Docker Desktop must be allowed to share the repository path for the bind mount;
on Linux this just works.)

## Run on the board

```bash
scp dist/rebot-edge pi@raspberrypi.local:~
ssh pi@raspberrypi.local
chmod +x rebot-edge

sudo systemctl start can0-setup.service      # bring can0 up at 1 Mbit/s
ip -details link show can0

./rebot-edge --check trajectories/seq_pino_peel.json   # offline sanity check
./rebot-edge trajectories/seq_pino_peel.json           # dry run: reads poses, no frames
sudo ./rebot-edge trajectories/seq_pino_peel.json --execute --track
```

The dry run still opens the CAN socket and reads `0x7019` from every motor
(read-only, no torque). `--check` is the only mode that touches nothing at all.

`--execute` prints the drift between the plan's start pose and the measured one,
refuses to start if it is more than 20°, eases onto the plan start with
`--ramp`, streams the waypoints, then ramps home and disables torque. Ctrl-C or
SIGTERM interrupts at the next tick, ramps home and then disables.

## Pose snapshot

The planner's `--joints` input is still a read-only pose JSON. The former
`dump_joints.py` is replaced by the `dump` subcommand:

```bash
./rebot-edge dump --ids 1,2,3,4,5,6,7 > current.json
# {"ids":[...],"q":[...],"timestamp":"...Z","channel":"can0","note":"..."}
```

`dump` reads parameter `0x7019` from each motor, enables nothing and sends no
torque. It needs `can0` up, like the player.

## Input

The file is the planner's `rebot-joint-trajectory` format. Only a subset is used
at runtime; everything else is shown in the banner.

```jsonc
{
  "format": "rebot-joint-trajectory",
  "control_hz": 30,                    // replayed at this rate (clamped 5..200)
  "ids": [1, 2, 3, 4, 5, 6, 7],        // motor IDs, 1..7, unique
  "limits": [[-2.8, 2.8], ...],        // re-checked here, +/- 2 mrad tolerance
  "duration_s": 22.0,                  // banner only
  "hold_start_s": 10.0, "hold_end_s": 12.0,
  "waypoints": [
    {"t": 0.033, "q": [0.0049, ...], "tau": [-0.0, 1.54, ...]}
  ]
}
```

Checks performed before anything is enabled:

- `format`, at least two waypoints, monotonic `t`, one `q` per motor;
- every `q` inside the file's `limits` (when the lengths match) and outside no
  limit by more than 2 mrad;
- at most 0.5 rad between consecutive waypoints (sanity, as in Python);
- every `q` inside the MIT frame's ±4π range, rather than letting the frame
  encoder clip it silently;
- `tau` present on every waypoint or none, finite, one per motor;
- finite numbers throughout, no duplicate motor IDs.

## Options

| option | meaning |
|---|---|
| `--channel IFACE` | SocketCAN interface, default `can0` |
| `--model MODEL` | fallback MIT model, default `rs-00` |
| `--model-spec SPEC` | per-joint models, default `1:rs-06,2:rs-06,3:rs-06` |
| `--kp SPEC` | default `50,2:150,3:150` (vendor gains) |
| `--kd SPEC` | default `3,2:10,3:10,4:5,5:4,6:4,7:4` |
| `--ramp S` | ease-on time, default 2.0 s (clamped 0.5..10) |
| `--speed X` | time scale, default 1.0 (clamped 0.1..5) |
| `--delay S` | hold the start pose before moving |
| `--max-follow-error RAD` | abort when a joint lags by more than this (checked twice a second) |
| `--log FILE` | write commanded/measured angles as `rebot-trajectory-log` JSON |
| `--allow-kp-mismatch` | replay a sag-compensated file at another `kp` |
| `--stay-enabled` | leave torque on at exit |
| `--active-report` | enable Type-24 motor active reporting during playback |
| `--track` | print measured angles at 25/50/75/100 % |
| `--execute` | actually move; without it the player is read-only |
| `--check` | validate/describe the file offline, no CAN |
| `--fast-frames` | accepted for command-line compatibility; frames are always hand-built |

`--sdk-path` is rejected with a hint: there is no Python SDK to find.

## Higher rates

`control_hz` is accepted up to 200 Hz. Playback is driven by the waypoint
timestamps, so generate a trajectory at the rate you want — either replan with
`plan_with_pinocchio.py --hz 100` (which also recomputes `tau`) or resample an
existing file (which preserves its path but linearly interpolates `tau`).
`--speed` is not a rate control: it shortens the whole motion and makes the arm
move faster.

Telemetry never waits in the control tick. With `--active-report` a reader
thread owns the receive side; otherwise each tick drains the status replies
already queued and keeps the last sample per joint. A 200 Hz run with `--log
--track` and no active reporting measured mean 5.0 ms, p95 5.0 ms, max 7.0 ms,
0 late ticks of 4400. The snapshot is typically the previous tick's reply, so a
logged `meas` can trail `cmd` by one tick — 5 ms at 200 Hz, 33 ms at 30 Hz.
Keep that in mind if the log is used for trim analysis.

## Motor active reporting

`--active-report` starts the reader thread, switches every motor to Type-24
active reporting after bring-up, and sends the disable frame on the way out
(including errors and SIGINT/SIGTERM; `CanBus::drop` retries if needed). The
request is the manual's `01 02 03 04 05 06 F_CMD` Type-24 frame, with `F_CMD=01`
to enable and `00` to disable.

Reports are Type-24 frames carrying the same position, velocity, torque and
temperature payload as Type 2. This firmware sends Type 24 even though the
manual summary calls it "report type 2", so the reader accepts both. The
interval comes from `EPScan_time` (0x7026, 5 ms units); the motor default is
10 ms (about 100 Hz), and the floor measured on this unit was roughly 13-17 ms
at the lowest setting, so 200 Hz reports are not available. Command replies
continue, so active reporting adds telemetry rather than replacing per-command
feedback.

Measured on this arm, with `--log --track`:

| control rate | actual | mean / p95 / max tick | late ticks | new CAN RX errors |
|---|---|---|---|---|
| 100 Hz | 22.00 s | 10.0 / 10.0 / 10.6 ms | 0 of 2200 | 5 |
| 200 Hz | 22.65 s | 5.1 / 6.4 / 12.7 ms | 24 of 4400 | 28 |

At 100 Hz the combination is usable: the control loop holds its 10 ms deadline,
though the MCP2515 still counted five RX errors from the report bursts. At
200 Hz the extra ~700 report frames/s (about 2,100 frames/s total) overloads the
Pi Zero W and MCP2515: the 22 s motion stretched to 22.65 s, 24 ticks were late,
and tracking degraded (gripper peak 0.77 rad vs 0.29 rad at 100 Hz active).
Use active reporting at 100 Hz or below; at 200 Hz leave it off. Both runs
switched reporting off at exit — zero Type-24 frames were seen afterwards.

## Differences from the Python player

- **One static binary.** No Python, numpy, `python-can` or `RobStride_Control`
  on the board. MIT frames are built with integer scaling in `mit_frame`; the
  tests pin the bytes to the Python implementation.
- **`--check`.** Validates and prints the plan on the workstation or in CI.
- **Stricter input checks.** Duplicate IDs, non-finite values, partially present
  `tau`, and angles outside ±4π are refused; the Python player would either
  crash or let the frame encoder clip.
- **A partial bring-up disables torque.** If a motor accepts ENABLE but never
  returns a status frame, the Rust player disables every motor instead of
  leaving torque on while it reports the error. The Python player's `enabled`
  flag is only set after the whole loop, so that failure leaves the earlier
  motors enabled.
- **Reads have timeouts.** A missing motor reports after 500 ms instead of
  hanging forever in a blocking parameter read.
- **Non-blocking telemetry.** With `--active-report`, a reader thread owns the
  CAN receive side and publishes the latest status per joint. Without it, the
  playback loop drains already-queued status frames with a zero-timeout read,
  so `--log`, `--track` and follow-error checks never wait on the bus either
  (the Python player blocks up to 12 ms per waypoint).
- **`operstate` accepts `unknown`.** CAN interfaces (and `vcan`) often report
  `unknown` when administratively up; only `down` is treated as down, so the
  player also works against `vcan0` for bench tests.
- **Log timestamps are UTC** (`...Z`), not local time. Everything else in the
  log follows the Python `rebot-trajectory-log` format.
- **Dump is included.** `rebot-edge dump` replaces the former Python
  `dump_joints.py` and produces the planner's `--joints` input. It enables
  nothing; the playback path still needs only the trajectory.

## Measured on the reference arm

Raspberry Pi Zero W, kernel `6.6.20+rpt-rpi-v6`, `can0` at 1 Mbit/s (mcp251x),
2026-11-04. The same 22 s peel/lift/return path was played seven times; the 50,
100 and 200 Hz files were resampled from the 30 Hz plan (same path, `tau`
linearly interpolated between the 30 Hz samples):

| rate | waypoints | actual | mean / p95 / max tick | late ticks | post-run drift |
|---|---|---|---|---|---|
| 30 Hz (`--log`, blocking) | 660 | 22.00 s | 33.3 / 33.4 / 147.0 ms | 2 of 659 | 0.0056 rad |
| 50 Hz | 1101 | 22.00 s | 20.0 / 20.0 / 21.8 ms | 0 of 1100 | 0.0048 rad |
| 100 Hz | 2201 | 22.00 s | 10.0 / 10.0 / 13.4 ms | 0 of 2200 | 0.0053 rad |
| 200 Hz (unlogged) | 4401 | 22.00 s | 5.0 / 5.0 / 7.2 ms | 0 of 4400 | 0.0187 rad |
| 200 Hz (`--log --track`, no-wait) | 4401 | 22.00 s | 5.0 / 5.0 / 7.0 ms | 0 of 4400 | 0.0181 rad |
| 100 Hz + active report | 2201 | 22.00 s | 10.0 / 10.0 / 10.6 ms | 0 of 2200 | 0.0183 rad |
| 200 Hz + active report | 4401 | **22.65 s** | 5.1 / 6.4 / 12.7 ms | 24 of 4400 | 0.0185 rad |

All runs exited 0, returned to the start pose and disabled torque. The 30 Hz row
predates the zero-wait/reader split, so its 147 ms maximum and two late ticks
are the blocking status poll. The active-report rows add 700 Type-24 frames/s
to the command replies; 100 Hz is the highest rate that stayed on schedule on
this board.

The 200 Hz post-run drift is a joint-6 wrist sag after torque is dropped (the
200 Hz runs started about 0.019 rad off and the align ramp corrected it), not a
tracking error at speed. Per-waypoint tracking: at 30 Hz, joints 1-6 peak at
1-87 mrad and the gripper at 0.41 rad; at 200 Hz without active reporting
(one-tick-old snapshot) joints 1-6 peak at 1.3-66 mrad and the gripper at
0.28 rad. With active reporting at 100 Hz the gripper peak is 0.30 rad; at
200 Hz it climbs to 0.77 rad as the timing slips. `--track` and `--log` share
the same snapshot instead of competing for the status stream.

## Safety

- Nothing moves without `--execute`.
- Torque is never enabled before the measured pose has been read and compared
  with the plan start.
- A fault frame, a failed status read or a follow-error aborts playback; the arm
  is ramped back to the pose captured at enable before torque is dropped.
- A gravity-loaded joint will sag when torque drops. Make sure the arm is
  resting safely before running, as with the Python demo.
- Active reporting is telemetry only, but it is switched off on exit. If the
  process is SIGKILLed, the motors keep reporting until the next power cycle;
  use `--active-report` with the normal Ctrl-C/SIGTERM path.

## Layout

```
examples/edge-rust/
  src/main.rs              the whole player: CLI, JSON, CAN, MIT frames, playback
  testdata/mini_trajectory.json   fixture used by the parser tests
  Dockerfile               cross-build (musl) + test + host-run stages
  Cargo.toml / Cargo.lock  pinned dependencies (libc, serde, serde_json)
```
