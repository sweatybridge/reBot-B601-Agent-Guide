## Demo Development and Validation on a Raspberry Pi Zero W

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

This documents how the pure-Python motion demo ([`examples/edge/gradual_motion_demo.py`](../../examples/edge/gradual_motion_demo.py)) was built and validated on a 32-bit Raspberry Pi, and — more usefully — the checks that caught real defects before they reached the arm. Follow the same order for any new motion script on this class of hardware.

**Verified baseline**

| Item | Value |
|---|---|
| Board | Raspberry Pi Zero W Rev 1.1 (BCM2835, 512 MB) |
| Architecture | `armv6l` (32-bit, no NEON) |
| OS / kernel | Raspbian GNU/Linux 12 (bookworm) / `6.6.20+rpt-rpi-v6` |
| Python | 3.11.2 (system) |
| CAN | RS485 CAN HAT, MCP2515 over SPI0 CE0, 12 MHz crystal, `INT` = GPIO25 |
| Library | Seeed `RobStride_Control` → `robstride_dynamics` (pure Python) |
| Packages | `python3-can` 4.1.0, `python3-numpy` 1.24.2, `python3-tqdm` 4.64.1 (all from apt) |

## Gate 1: Can the documented stack run at all?

Probe before installing anything (see `AGENTS.md` Step 0):

```bash
python3 -m venv ~/mb-probe
~/mb-probe/bin/pip download motorbridge --no-deps --only-binary=:all: -d /tmp/mb-probe
# ERROR: No matching distribution found for motorbridge
```

`motorbridge` has no `armv6l`/`armv7l` wheels — PyPI carries `linux_x86_64`, `linux_aarch64`, `macos_arm64` and `win_amd64` builds only — and its sdist is a thin Python wrapper around a prebuilt `libmotor_abi.so` + `ws_gateway` binary (`abi.py` resolves `MOTORBRIDGE_LIB`, `motorbridge/lib/`, `target/release/`). On `armv6l` there is nothing to load, so a source install yields an importable module that fails at runtime. Miniforge is no help either — no `armv6l`/`armv7l` builds.

**Verdict**: motorbridge and everything layered on it (Motorbridge Studio, the Zero Calibration UI, `reBotArm_control_py`, the ROS 2 workspace) cannot run on this board. Keep the CAN path and choose a different client library.

## Gate 2: Get `can0` up, and make it survive a reboot

The HAT was installed but dead: SPI commented out, no overlay, `can0` absent.

```bash
grep -n "spi=on\|mcp2515" /boot/firmware/config.txt   # bookworm; /boot/config.txt on older
# dtparam=spi=on
# dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
sudo reboot
sudo dmesg | grep mcp251          # mcp251x spi0.0 can0: MCP2515 successfully initialized.
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set can0 up
ip -details link show can0        # can state ERROR-ACTIVE, bitrate 1000000, clock 6000000
```

Two things that cost time and are worth knowing up front:

1. **`clock 6000000` is correct.** The driver reports half the crystal (`can.clock.freq = freq / 2` in `mcp251x.c`); a 12 MHz HAT always reads 6 MHz here. Do not "fix" the overlay because of it.
2. **Persisting `can0` is a boot-ordering problem, not an `ip link` problem.** The first systemd unit failed with `Cannot find device "can0"`: on a Zero W the `mcp251x` driver only creates the interface ~45–50 s into boot, after the unit had already run. The working unit waits for the netdev (`.device` dependency plus a bounded loop in `ExecStartPre`) — see [CAN HAT Setup](setup-can-hat.md#step-6-make-it-persistent-recommended). Verify with a real `reboot`, not `systemctl start`.

Full procedure and troubleshooting table: [CAN HAT Setup](setup-can-hat.md).

## Gate 3: Prove bus and motors are healthy before writing motion code

Three read-only checks, cheapest first.

```bash
# 1. MCP2515 loopback: proves SPI + driver + bitrate, no motors involved
sudo ip link set can0 type can bitrate 1000000 loopback on
# ... send one frame with a CAN_RAW socket and read it back ...

# 2. Bus presence: in normal mode, send one frame to an ID the motors ignore
#    ACKed -> stays ERROR-ACTIVE, tx_errors=0; nothing there -> BUS-OFF

# 3. Motor presence: ping every ID (read-only, no torque)
RobstrideBus.scan_channel("can0", 1, 9)     # -> motors 1..7 with UUIDs
bus.read(name, ParameterType.MECHANICAL_POSITION)   # -> joint angles in rad
```

The loopback test needs no extra packages — a `socket(AF_CAN, SOCK_RAW, CAN_RAW)` socket works with the stdlib, provided `CAN_RAW_RECV_OWN_MSGS` is enabled and the interface is in loopback mode. That matters because it lets you separate "HAT broken" from "motors unpowered" without installing anything.

## Gate 4: Design the motion script around refusals, not features

| Decision | Reason |
|---|---|
| Dry run is the default; `--execute` required | A stray `python3 demo.py` must not move a 6-DOF arm |
| Start pose read from the **ENABLE status frame**, never assumed | The arm's zero has already changed once between sessions; assuming zero is how arms get commanded to move when they should not |
| `\|delta\| ≤ 0.35 rad`, gains clamped, targets kept 0.2 rad inside the ±4π MIT range | Flags may only make motion smaller |
| Fault frames (comm type 21) raise instead of being drained | Otherwise the loop keeps commanding a motor that already faulted |
| Every leg ramps from the **last commanded** position | See defect 1 |
| Exit ramps home, holds, then disables (or `--stay-enabled`) | Crash-safe, and makes the gravity-sag trade-off explicit |

## Gate 5: Validate with measurement, not with wall-clock

Three defects were found during validation. All three produced plausible-looking output first.

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | Return leg "took 3.03 s" so it looked gradual | Interpolation always started from the bring-up pose, so a leg whose target *is* the start pose emitted it on the first frame — a step, followed by 3 s of holding | Interpolate from the last commanded position (`self.last`) |
| 2 | `WARNING: Invalid device ID, got 7, expected 1` in `--track` output | Status frames arrive in arbitrary order once several motors are enabled; "read the next frame" attributed another joint's position | `read_status()` filters on device ID, fails after a timeout |
| 3 | Joints 3–4 settled 0.05–0.14 rad below target | Arm now carries its own weight; a single `--kp` cannot suit an unloaded base joint and a loaded wrist joint | Per-joint gains (`--kp 4:45,5:35`) |

Defect 1 is the one to internalise: **the loop running for the full duration proves nothing about the motion.** `--track` (measured position at 25/50/76/100 % of each leg) is what turned it into evidence.

Defect 2 matters beyond logging: the same call seeded the start pose at bring-up, where a mis-attributed reading becomes the target of the first command.

## Measurements

Single joint, δ = 0.0524 rad (3°), kp 15 / kd 0.6, 30 Hz — measured positions:

| leg | 24 % | 50 % | 76 % | 100 % |
|---|---|---|---|---|
| outbound | +0.0284 | +0.0456 | +0.0629 | +0.0736 |
| return | +0.0717 | +0.0548 | +0.0376 | +0.0268 |

Both legs ~3.03 s for a 3 s target (30 Hz held), lands within ~2 mrad of start.

All 7 joints, δ = 0.1048 rad (6°), `--track`, tracking error at the far point (measured − commanded):

| joint | kp 15 | kp `4:45 5:35` |
|---|---|---|
| 1 | −0.002 | −0.003 |
| 2 | −0.008 | −0.005 |
| 3 | −0.050 | −0.037 |
| 4 | −0.141 | −0.045 |
| 5 | −0.008 | −0.004 |
| 6 | −0.007 | −0.003 |
| 7 | −0.001 | +0.002 |

Joints 3 and 4 remain short — joint 4 even at 45 Nm/rad, joint 3 still at the 20 base — and both track to ~2 mrad when the arm is at rest, which points at load rather than mistuning. Gravity compensation is the correct fix, and it lives in `reBotArm_control_py`, which needs motorbridge. Timing: 3.35–3.40 s per 3 s leg at 20 Hz, i.e. **~18 Hz effective for 7 joints** — about the Zero W's ceiling over this HAT.

## Checklist for the next Pi Zero W session

1. `dpkg -s python3-can python3-numpy python3-tqdm` — install from apt, never pip (avoids building numpy on a 512 MB armv6 CPU)
2. `ip -details link show can0` — `ERROR-ACTIVE`, `bitrate 1000000`
3. Motor scan before any motion; confirm IDs 1–7
4. **Dry run first** — the plan should match what you expect to see physically
5. Validate one joint before seven; the others being limp hides gravity load
6. Use `--track`; compare a leg's `done in +X.XXs` against `--duration` to catch loop overrun
7. Keep `--hz ≤ 20` for 7 joints, `≤ 30` for 1–2
8. After torque drops the arm sags — re-read positions before the next run rather than assuming the previous start pose

## Reproducing the runs

```bash
# one-time setup (apt, not pip: no compiling numpy on a 512 MB armv6 CPU)
curl -sL -o /tmp/rs.tar.gz https://codeload.github.com/Seeed-Projects/RobStride_Control/tar.gz/refs/heads/master
tar xzf /tmp/rs.tar.gz && mv RobStride_Control-master ~/RobStride_Control
sudo apt-get install -y python3-can python3-numpy python3-tqdm

# single joint, 3 degrees, measured
python3 edge/gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
    --execute --joints 1 --delta 0.0524 --duration 3 --kp 15 --hz 30 --track

# all joints, 6 degrees, stiffer wrist
python3 edge/gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
    --execute --joints all --delta 0.1048 --kp 4:45,5:35 --hz 20 --track
```

## Offline cartesian planning

The Pi cannot do IK in a control loop: one damped-least-squares solve costs
~1.3 s on a Zero W, so 20 Hz is hopeless. The split that works — and the reason
the examples are laid out as `edge/`, `host/` and `trajectories/`:

| Stage | Where | File | Job |
|---|---|---|---|
| 1. Snapshot | edge | `edge/dump_joints.py` | Read `0x7019` from motors 1-6, print JSON. Read-only, no torque. |
| 2. Plan | host | `host/plan_cartesian.py` + `host/rebot_ik.py` | FK at the current pose, move a cartesian delta, solve IK per waypoint, write a joint-space trajectory |
| 3. Play | edge | `edge/play_trajectory.py` | Stream the waypoints at `control_hz`. No IK, no numpy needed. |

```bash
# on the edge device
python3 edge/dump_joints.py --sdk-path ~/RobStride_Control/python > current.json
# on the host
python3 host/plan_cartesian.py --joints current.json --delta 0,0,0.10 \
    --duration 5 --pause 1.0 --hz 20 --out trajectories/raise10cm.json
# back on the edge device
python3 edge/play_trajectory.py trajectories/raise10cm.json            # dry run
python3 edge/play_trajectory.py trajectories/raise10cm.json --execute
```

Kinematics come from Seeed's URDF (`ReBot_Arm_RS.urdf`), transcribed into
`host/rebot_ik.py`. Six revolute joints with a **non-spherical wrist** — the last three
axes are 104 mm and 60 mm apart — so there is no closed form; it is numeric.

### Control-loop budget on this board

Measured with motor 7 added to the six arm joints:

| quantity | value |
|---|---|
| one tick of 7 MIT frames | **50 ms** (drain of the 7 replies adds 3.8 ms; ~72 ms/tick with motors enabled) |
| 20 Hz budget | 50 ms — **exceeded**, a 25 s trajectory plays in 35 s |
| 15 Hz budget | 66.7 ms — marginal, a 22.1 s trajectory plays in 23.9 s |
| one tick of 6 MIT frames | ~43 ms — fits 20 Hz, which is why 6-joint runs kept time |

The cost is inside `write_operation_frame`, which does ~6 numpy scalar operations
per joint at ~1 ms each on a 1 GHz ARMv6. Two consequences: keep 7-joint
trajectories at 15 Hz (12 Hz if you want margin), and do not chase timing
problems with read-side tuning — this was misdiagnosed as a sampling cost twice
before the write path was measured.

**The fix is to stop calling numpy.** `edge/mit_frames.MitSender` builds the same MIT
frame with plain arithmetic and `struct`: 8.2 ms per 7-joint tick instead of
50.6 ms (6.1x), enough for a comfortable 30-40 Hz. `edge/test_mit_frames.py` compares
its bytes against `write_operation_frame` over positions, gains and torques in
and out of range - 488 frames, 0 mismatches - before it is trusted with a motor.

At 30 Hz with hand-built frames the same sequence ran **22.00 s of 22.00 s
planned with 33.3 ms mean tick spacing, max 34.8 ms, 0 of 660 ticks late**. The
previous 15 Hz run lagged 8-9 % with visible stepping, because a 66 ms command
interval at kp 30-45 makes the motor execute discrete position steps and then
dwell between them.

### Result of a 10 cm raise (validated)

Planned: 221 waypoints, 5 s up, 1 s hold, 5 s down, 20 Hz, peak joint speed
0.137 rad/s, worst IK residual 0.49 mm.

| quantity | value |
|---|---|
| planned rise at the tool | 99.5 mm |
| **measured** rise (FK of measured joints) | **78.8 mm** |
| lateral drift during the lift | 0.9 mm |
| return to start (measured) | −1.7 mm |
| playback time vs plan | 11.00 s / 11.00 s |
| joint error at the top | j2 −3 mrad, j3 −18 mrad, j4 −43 mrad |

So the *path* is right — vertical, straight, and it comes home — but the tool
finishes 21 mm low. That is joint compliance under gravity, not an IK error: the
planner's own residual is 0.5 mm. Joint 4 carries the wrist and is the one that
gives (43 mrad ≈ 2.5°).

A second run at **20 cm** (same profile, 180.9 mm measured of 199.5 mm planned)
came up 18.6 mm short. A third, compound run — up 25 cm, then forward 10 cm with
a 0.5 s corner pause — reached 225.2 mm of 250.2 mm vertically and 102.7 mm of
99.8 mm forward: **25 mm off in z, 3 mm in x**.

So the shortfall sits around 18-25 mm for 10-25 cm moves and grows only slowly as
the arm extends, i.e. it is dominated by wrist compliance rather than a scale
error. Accuracy therefore improves in percentage terms with amplitude: 21 % short
at 10 cm, 9 % at 20 cm, 10 % at 25 cm (with the extra reach offsetting the gain).

Gravity feedforward was modelled (`host/rebot_ik.gravity_torques`, validated against
`dU/dq` to 3.5e-9 Nm) and then deliberately **not** applied. Comparing the model
against the droop the arm actually shows at the top of the lift:

| joint | kp·droop (implied load) | URDF model | ratio |
|---|---|---|---|
| j2 | 0.10 Nm | 1.75 Nm | 0.06 |
| j3 | 0.74 Nm | 5.81 Nm | 0.13 |
| j4 | 1.97 Nm | 1.67 Nm | 1.18 |

Joint 4 agrees — the wrist really does carry the gripper. Joints 2 and 3 carry a
tenth of what the model predicts, i.e. the upper arm is partly supported at this
pose. Feeding the modelled 5.8 Nm into joint 3 would drive the arm up against
whatever is holding it. So the 78 mm stands as the honest result for this setup,
and feedforward would have to be calibrated per joint against measured load
first.

### Gripper (motor 7)

Motor 7 is not part of the arm chain, so no IK applies: `plan_gripper.py` writes a
one-motor trajectory that `edge/play_trajectory.py` replays like any other, and the
arm's other joints stay disabled throughout.

Two things measured on the reference unit:

- **The closed stop sits at about −0.015 rad, not 0.** The encoder reads −0.015
  with the jaws shut, and commanding below that changes nothing: the player's
  align ramp asked for −0.0277 and the gripper stayed at −0.015 for the whole
  leg. Plan `--start-rad`/`--close-rad` at the measured stop so the motor does
  not sit pressing against it (at kp 10 that is a steady ~0.13 Nm for no gain).
- **Tracking to 4.12 rad is clean** (measured 4.083-4.111, no stall), so the
  stroke extends past 4.12 rad and Seeed's 0-5 rad closed-to-open range is
  plausible.

Radians to centimetres is pinion geometry that the URDF does not model (it makes
the fingers prismatic, 0.05 m + 0.0715 m). The default 0.412 rad/cm comes from
assuming 5 rad spans that ~12 cm stroke, and it is a linear map through the
closed stop, so a single ruler measurement fixes it.

### Measuring the lift by hand

The tool moves by pure translation (orientation is held constant), so **any
single point on the gripper** rises by the same amount — measure that against a
fixed external reference. Do **not** measure between two joints: two points on
different links describe a chord, not a displacement. The j2→j5 centre-to-centre
chord grows by 88.6 mm for a 99.4 mm tool rise, and an edge-to-edge version of
the same measurement reads differently again, which is plenty to make a 78 mm
result look like "about 100 mm".

### Two IK details that bit during development

1. **The joint-limit margin must not push a joint that already sits inside it.**
   Joint 2 rests at 0.005 rad against a 0 rad limit; enforcement with a 0.02 rad
   margin silently moved the seed to 0.02 and every solve stalled ~1 mm short.
   The band is now `min(lo+margin, seed) .. max(hi−margin, seed)`.
2. **The band must also include the plan's start pose** (`anchor=`). Without it a
   return leg stalls in the last 4 % — it cannot get back down to the pose it
   began from, because that pose is inside the margin band.

## Known limits

- **Cartesian motion is planned offline.** `host/rebot_ik.py` + `host/plan_cartesian.py` run on a workstation; the Pi only replays joint waypoints. Accuracy at the tool is limited by joint compliance (21 mm of a 100 mm lift at kp 30-45), not by the planner.
- **No gravity compensation on the arm.** A model exists and is numerically validated (`host/rebot_ik.gravity_torques`), but it is not wired into the control path: measured joint loads disagree with it on joints 2-3 (arm partly supported), and applying it blindly would push the arm up. Calibrate per joint against measured droop before using it.
- **`write_id()` in the vendored library is broken**: it calls `CommunicationType.SET_CAN_ID`, which does not exist (the constant is `SET_DEVICE_ID = 7`). Irrelevant once IDs are set, but do not trust that method.
