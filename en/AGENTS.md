# B601-RS Robotic Arm User Guide

> This document applies only to the **B601-RS robotic arm**.
> Except for commands that require explicit user permission, all steps are executed by the agent with your consent. In non-exceptional cases, do not ask the user to handle things manually in the terminal.
> Do not use motorbridge-cli commands not mentioned in this document, as they may start the motors and cause unexpected behavior.

## Environment State Tracking

During the workflow, check and update `memory/local-machine-env.md` to record the current machine's environment, avoiding redundant initialization:

- **Before starting**: Read this file — if the control host, conda paths, motorbridge info, CAN interface, or motor configuration already exist, you can skip those steps
- **After each step**: Append new information as instructed in the reference documents
- **If the file doesn't exist**: Skip the read step and proceed normally

Record at minimum: control host type (local / remote SSH), SSH target, CPU architecture, CAN interface name and bitrate, motorbridge CLI path, configured motor IDs.

---

## Step 0: Confirm the Control Host and the CAN Path

`motorbridge` talks to the arm over a **local** CAN interface (`can0`). It has no network/remote transport — `socketcan`, `socketcanfd`, `dm-serial`, `dm-device` and `mcu-serial` are all local. So the machine running motorbridge **must be the machine physically wired to the arm's CAN bus**. Establish which case you are in before touching anything:

| Case | motorbridge runs on | The agent reaches it via |
|------|---------------------|--------------------------|
| **A. Local desktop** | The user's PC (Windows / macOS / Linux) with a USB-CAN adapter | Directly |
| **B. Remote SBC over SSH** | A Raspberry Pi / Jetson / other board with a CAN HAT or USB-CAN on it | `ssh <user>@<host>` |

**For case B**, all of the following applies for the rest of the document:

- Every command runs over SSH, e.g. `ssh <user>@<host> "motorbridge-cli scan --vendor robstride --channel can0 ..."`
- The same rule as local applies: do not hand the commands back to the user to type into a terminal on the board
- The board usually has no display, so the browser steps (Motorbridge Studio) happen on the **user's** machine and need a port-forward — see [Step 3](#step-3-zero-calibration)
- Long-running processes (the gateway) must survive the SSH session: use `setsid nohup ... >/tmp/gw.log 2>&1 &` or `tmux`, and record the PID

### Host support check (do this first)

`motorbridge` ships prebuilt `libmotor_abi.so` + `ws_gateway` binaries inside its wheels. There is no usable sdist fallback — a plain source install gives you the Python wrapper and then fails to load the ABI.

| `uname -m` | Wheel available | Verdict |
|------------|-----------------|---------|
| `x86_64` / `amd64` | Yes | Continue |
| `aarch64` / `arm64` (64-bit OS) | Yes | Continue |
| `armv6l` / `armv7l` (32-bit Raspberry Pi OS) | **No** | **Stop** — see below |

Probe without installing anything:

```bash
python3 -m venv ~/mb-probe
~/mb-probe/bin/pip download motorbridge --no-deps --only-binary=:all: -d /tmp/mb-probe
```

`No matching distribution found for motorbridge` means the platform is unsupported. **Do not** retry with `--no-binary`, and do not continue to Step 1. Tell the user which of these applies and let them choose:

1. Use a 64-bit host board (Pi Zero 2 W / Pi 3 / Pi 4 / Pi 5 with a 64-bit OS, or any x86_64 Linux box) — same CAN HAT, `pip install motorbridge` just works
2. Build `motor_abi` + `ws_gateway` from source for that architecture (hours on a Pi Zero W, and outside the scope of this guide)
3. Move the CAN adapter to a supported host
4. Keep the 32-bit board and drive the motors with the pure-Python `robstride_dynamics` driver — verified working on a Pi Zero W. Joint-space only on the board, but an offline planner (`examples/host/plan_with_pinocchio.py`, run in a Docker container) turns cartesian moves into joint trajectories the Pi replays, with Pinocchio gravity feedforward baked into each waypoint. Still no Studio calibration UI. See [Demo Validation on Pi Zero W](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/demo-validation-pi-zero.md)

> The CAN HAT itself is configured in Step 1 and can be set up on a 32-bit board regardless; it is only the Python/ABI side that needs a supported architecture.

---

## Quick Start

Please confirm which form of robotic arm you have:

| Form | Description | Starting Step |
|------|-------------|---------------|
| Kit (unassembled) | Requires user assembly | Step 0 → Step 1 |
| Assembled | Pre-assembled and ready to use | Step 0 → Step 3 |

---

## Step 1: Environment Initialization

See [Environment Setup Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-environment.md) to complete the following:

- [ ] Python environment setup (Miniforge or existing Python)
- [ ] motorbridge installation
- [ ] CAN path configured for your hardware

The CAN path depends on the adapter, not just the OS:

| Adapter | Reference |
|---------|-----------|
| PCAN-USB (Linux / Windows / macOS) | [Environment Setup](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-environment.md) |
| CANable / candleLight (SocketCAN) | [Environment Setup](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-environment.md) |
| **CAN HAT, MCP2515 over SPI** (typical Raspberry Pi) | [CAN HAT Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-can-hat.md) |
| Damiao USB serial | [Environment Setup](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-environment.md) |

After completion, verify motorbridge-cli is available:

```bash
motorbridge-cli --help
```

> **Windows users**: Use Miniforge to create a conda environment — do not use system Python directly. If you already use `uv`, an isolated uv venv is an accepted alternative (see the uv section of the reference) — either way, never install into a system Python. No can0 configuration needed. See [Environment Setup Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-environment.md) for details.
>
> **32-bit Raspberry Pi OS users**: Miniforge ships no `armv6l`/`armv7l` builds, and motorbridge has no wheels for those architectures. Step 0 must be resolved first.

---

## Step 2: Write Motor IDs

See [Write Motor ID Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/write-motor-id.md)

This reference guides you through:

1. **Confirm the CAN interface** (`can0` at bitrate `1000000`; on a CAN HAT it must be up **and** persistent — see the [CAN HAT Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/setup-can-hat.md))
2. **Set motor IDs one by one** (7 motors total, ID range 1–7)
   - Connect only one motor at a time
   - Scan to confirm the current motor ID (default: 127, some batches pre-set to 1–7)
   - Modify to the user-specified target ID
   - > **Note**: robstride `id-set` may report `store_parameters failed` timeout error — the ID is actually written, verify by scanning
3. **Final verification** — Scan to confirm all motor IDs (1–7) are correctly written

---

## Step 3: Zero Calibration

See the [Zero Calibration Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/write-motor-zero.md).

This reference guides you through:

1. **Start motorbridge-gateway** (bound to `127.0.0.1:9002`, using `can0`)
2. **Guide the user to open Motorbridge Studio** to complete zero calibration

> **Remote host (case B)**: the gateway listens on the board's loopback and Studio runs in the user's browser, so the port must be forwarded:
>
> ```bash
> ssh -L 9002:127.0.0.1:9002 <user>@<host>
> ```
>
> Start the gateway on the board first (detached, so it outlives the SSH session), keep the forwarding SSH session open, then have the user open <https://motorbridge.github.io/motorbridge-studio/> pointed at `127.0.0.1:9002`. The [Zero Calibration Reference](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/references/write-motor-zero.md) has the full sequence.

---

## Step 4 (Optional): ROS 2

ROS 2 support exists, but it is **not** an alternative to Steps 0–3 — it sits on top of the same low-level stack and needs correct motor IDs and zero positions first.

See the Seeed tutorial: [reBot Arm B601-RS ROS 2 Integration](https://wiki.seeedstudio.com/rebot_arm_b601_rs_ros2_integration/)

What it requires, and why it changes the platform choice:

| Requirement | Detail |
|-------------|--------|
| Host OS | Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Jazzy) |
| ROS packages | `ros-${ROS_DISTRO}-pinocchio`, `control-msgs`, `trajectory-msgs`, `rviz2`, optional MoveIt 2 |
| Workspace | `Seeed-Projects/reBotArmController_ROS2` + `Seeed-Projects/reBotArm_control_py` |
| CAN access | The ROS 2 node opens `can0` **locally**, so the workspace must run on the machine wired to the arm |
| motorbridge | Required — the same wheel/architecture restriction from Step 0 applies |

Practical consequence: ROS 2 on this arm means an Ubuntu x86_64 (or 64-bit ARM Ubuntu) host with a CAN adapter, not a 32-bit Raspberry Pi OS board. ROS 2 on Raspbian 12 / `armv6l` is not a supported combination.

---

## Reference Documents Status

| Reference | File | Status |
|-----------|------|--------|
| Environment Setup | `references/setup-environment.md` | Completed |
| CAN HAT (MCP2515 over SPI) | `references/setup-can-hat.md` | Completed |
| Demo Development & Validation (Pi Zero W) | `references/demo-validation-pi-zero.md` | Completed |
| Write Motor ID | `references/write-motor-id.md` | Completed |
| Zero Calibration | `references/write-motor-zero.md` | Completed |
