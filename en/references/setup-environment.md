## Environment Setup

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

## Before You Begin

Before starting, check `../memory/local-machine-env.md` to see if the current machine's environment configuration already exists. If conda path and motorbridge info are already recorded, you can use them directly; if not, or if the information is incomplete, follow the steps below to re-initialize.

## Remote Control Host (SSH)

If motorbridge will run on a board that is not this machine (Raspberry Pi, Jetson, …), read this first:

- Every command in this document and in the other references runs **on that board**, over `ssh <user>@<host> "<command>"`
- motorbridge has **no network transport** — it opens the CAN interface locally. The board must be the machine wired to the arm
- Record the SSH target in `../memory/local-machine-env.md` so later steps do not re-ask
- The conda environment must be activated inside the same SSH command that needs it, e.g.
  `ssh <user>@<host> "source ~/miniforge3/etc/profile.d/conda.sh && conda activate rebot && motorbridge-cli --help"`
- The gateway (Zero Calibration step) needs an `ssh -L 9002:127.0.0.1:9002` tunnel, not `0.0.0.0` binding

## Architecture Check Before Installing

`motorbridge` publishes wheels with prebuilt `libmotor_abi.so` + `ws_gateway` binaries. Supported targets: **Linux x86_64**, **Linux aarch64**, macOS arm64, Windows x86_64.

```bash
uname -m                                   # x86_64 | aarch64 -> OK ; armv6l | armv7l -> stop
python3 -m venv ~/mb-probe
~/mb-probe/bin/pip download motorbridge --no-deps --only-binary=:all: -d /tmp/mb-probe
```

If the download reports `No matching distribution found for motorbridge`, the platform is unsupported — a source install only provides the Python wrapper and cannot load the ABI. Resolve the host choice (see `AGENTS.md` Step 0) before continuing; on 32-bit Raspberry Pi OS, Miniforge has no build for that architecture either.

## Step 0: Check for Existing Installation

Before starting, check whether motorbridge is already installed:

```bash
# Linux / macOS / Git Bash:
motorbridge-cli --help 2>/dev/null || python -m motorbridge --help 2>/dev/null || echo "motorbridge not installed"

# Windows PowerShell (no 2>/dev/null):
motorbridge-cli --help *>&1 | Out-Null; if ($LASTEXITCODE -ne 0) { python -m motorbridge *>&1 | Out-Null; if ($LASTEXITCODE -ne 0) { echo "motorbridge not installed" } }
```

- If help text is displayed, motorbridge is **already installed** — skip Step 1 and Step 2, proceed directly to the next steps
- If `motorbridge not installed` is shown, continue with the steps below

---

## Step 1: Prepare Python Environment

### Linux / Jetson / Raspberry Pi

> Run these commands in **bash/zsh**.
>
> **Architecture note**: `$(uname -m)` expands to `aarch64` on a 64-bit Raspberry Pi OS (works), but to `armv6l` / `armv7l` on 32-bit Raspberry Pi OS — **Miniforge ships no build for those**, and the download below returns 404. Combined with the missing motorbridge wheels, a 32-bit Raspberry Pi cannot run the low-level stack; see [Architecture Check](#architecture-check-before-installing).

Install Miniforge:

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

If `wget` is not available, use `curl` instead:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

Create and activate a virtual environment:

```bash
conda create -y -n rebot python=3.12
conda activate rebot
```

### macOS

> Run these commands in **bash/zsh**.
>
> **Note**: On macOS, `$(uname)` returns `Darwin`, but Miniforge installers use `MacOSX`. The URL must specify it explicitly.

Install Miniforge:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
bash Miniforge3-MacOSX-$(uname -m).sh
```

Create and activate a virtual environment:

```bash
conda create -y -n rebot python=3.12
conda activate rebot
```

### Windows

> Run these commands in **PowerShell**. If using Git Bash, replace `Invoke-WebRequest` with `curl` commands.

#### Using Miniforge (Recommended)

> **Note: Windows users should download Miniforge themselves — do not have the Agent download it for you.** The Agent environment (Git Bash) uses `curl` to download from GitHub at a very slow speed, even with a proxy.

1. **Download Miniforge** (choose one method):

   **Method 1: PowerShell download (Recommended)**

   ```powershell
   Invoke-WebRequest -Uri "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86_64.exe" -OutFile "$env:USERPROFILE\Downloads\Miniforge3-Windows-x86_64.exe"
   ```

   **Method 2: Browser download**

   Open the Miniforge Release page in your browser, find the latest `Miniforge3-Windows-x86_64.exe` and download:

   ```text
   https://github.com/conda-forge/miniforge/releases
   ```

   > Do not use the system Python (e.g. `C:\Python314\python`) to directly `pip install motorbridge`, as this may fail. You must use a conda virtual environment created by Miniforge.

2. Double-click the installer to complete installation, then open **Anaconda Prompt** (or Miniforge Prompt), create and activate the virtual environment:

> **Write to memory**: After installation, record the actual Miniforge install path to `../memory/local-machine-env.md`:
>
> ```markdown
> ## conda
> - Install path: `F:\miniforge` (Git Bash: `/f/miniforge`)
> - conda.sh: `/f/miniforge/etc/profile.d/conda.sh`
> ```

   ```bash
   conda create -y -n rebot python=3.12 --no-shortcuts
   conda activate rebot
   ```

   > If `conda activate` fails with "CommandNotFoundError: Your shell has not been properly configured",
   > run `conda init cmd.exe` (or `conda init powershell`) first, then restart the terminal and activate again.

   **Git Bash users**: If `conda` is not found, Git Bash hasn't loaded the conda environment. Initialize it first:

   ```bash
   # Temporary (current terminal only), replace <install-path> with your actual path
   source <install-path>/etc/profile.d/conda.sh

   # Permanent (write to bashrc, run once)
   echo 'source <install-path>/etc/profile.d/conda.sh' >> ~/.bashrc
   source ~/.bashrc
   ```

   > Default install path example: `~/miniforge3` (i.e. `C:\Users\<Username>\miniforge3`).

   **PowerShell users**: If `conda` is not found, initialize PowerShell first:

   ```powershell
   # Initialize PowerShell (run once)
   conda init powershell
   ```

   Close and reopen PowerShell, then `conda` will be available. Alternatively, use the **Anaconda Prompt** that comes with Miniforge — no extra configuration needed.

#### Alternative: uv

> Use this only if the user already has `uv` and prefers it to Miniforge. The requirement this section exists to satisfy is **isolation**, not conda itself: a uv venv has its own interpreter and its own `Scripts\` directory, and never touches a system Python.

```powershell
uv venv .rebot-venv --python 3.12
uv pip install --python .rebot-venv\Scripts\python.exe motorbridge
.\.rebot-venv\Scripts\motorbridge-cli.exe --help
```

- Keep Python 3.12 to match the rest of this guide. `motorbridge` publishes Windows wheels for cp310–cp314, so other versions generally work too.
- Replace every `conda activate rebot` in the other references with `.\.rebot-venv\Scripts\Activate.ps1` — or skip activation entirely and call `.\.rebot-venv\Scripts\motorbridge-cli.exe` and `.\.rebot-venv\Scripts\motorbridge-gateway.exe` by full path. If `Activate.ps1` is blocked by execution policy, use the full-path form.
- `uv tool install motorbridge` is an alternative that puts `motorbridge-cli` and `motorbridge-gateway` on PATH (`%USERPROFILE%\.local\bin`), so commands in the other references run unchanged.
- If `uv` cannot write its default cache (restricted or sandboxed host), redirect it: set `$env:UV_CACHE_DIR` and `$env:UV_PYTHON_INSTALL_DIR` to writable directories.
- `uv` ignores `pip.ini`, so the `global.target` check in Step 2 does not apply.

> **Write to memory**: record the uv venv path instead of the conda fields:
>
> ```markdown
> ## Python Environment (uv)
> - venv path: `D:\work\reBot-B601-Agent-Guide\.rebot-venv`
> - Activate: `.rebot-venv\Scripts\Activate.ps1`
> ```

---

## Step 2: Install motorbridge

> **Windows users**: If you have previously configured a global `target` path for pip, `pip install` may install to the wrong location.
> Check first:
>
> ```bash
> python -m pip config list
> ```
>
> If the output includes `global.target=...`, edit `C:\Users\<YourUsername>\AppData\Roaming\pip\pip.ini`,
> add a `#` before the `target` line to comment it out, then continue.

```bash
pip install --force-reinstall --no-cache-dir motorbridge
```

> Do not pin to a specific version — use the latest stable release. After installation, verify:

```bash
motorbridge-cli --help
```

If the command is not found, check whether the Python Scripts directory is in PATH. Windows users should use the full path.

> **Write to memory**: After successful installation, record the actual motorbridge-cli path to `../memory/local-machine-env.md`:
>
> ```markdown
> ## motorbridge
> - CLI: `/f/miniforge/envs/rebot/Scripts/motorbridge-cli`
> - Installation: pip install in conda rebot environment
> ```

---

## Step 3: Platform-Specific Configuration

Complete the following setup based on your operating system and device type:

### Linux — robstride (CAN HAT, MCP2515 over SPI)

If the CAN controller is a HAT on the 40-pin SPI header (typical Raspberry Pi setup) instead of a USB adapter, `peak_usb` does not apply and `can0` will not exist until the overlay is enabled. Follow [CAN HAT Setup](setup-can-hat.md) — it covers the overlay, the reboot, `can0` at 1 Mbit/s, persistence across reboots, and a loopback self-test.

### Linux — robstride (USB-CAN: PCAN-USB / CANable)

```bash
# Load PCAN-USB driver (skip for CANable/candleLight — they use gs_usb)
sudo modprobe peak_usb
ip -br link

# Configure can0
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# Verify status
ip -br link show can0
```

### macOS — robstride (USB-CAN)

macOS uses PCAN-USB — **no can0 configuration needed**, but the PCBUSB runtime library (`libPCBUSB.dylib`) must be installed.

1. Install the PCBUSB library:

```bash
curl -L -o macOS_Library_for_PCANUSB_v0.13.tar.gz \
  https://raw.githubusercontent.com/tianrking/motorbridge/main/third_party/pcan/macos/macOS_Library_for_PCANUSB_v0.13.tar.gz
tar -xzf macOS_Library_for_PCANUSB_v0.13.tar.gz
cd PCBUSB
sudo ./install.sh
```

2. Configure `DYLD_LIBRARY_PATH` so `motorbridge-gateway` can find the PCBUSB library at runtime. Create an activation script in the conda environment — it takes effect automatically on every `conda activate rebot`:

```bash
mkdir -p "$CONDA_PREFIX/etc/conda/activate.d"
cat > "$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh" << 'EOF'
export DYLD_LIBRARY_PATH="/usr/local/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
EOF
```

> **Note**: On macOS, `motorbridge-gateway` calls `dlopen("PCBUSB")` which does not search `/usr/local/lib/` by default. `DYLD_LIBRARY_PATH` is required to make it discoverable. `motorbridge-cli` is not affected.

3. Verify the installation:

```bash
# Check Python package and CLI
python -c "import motorbridge; print('motorbridge OK')"
motorbridge-cli --help

# Check that the PCBUSB runtime can be loaded
python -c "import ctypes; ctypes.CDLL('libPCBUSB.dylib'); print('PCBUSB load OK')"

# Confirm the environment variable is set
echo $DYLD_LIBRARY_PATH
```

> **Write to memory**: After installation, update `../memory/local-machine-env.md`:
>
> ```markdown
> - PCBUSB library: installed (macOS)
> - DYLD_LIBRARY_PATH: auto-set to `/usr/local/lib` via `$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh`
> ```

### Windows — robstride (USB-CAN)

Windows uses the PCAN-USB driver directly — **no can0 configuration needed**.

1. On first use, download and install the driver from the [PCAN-USB official page](https://www.peak-system.com/products/hardware/external-pc-interfaces/pcan-usb/)
2. Plug in the PCAN-USB adapter
3. No additional configuration required — use `motorbridge-cli` directly

> **Write to memory**: After installing the PCAN-USB driver, update `../memory/local-machine-env.md`:
>
> ```markdown
> - PCAN-USB driver: installed
> ```

### Linux — damiao (USB serial)

1. Confirm the serial device exists:

```bash
ls -l /dev/ttyACM0
```

2. Check permissions. The output should be `crw-rw----` with group `dialout`. Verify the current user is in the group:

```bash
groups
```

If the output does not include `dialout`, run:

```bash
sudo usermod -aG dialout $USER
```

> After joining the group, you need to **log out and log back in** (or reboot) for it to take effect. After re-login, confirm again that `groups` output includes `dialout`, otherwise the serial port will be inaccessible.

### Windows — damiao (USB serial)

1. Plug in the damiao motor USB serial cable
2. Confirm the COM port number in Device Manager
3. Use `--serial-port COMx` to specify the corresponding port (e.g. `COM3`)

> **Write to memory**: After confirming the COM port, update `../memory/local-machine-env.md`:
>
> ```markdown
> - damiao serial port: COMx
> ```
