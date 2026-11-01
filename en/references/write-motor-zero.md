## Zero Calibration

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

## Prerequisites

- Motor IDs have been correctly set via the write motor ID process
- `can0` is configured and UP (Linux needs configuration in environment setup; Windows/macOS do not)

## Steps

### 1. Activate Environment and Start Gateway

**Linux:**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0
```

**macOS (routing mode recommended):**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002
```

macOS uses PCAN-USB — no can0 configuration needed. In routing mode, only `--bind` is needed — the web UI will auto-select vendor/channel/ID. To specify a channel explicitly:

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --channel can0@1000000
```

**Windows (routing mode recommended):**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002
```

In routing mode, only `--bind` is needed — the web UI will auto-select vendor/channel/ID. To specify a channel explicitly:

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0
# or with bitrate
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0@1000000
```

> **Windows `conda activate` tip**: If you get "CommandNotFoundError", conda hasn't been initialized for this shell. Use Miniforge Prompt, or the full path: `<miniforge_install_path>\Scripts\activate.bat rebot`. Git Bash users see the initialization steps in the [Environment Setup Reference](setup-environment.md).

Keep the Gateway running — do not close this terminal.

> **Important: The Gateway exclusively occupies the CAN bus**. While the Gateway is running, `motorbridge-cli scan` and other CLI commands will fail to communicate with motors and return `0 motor(s) found`.
> To scan motor IDs while the Gateway is running, you must stop it first:
>
> ```bash
> # Linux
> pkill -f ws_gateway
> ```
>
> ```cmd
> :: Windows cmd
> taskkill /F /IM ws_gateway.exe
> ```
>
> ```powershell
> # PowerShell
> Stop-Process -Name "ws_gateway" -Force -ErrorAction SilentlyContinue
> ```
>
> **Windows note**: `motorbridge-gateway.exe` is only a launcher — the process that owns the bus and the port is its child `ws_gateway.exe`. Killing the launcher alone leaves `ws_gateway.exe` running, so the port stays bound and the next start fails with `Os { code: 10048, kind: AddrInUse }`. Kill `ws_gateway.exe`; the launcher exits with it.
>
> Restart the Gateway after scanning is complete. It's recommended to complete all CLI scan and motor ID setup work before starting the Gateway.

### 2. Open Motorbridge Studio

Direct the user to: https://motorbridge.github.io/motorbridge-studio/

Complete the zero calibration via the web interface.
