## Zero Calibration

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

## Prerequisites

- Motor IDs have been correctly set via the write motor ID process
- `can0` is configured and UP (Linux needs configuration in environment setup; a Raspberry Pi CAN HAT needs the SPI overlay and reboot first — see [CAN HAT Setup](setup-can-hat.md); Windows/macOS do not)
- On a CAN HAT, `can0` must be up **and** persistent: after a reboot the gateway otherwise fails with `SocketCAN write failed: Network is down`

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
> taskkill /F /IM motorbridge-gateway.exe
> ```
>
> ```powershell
> # PowerShell
> Stop-Process -Name "motorbridge-gateway" -ErrorAction SilentlyContinue
> ```
>
> Restart the Gateway after scanning is complete. It's recommended to complete all CLI scan and motor ID setup work before starting the Gateway.

### 1b. Remote Host over SSH

When the arm is wired to a board (Raspberry Pi / Jetson) and the user's browser is on another machine, the gateway stays bound to the board's loopback and the port is forwarded to the user's machine. Do **not** bind `0.0.0.0` — that requires `MOTORBRIDGE_WS_TOKEN` and exposes motor control to the network.

1. Start the gateway on the board, detached so it survives the SSH session:

```bash
ssh <user>@<host> "source ~/miniforge3/etc/profile.d/conda.sh && conda activate rebot && \
  setsid nohup motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0 \
  >/tmp/motorbridge-gateway.log 2>&1 &"
```

2. Confirm it is listening (and check the log if not):

```bash
ssh <user>@<host> "ss -ltnp | grep 9002; tail -5 /tmp/motorbridge-gateway.log"
```

3. Keep a forwarding session open on the user's machine:

```bash
ssh -L 9002:127.0.0.1:9002 <user>@<host>
```

4. The user then opens Motorbridge Studio and points it at `127.0.0.1:9002` — that address is now the board's gateway through the tunnel.

5. When calibration is finished, stop the gateway on the board:

```bash
ssh <user>@<host> "pkill -f ws_gateway"
```

> The tunnel must stay open for the whole calibration. If the browser reports a connection failure, check the SSH session first, then `ss -ltnp` on the board — not the address in the web UI.

### 2. Open Motorbridge Studio

Direct the user to: https://motorbridge.github.io/motorbridge-studio/

Complete the zero calibration via the web interface.
