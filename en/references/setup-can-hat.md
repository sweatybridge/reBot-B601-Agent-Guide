## CAN HAT on a Raspberry Pi (MCP2515 over SPI)

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

Use this reference when the arm's CAN bus reaches the control host through a **CAN HAT** — an MCP2515 controller wired to the 40-pin SPI header — instead of a USB-CAN adapter. It replaces the `peak_usb` / `can0` steps in [Environment Setup](setup-environment.md) for that hardware.

**Verified on**: Raspberry Pi Zero W Rev 1.1, Raspbian GNU/Linux 12 (bookworm), kernel 6.6.20, Waveshare RS485 CAN HAT (12 MHz crystal). The same steps apply to any Raspberry Pi with a 40-pin header and to other SPI MCP2515 boards.

## Before You Begin

- [ ] The HAT is fully seated on the 40-pin header (a half-seated HAT usually fails at step 3)
- [ ] `CAN_H` / `CAN_L` from the HAT go to the arm's `H` / `L` (H-H, L-L, never crossed)
- [ ] The arm is powered from its own supply. **Never power the arm from the Pi's 5 V rail.**
- [ ] Bus termination: the HAT has an onboard 120 Ω resistor (jumper/solder bridge). Enable it only if the HAT is an end node; the arm already terminates its end. Two 120 Ω in parallel (60 Ω) at 1 Mbit/s usually still works on a bench, but one terminator per end is correct.

## Step 1: Check the HAT's Crystal

The overlay must be told the real oscillator frequency, otherwise the controller is configured for the wrong bitrate and the motors never answer.

- **12 MHz** (current boards, silkscreen/imprint marks the crystal `12.000`) → use `oscillator=12000000`
- **8 MHz** (boards bought before August 2019) → use `oscillator=8000000`

## Step 2: Enable SPI and the Overlay

Edit the firmware config file for your OS release:

| OS release | Path |
|---|---|
| Bookworm (12) and later | `/boot/firmware/config.txt` |
| Bullseye (11) and earlier | `/boot/config.txt` |

Append (once — check with `grep mcp2515 <file>` first):

```bash
sudo tee -a /boot/firmware/config.txt >/dev/null <<'EOF'

# B601-RS: RS485 CAN HAT (MCP2515, 12MHz crystal, INT=GPIO25)
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
EOF
```

- `interrupt=25` is BCM GPIO25 — the HAT's `INT` pin. If the pin is not connected or the number is wrong, the controller still initializes but **loses received frames**, which looks like "motors randomly stop answering".
- `spimaxfrequency=2000000` is the Waveshare default and is safe at 1 Mbit/s. If you see `rx_over_errors` climbing under load, you can raise it (the MCP2515 supports up to 10 MHz).

## Step 3: Reboot and Verify the Controller

```bash
sudo reboot
```

After it comes back:

```bash
sudo dmesg | grep -i mcp251
```

Expected:

```
mcp251x spi0.0 can0: MCP2515 successfully initialized.
```

If you instead see `Cannot initialize MCP2515. Wrong wiring?`, the HAT is not seated, SPI is off, or the overlay is missing.

## Step 4: Bring `can0` Up at 1 Mbit/s

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 65536
```

Verify:

```bash
ip -details link show can0
```

Expected highlights:

```
can state ERROR-ACTIVE restart-ms 100
	  bitrate 1000000 sample-point 0.666
	  clock 6000000
```

> **`clock 6000000` is correct, not a misconfiguration.** The `mcp251x` driver reports half the crystal frequency (`can.clock.freq = freq / 2` in the driver), so a 12 MHz HAT always shows 6 MHz here. `bitrate 1000000` is the value that matters.

## Step 5: Self-Test the HAT Without the Arm

This proves SPI, the driver and the bitrate are right before you blame the motors. It needs no extra packages — the loopback happens inside the MCP2515, so it does **not** test the transceiver or the cabling.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback on
sudo ip link set can0 up
sudo python3 - <<'PY'
import socket, struct
SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, CAN_RAW_LOOPBACK = 101, 4, 3
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, 1)
s.bind(("can0",))
s.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 1)
s.setsockopt(SOL_CAN_RAW, CAN_RAW_LOOPBACK, 1)
s.settimeout(1.0)
s.send(struct.pack("=IB3x8s", 0x123, 2, b"\xde\xad" + b"\x00" * 6))
cid, dlc = struct.unpack("=IB3x", s.recv(16)[:8])
print("loopback ok: id=0x%X dlc=%d" % (cid, dlc))
PY
```

Expected: `loopback ok: id=0x123 dlc=2`. Then restore normal mode:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 loopback off
sudo ip link set can0 up
```

> **Bus presence check (optional)**: in normal mode, send one frame to an ID the motors ignore (e.g. `0x7FF`). If a powered node is on the bus it ACKs the frame and `ip -details link show can0` stays `ERROR-ACTIVE` with `tx_errors=0`. With nothing on the bus, the controller retries and ends in `ERROR-PASSIVE`/`BUS-OFF`. This distinguishes "arm unpowered / unplugged" from "arm not answering".

## Step 6: Make It Persistent (Recommended)

`can0` comes back DOWN after every reboot unless something configures it. A oneshot unit is the most portable way — it does not depend on `ifupdown`, `dhcpcd` or NetworkManager:

```bash
sudo tee /etc/systemd/system/can0-setup.service >/dev/null <<'EOF'
[Unit]
Description=Configure can0 (MCP2515 CAN HAT) at 1 Mbit/s
After=sys-subsystem-net-devices-can0.device
Wants=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
# mcp251x loads late on slow boards; wait for the netdev instead of racing it
ExecStartPre=/bin/sh -c "for i in $(seq 1 150); do [ -e /sys/class/net/can0 ] && exit 0; sleep 0.2; done; exit 1"
ExecStart=/sbin/ip link set can0 down
ExecStart=/sbin/ip link set can0 type can bitrate 1000000 restart-ms 100
ExecStart=/sbin/ip link set can0 up
ExecStart=/sbin/ip link set can0 txqueuelen 65536
ExecStartPost=/bin/sh -c "/sbin/ip -details link show can0 | /bin/grep -q 'state UP'"

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now can0-setup.service
```

Verify it survives a real boot — not just `systemctl start`:

```bash
sudo systemctl reboot
# after reboot
systemctl is-active can0-setup.service     # -> active
ip -details link show can0 | grep -E "can state|bitrate"
```

> **Timing note**: on a Pi Zero W the `mcp251x` driver initializes `can0` around 45–50 s into boot. Without the `ExecStartPre` wait (or the `.device` dependency) the unit starts first, fails with `Cannot find device "can0"`, and silently leaves the bus down.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `dmesg` has no `mcp251x` line | SPI off, overlay missing, or HAT not seated | Re-check step 2, `dtparam=spi=on` must be uncommented |
| `Cannot initialize MCP2515. Wrong wiring?` | HAT not fully seated / SPI disabled | Reseat the HAT, reboot |
| `can0` exists but motors never answer | Wrong `oscillator=` value → wrong real bitrate | Check the crystal (step 1) and re-apply the overlay |
| `bus-off` right after `up` | No other node on the bus, or wiring crossed | Check power to the arm, swap H/L |
| `rx_over_errors` climbing | SPI too slow for the traffic | Raise `spimaxfrequency` (e.g. 10000000) |
| `ip link set can0` works, then down after reboot | Persistence unit missing/failed | Step 6, `journalctl -u can0-setup.service -b` |
| `RTNETLINK answers: Operation not permitted` | `sudo` missing | Prefix with `sudo` |
