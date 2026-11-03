## Write Motor ID

> **Note: This reference document is a beta version and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**
> **Reminder: Ensure motors are correctly connected and powered on before executing this process. Observe electrical safety precautions.**

## Prerequisites

Environment setup has been completed or motorbridge is already installed.

Verify motorbridge-cli is available before proceeding:

```bash
motorbridge-cli --help 2>/dev/null || echo "motorbridge-cli not found"
```

If the command is not found:
- Linux/macOS: ensure the conda environment is activated (`conda activate rebot`)
- Windows: use the full path, e.g. `<Python_install_dir>\Scripts\motorbridge-cli.exe`

> **Important: Motors must be powered on (RS: 48V, DM: 24V) to respond to scans.** If a scan fails, first ask the user to verify that power is applied.

## Identify Device Type

Before proceeding, identify which protocol the motors use:

| Type | Connection | Description |
|------|------------|-------------|
| robstride | USB-CAN (PCAN-USB / CANable) | Linux uses socketcan `can0`; Windows driver auto-detects |
| robstride | CAN HAT (MCP2515 over SPI) | Raspberry Pi: configure once with [CAN HAT Setup](setup-can-hat.md), then `can0` at 1 Mbit/s |
| damiao | USB serial | Linux uses `/dev/ttyACM0`; Windows uses `COMx` |

> **Remote host**: if these motors are wired to a board you reach over SSH, prefix every command with `ssh <user>@<host> "…"`. When the board has a conda environment, chain the activation into the same command (`"source ~/miniforge3/etc/profile.d/conda.sh && conda activate rebot && motorbridge-cli ..."`).

## CLI Command Path Convention

All commands below use `motorbridge-cli` as the command name. In practice:

- **Linux/macOS** (conda environment activated): run `motorbridge-cli` directly
- **Windows** (conda environment not in PATH): use the full path, e.g.:

  ```
  <miniforge_path>\envs\rebot\Scripts\motorbridge-cli.exe
  ```

  Or run `conda activate rebot` first, then use `motorbridge-cli` directly.

---

## Damiao vs RobStride ID Writing Differences

| | Damiao | RobStride |
|---|---|---|
| Device ID register | `ESC_ID` (rid=8) | `device_id` |
| Feedback ID register | `MST_ID` (rid=7) | `host_id` (not modified) |
| Default device ID | `0x01` | `127` (0x7F), but some batches may be pre-set to 1–7 |
| Default feedback ID | `0x11` (= motor_id + 0x10) | `0xFD` (fixed value) |
| Modify feedback ID when changing ID | **Yes**, `feedback_id = motor_id + 0x10` | **No**, only device_id is changed |
| Parameters | `--motor-id` / `--new-motor-id` + `--feedback-id` / `--new-feedback-id` | `--motor-id` / `--new-motor-id` |

> Damiao motors have a fixed offset between `feedback_id` and `motor_id`: `feedback_id = motor_id + 0x10`.
> For example: motor_id=0x01 → feedback_id=0x11, motor_id=0x05 → feedback_id=0x15.

---

## Step 1: Device Initialization

Complete initialization based on your OS and device type.

### Linux — robstride (USB-CAN)

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

### Linux — robstride (CAN HAT, MCP2515 over SPI)

The HAT does not use `peak_usb`; `can0` only appears after the SPI overlay is enabled and the board has rebooted. Follow [CAN HAT Setup](setup-can-hat.md) once, then confirm the interface before scanning:

```bash
ip -details link show can0 | grep -E "can state|bitrate"
# can state ERROR-ACTIVE / bitrate 1000000
```

### Windows — robstride (USB-CAN)

PCAN-USB driver works automatically after installation — **no can0 setup required**. Just confirm the adapter is plugged in.

### Linux — damiao (USB serial)

1. Confirm the serial device exists: `ls -l /dev/ttyACM0`
2. Confirm user is in `dialout` group: `groups`
3. If not: `sudo usermod -aG dialout $USER` (requires re-login)

### Windows — damiao (USB serial)

1. Confirm COM port number in Device Manager
2. Use `COMx` for `--serial-port` in subsequent commands (e.g. `COM3`)

---

## Step 2: Check Existing Motors

Run the appropriate command for your device type:

**robstride (Linux):**
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 8
```

**robstride (Windows):** same command as Linux — `--channel can0` is still required (PCAN driver maps it automatically at the lower level):
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 8
```

**damiao (Linux):**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 32
```

**damiao (Windows):** replace `--serial-port /dev/ttyACM0` with `--serial-port COMx`

Decision logic:
- **robstride**: If motors 1–7 are detected, inform the user that the robotic arm may already be a finished product and ID modification is not needed. This process does not need to be executed.
- **damiao**: Determine whether modification is needed based on detected IDs.

---

## Step 3: Modify Motor ID

### Connect a New Motor

1. Ask the user to **power off** and unplug the cable from the current motor
2. Connect the cable to the next motor
3. **Turn on power** (RS: 48V, DM: 24V; motors must be powered to communicate)
4. After the user confirms power is on, proceed to **Find Current Motor ID**

### Find Current Motor ID

Ensure the user has **only one motor connected** before proceeding.

> **Scan strategy: scan both factory default range and 1–7 range simultaneously.** Some batches ship with IDs pre-set to 1–7 rather than the default 127.

**robstride (Linux / Windows, same command):**

```bash
# Scan both ranges simultaneously to determine motor status
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7 --timeout-ms 300
motorbridge-cli scan --vendor robstride --channel can0 --start-id 126 --end-id 127 --timeout-ms 300
```

> On Windows, `--channel can0` is still required — the PCAN driver maps it automatically at the lower level. No can0 interface configuration is needed beforehand.

**damiao:**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 1
```

- **robstride** hits **127**: Motor has not had its ID set. Proceed to **Modify Current Motor ID**.
- **robstride** hits **a value in 1–7**: ID is pre-set. If it matches the target ID, this motor is done; otherwise ask the user if they want to re-modify.
- **damiao** hits **0x01**: Motor has not had its ID set. Proceed to **Modify Current Motor ID**.
- **Default ID not hit**: Expand scan range (1–32 or 1–50). If a specific ID is found, inform the user and ask if they want to re-modify.
- **No motors detected**: Ask the user to check: (1) Is power on (RS: 48V, DM: 24V)? (2) Are CAN/serial cables firmly connected? (3) Is the motor functional?

### Modify Current Motor ID

After confirming the motor's current ID, execute the modification:

**robstride (Linux / Windows, same command):**
```bash
# Example: change ID 127 to 5
motorbridge-cli id-set --vendor robstride --channel can0 --motor-id 127 --new-motor-id 5
```

**damiao:**
```bash
# Example: change ESC_ID 0x05 to 1, MST_ID automatically changes from 0x15 to 0x11
motorbridge-cli id-set \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --motor-id 5 \
    --feedback-id 0x15 \
    --new-motor-id 1 \
    --new-feedback-id 0x11
```

> **RobStride id-set timeout note**: After running `id-set` on robstride motors, you may see a `store_parameters failed: response ack timeout` error (exit code 2). This occurs because the motor restarts after writing the ID, causing the confirmation to time out. **The ID has actually been written successfully.** Do not retry when this error occurs — proceed directly to **Scan Confirmation** to verify the ID has changed.

On successful write, Damiao motors will output something like:

```
write rid=7 (MST_ID) <= 0x11
write rid=8 (ESC_ID) <= 0x1
store_parameters sent
verify rid=8 (ESC_ID): 0x1
verify rid=7 (MST_ID): 0x11
verify ok
```

> Damiao motors restart after ID modification. The command may end with a `get_register_u32 failed` timeout message. This indicates verification failed — immediately run a **scan** to confirm whether the ID was actually written. If not, re-modify.

### Scan Confirmation

Immediately after modification, scan to confirm:

```bash
# robstride: scan both old and new ID
motorbridge-cli scan --vendor robstride --channel can0 --start-id <new_id> --end-id <new_id> --timeout-ms 300

# damiao: same, with transport parameters
```

- If new ID hits and old ID doesn't respond → modification successful
- If old ID still hits → modification failed, re-run `id-set`

---

## Step 4: Final Confirmation

After all 7 motors are configured, connect all motors simultaneously and run a final scan:

**robstride (Linux / Windows, same command):**
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7 --timeout-ms 500
```

**damiao:**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 32
```

On successful scan, the CLI will output something like:

```
[hit] probe=0x01 vendor=robstride via=ping feedback_id=0xFD device_id=1 responder_id=254
[hit] probe=0x02 vendor=robstride via=ping feedback_id=0xFD device_id=2 responder_id=254
...
scan done: 7 motor(s) found
```

Inform the user which motors were detected. Once the user confirms all IDs 1–7 are correct, this process is complete; if any are missing, go back to re-modify the corresponding motor.
