# Local Machine Environment Configuration

> **Usage**: This file stores environment information for the current machine to avoid redundant setup and configuration.
> The Agent will automatically read and update this file during the workflow.
>
> **File location**: Place this file in the current project or a fixed path. Ensure `memory/local-machine-env.md` is accessible before running the workflow.

---

## Control Host

<!-- Automatically written by Agent after the control host is identified -->

- Host type (local / remote SSH):
- SSH target (`user@host`):
- OS / release:
- Architecture (`uname -m`):
- motorbridge wheel supported (yes / no):

## CAN Interface

<!-- Automatically written by Agent after the CAN path is configured -->

- Adapter (PCAN-USB / CANable / CAN HAT MCP2515 / damiao serial):
- Interface name and bitrate (`can0` @ 1000000):
- Configured persistently across reboots (yes / no, and how):
- CAN HAT overlay line / config file:

## conda

<!-- Automatically written by Agent after Miniforge installation -->

- Install path:
- conda.sh:

## motorbridge

<!-- Automatically written by Agent after motorbridge installation -->

- CLI path:
- Installation method:

## PCAN-USB

<!-- Automatically written by Agent after driver installation -->

- Driver status:

## Damiao Serial Port

<!-- Automatically written by Agent after COM port is confirmed -->

- Serial port:

## Motor Configuration

<!-- Automatically written by Agent after device information is collected -->

- Motor type (robstride / damiao):
- Device version (unassembled kit / assembled):
- Target ID range:

### Configured Motors

<!-- Automatically appended by Agent after each motor ID is set -->

- Configured IDs:
- Final scan time:
