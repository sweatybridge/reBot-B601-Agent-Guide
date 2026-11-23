---
name: rebot-motor-rezero
description: Re-zero a RobStride motor on the reBot B601 bench via raw SocketCAN frames on the Pi Zero W (armv6l cannot run motorbridge/Motorbridge Studio). Use when joint-zero drift is flagged, the gripper j7 reads past the closed stop at physical rest, or the user asks to re-zero / zero-calibrate a motor on this bench. Covers diagnosis, the verified type-6/type-22 protocol, a stdlib-only script, and verification.
version: 1.0.0
---

# reBot motor re-zero over raw CAN (Pi Zero W)

The bench edge board is a **Pi Zero W Rev 1.1 (armv6l, 32-bit Raspbian 12)**.
motorbridge, its gateway, and the Motorbridge Studio zero-calibration UI
**cannot run on it** (no armv6l wheels; see
`en/references/demo-validation-pi-zero.md`). The Rust edge player only reads
parameter 0x7019 and plays trajectories — it has no zero-write. The documented
Studio flow therefore does not apply; the zero is set with **two raw CAN
frames** sent from a stdlib-only Python script on the Pi itself.

## When to use

- A joint reads far from its rest reference while the arm is physically at
  rest (e.g. gripper j7 ≈ −0.186 rad when the jaws-closed rest reference is 0).
- The oracle flags `joint-zeros` (constant per-joint bias across poses,
  > 30 mrad, spread ≤ 5 mrad — `examples/host/app/oracle.py`).
- Spec §7.2: zero drift is a re-zeroing job, never absorbed into the camera
  rig; the tag cannot observe joint zeros.

## Gripper convention (j7)

- Zero = **jaws fully closed stop**. `MOTOR7_CLOSED = −0.015` rad (close
  margin into the stop), `MOTOR7_OPEN = 4.90` rad
  (`examples/host/app/plan_to_target.py`). Configured j7 limits: [−0.1, 5.0].
- A reading *below* the closed stop at rest cannot be jaw position (rigid
  jaws cannot pass the stop) — it is a zero-reference offset.
- Arm joints 1–6 rest reference = 0; ≤ ~10 mrad at rest is normal slop, not
  drift (oracle flag threshold is 30 mrad).

## Protocol (verified — do not guess)

Sources: RobStride RS00 manual §4.1.5 / §4.1.9,
`RobStride/Python_Sample` → `robstride_dynamics/protocol.py`, and the frame
layout in `examples/edge-rust/src/main.rs` (`ext_id`, `read_position`).

- 29-bit extended ID: `(comm_type << 24) | (host_id << 8) | motor_id`,
  host_id = **0xFF**. SocketCAN `can_id` needs the **0x80000000 EFF flag** or
  the kernel sends an 11-bit frame the motors silently ignore.
- **Type 17** read parameter: data[0:2] = index (LE), rest 0. Reply: type 17,
  value in data[4:8]. `0x7019` = mechanical position (float32 LE, rad);
  `0x7029` = zero_sta (uint8, 1 = zero set). Replies may echo the request
  (motor id in ID low byte) or use the receive layout (host in low byte,
  motor in bits 15–8) — accept both.
- **Type 4** disable: 8 zero bytes (no torque during the zero set).
- **Type 6** set mechanical zero: data `01 00 00 00 00 00 00 00`
  (Byte[0]=1) → sets the **current position to 0**. Reply: type-2 feedback.
- **Type 22** save parameters: data `01 02 03 04 05 06 07 08` → persists to
  flash. **The firmware acknowledges with a type-0 ID broadcast, NOT type 2**
  — listening only for type 2 reports a false "no response".
- Type 18 parameter writes alone are "lost in power failure" — always follow
  with the type-22 save.

### Traps

- **0x7016 is the position TARGET (`lof_ref`), not the zero.** Zero-set is a
  communication type (6), not a parameter write. Never improvise a zero
  write from a guessed parameter index.
- Zero only the affected motor ID — frames are addressed; other joints
  ignore them. Verify joints 1–6 unchanged afterwards.
- Never zero while the motor is enabled/holding: the position loop's
  reference frame would shift under torque. Disable (type 4) first.
- Do not use motorbridge-cli commands outside the guide (they may enable
  motors). Raw frames below send no motion commands.

## Preconditions (all required)

1. Edge **idle**: `GET /health` via the planner container shows
   `playing: false`, `can_up: true`.
2. Arm at rest; for j7 the **jaws physically fully closed — user confirms**
   (that position becomes the new zero).
3. **Explicit user consent**: this is a persistent flash write, outside the
   guide's documented Studio flow. Undoing it means re-zeroing at a different
   physical position; the old offset is not stored anywhere.
4. Read-only before/after evidence: `/pose` via the planner container
   (`REBOT_EDGE_URL`, Bearer `REBOT_EDGE_TOKEN`), e.g.
   `docker exec rebot-capture-planner-1 python -c "..."`.

## Access

- SSH: **`qh812@raspberrypi`** (the `qiao` user's key auth is denied).
  Key auth works from the Windows host with `-o BatchMode=yes`.
- Pipe the script: `Get-Content script.py -Raw | ssh -o BatchMode=yes
  qh812@raspberrypi "python3 -"`. Pi has Python 3.11 (stdlib only —
  `socket(AF_CAN, SOCK_RAW, CAN_RAW)` needs no root and no installs).
- The edge's own CAN socket coexists fine: every CAN_RAW socket receives
  copies of all frames; just ensure the edge is not playing.

## Script (stdlib only; set MOTOR to the affected id)

```python
import socket, struct, sys, time

MOTOR, HOST, EFF = 7, 0xFF, 0x80000000

def ext_id(comm):
    return (comm << 24) | (HOST << 8) | MOTOR

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(("can0",))
s.settimeout(0.5)

def send(comm, data):
    s.send(struct.pack("=IB3x8s", ext_id(comm) | EFF, 8, data.ljust(8, b"\x00")))

def match(cid, comm):
    cid &= 0x1FFFFFFF
    c, extra, dev = (cid >> 24) & 0x1F, (cid >> 8) & 0xFFFF, cid & 0xFF
    return c == comm and (dev == MOTOR or extra & 0xFF == MOTOR)

def read_pos():
    send(17, struct.pack("<H6x", 0x7019))
    end = time.time() + 1.0
    while time.time() < end:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        cid = struct.unpack("=I", raw[:4])[0]
        if match(cid, 17):
            return struct.unpack("<f", raw[12:16])[0]
        if (cid & 0x1FFFFFFF) >> 24 == 21:
            sys.exit("FAULT frame on the bus: %s" % raw[8:16].hex())
    return None

def feedback():
    end = time.time() + 1.0
    while time.time() < end:
        try:
            raw = s.recv(16)
        except socket.timeout:
            continue
        cid = struct.unpack("=I", raw[:4])[0]
        if match(cid, 2):
            return raw[8:16]
        if (cid & 0x1FFFFFFF) >> 24 == 21:
            sys.exit("FAULT frame on the bus: %s" % raw[8:16].hex())
    return None

before = read_pos()
if before is None:
    sys.exit("no reply to the 0x7019 read -- is the bus up?")
print("before: %+.6f rad" % before)
if abs(before) < 0.02:
    sys.exit("already within 20 mrad of zero -- nothing to do")

send(4, b"\x00" * 8)              # disable: no torque during the zero set
print("disable feedback:", (feedback() or b"NONE").hex())
send(6, b"\x01" + b"\x00" * 7)    # type 6: current position -> zero
print("zero-set feedback:", (feedback() or b"NONE").hex())
send(22, bytes(range(1, 9)))      # type 22: save to flash
time.sleep(0.5)                   # ack is a type-0 broadcast; drain it
end = time.time() + 2.0
while time.time() < end:
    try:
        raw = s.recv(16)
    except socket.timeout:
        continue
    cid = struct.unpack("=I", raw[:4])[0] & 0x1FFFFFFF
    if (cid >> 24) & 0x1F == 0 and (cid & 0xFF == MOTOR or (cid >> 8) & 0xFF == MOTOR):
        print("save ack (type-0 broadcast):", raw[8:16].hex())
        break
else:
    print("save ack: NONE -- re-send type 22 and listen before trusting persistence")

time.sleep(0.2)
after = read_pos()
if after is None:
    sys.exit("no reply to the verification read")
print("after: %+.6f rad" % after)
print("OK" if abs(after) < 0.02 else "VERIFY FAILED -- zero did not take")
```

## Verification

1. Script prints `after ≈ 0` (±20 mrad) and `OK`.
2. Read `zero_sta` (0x7029, uint8 in reply data[4]) = **1**.
3. Independent `/pose` read via the planner: target joint ≈ 0, **all other
   joints unchanged** (compare with the pre-run snapshot).
4. Persistence across power cycle is proven only by an actual reboot: at the
   next bench power-up, `/pose` at rest must still read ≈ 0. If the old
   offset returns, the save did not stick — redo type 22 (listen for the
   type-0 ack) and re-verify.

## Bench context pitfalls

- The stack drifts between sessions (state dir, capture port/TLS, live rig).
  Containers bind-mount the main checkout: a git branch switch silently
  swaps live code. Verify current branch + live state location before
  diagnosing.
- A single rest snapshot cannot separate zero drift from rest slop for
  joints 1–6; the definitive test is the oracle's constant-bias signature
  on the next executed run. For j7, a reading past the closed stop is
  decisive on its own.
- After a j7 re-zero at the closed stop, the code constants line up again
  (close −0.015 presses the designed margin into the stop; open 4.9 opens
  as intended). No code change is needed.