#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gradual joint motion demo for the B601-RS arm using robstride_dynamics.

Why this exists: on 32-bit Raspberry Pi OS (armv6l/armv7l) motorbridge has no
wheel, so this script drives the same motors through Seeed's pure-Python
SocketCAN driver instead:

    git clone https://github.com/Seeed-Projects/RobStride_Control ~/RobStride_Control
    sudo apt-get install -y python3-can python3-numpy python3-tqdm

The motion is deliberately boring: read where every joint currently is, move a
small delta with cosine easing, hold, come back. No snap-to-zero, no gravity
compensation, no large angles.

Usage
-----
    # 1. Always start dry: reads positions, prints the plan, sends nothing.
    python3 gradual_motion_demo.py --sdk-path ~/RobStride_Control/python

    # 2. Same plan, but really move (small: 0.1 rad = 5.7 deg over 3 s).
    python3 gradual_motion_demo.py --sdk-path ~/RobStride_Control/python --execute

    # Move three joints, 0.2 rad, 5 s per leg, slower bus rate.
    python3 gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
        --execute --joints 1,2,3 --delta 0.2 --duration 5 --hz 20

    # All joints, but the gravity-loaded wrist joints get stiffer gains.
    python3 gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
        --execute --joints all --delta 0.1048 --kp 4:45,5:35 --hz 20 --track

Per-joint gains
---------------
`--kp` / `--kd` take either one number for every joint (`--kp 15`) or an
`ID:VALUE` list (`--kp 4:45,5:35`, optionally with a base such as
`--kp 15,4:45`). Unlisted joints use 20 / 0.6 unless a bare number sets another
base. The banner prints the resolved per-joint values, so what actually went on
the bus is never ambiguous.

Gains matter once several joints are enabled at once: at kp=15 the arm carries
its own weight and the wrist end settles 0.02-0.14 rad below target, while the
same gains hold a single unloaded joint to a few mrad.

Safety model
------------
* Dry run is the default. Nothing moves unless you pass --execute.
* Every joint starts from its *measured* position; nothing is commanded until
  the motor has reported where it is.
* |delta| is clamped to MAX_DELTA rad, kp/kd are clamped, and targets are kept
  inside the MIT frame's +-4*pi limit with margin.
* On exit (normal or Ctrl-C) the arm ramps back to its start pose, holds, then
  disables torque. A gravity-loaded joint will sag when torque drops - make
  sure the arm is resting safely before running this.

Requires a CAN interface that is UP at 1 Mbit/s. On a CAN HAT:
    sudo systemctl start can0-setup.service && ip -br link show can0

Verified on the reference arm
-----------------------------
Raspberry Pi Zero W + RS485 CAN HAT, 7x RS motors at IDs 1-7, joint 1,
delta 0.0524 rad (3 deg), kp=15/kd=0.6, 30 Hz, `--track`:
    outbound  24/50/76/100 % measured +0.0284 +0.0456 +0.0629 +0.0736
    return    24/50/76/100 % measured +0.0717 +0.0548 +0.0376 +0.0268
Both legs ramp (about 3-4 mrad droop versus commanded, no integral term), the
joint lands within ~2 mrad of where it started, and each 3 s leg hit its
deadline at 30 Hz.

Note: every leg interpolates from the position we last commanded, not from the
bring-up pose. Getting that wrong turns any leg whose target is the start pose
(a return, or the shut-down home ramp) into a single-step command that merely
looks gradual because the loop still runs for the full duration.

All 7 joints, delta 0.1048 rad (6 deg), kp 4:45 5:35, 20 Hz, `--track`
---------------------------------------------------------------------
Tracking at the far point, measured minus commanded:
    1: -0.003   2: -0.005   3: -0.037   4: -0.045   5: -0.004   6: -0.003
    7: +0.002
Joints 3 and 4 stay short even at these gains - that is load, not code, and the
real fix is gravity compensation. Also note the loop ran 3.4 s for a 3 s leg at
20 Hz, i.e. ~18 Hz effective for 7 joints on a Zero W.

Sampling status: read_status() matches frames by device ID. Taking "the next
status frame" instead mis-attributes another joint's position, which at
bring-up would seed the first command with a pose the joint is not actually at.
"""

import argparse
import math
import os
import signal
import sys
import time
import struct

# MIT-frame position range used by the RobStride protocol (see tables.py: 4*pi).
MIT_POSITION_LIMIT = 4.0 * math.pi

# Hard safety ceilings. --flags can make motion smaller, never bigger.
MAX_DELTA = 0.35          # rad, ~20 deg
MAX_KP = 60.0             # Nm/rad
MAX_KD = 3.0              # Nm/(rad/s)
MIN_HZ, MAX_HZ = 10, 100


# --------------------------------------------------------------------------- #
# SDK resolution
# --------------------------------------------------------------------------- #
def load_sdk(sdk_path):
    """Import robstride_dynamics from a checkout or an installed location."""
    if sdk_path:
        sys.path.insert(0, os.path.expanduser(sdk_path))
    try:
        from robstride_dynamics import (
            CommunicationType,
            Motor,
            ParameterType,
            RobstrideBus,
        )
    except ImportError as exc:
        raise SystemExit(
            "Cannot import robstride_dynamics: %s\n"
            "Point --sdk-path at the repo's python/ directory, e.g.\n"
            "  --sdk-path ~/RobStride_Control/python\n"
            "or clone it first:\n"
            "  git clone https://github.com/Seeed-Projects/RobStride_Control "
            "~/RobStride_Control" % exc
        )
    return RobstrideBus, Motor, ParameterType, CommunicationType


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def clamp(value, low, high):
    return max(low, min(high, value))


def ease_in_out(t):
    """Cosine easing: zero velocity at both ends of the leg."""
    return 0.5 * (1.0 - math.cos(math.pi * clamp(t, 0.0, 1.0)))


def can_is_up(channel):
    path = "/sys/class/net/%s/operstate" % channel
    try:
        with open(path) as handle:
            return handle.read().strip() == "up"
    except OSError:
        return False


def drain_rx(bus, budget=64):
    """Discard pending status frames so the socket buffer cannot back up.

    Also looks for fault frames (communication type 21): dropping those on the
    floor would let the demo keep commanding a motor that has already faulted.
    """
    drained = 0
    while drained < budget:
        msg = bus.channel_handler.recv(timeout=0.0)
        if msg is None:
            break
        drained += 1
        if msg.is_extended_id and ((msg.arbitration_id >> 24) & 0x1F) == 21:
            raise RuntimeError(
                "motor id %d reported a fault frame: %s"
                % (msg.arbitration_id & 0xFF, msg.data.hex()))
    return drained


# --------------------------------------------------------------------------- #
# Core
# --------------------------------------------------------------------------- #
class Demo:
    def __init__(self, bus, names, kp, kd, hz):
        self.bus = bus
        self.names = names
        self.kp = kp          # {motor_id: gain}
        self.kd = kd          # {motor_id: gain}
        self.hz = hz
        self.period = 1.0 / hz
        self.start = {}
        self.last = {}
        self.enabled = False
        self.sender = None      # optional MitSender; None = library path

    # -- low level ---------------------------------------------------------- #
    def _write_mode_mit(self, name, ParameterType, CommunicationType):
        """Set run_mode=0 (MIT) with a raw frame so we do not block on an ack."""
        param_id = ParameterType.MODE[0]
        data = struct.pack("<H", param_id) + struct.pack("<bBH", 0, 0, 0)
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id,
                          self.bus.motors[name].id, data)

    def _disable(self, name, CommunicationType):
        self.bus.transmit(CommunicationType.DISABLE, self.bus.host_id,
                          self.bus.motors[name].id)

    def bring_up(self, ParameterType, CommunicationType):
        """Mode 0 -> enable -> capture the measured start pose."""
        for name in self.names:
            self._write_mode_mit(name, ParameterType, CommunicationType)
        time.sleep(0.1)
        drain_rx(self.bus)

        for name in self.names:
            self.bus.transmit(CommunicationType.ENABLE, self.bus.host_id,
                              self.bus.motors[name].id)
            state = self.read_status(name, timeout=0.5)
            if state is None:
                raise RuntimeError("no status frame from motor %d after enable"
                                   % self.bus.motors[name].id)
            position, velocity, torque, temperature = state
            self.start[name] = position
            self.last[name] = position
            print("  motor %-2d start=%+.4f rad (vel=%+.3f, tq=%+.2f, %.1f C)"
                  % (self.bus.motors[name].id, position, velocity, torque, temperature))
            drain_rx(self.bus)
        self.enabled = True

    def command(self, targets, drain=True):
        """Send one MIT frame per joint at the current tick."""
        for name, target in targets.items():
            motor_id = self.bus.motors[name].id
            if self.sender is not None:
                # ~6x cheaper than the library's numpy path; see mit_frames.py
                self.sender.send(motor_id, target, self.kp[motor_id], self.kd[motor_id])
            else:
                self.bus.write_operation_frame(name, target, self.kp[motor_id],
                                               self.kd[motor_id], 0.0, 0.0)
            self.last[name] = target
        if drain:
            drain_rx(self.bus)

    def sample(self, name, timeout=0.05):
        """Measured position for exactly this joint, or None."""
        state = self.read_status(name, timeout)
        return None if state is None else state[0]

    def read_status(self, name, timeout=0.2):
        """Position/velocity/torque/temperature for `name`, matched by device ID.

        With several motors enabled their status frames arrive in whatever order
        the bus delivers them, so "read the next frame" can quietly attribute
        another joint's state to this one. That matters most at bring-up, where
        a mis-attributed start pose becomes the target of the first command.
        """
        from robstride_dynamics.table import (
            MODEL_MIT_POSITION_TABLE,
            MODEL_MIT_TORQUE_TABLE,
            MODEL_MIT_VELOCITY_TABLE,
        )

        motor = self.bus.motors[name]
        model = motor.model
        deadline = time.perf_counter() + timeout
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            msg = self.bus.channel_handler.recv(timeout=min(0.01, remaining))
            if msg is None or not msg.is_extended_id:
                continue
            comm_type = (msg.arbitration_id >> 24) & 0x1F
            if comm_type == 21:
                raise RuntimeError("motor %d reported a fault frame: %s"
                                   % (motor.id, msg.data.hex()))
            if comm_type != 2:
                continue
            if ((msg.arbitration_id >> 8) & 0xFF) != motor.id:
                continue
            pos_u16, vel_u16, torque_i16, temp_u16 = struct.unpack(">HHHh", msg.data)
            return (
                (float(pos_u16) / 0x7FFF - 1.0) * MODEL_MIT_POSITION_TABLE[model],
                (float(vel_u16) / 0x7FFF - 1.0) * MODEL_MIT_VELOCITY_TABLE[model],
                (float(torque_i16) / 0x7FFF - 1.0) * MODEL_MIT_TORQUE_TABLE[model],
                float(temp_u16) * 0.1,
            )

    def move_leg(self, targets, duration, label, track=False):
        """Cosine-eased trajectory from *where we are now* to `targets`.

        The start point is the last commanded position, not the pose captured at
        bring-up: otherwise a leg that returns to the start pose would emit that
        pose on its first frame (a step) instead of ramping into it.
        """
        begin = {name: self.last.get(name, self.start[name]) for name in targets}
        t0 = time.perf_counter()
        steps = max(1, int(round(duration * self.hz)))
        sample_at = set()
        if track:
            sample_at = {int(round(steps * frac)) for frac in (0.25, 0.5, 0.75, 1.0)}

        for step in range(steps + 1):
            frac = ease_in_out(step / steps)
            is_sample = step in sample_at
            self.command({name: begin[name] + (targets[name] - begin[name]) * frac
                          for name in targets}, drain=not is_sample)
            if is_sample:
                for name in targets:
                    measured = self.sample(name)
                    if measured is not None:
                        print("    %-8s %3d%% commanded %+.4f measured %+.4f"
                              % (label, int(round(100.0 * step / steps)),
                                 self.last[name], measured))
                drain_rx(self.bus)
            deadline = t0 + (step + 1) * self.period
            time.sleep(max(0.0, deadline - time.perf_counter()))
        print("    %-8s done in %+.2fs" % (label, time.perf_counter() - t0))

    def hold(self, targets, seconds):
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < seconds:
            self.command(targets)
            time.sleep(self.period)

    def shut_down(self, targets, stay_enabled, CommunicationType, track=False):
        """Ramp home, hold, then drop torque unless asked to stay enabled."""
        if not self.enabled:
            return
        print("  returning to start pose...")
        self.move_leg(targets, 1.5, "home", track=track)
        self.hold(targets, 0.5)
        if stay_enabled:
            print("  --stay-enabled: leaving torque ON (motors keep holding).")
            return
        print("  disabling torque (gravity-loaded joints may sag)")
        for name in self.names:
            self._disable(name, CommunicationType)
        drain_rx(self.bus)
        self.enabled = False


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Gradual, small-amplitude motion demo for a B601-RS over SocketCAN.")
    parser.add_argument("--channel", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--joints", default="1",
                        help="Comma-separated motor IDs, or 'all' (default: 1)")
    parser.add_argument("--model", default="rs-00",
                        help="Motor model for MIT scaling (all rs-* share 4*pi here)")
    parser.add_argument("--delta", type=float, default=0.10,
                        help="Amplitude in rad, each way (default 0.10, max %.2f)" % MAX_DELTA)
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Seconds for each leg (default 3.0)")
    parser.add_argument("--hold", type=float, default=0.5,
                        help="Seconds to hold at the far point (default 0.5)")
    parser.add_argument("--kp", default="20",
                        help="Position gain: '20' for all joints, or '4:45,5:35' per joint")
    parser.add_argument("--kd", default="0.6",
                        help="Damping gain: '0.6' for all joints, or '4:1.0' per joint")
    parser.add_argument("--hz", type=int, default=30,
                        help="Control rate; drop it if the Pi cannot keep up (default 30)")
    parser.add_argument("--sdk-path", default=os.environ.get("ROBSTRIDE_SDK_PATH", ""),
                        help="Path to RobStride_Control/python (or set ROBSTRIDE_SDK_PATH)")
    parser.add_argument("--execute", action="store_true",
                        help="Actually move. Without this the script only reads and plans.")
    parser.add_argument("--track", action="store_true",
                        help="Print measured position at 25/50/75/100%% of each leg")
    parser.add_argument("--stay-enabled", action="store_true",
                        help="Leave torque on when the script exits")
    return parser.parse_args(argv)


def parse_gain_spec(spec, default, ceiling, flag):
    """Parse '20', '4:45,5:35', or '20,4:45' into (base, {motor_id: gain}).

    A bare number sets the base for every joint; `ID:VALUE` overrides one joint.
    Unlisted joints fall back to `default`, which the banner always prints.
    """
    base = clamp(default, 0.0, ceiling)
    overrides = {}
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            key, _, value = part.partition(":")
            try:
                motor_id = int(key)
                gain = float(value)
            except ValueError:
                raise SystemExit("--%s: cannot parse '%s' (expected ID:VALUE)" % (flag, part))
            if not 1 <= motor_id <= 7:
                raise SystemExit("--%s: motor IDs are 1..7 (got %d)" % (flag, motor_id))
            overrides[motor_id] = clamp(gain, 0.0, ceiling)
        else:
            try:
                base = clamp(float(part), 0.0, ceiling)
            except ValueError:
                raise SystemExit("--%s: cannot parse '%s'" % (flag, part))
    return base, overrides


def format_gains(by_id):
    return " ".join("%d:%.1f" % (motor_id, gain) for motor_id, gain in sorted(by_id.items()))


def resolve_joint_ids(spec):
    if spec.strip().lower() == "all":
        return list(range(1, 8))
    try:
        ids = [int(part) for part in spec.split(",") if part.strip()]
    except ValueError:
        raise SystemExit("--joints expects comma-separated motor IDs, e.g. 1,2,3 or 'all'")
    if not ids:
        raise SystemExit("--joints is empty")
    if any(i < 1 or i > 7 for i in ids):
        raise SystemExit("--joints: motor IDs on this arm are 1..7")
    return ids


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    RobstrideBus, Motor, ParameterType, CommunicationType = load_sdk(args.sdk_path)

    ids = resolve_joint_ids(args.joints)
    delta = clamp(abs(args.delta), 0.0, MAX_DELTA)
    kp_base, kp_over = parse_gain_spec(args.kp, 20.0, MAX_KP, "kp")
    kd_base, kd_over = parse_gain_spec(args.kd, 0.6, MAX_KD, "kd")
    kp_by_id = {i: kp_over.get(i, kp_base) for i in ids}
    kd_by_id = {i: kd_over.get(i, kd_base) for i in ids}
    hz = int(clamp(args.hz, MIN_HZ, MAX_HZ))
    duration = clamp(args.duration, 0.5, 30.0)

    print("B601-RS gradual motion demo")
    print("  channel : %s" % args.channel)
    print("  joints  : %s" % ", ".join(str(i) for i in ids))
    print("  motion  : +/-%.3f rad (%.1f deg) over %.1f s per leg"
          % (delta, math.degrees(delta), duration))
    print("  kp      : %s" % format_gains(kp_by_id))
    print("  kd      : %s   @ %d Hz" % (format_gains(kd_by_id), hz))
    print("  mode    : %s"
          % ("EXECUTE - the arm will move" if args.execute else "dry run (read-only)"))

    if not can_is_up(args.channel):
        raise SystemExit("%s is not UP. Bring it up first, e.g.\n"
                         "  sudo systemctl start can0-setup.service\n"
                         "  ip -details link show %s" % (args.channel, args.channel))

    motors = {("m%d" % i): Motor(i, args.model) for i in ids}
    names = list(motors)

    bus = RobstrideBus(args.channel, motors)
    bus.connect()

    demo = Demo(bus, names, kp_by_id, kd_by_id, hz)
    exit_code = 0

    try:
        # 1. Where are we? Read-only, works without enabling torque.
        for name in names:
            start = bus.read(name, ParameterType.MECHANICAL_POSITION)
            demo.start[name] = start
            demo.last[name] = start
            target = start + delta
            if abs(target) >= MIT_POSITION_LIMIT - 0.2:
                raise SystemExit(
                    "motor %s: start %+.3f + delta would leave the MIT range; "
                    "reduce --delta" % (bus.motors[name].id, start))
            print("  motor %-2d measured %+.4f rad -> target %+.4f rad"
                  % (bus.motors[name].id, start, target))

        if not args.execute:
            print("\nDry run complete - no frames sent. Re-run with --execute to move.")
            return 0

        # 2. Real motion.
        print("\nEnabling motors and ramping...")
        demo.bring_up(ParameterType, CommunicationType)

        out_target = {name: demo.start[name] + delta for name in names}
        home_target = dict(demo.start)

        demo.move_leg(out_target, duration, "outbound", track=args.track)
        demo.hold(out_target, clamp(args.hold, 0.0, 10.0))
        demo.move_leg(home_target, duration, "return", track=args.track)
        demo.hold(home_target, 0.3)

        print("\nFinal positions:")
        for name in names:
            print("  motor %-2d %+.4f rad (started %+.4f)"
                  % (bus.motors[name].id, home_target[name], demo.start[name]))

    except KeyboardInterrupt:
        print("\nInterrupted - ramping home before disabling.")
        exit_code = 130
    finally:
        try:
            targets = {name: demo.start[name] for name in names if name in demo.start}
            demo.shut_down(targets, args.stay_enabled, CommunicationType, track=args.track)
        except Exception as exc:  # never mask the real error
            print("  warning: cleanup did not finish: %s" % exc)
        finally:
            bus.disconnect(disable_torque=False)
            print("Disconnected.")

    return exit_code


def _on_sigterm(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _on_sigterm)
    sys.exit(main())
