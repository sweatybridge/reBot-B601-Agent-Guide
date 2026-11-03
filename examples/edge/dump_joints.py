#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read the current joint angles from the arm as JSON (read-only, no torque).

EDGE DEVICE - run this on the board wired to the CAN bus. It enables nothing and commands
nothing: it just reads parameter 0x7019 from every motor, so it is safe to run
at any time, including while the arm is resting.

    python3 dump_joints.py --sdk-path ~/RobStride_Control/python > current.json

The output feeds plan_cartesian.py / plan_sequence.py in ../host/ on the workstation:

    {"ids": [1,2,3,4,5,6], "q": [0.0138, ...], "timestamp": "..."}
"""

import argparse
import contextlib
import json
import os
import signal
import sys
import time

DEFAULT_IDS = "1,2,3,4,5,6"


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Dump joint angles as JSON (read-only).")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--ids", default=DEFAULT_IDS, help="motors to read (default 1..6)")
    parser.add_argument("--model", default="rs-00")
    parser.add_argument("--sdk-path", default=os.environ.get("ROBSTRIDE_SDK_PATH", ""),
                        help="Path to RobStride_Control/python (or set ROBSTRIDE_SDK_PATH)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    if args.sdk_path:
        sys.path.insert(0, os.path.expanduser(args.sdk_path))
    try:
        from robstride_dynamics import Motor, ParameterType, RobstrideBus
    except ImportError as exc:
        raise SystemExit("cannot import robstride_dynamics: %s\n"
                         "pass --sdk-path ~/RobStride_Control/python" % exc)

    ids = [int(part) for part in args.ids.split(",") if part.strip()]
    motors = {("m%d" % i): Motor(i, args.model) for i in ids}

    # The SDK prints status lines on stdout; keep stdout clean for the JSON.
    with contextlib.redirect_stdout(sys.stderr):
        bus = RobstrideBus(args.channel, motors)
        bus.connect()
        try:
            values = {}
            for name in motors:
                values[name] = bus.read(name, ParameterType.MECHANICAL_POSITION)
        finally:
            bus.disconnect(disable_torque=False)

    payload = {
        "ids": ids,
        "q": [round(float(values["m%d" % i]), 6) for i in ids],
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "channel": args.channel,
        "note": "read-only snapshot, parameter 0x7019 (mechanical position, rad)",
    }
    sys.stdout.write(json.dumps(payload, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    sys.exit(main())
