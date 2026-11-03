#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plan a gripper open/hold/close trajectory (motor 7 only).

Cartesian planning does not apply to the gripper: it is a single motor driving a
rack, not part of the arm's kinematic chain. This just writes a one-motor
trajectory that play_trajectory.py (../edge/) replays like any other.

    python3 plan_gripper.py --open-cm 10 --out ../trajectories/gripper10cm.json
    # on the edge device
    python3 play_trajectory.py ../trajectories/gripper10cm.json --execute --kp 7:15 --kd 7:1.0 --delay 10

Jaw width is not in the URDF (the fingers are modelled as prismatic joints with
0.05 m / 0.0715 m travel, i.e. ~12 cm of stroke, and Seeed's ROS 2 config treats
0..5 rad as closed..open). That gives 0.412 rad per cm, which is the default
`--rad-per-cm`. Measure the real opening and correct that number if it disagrees
- it is a linear map through closed = 0 rad, so one measurement is enough.
"""

import argparse
import json
import math
import sys
import time

FORMAT = "rebot-joint-trajectory"
MOTOR_ID = 7
RAD_LIMITS = (0.0, 5.0)          # closed .. fully open, from Seeed's ROS 2 config


def ease_in_out(t):
    return 0.5 * (1.0 - math.cos(math.pi * min(max(t, 0.0), 1.0)))


def ramp(start, end, duration, hz):
    steps = max(1, int(round(duration * hz)))
    return [start + (end - start) * ease_in_out(i / steps) for i in range(1, steps + 1)]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Plan a gripper open/hold/close trajectory.")
    parser.add_argument("--open-cm", type=float, default=10.0, help="target jaw width in cm")
    parser.add_argument("--rad-per-cm", type=float, default=0.412,
                        help="joint-7 radians per cm of jaw travel (default 0.412)")
    parser.add_argument("--start-rad", type=float, default=None,
                        help="where the gripper is now; omit to assume closed (0 rad)")
    parser.add_argument("--close-rad", type=float, default=0.0,
                        help="radians to return to, i.e. closed (default 0.0)")
    parser.add_argument("--duration", type=float, default=2.5, help="seconds per leg")
    parser.add_argument("--hold", type=float, default=5.0, help="seconds held open")
    parser.add_argument("--hz", type=int, default=20)
    parser.add_argument("--out", default="gripper.json")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    target = args.open_cm * args.rad_per_cm
    start = args.start_rad if args.start_rad is not None else args.close_rad
    if not (RAD_LIMITS[0] <= target <= RAD_LIMITS[1]):
        raise SystemExit("%.1f cm -> %.2f rad is outside the 0..5 rad range; "
                         "check --rad-per-cm" % (args.open_cm, target))

    step = 1.0 / args.hz
    waypoints = [{"t": 0.0, "q": [round(start, 6)]}]
    t = 0.0

    print("Gripper plan")
    print("  jaw width : %.1f cm = %.3f rad (%.3f rad/cm)" % (args.open_cm, target, args.rad_per_cm))
    print("  profile   : %.1f s open, %.1f s hold, %.1f s close at %d Hz"
          % (args.duration, args.hold, args.duration, args.hz))

    hold_start = hold_end = None
    for label, values in (
        ("open", ramp(start, target, args.duration, args.hz)),
        ("hold", [target] * max(1, int(round(args.hold * args.hz)))),
        ("close", ramp(target, args.close_rad, args.duration, args.hz)),
        ("settle", [args.close_rad] * max(1, int(round(0.5 * args.hz)))),
    ):
        if label == "hold":
            hold_start = t
        for value in values:
            t += step
            waypoints.append({"t": round(t, 4), "q": [round(value, 6)]})
        if label == "hold":
            hold_end = t
        print("  %-6s leg: %3d waypoints, t = %.2f s" % (label, len(values), t))

    data = {
        "format": FORMAT,
        "version": 1,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "control_hz": args.hz,
        "ids": [MOTOR_ID],
        # Widen the nominal range to include where the gripper actually is: the
        # encoder reads a few tens of mrad below "closed = 0" on this unit, and
        # the player rejects any waypoint outside the limits it is given.
        "limits": [[min(RAD_LIMITS[0], start, args.close_rad),
                    max(RAD_LIMITS[1], target)]],
        "open_cm": args.open_cm,
        "rad_per_cm": args.rad_per_cm,
        "duration_s": waypoints[-1]["t"],
        "hold_start_s": hold_start,
        "hold_end_s": hold_end,
        "waypoints": waypoints,
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(data, handle, separators=(",", ":"))
    print("\nwrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
