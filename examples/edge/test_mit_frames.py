#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Prove MitSender emits byte-identical frames to write_operation_frame.

EDGE DEVICE - run on the board (needs the SDK importable). It intercepts the CAN send call, lets
the library build its frame, then builds the same frame with MitSender and
compares arbitration ID and payload. Nothing is transmitted.

    python3 test_mit_frames.py --sdk-path ~/RobStride_Control/python
"""

import argparse
import itertools
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Byte-compare MitSender against the library.")
    parser.add_argument("--sdk-path", default=os.environ.get("ROBSTRIDE_SDK_PATH", ""))
    parser.add_argument("--model", default="rs-00")
    parser.add_argument("--samples", type=int, default=500)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.sdk_path:
        sys.path.insert(0, os.path.expanduser(args.sdk_path))
    from robstride_dynamics import Motor, RobstrideBus

    from mit_frames import MitSender

    bus = RobstrideBus("can0", {"m1": Motor(1, args.model)})
    bus.connect()                       # the socket only exists after connect()
    sender = MitSender(bus, args.model)  # caches the real handler; we only call build()

    captured = []
    real_channel = bus.channel_handler

    class Harness:
        """Stand-in channel handler: records frames instead of sending them."""

        def __init__(self):
            self.send = self.record

        def record(self, message):
            captured.append(message)

    bus.channel_handler = Harness()

    rng = random.Random(20241102)
    span = sender.position_range
    edge_positions = [0.0, span, -span, span * 1.5, -span * 1.5, 0.5]
    edge_kp = [0.0, 20.0, 45.0, 60.0, sender.kp_range, sender.kp_range * 2]
    edge_kd = [0.0, 0.6, 3.0, sender.kd_range * 1.5]
    edge_torque = [0.0, sender.torque_range, -sender.torque_range, 0.5]

    cases = []
    for position in edge_positions:
        for kp, kd in itertools.product(edge_kp, edge_kd):
            cases.append((position, kp, kd, 0.0))
    for torque in edge_torque:
        for position, kp in itertools.product(edge_positions, edge_kp):
            cases.append((position, kp, 0.6, torque))
    for _ in range(200):
        cases.append((rng.uniform(-span * 1.2, span * 1.2),
                      rng.uniform(0.0, sender.kp_range),
                      rng.uniform(0.0, sender.kd_range),
                      rng.uniform(-sender.torque_range, sender.torque_range)))

    mismatches = 0
    compared = 0
    for position, kp, kd, torque in cases:
        captured.clear()
        bus.write_operation_frame("m1", position, kp, kd, 0.0, torque)
        if not captured:
            print("no frame captured - harness not wired correctly")
            return 2
        library_frame = captured[-1]
        ours = sender.build(1, position, kp, kd, 0.0, torque)
        compared += 1
        if (library_frame.arbitration_id != ours.arbitration_id
                or bytes(library_frame.data) != bytes(ours.data)
                or library_frame.is_extended_id != ours.is_extended_id):
            mismatches += 1
            if mismatches <= 5:
                print("MISMATCH pos=%.6f kp=%.3f kd=%.3f tq=%.3f" % (position, kp, kd, torque))
                print("  library: id=0x%08X data=%s" % (library_frame.arbitration_id,
                                                        bytes(library_frame.data).hex()))
                print("  ours   : id=0x%08X data=%s" % (ours.arbitration_id,
                                                        bytes(ours.data).hex()))

    bus.channel_handler = real_channel
    bus.disconnect(disable_torque=False)
    print("compared %d frames (model %s): %d mismatches" % (compared, args.model, mismatches))
    print("PASS - byte-identical" if mismatches == 0 else "FAIL")
    return 0 if mismatches == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
