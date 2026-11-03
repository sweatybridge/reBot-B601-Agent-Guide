#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build RobStride MIT frames without numpy.

EDGE DEVICE - used by play_trajectory.py --fast-frames and by the demo.

`write_operation_frame` from robstride_dynamics costs ~7 ms per joint on a Pi
Zero W because it runs ~6 numpy scalar operations per call. A 7-joint tick then
takes 50 ms, which caps the control rate at 20 Hz and turns a slow motion into a
visible staircase.

This module produces the *identical* frame with plain Python arithmetic and
struct - measured 8.2 ms per 7-joint tick (6x faster), so 30-40 Hz is
comfortable. `test_mit_frames.py` proves the bytes match the library's for a
sweep of positions and gains; run it before trusting this with a motor.

The scaling below mirrors `RobstrideBus.write_operation_frame` exactly: position
and velocity are mapped onto the model's MIT range, kp/kd onto the model's gain
ranges, torque travels in the extended-ID "extra data" field.
"""

import struct

import can

COMM_OPERATION_CONTROL = 1      # CommunicationType.OPERATION_CONTROL


class MitSender:
    """Drop-in replacement for write_operation_frame, ~6x cheaper."""

    def __init__(self, bus, model="rs-00"):
        from robstride_dynamics.table import (
            MODEL_MIT_KD_TABLE,
            MODEL_MIT_KP_TABLE,
            MODEL_MIT_POSITION_TABLE,
            MODEL_MIT_TORQUE_TABLE,
            MODEL_MIT_VELOCITY_TABLE,
        )

        self.bus = bus
        self.channel = bus.channel_handler
        self.position_range = float(MODEL_MIT_POSITION_TABLE[model])
        self.velocity_range = float(MODEL_MIT_VELOCITY_TABLE[model])
        self.kp_range = float(MODEL_MIT_KP_TABLE[model])
        self.kd_range = float(MODEL_MIT_KD_TABLE[model])
        self.torque_range = float(MODEL_MIT_TORQUE_TABLE[model])

    def build(self, motor_id, position, kp, kd, velocity=0.0, torque=0.0):
        """Return the can.Message the library would send, byte for byte."""
        span = self.position_range

        # The library clips each value into its range first, then scales.
        position = min(max(position, -span), span)
        velocity = min(max(velocity, -self.velocity_range), self.velocity_range)
        kp = min(max(kp, 0.0), self.kp_range)
        kd = min(max(kd, 0.0), self.kd_range)
        torque = min(max(torque, -self.torque_range), self.torque_range)

        def clamp_u16(value):
            return 0 if value < 0 else (0xFFFF if value > 0xFFFF else value)

        position_u16 = clamp_u16(int(((position / span) + 1.0) * 0x7FFF))
        velocity_u16 = clamp_u16(int(((velocity / self.velocity_range) + 1.0) * 0x7FFF))
        kp_u16 = clamp_u16(int((kp / self.kp_range) * 0xFFFF))
        kd_u16 = clamp_u16(int((kd / self.kd_range) * 0xFFFF))
        torque_u16 = clamp_u16(int(((torque / self.torque_range) + 1.0) * 0x7FFF))

        data = struct.pack(">HHHH", position_u16, velocity_u16, kp_u16, kd_u16)
        extended_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id
        return can.Message(arbitration_id=extended_id, is_extended_id=True,
                           dlc=len(data), data=data)

    def send(self, motor_id, position, kp, kd, velocity=0.0, torque=0.0):
        self.channel.send(self.build(motor_id, position, kp, kd, velocity, torque))
