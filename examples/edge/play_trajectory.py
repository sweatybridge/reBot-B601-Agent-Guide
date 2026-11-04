#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Play a planned joint-space trajectory on the edge device (no IK here).

EDGE DEVICE - runs on the board wired to the CAN bus (Raspberry Pi + CAN HAT).
The workstation computes the trajectory with `plan_cartesian.py` or
`plan_sequence.py` (see ../host/); this script only sends joint angles on time.
That keeps the Pi's job to something a Zero W can actually do: read the file,
enable, stream frames at a fixed rate.

    python3 play_trajectory.py ../trajectories/seq_30hz.json            # dry run
    python3 play_trajectory.py ../trajectories/seq_30hz.json --execute  # move

Safety, on top of the checks the planner already did:

* dry run is the default; nothing moves without --execute
* the trajectory's own joint limits are re-checked here, so a hand-edited or
  truncated file cannot drive the arm somewhere the planner would have refused
* the arm first ramps (2 s, eased) onto the trajectory's start pose, so a little
  drift since planning is not a jump
* any fault frame aborts, and the arm is ramped back to its start pose before
  torque is dropped (or kept on with --stay-enabled)

Shares the control loop, status sampling and shutdown path with
`gradual_motion_demo.py` and the frame builder with `mit_frames.py` - keep the
three files in this directory together.
"""

import argparse
import json
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gradual_motion_demo import (  # noqa: E402
    DEFAULT_MODEL_SPEC,
    Demo,
    can_is_up,
    clamp,
    drain_rx,
)

TRAJECTORY_FORMAT = "rebot-joint-trajectory"
MAX_JOINT_STEP = 0.5          # rad between consecutive waypoints, sanity only
LIMIT_TOL = 0.002             # encoders read a few mrad past a stop at rest
MAX_KP = 150.0                # safety ceiling for --kp, Nm/rad (the vendor's j2/j3 value)
MAX_KD = 10.0                 # safety ceiling for --kd (vendor uses 10 on j2/j3)


def load_trajectory(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if data.get("format") != TRAJECTORY_FORMAT:
        raise SystemExit("%s: not a %s file (found %r)"
                         % (path, TRAJECTORY_FORMAT, data.get("format")))
    waypoints = data.get("waypoints") or []
    if len(waypoints) < 2:
        raise SystemExit("%s: needs at least two waypoints" % path)
    ids = data.get("ids")
    if not ids or any(not isinstance(i, int) or i < 1 or i > 7 for i in ids):
        raise SystemExit("%s: 'ids' must list motor IDs 1..7" % path)
    for i, wp in enumerate(waypoints):
        if len(wp.get("q") or []) != len(ids):
            raise SystemExit("%s: waypoint %d has %d values for %d motors"
                             % (path, i, len(wp.get("q") or []), len(ids)))
    times = [float(wp["t"]) for wp in waypoints]
    if any(b < a for a, b in zip(times, times[1:])):
        raise SystemExit("%s: waypoint times are not monotonic" % path)
    return data


def check_against_limits(data):
    """Re-apply the planner's joint limits; ignore if the file omits them."""
    limits = data.get("limits")
    qs = [wp["q"] for wp in data["waypoints"]]
    if limits and len(limits) == len(data["ids"]):
        for j, (lo, hi) in enumerate(limits):
            column = [q[j] for q in qs]
            if (min(column) < lo - LIMIT_TOL) or (max(column) > hi + LIMIT_TOL):
                raise SystemExit("joint %d leaves its limit [%.3f, %.3f]: %.3f..%.3f"
                                 % (data["ids"][j], lo, hi, min(column), max(column)))
    for a, b in zip(qs, qs[1:]):
        step = max(abs(x - y) for x, y in zip(a, b))
        if step > MAX_JOINT_STEP:
            raise SystemExit("waypoints jump %.3f rad, above the %.2f sanity limit"
                             % (step, MAX_JOINT_STEP))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Play a planned joint-space trajectory (dry run by default).")
    parser.add_argument("trajectory", help="trajectory JSON from plan_cartesian.py")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--model", default="rs-00",
                        help="Fallback motor model for MIT scaling (default: rs-00)")
    parser.add_argument("--model-spec", default=DEFAULT_MODEL_SPEC,
                        help="Per-joint models, ID:MODEL,... The B601-RS uses RS-06 on "
                             "joints 1-3 and RS-00 on 4-7, and the MIT frame scales kp, "
                             "kd and torque by the model, so a wrong table silently "
                             "changes the gains tenfold. Pass '' for a uniform arm "
                             "(default: %s)" % DEFAULT_MODEL_SPEC)
    parser.add_argument("--kp", default="50,2:150,3:150",
                        help="vendor rebotarm_rs.yaml: 50, stiffer 150 on j2/j3")
    parser.add_argument("--kd", default="3,2:10,3:10,4:5,5:4,6:4,7:4",
                        help="vendor rebotarm_rs.yaml: 3, 10 on j2/j3, 4-5 on the wrist")
    parser.add_argument("--ramp", type=float, default=2.0,
                        help="seconds to ease onto the trajectory start pose (default 2.0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="time scale, 0.5 = twice as slow (default 1.0)")
    parser.add_argument("--sdk-path", default=os.environ.get("ROBSTRIDE_SDK_PATH", ""))
    parser.add_argument("--max-follow-error", type=float, default=0.0,
                        help="abort if measured position lags the command by more than this "
                             "(rad, checked twice a second; 0 disables)")
    parser.add_argument("--allow-kp-mismatch", action="store_true",
                        help="play a sag-compensated trajectory at a kp other than the "
                             "one it was compensated for; the arm will miss by the "
                             "difference")
    parser.add_argument("--log", default="",
                        help="write every waypoint's commanded and measured angles here; "
                             "host/trim_trajectory.py turns that into a corrected trajectory")
    parser.add_argument("--fast-frames", action="store_true",
                        help="build MIT frames without numpy (6x cheaper, needed above ~20 Hz)")
    parser.add_argument("--delay", type=float, default=0.0,
                        help="seconds to wait (holding the start pose) before the motion starts")
    parser.add_argument("--execute", action="store_true",
                        help="Actually move. Without this the script only reads and checks.")
    parser.add_argument("--stay-enabled", action="store_true",
                        help="Leave torque on when the script exits")
    parser.add_argument("--track", action="store_true",
                        help="Print measured position every quarter of the trajectory")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    data = load_trajectory(args.trajectory)
    check_against_limits(data)

    from gradual_motion_demo import load_sdk
    RobstrideBus, Motor, ParameterType, CommunicationType = load_sdk(args.sdk_path)
    from gradual_motion_demo import format_gains, parse_gain_spec, parse_model_spec

    ids = data["ids"]
    names = ["m%d" % i for i in ids]
    model_by_id = parse_model_spec(args.model_spec, args.model, ids)
    start_q = [float(v) for v in data["waypoints"][0]["q"]]
    hz = int(clamp(float(data.get("control_hz", 20)), 5, 50))
    speed = clamp(args.speed, 0.1, 5.0)

    kp_base, kp_over = parse_gain_spec(args.kp, 50.0, MAX_KP, "kp")
    kd_base, kd_over = parse_gain_spec(args.kd, 3.0, MAX_KD, "kd")
    kp_by_id = {i: kp_over.get(i, kp_base) for i in ids}
    kd_by_id = {i: kd_over.get(i, kd_base) for i in ids}

    # A sag-compensated or trimmed file has q_cmd = q_plan + correction baked in, so
    # the correction is only right at the kp it was computed for. Playing it
    # stiffer undershoots; softer overshoots, and the arm climbs above the plan.
    bound = data.get("sag_compensation") or data.get("trim") or {}
    bound_kp = {int(key): float(value) for key, value in (bound.get("kp") or {}).items()}
    mismatched = [i for i in ids
                  if i in bound_kp and abs(kp_by_id[i] - bound_kp[i]) > 0.02 * bound_kp[i]]

    # Optional modelled feedforward: `tau` per waypoint, in `ids` order. Unlike the
    # position trims this is an absolute torque, so it is not tied to kp.
    has_tau = any("tau" in waypoint for waypoint in data["waypoints"])
    if has_tau:
        for index, waypoint in enumerate(data["waypoints"]):
            values = waypoint.get("tau") or []
            if len(values) != len(ids):
                raise SystemExit("waypoint %d has %d tau values for %d motors"
                                 % (index, len(values), len(ids)))
        peak_tau = max(abs(float(v)) for waypoint in data["waypoints"]
                       for v in waypoint["tau"])
    else:
        peak_tau = 0.0

    print("Planned trajectory")
    print("  file      : %s" % args.trajectory)
    print("  motors    : %s" % ", ".join(str(i) for i in ids))
    print("  created   : %s" % data.get("created", "?"))
    print("  duration  : %.2f s at %d Hz (x%.2f speed -> %.2f s)"
          % (data.get("duration_s", 0), hz, speed, data.get("duration_s", 0) / speed))
    print("  cartesian : start %s -> end %s (delta %s m)"
          % (fmt_vec(data.get("start_tcp")), fmt_vec(data.get("end_tcp")),
             fmt_vec(data.get("delta_xyz"))))
    print("  peak speed: %.3f rad/s, worst IK residual %.3f mm"
          % (data.get("peak_joint_speed", 0), 1000 * data.get("worst_ik_residual_m", 0)))
    print("  kp        : %s" % format_gains(kp_by_id))
    if has_tau:
        print("  tau       : modelled gravity feedforward, peak %.2f Nm" % peak_tau)
    print("  models    : %s" % ", ".join("%d:%s" % (i, model_by_id[i]) for i in ids))
    if bound:
        print("  corrected : %s%s"
              % (", ".join("%d:%.1f" % (i, v) for i, v in sorted(bound_kp.items())),
                 "" if not mismatched else "   <- MISMATCH with --kp"))
    print("  mode      : %s" % ("EXECUTE - the arm will move" if args.execute else "dry run (read-only)"))

    if mismatched:
        detail = ", ".join("joint %d: file %.1f, playing %.1f"
                           % (i, bound_kp[i], kp_by_id[i]) for i in mismatched)
        if not args.allow_kp_mismatch:
            raise SystemExit(
                "this trajectory was corrected for a different kp (%s).\n"
                "The offset is tau/kp, so replaying it at another gain either "
                "under- or overshoots; at a softer kp the arm climbs above the "
                "plan. Play it with the same --kp, or pass --allow-kp-mismatch if "
                "you want the difference." % detail)
        print("  warning   : playing a sag-compensated file at a different kp (%s)"
              % detail)

    if not can_is_up(args.channel):
        raise SystemExit("%s is not UP. Bring it up first:\n"
                         "  sudo systemctl start can0-setup.service" % args.channel)

    motors = {name: Motor(motor_id, model_by_id[motor_id])
              for name, motor_id in zip(names, ids)}
    bus = RobstrideBus(args.channel, motors)
    bus.connect()
    demo = Demo(bus, names, kp_by_id, kd_by_id, hz)
    if args.fast_frames:
        from mit_frames import MitSender
        demo.sender = MitSender(bus, args.model, models=model_by_id)
        print("  frames    : hand-built (numpy-free)")
    exit_code = 0

    try:
        # Where is the arm now, versus where the plan begins?
        drift = {}
        for name, motor_id, target in zip(names, ids, start_q):
            current = bus.read(name, ParameterType.MECHANICAL_POSITION)
            demo.start[name] = current
            demo.last[name] = current
            drift[name] = target - current
            print("  motor %-2d now %+.4f  plan starts %+.4f  (drift %+.4f)"
                  % (motor_id, current, target, drift[name]))
        worst_drift = max(abs(v) for v in drift.values())
        print("  worst drift from plan start: %.4f rad" % worst_drift)
        if worst_drift > math.radians(20):
            raise SystemExit("the arm is more than 20 deg from the plan's start pose; "
                             "re-plan from a fresh dump_joints.py instead of ramping across")

        if not args.execute:
            print("\nDry run complete - no frames sent. Re-run with --execute to move.")
            return 0

        demo.bring_up(ParameterType, CommunicationType)

        # Ease onto the planned start pose (uses the measured pose as the origin).
        if worst_drift > 1e-4:
            print("\nAligning to the planned start pose...")
            demo.move_leg({n: t for n, t in zip(names, start_q)}, clamp(args.ramp, 0.5, 10.0),
                          "align", track=args.track)

        print("\nPlaying %d waypoints..." % len(data["waypoints"]))
        if args.delay > 0:
            print("  waiting %.0f s while holding the start pose - get your ruler ready"
                  % args.delay)
            deadline = time.perf_counter() + args.delay
            announced = -1
            while time.perf_counter() < deadline:
                demo.command({n: t for n, t in zip(names, start_q)})
                remaining = int(math.ceil(deadline - time.perf_counter()))
                if remaining != announced and remaining >= 0:
                    announced = remaining
                    print("    starting in %2d s" % remaining)
                time.sleep(demo.period)
            print("    GO")

        # Tell the operator when the arm is actually parked at the far point,
        # so a measurement taken by hand lines up with the right seconds.
        if data.get("hold_start_s") is not None:
            print("  far point reached at t=+%.1f s, held until t=+%.1f s (hold %.1f s)"
                  % (data["hold_start_s"], data["hold_end_s"],
                     data["hold_end_s"] - data["hold_start_s"]))

        t0 = time.perf_counter()
        total = float(data["waypoints"][-1]["t"]) / speed
        sample_at = {round(len(data["waypoints"]) * f) for f in (0.25, 0.5, 0.75, 1.0)}
        next_follow_check = 0.5
        abort_reason = None
        stamps = []
        logging = bool(args.log)
        logged = []
        held = {name: demo.start.get(name, 0.0) for name in names}
        for index, wp in enumerate(data["waypoints"]):
            targets = {n: float(v) for n, v in zip(names, wp["q"])}
            sample = args.track and index in sample_at
            elapsed = float(wp["t"]) / speed
            stamps.append(time.perf_counter())

            # Blocking reads inside the control loop cost real time: sampling all
            # seven joints at 20 ms each, twice a second, stretched a 25 s
            # trajectory to 34 s. Read only when a check is due, keep the reads
            # short, and skip draining so the reply frames are already queued.
            check_due = args.max_follow_error > 0 and elapsed >= next_follow_check
            torques = ({name: float(value) for name, value in zip(names, wp["tau"])}
                       if "tau" in wp else None)
            demo.command(targets, drain=not (sample or check_due or logging),
                         torques=torques)

            if logging:
                # One poll after the frames: the status replies belong to the
                # command just sent, so cmd/meas line up in the log. ~6 ms.
                for name, state in demo.poll_states(names, 0.012).items():
                    held[name] = state[0]
                logged.append({"t": round(elapsed, 4),
                               "cmd": [targets[name] for name in names],
                               "meas": [held[name] for name in names]})

            if check_due:
                next_follow_check = elapsed + 1.0
                for name in names:
                    measured = demo.sample(name, timeout=0.005)
                    if measured is None:
                        continue
                    gap = abs(targets[name] - measured)
                    if gap > args.max_follow_error:
                        abort_reason = ("motor %d is not following: commanded %+.3f, "
                                        "measured %+.3f (%.2f rad behind)"
                                        % (bus.motors[name].id, targets[name], measured, gap))
                        break
            if abort_reason:
                break

            if sample:
                for name in names:
                    measured = demo.sample(name, timeout=0.02)
                    if measured is not None:
                        print("    t=%5.2fs commanded %+.4f measured %+.4f"
                              % (float(wp["t"]) / speed, targets[name], measured))
            if sample or check_due:
                drain_rx(bus)
            deadline = t0 + float(wp["t"]) / speed
            time.sleep(max(0.0, deadline - time.perf_counter()))
        if abort_reason:
            print("  ABORTED at t=%.2fs: %s" % (elapsed, abort_reason))
            exit_code = 1
        else:
            print("  played in %.2f s (planned %.2f s)" % (time.perf_counter() - t0, total))

        if len(stamps) > 2:
            gaps = [1000 * (b - a) for a, b in zip(stamps, stamps[1:])]
            gaps_sorted = sorted(gaps)
            p95 = gaps_sorted[int(0.95 * (len(gaps_sorted) - 1))]
            nominal = 1000.0 / hz
            late = sum(1 for gap in gaps if gap > 1.5 * nominal)
            print("  tick spacing: mean %.1f ms, p95 %.1f ms, max %.1f ms "
                  "(target %.1f ms, %d of %d ticks >1.5x target)"
                  % (sum(gaps) / len(gaps), p95, max(gaps), nominal, late, len(gaps)))

        print("\nFinal angles vs plan:")
        for name, motor_id, target in zip(names, ids, [float(v) for v in data["waypoints"][-1]["q"]]):
            print("  motor %-2d planned %+.4f  last commanded %+.4f"
                  % (motor_id, target, demo.last.get(name, float("nan"))))

    except KeyboardInterrupt:
        print("\nInterrupted - ramping home before disabling.")
        exit_code = 130
    finally:
        try:
            if logging and logged:
                payload = {
                    "format": "rebot-trajectory-log",
                    "version": 1,
                    "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "trajectory": args.trajectory,
                    "ids": list(ids),
                    "kp": {str(i): kp_by_id[i] for i in ids},
                    "kd": {str(i): kd_by_id[i] for i in ids},
                    "control_hz": hz,
                    "speed": speed,
                    "samples": logged,
                }
                with open(args.log, "w", encoding="utf-8") as handle:
                    json.dump(payload, handle, separators=(",", ":"))
                print("  log       : %s (%d waypoints)" % (args.log, len(logged)))
            home = {n: demo.start[n] for n in names if n in demo.start}
            demo.shut_down(home, args.stay_enabled, CommunicationType, track=args.track)
        except Exception as exc:
            print("  warning: cleanup did not finish: %s" % exc)
        finally:
            bus.disconnect(disable_torque=False)
            print("Disconnected.")

    return exit_code


def fmt_vec(values):
    if not values:
        return "?"
    return "[%s]" % " ".join("%+.4f" % float(v) for v in values)


def _on_sigterm(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _on_sigterm)
    sys.exit(main())
