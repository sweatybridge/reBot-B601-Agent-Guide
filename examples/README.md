# B601-RS examples

Two machines, two directories, one hand-off.

```
edge/          runs on the board wired to the CAN bus (Raspberry Pi Zero W + CAN HAT)
host/          runs on a workstation; needs numpy only, never touches CAN
trajectories/  the hand-off: joint-space JSON produced on host, replayed on edge
```

Why the split exists: `motorbridge` has no armv6l/armv7l wheels, and a single
damped-least-squares IK solve costs ~1.3 s on a Pi Zero W. So **all planning
happens on the host** and the edge device only ever sends joint angles it was
handed.

## The loop

```
        edge/dump_joints.py            host/plan_*.py            edge/play_trajectory.py
  arm ──────────────────────► current.json ──────────────► seq.json ──────────────────────► arm
        (read 0x7019,          (IK + timing,             (fixed-rate frames,
         no torque)             no hardware)              dry run by default)
```

```bash
# 1. edge: snapshot where the arm is (read-only)
python3 edge/dump_joints.py --ids 1,2,3,4,5,6,7 \
    --sdk-path ~/RobStride_Control/python > current.json

# 2. host: turn a cartesian move into joint space
python3 host/plan_sequence.py --joints current.json --hz 30 \
    --out trajectories/seq.json \
    --steps "grip:open:2.5; movej:0.20,0,0.30:4:0.1:5; grip:to:6:2.5; \
             hold:2; homej:5; grip:open:2.5; grip:close:2.5"

# 3. edge: replay it (dry run first, then --execute)
python3 edge/play_trajectory.py trajectories/seq.json
python3 edge/play_trajectory.py trajectories/seq.json --execute \
    --fast-frames --kp 2:30,3:40,4:45,5:35,7:10 --kd 7:1.0 --delay 2
```

## edge/

| file | job |
|---|---|
| `dump_joints.py` | read parameter `0x7019` from each motor, print JSON. Enables nothing |
| `play_trajectory.py` | stream a trajectory at `control_hz`; dry run by default |
| `gradual_motion_demo.py` | the original joint-space demo (cosine ease, own gains) |
| `mit_frames.py` | MIT frame builder without numpy, 6x faster (needed above ~20 Hz) |
| `test_mit_frames.py` | byte-compares that builder against the library |

`play_trajectory.py` imports the control loop from `gradual_motion_demo.py` and
the frame builder from `mit_frames.py`, so keep this directory together.

## host/

| file | job |
|---|---|
| `rebot_ik.py` | FK + damped-least-squares IK from Seeed's URDF |
| `plan_cartesian.py` | one or more straight cartesian segments |
| `plan_sequence.py` | arm + gripper script with `move`/`movej`/`home`/`homej`/`joint`/`grip`/`hold` |
| `plan_gripper.py` | motor 7 only: open, hold, close |

`plan_cartesian.py` and `plan_sequence.py` import `rebot_ik.py` from the same
directory.

## Conventions

- `base_link` frame: **x forward, y left, z up**; deltas are metres.
- Trajectory JSON is the contract between the two halves: `ids`, `limits`,
  `control_hz`, `waypoints[{t, q}]`, plus optional `hold_start_s`/`hold_end_s`
  used for measurement windows. The edge device re-checks the limits it is given.
- 7 joints at 30 Hz needs `--fast-frames`; without it the library's numpy path
  costs ~50 ms per tick and the motion stretches ~9 %.
- Dry run is the default everywhere. Nothing moves without `--execute`.
