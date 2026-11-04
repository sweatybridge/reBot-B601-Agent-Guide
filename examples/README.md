# B601-RS trajectory pipeline

One trajectory, two machines, one hand-off.

```
edge/          runs on the board wired to the CAN bus (Raspberry Pi Zero W + MCP2515 HAT)
host/          planning runs in a Docker container (Pinocchio has no Windows wheels)
trajectories/  the hand-off: joint-space JSON with gravity feedforward
```

Why the split: `motorbridge` has no armv6l wheels and a damping-least-squares IK
solve costs ~1.3 s on a Pi Zero W, so **all planning happens off the board** and
the edge only ever sends joint angles and torques it was handed.

## The loop

```
  edge/dump_joints.py        host/plan_with_pinocchio.py (container)       edge/play_trajectory.py
arm ────────────────► current.json ──────────────────────────────► seq.json ──────────────────────► arm
     (read 0x7019,         (their IK + Pinocchio g(q),                 (30 Hz frames,
      no torque)            one tau per waypoint)                       dry run by default)
```

```bash
# 1. edge: snapshot where the arm is (read-only)
python3 edge/dump_joints.py --ids 1,2,3,4,5,6,7 \
    --sdk-path ~/RobStride_Control/python > current.json

# 2. host: plan with Seeed's stack, inside the container
docker build -t rebot-pino -f host/Dockerfile.pinocchio .
pwsh -File host/repeat_pino.ps1            # plan and save

# 3. edge: replay it (dry run first)
python3 edge/play_trajectory.py trajectories/seq_pino_peel.json
python3 edge/play_trajectory.py trajectories/seq_pino_peel.json \
    --sdk-path ~/RobStride_Control/python --execute --fast-frames --log run.json
```

`host/repeat_pino.ps1 -Replay` does all three steps in one command (it copies the
JSON to the board and replays it). On a POSIX host, the container can be run with
a bind mount instead of `docker cp`:

```bash
docker run --rm -v "$PWD:/work" rebot-pino python \
    /work/host/plan_with_pinocchio.py --joints /work/current.json --out /work/seq.json
```

## The trajectory

`trajectories/seq_pino_peel.json` — 22.00 s, 660 waypoints at 30 Hz, motors 1-7:

| phase | time | what happens |
|---|---|---|
| peel + jaws open | 0 - 2.5 s | elbow +8° lifts the arm off its stop while the jaws open to 4.90 rad |
| transfer | 2.5 - 7.5 s | straight line +0.20 m forward, +0.30 m up, gripper held flat |
| close | 7.5 - 10 s | jaws to 2.46 rad (6 cm) |
| hold | 10 - 12 s | measurement window |
| return | 12 - 17 s | exact retrace |
| jaws | 17 - 22 s | open, close, back on the stops |

Every waypoint carries `tau`, the Pinocchio gravity torque of that pose, which
`play_trajectory.py` sends in the MIT frame's torque field.

![trajectory](trajectories/seq_pino_peel.png)

## Measured on the arm

| | hold error | orientation | dip at lift start |
|---|---|---|---|
| plan + `tau`, no peel | 61 mm | 6.6° | -15 mm |
| + elbow peel | 60 mm | 6.3° | 0.0 mm |
| **+ vendor gains (default)** | **23.6 mm** | **2.64°** | 0.0 mm |

The residual is the model's own error: the URDF under-predicts j2/j3 by ~3 Nm at
the folded pose, and `sag = residual / kp`. Stiffer joints therefore help
directly, which is why the vendor gains (kp 150 on j2/j3) are the default.

## The model

`host/rebot_ik.py` carries the B601-RS chain transcribed from Seeed's
`urdf/RS/urdf/ReBot_Arm_RS.urdf`; the container reads the original file through
Pinocchio. The two were checked against each other: joint origins, axes and
limits identical, link masses and CoMs agreeing to 4e-9 m, gravity torques
agreeing to 0.000 Nm.

![model](host/arm_model.png)

## Vendor configuration

`config/rebotarm_rs.yaml` in `reBotArm_control_py` is the reference:

| item | vendor | this repo |
|---|---|---|
| motor models | rs-06 on j1-j3, rs-00 on j4-j6 + gripper | same (`--model-spec` default) |
| motor ids | 0x01-0x07 | 1-7 |
| MIT kp | 50 / 150 / 150 / 50 / 50 / 50 / 50 | same (default) |
| MIT kd | 3 / 10 / 10 / 5 / 4 / 4 / 4 | same (default) |
| URDF / end frame | `ReBot_Arm_RS.urdf`, `gripper_end` | same model |

Not adopted: `tau_scale` 1.0/0.98/0.98/1/1/1 (2 % next to the model's 30-40 %
gap) and CAN `rate: 500` (this rig runs 1 Mbit/s).

## Files

| file | job |
|---|---|
| `edge/dump_joints.py` | read parameter `0x7019` from each motor, print JSON. Enables nothing |
| `edge/play_trajectory.py` | stream a trajectory at `control_hz`, send its `tau`; dry run by default |
| `edge/gradual_motion_demo.py` | the control loop (MIT frames, ramps, shutdown) that the player uses |
| `edge/mit_frames.py` | MIT frame builder without numpy, 6x faster (needed at 30 Hz) |
| `host/Dockerfile.pinocchio` | python 3.11 + Pinocchio + `reBotArm_control_py`, pinned |
| `host/plan_with_pinocchio.py` | plans with their IK, adds `tau`, writes the JSON (runs in the container) |
| `host/repeat_pino.ps1` | one-shot: dump pose, plan, save, optionally replay |
| `host/rebot_ik.py` | the URDF transcription (model reference, used by the docs) |
| `trajectories/seq_pino_peel.json` | the trajectory |

## Notes

- **The peel** exists because the parked pose rests on the j2 and j3 lower stops
  (both within 7 mrad of 0.000). Peeling the wrist and shoulder instead leaves j3
  pinned and the dip grows to -18 mm; the elbow is the binding stop, and 8° over
  the jaw-opening leg removes the dip entirely.
- **Dry run is the default.** Nothing moves without `--execute`.
- **7 joints at 30 Hz needs `--fast-frames`**; without it the library's numpy path
  costs ~50 ms per tick and the motion stretches ~9 %.
- **`--payload kg`** on the planner folds a held mass into `tau` (1 kg at the tool
  adds -2.77 / +5.08 / +2.84 Nm on j2/j3/j4 at the parked pose).
- **Limits are re-checked on the edge**; a waypoint outside the URDF range is
  refused before any frame is sent.
- The trajectory is bound to the gains only through the file's `trim` block (this
  file has none), so `--kp`/`--kd` can be changed freely.
