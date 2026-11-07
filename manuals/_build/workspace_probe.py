"""Pure-stdlib FK probe for the B601-RS cleaning envelope.

No numpy on this machine, so this is a small 4x4 matrix implementation. It
samples joint space and keeps the poses where a tool-mounted pad (80 mm out
along the tool axis) faces down onto the counter, then reports the horizontal
radius the pad can reach at counter height.

The numbers feed the "working envelope" values quoted in the manuals.
"""

import math
import random

CHAIN = (
    ("joint1", (-0.00034283, -0.00098683, 0.075), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), -2.80, 2.80),
    ("joint2", (0.020343, 0.027237, 0.07), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0), 0.00, 3.14),
    ("joint3", (-0.236, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 0.00, 3.14),
    ("joint4", (0.228, -0.072746, 0.0045), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), -1.57, 1.57),
    ("joint5", (0.087, -0.048, -0.03075), (-1.5708, 0.0, 0.0), (0.0, 0.0, -1.0), -1.57, 1.57),
    ("joint6", (0.0365, 0.0, 0.048), (0.0, 1.5708, 0.0), (0.0, 0.0, -1.0), -3.14, 3.14),
)
TOOL_XYZ = (0.0, 0.0, 0.16621)
TOOL_RPY = (3.1416, -1.5708, 0.0)


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def tf(rot=None, trans=(0, 0, 0)):
    m = [row[:] for row in (rot or [[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
    return [
        [m[0][0], m[0][1], m[0][2], trans[0]],
        [m[1][0], m[1][1], m[1][2], trans[1]],
        [m[2][0], m[2][1], m[2][2], trans[2]],
        [0, 0, 0, 1],
    ]


def rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return mat_mul3(rz, mat_mul3(ry, rx))


def mat_mul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def axis_rot(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def forward(q):
    t = tf()
    for (_n, xyz, r, axis, _lo, _hi), ang in zip(CHAIN, q):
        t = mat_mul(t, tf(rpy(*r), xyz))
        t = mat_mul(t, tf(axis_rot(axis, ang)))
    t = mat_mul(t, tf(rpy(*TOOL_RPY), TOOL_XYZ))
    return t


def main():
    random.seed(7)
    samples = 400_000
    best = []
    n_down = 0
    for _ in range(samples):
        q = [random.uniform(lo, hi) for _n, _x, _r, _a, lo, hi in CHAIN]
        t = forward(q)
        zx, zy, zz = t[0][2], t[1][2], t[2][2]
        if zz > -0.94:  # tool axis must point down within ~20 deg
            continue
        n_down += 1
        # pad contact 80 mm below the tool along the tool axis
        px = t[0][3] + 0.080 * zx
        py = t[1][3] + 0.080 * zy
        pz = t[2][3] + 0.080 * zz
        if -0.005 <= pz <= 0.060:
            best.append((math.hypot(px, py), pz, px, py, q))
    best.sort(key=lambda r: r[0], reverse=True)
    print(f"samples={samples} down_poses={n_down} counter_poses={len(best)}")
    if best:
        print("max radius at counter:", round(best[0][0] * 1000, 1), "mm at z", round(best[0][1] * 1000, 1))
        for pct in (50, 90, 99):
            idx = int(len(best) * pct / 100) - 1
            print(f"p{pct} radius:", round(best[idx][0] * 1000, 1), "mm")
        # radius by approach direction octant
        octants = {}
        for r, pz, px, py, _q in best:
            ang = math.degrees(math.atan2(py, px)) % 360
            octant = int(ang // 45) * 45
            octants[octant] = max(octants.get(octant, 0), r)
        for k in sorted(octants):
            print(f"  {k:3d}-{k+45:3d} deg: {octants[k]*1000:6.1f} mm")


if __name__ == "__main__":
    main()
