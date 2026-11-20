"""Can the B601-RS work *under* an upper cabinet?

Samples joint space for poses that put the tool on the worktop below a
350 mm-deep cabinet whose underside is 450 mm above the worktop. The base sits
on the counter at y=base_y, in front of the cabinet; the arm folds forward
under the cabinet. Rejects poses where any link capsule intersects the cabinet
AABB. Reports how far behind the cabinet front edge the tool reaches, the
highest link point of that pose, and its clearance below the underside. Pure
stdlib, same chain as `workspace_probe.py`.
"""

import math
import random

from workspace_probe import CHAIN, TOOL_XYZ, TOOL_RPY, forward, mat_mul, tf, rpy


def joint_points(q):
    """World positions of joint origins plus the tool point."""
    t = tf()
    pts = []
    for (_n, xyz, r, axis, _lo, _hi), ang in zip(CHAIN, q):
        t = mat_mul(t, tf(rpy(*r), xyz))
        pts.append((t[0][3], t[1][3], t[2][3]))
        from workspace_probe import axis_rot
        t = mat_mul(t, tf(axis_rot(axis, ang)))
    t = mat_mul(t, tf(rpy(*TOOL_RPY), TOOL_XYZ))
    tool = (t[0][3], t[1][3], t[2][3])
    zaxis = (t[0][2], t[1][2], t[2][2])
    return pts + [tool], tool, zaxis


def point_box_distance(p, box):
    x, y, z = p
    x0, y0, z0, x1, y1, z1 = box
    dx = max(x0 - x, 0, x - x1)
    dy = max(y0 - y, 0, y - y1)
    dz = max(z0 - z, 0, z - z1)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def segment_box_clear(p, q, box, radius, samples=24):
    for i in range(samples + 1):
        t = i / samples
        x = p[0] + (q[0] - p[0]) * t
        y = p[1] + (q[1] - p[1]) * t
        z = p[2] + (q[2] - p[2]) * t
        if point_box_distance((x, y, z), box) < radius:
            return False
    return True


def probe(cabinet_depth=0.35, cabinet_bottom=0.45, base_y=0.55, samples=300_000):
    # cabinet against the wall (y=0), 600 mm wide; the base sits on the counter
    # at y=base_y, in front of the cabinet edge, and folds forward under it
    box = (-0.30, 0.0, cabinet_bottom, 0.30, cabinet_depth, 2.25)
    radii = [0.055, 0.055, 0.050, 0.045, 0.035, 0.030, 0.020]
    random.seed(5)
    best = None
    hits = 0
    for _ in range(samples):
        q = [random.uniform(lo, hi) for _n, _x, _r, _a, lo, hi in CHAIN]
        pts, tool, zaxis = joint_points(q)
        pts = [(x, y + base_y, z) for x, y, z in pts]
        tool = (tool[0], tool[1] + base_y, tool[2])
        # tool on the worktop, pointing down, under the cabinet
        if not (0.005 < tool[2] < 0.060):
            continue
        if zaxis[2] > -0.90:
            continue
        if not (0.06 < tool[1] < cabinet_depth - 0.03 and -0.22 < tool[0] < 0.22):
            continue
        hits += 1
        ok = True
        # skip the base link (inside the counter) and check the moving links
        for i in range(1, len(pts) - 1):
            if not segment_box_clear(pts[i], pts[i + 1], box, radii[i] + 0.008):
                ok = False
                break
        if not ok:
            continue
        depth = cabinet_depth - tool[1]
        if best is None or depth > best[1]:
            best = (q, depth, tool, max(p[2] for p in pts))
    return hits, best


if __name__ == "__main__":
    for cab_depth in (0.20, 0.25, 0.30, 0.35):
        for bottom in (0.40, 0.45, 0.50):
            hits, best = probe(cabinet_depth=cab_depth, cabinet_bottom=bottom, samples=200_000)
            if best:
                _q, depth, tool, maxz = best
                print(f"depth={cab_depth:.2f} bottom={bottom:.2f}: tool {depth*1000:.0f} mm behind edge "
                      f"(y={tool[1]:.3f} m); highest link {maxz*1000:.0f} mm; "
                      f"clearance {(bottom - maxz)*1000:.0f} mm; candidates={hits}")
            else:
                print(f"depth={cab_depth:.2f} bottom={bottom:.2f}: NO collision-free pose; candidates={hits}")
