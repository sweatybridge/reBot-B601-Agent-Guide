"""Reach bands for the B601-RS, sampled in pure Python (no numpy).

Outputs the numbers used in the manuals:
  * tool-down pad reach at several heights above the counter
  * horizontal-surface reach vs. wall (backsplash) approach
  * vertical-surface (backsplash) reachable band
"""

import math
import random

from workspace_probe import CHAIN, forward

random.seed(11)


def sample(n=500_000):
    down = []
    wall = []
    for _ in range(n):
        q = [random.uniform(lo, hi) for _n, _x, _r, _a, lo, hi in CHAIN]
        t = forward(q)
        zx, zy, zz = t[0][2], t[1][2], t[2][2]
        tx, ty, tz = t[0][3], t[1][3], t[2][3]
        # Pad contact 80 mm along the tool axis. Tool-down: pad near counter.
        if zz < -0.94:
            px, py, pz = tx + 0.080 * zx, ty + 0.080 * zy, tz + 0.080 * zz
            if -0.005 <= pz <= 0.50:
                down.append((math.hypot(px, py), pz))
        # Wall approach: tool axis horizontal, pad in front of tool, 0..0.5 m up.
        if abs(zz) < 0.20:
            px, py, pz = tx + 0.080 * zx, ty + 0.080 * zy, tz + 0.080 * zz
            if 0.0 <= pz <= 0.60:
                wall.append((math.hypot(px, py), pz))
    return down, wall


def band(points, lo, hi):
    vals = [r for r, z in points if lo <= z < hi]
    if not vals:
        return None
    vals.sort()
    return vals[-1], vals[int(len(vals) * 0.99) - 1]


down, wall = sample()
print("tool-down pad reach by height above counter (max / p99):")
for lo, hi in ((0.0, 0.06), (0.06, 0.15), (0.15, 0.30), (0.30, 0.50)):
    b = band(down, lo, hi)
    if b:
        print(f"  {lo*1000:3.0f}-{hi*1000:3.0f} mm: {b[0]*1000:6.1f} / {b[1]*1000:6.1f} mm  (n={sum(1 for r,z in down if lo<=z<hi)})")

print("wall-approach pad reach by height (max / p99):")
for lo, hi in ((0.0, 0.10), (0.10, 0.25), (0.25, 0.40), (0.40, 0.60)):
    b = band(wall, lo, hi)
    if b:
        print(f"  {lo*1000:3.0f}-{hi*1000:3.0f} mm: {b[0]*1000:6.1f} / {b[1]*1000:6.1f} mm  (n={sum(1 for r,z in wall if lo<=z<hi)})")

# Minimum reach: can the pad get close to the base? Useful for sink rims.
close = sorted(r for r, z in down if z < 0.06)
print("closest tool-down pad to base axis:", round(close[0] * 1000, 1), "mm")
