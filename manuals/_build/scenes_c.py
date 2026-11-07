"""Layout C scenes: overhead island, no wall, vertical approach."""

import math
import random

from draw import *  # noqa
from card import blob_pts, draw_blob, label_wall, particles, project_xy
import props as P


def cam_c(vfov=36):
    from scene3d import Camera
    return Camera((0.30, -0.53, 0.60), (0.30, 0.16, 0.00), 1584, 730, vfov=vfov)


def island(s):
    P.counter(s, backsplash=False)
    # crisp far edge; no floor plane, the panel stays a studio backdrop
    s.face([(-0.16, 0.421, -0.045), (0.74, 0.421, -0.045),
            (0.74, 0.421, 0.0), (-0.16, 0.421, 0.0)], "#BFB5A2", double=True)


def overhead(s, wrist=(0.30, 0.20, 0.27), tool="brush"):
    P.arm_from(s, (0.30, 0.09, 1.15), wrist, thick=0.022)
    wx, wy, wz = wrist
    if tool == "brush":
        P.jaws(s, wx, wy, wz - 0.010, wz - 0.045)
        s.set_layer(1)
        s.tube((wx, wy, wz - 0.045), (wx, wy, 0.030), 0.011, "#8A6A45", n=16)
        s.box(wx - 0.035, wy - 0.022, 0.000, wx + 0.035, wy + 0.022, 0.030,
              "#3A4240", outline="#202625")
        s.set_layer(0)
    elif tool == "vacuum":
        P.jaws(s, wx, wy, wz - 0.010, wz - 0.045)
        s.set_layer(1)
        s.tube((wx, wy, wz - 0.045), (wx, wy, 0.015), 0.012, "#2E3533", n=16)
        s.box(wx - 0.020, wy - 0.028, 0.004, wx + 0.020, wy + 0.028, 0.016,
              "#1F2625", outline=None)
        s.set_layer(0)
    elif tool == "pad":
        P.jaws(s, wx, wy, wz - 0.010, wz - 0.045)
        s.set_layer(1)
        s.tube((wx, wy, wz - 0.045), (wx, wy, 0.060), 0.012, "#8A8F8C", n=16)
        s.box(wx - 0.042, wy - 0.032, 0.018, wx + 0.042, wy + 0.032, 0.052,
              "#EDE9DD", outline="#C9C2B2")
        s.set_layer(0)
    elif tool == "grip":
        P.jaws(s, wx, wy, wz - 0.010, wz - 0.070)


# --------------------------------------------------------------------------- #
# C-01  Flour dust
# --------------------------------------------------------------------------- #
def build_c01(s, cam):
    island(s)
    P.flour_bag(s, 0.10, 0.06)
    P.sieve(s, 0.345, 0.205)
    P.rolling_pin(s, 0.56, 0.31, length=0.22)
    overhead(s, wrist=(0.285, 0.245, 0.16), tool="brush")


def overlay_c01(c, cam):
    particles(c, cam, 620, 0.33, 0.20, 0.12, 0.085, seed=101, color="#FFFDF6",
              rmin=0.6, rmax=2.0, z=0.002, over=SURFACE_HI)
    particles(c, cam, 260, 0.18, 0.12, 0.08, 0.06, seed=102, color="#FFFDF6",
              rmin=0.6, rmax=1.8, z=0.003, over=SURFACE_HI)
    label_wall(c, cam, (0.285, 0.245, 0.03), (74, 600),
               "FLOUR DUST  ·  MICROFIBRE WAND", size=14)
    label_wall(c, cam, (0.34, 0.205, 0.04), (770, 664),
               "OVERHEAD DROP 550 mm  ·  NO AIR BLAST", size=14)


# --------------------------------------------------------------------------- #
# C-02  Breadcrumbs
# --------------------------------------------------------------------------- #
def build_c02(s, cam):
    island(s)
    P.board(s, 0.113, 0.075, 0.32, 0.21)
    P.bread(s, 0.145, 0.105, w=0.17, d=0.10, h=0.070)
    P.knife(s, 0.30, 0.145, angle=0.10, length=0.18)
    overhead(s, wrist=(0.315, 0.245, 0.14), tool="vacuum")


def overlay_c02(c, cam):
    particles(c, cam, 420, 0.30, 0.20, 0.10, 0.07, seed=111, color="#B98A4E",
              rmin=1.0, rmax=3.4, z=0.002, over=SURFACE_HI, alpha=0.95)
    particles(c, cam, 180, 0.25, 0.17, 0.08, 0.05, seed=112, color="#C99A5E",
              rmin=1.0, rmax=3.0, z=0.024, over=WOOD)
    label_wall(c, cam, (0.315, 0.245, 0.03), (74, 600),
               "CRUMB 2–8 mm  ·  VACUUM + BRUSH", size=14)
    label_wall(c, cam, (0.30, 0.10, 0.08), (770, 664),
               "BOARD GROOVE  ·  CLEAN IN PLACE", size=14)


# --------------------------------------------------------------------------- #
# C-03  Honey drizzle
# --------------------------------------------------------------------------- #
def build_c03(s, cam):
    island(s)
    P.honey_jar_tipped(s, 0.16, 0.17, length=0.14, r=0.036)
    # spoon
    s.set_layer(0)
    s.tube((0.47, 0.30, 0.012), (0.55, 0.34, 0.020), 0.008, "#B9C2C1", n=12)
    s.cylinder(0.455, 0.296, 0.022, 0.004, 0.012, "#B9C2C1", top="#D6DBD9")
    overhead(s, wrist=(0.375, 0.255, 0.13), tool="pad")


def overlay_c03(c, cam):
    # a continuous honey run with a soft shoulder and a highlight
    path = []
    for i in range(24):
        t = i / 23
        x = 0.300 + 0.085 * t
        y = 0.175 + 0.075 * t + 0.012 * math.sin(t * 3.2)
        path.append((x, y))
    pts = [project_xy(cam, x, y, 0.004) for x, y in path]
    c.line(pts, fill=rgba("#B97A1E", 0.95, SURFACE_HI), width=16)
    c.line(pts, fill=rgba("#D89A38", 0.95, SURFACE_HI), width=10)
    c.line([(p[0], p[1] - 2) for p in pts], fill=rgba("#F0C36A", 0.75, SURFACE_HI), width=2)
    draw_blob(c, cam, blob_pts(0.392, 0.258, 0.050, 0.034, seed=121, n=18, rough=0.18),
              rgba("#C98A2E", 0.92, SURFACE_HI), z=0.003)
    draw_blob(c, cam, blob_pts(0.390, 0.255, 0.028, 0.018, seed=122, n=12, rough=0.2),
              rgba("#E3B15A", 0.8, "#C98A2E"), z=0.0035)
    label_wall(c, cam, (0.375, 0.255, 0.04), (74, 600),
               "HONEY 15 mm WIDE  ·  DWELL 15 s", size=14)
    label_wall(c, cam, (0.31, 0.18, 0.05), (770, 664),
               "ROLL-LIFT, DO NOT SMEAR", size=14)


# --------------------------------------------------------------------------- #
# C-04  Red wine spill
# --------------------------------------------------------------------------- #
def build_c04(s, cam):
    island(s)
    P.wine_bottle_tipped(s, 0.115, 0.145, length=0.18, r=0.038)
    overhead(s, wrist=(0.40, 0.255, 0.13), tool="pad")


def overlay_c04(c, cam):
    draw_blob(c, cam, blob_pts(0.405, 0.27, 0.085, 0.058, seed=131, n=20, rough=0.28),
              rgba("#6E1F2E", 0.88, SURFACE_HI), z=0.002)
    draw_blob(c, cam, blob_pts(0.400, 0.266, 0.060, 0.040, seed=132, n=16, rough=0.25),
              rgba("#511523", 0.75, "#6E1F2E"), z=0.0023)
    rnd = random.Random(133)
    for _ in range(26):
        a = rnd.uniform(0, 2 * math.pi)
        rr = rnd.uniform(0.09, 0.13)
        p = project_xy(cam, 0.405 + rr * math.cos(a), 0.27 + rr * 0.7 * math.sin(a), 0.003)
        r = rnd.uniform(1.5, 4.0)
        c.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r],
                  fill=rgba("#6E1F2E", 0.6, SURFACE_HI))
    label_wall(c, cam, (0.405, 0.27, 0.01), (74, 600),
               "WINE 120 mL  ·  PAD + BLOT, EDGE-IN", size=14)
    label_wall(c, cam, (0.35, 0.20, 0.05), (770, 664),
               "5 min WINDOW  ·  TEST FOR STAIN", size=14)


# --------------------------------------------------------------------------- #
# C-05  Vegetable trimmings
# --------------------------------------------------------------------------- #
def build_c05(s, cam):
    island(s)
    P.board(s, 0.105, 0.065, 0.30, 0.20)
    P.carrot(s, 0.135, 0.125, angle=0.15, length=0.15)
    P.peeler(s, 0.245, 0.205, angle=0.2)
    P.bowl(s, 0.49, 0.185, r=0.060, h=0.052)
    overhead(s, wrist=(0.275, 0.255, 0.12), tool="grip")


def overlay_c05(c, cam):
    peels = [
        (0.19, 0.19, 0.030, 2.4, 4.2, 0.008, "#D9782E"),
        (0.23, 0.24, 0.026, 0.3, 2.0, 0.007, "#E08A3C"),
        (0.30, 0.26, 0.032, 2.8, 4.7, 0.008, "#C96A24"),
        (0.17, 0.28, 0.024, 0.8, 2.6, 0.006, "#5C7D3A"),
    ]
    for (x, y, r, a0, a1, w, col) in peels:
        pts = []
        n = 9
        for i in range(n):
            a = a0 + (a1 - a0) * i / (n - 1)
            pts.append((x + (r + w) * math.cos(a), y + (r + w) * math.sin(a)))
        for i in range(n):
            a = a1 - (a1 - a0) * i / (n - 1)
            pts.append((x + (r - w) * math.cos(a), y + (r - w) * math.sin(a)))
        c.poly([project_xy(cam, px, py, 0.024) for px, py in pts], fill=col,
               outline="#9C5A1E", width=0.7)
    label_wall(c, cam, (0.275, 0.255, 0.03), (74, 600),
               "PEEL + STEM  ·  GRASP → BOWL", size=14)
    label_wall(c, cam, (0.49, 0.185, 0.05), (770, 664),
               "BITS < 5 mm  ·  VACUUM FINISH", size=14)


