"""Layout B scenes: corner bay, hob and sink on adjacent legs."""

import math
import random

from draw import *  # noqa
from card import blob_pts, draw_blob, label_wall, particles, project_xy
import props as P


def cam_b(eye=(0.36, -0.50, 0.50), target=(0.30, 0.14, 0.02), vfov=38):
    from scene3d import Camera
    return Camera(eye, target, 1584, 730, vfov=vfov)


def cam_b_top():
    from scene3d import Camera
    return Camera((0.30, -0.48, 0.62), (0.30, 0.14, 0.02), 1584, 730, vfov=40)


def cam_b_wall():
    from scene3d import Camera
    return Camera((0.30, -0.36, 0.27), (0.30, 0.42, 0.16), 1584, 730, vfov=37)


def crescent(c, cam, cx, cy, r, a0, a1, width, color, z=0.003, outline=None):
    pts = []
    n = 10
    for i in range(n):
        a = a0 + (a1 - a0) * i / (n - 1)
        pts.append((cx + (r + width / 2) * math.cos(a), cy + (r + width / 2) * math.sin(a)))
    for i in range(n):
        a = a1 - (a1 - a0) * i / (n - 1)
        pts.append((cx + (r - width / 2) * math.cos(a), cy + (r - width / 2) * math.sin(a)))
    c.poly([project_xy(cam, x, y, z) for x, y in pts], fill=color, outline=outline, width=0.7)


def wall_splatter(c, cam, n=70, x0=0.12, x1=0.48, z0=0.06, z1=0.31, colors=("#B23A22", "#D45B32")):
    rnd = random.Random(88)
    for _ in range(n):
        x = rnd.uniform(x0, x1)
        z = rnd.uniform(z0, z1)
        p = cam.project((x, 0.416, z))
        r = rnd.uniform(1.6, 6.0)
        c.ellipse([p[0] - r, p[1] - r * 0.85, p[0] + r, p[1] + r * 0.85],
                  fill=rgba(colors[0], 0.75, "#EFE9DD"))
        c.ellipse([p[0] - r * 0.35, p[1] - r * 0.55, p[0] + r * 0.15, p[1] - r * 0.15],
                  fill=rgba(colors[1], 0.8, "#EFE9DD"))
        if rnd.random() < 0.30:
            c.line([(p[0], p[1] + r), (p[0], p[1] + r * rnd.uniform(2.0, 4.0))],
                   fill=rgba(colors[0], 0.45, "#EFE9DD"), width=max(1, r * 0.35))


# --------------------------------------------------------------------------- #
# B-01  Cooled grease ring
# --------------------------------------------------------------------------- #
def build_b01(s, cam):
    P.counter(s)
    P.hob(s, 0.10, 0.06, 0.50, 0.32)
    P.pan(s, 0.48, 0.32, r=0.078)
    P.arm_from(s, (-0.55, -0.12, 0.52), (0.255, 0.205, 0.15), elbow=(0.03, 0.03, 0.33))
    P.jaws(s, 0.255, 0.205, 0.14, 0.105)
    P.scraper(s, 0.255, 0.205, 0.012, angle=0.5)


def overlay_b01(c, cam):
    cx, cy, z = 0.238, 0.168, 0.0145
    outer = []
    inner = []
    for i in range(48):
        a = 2 * math.pi * i / 48
        outer.append(project_xy(cam, cx + 0.100 * math.cos(a), cy + 0.100 * math.sin(a), z))
        inner.append(project_xy(cam, cx + 0.076 * math.cos(-a), cy + 0.076 * math.sin(-a), z))
    c.poly(outer + inner, fill=rgba("#6B3A18", 0.82, "#2B3231"), outline=None)
    rnd = random.Random(12)
    for _ in range(45):
        a = rnd.uniform(0, 2 * math.pi)
        rr = rnd.uniform(0.076, 0.105)
        p = project_xy(cam, cx + rr * math.cos(a), cy + rr * math.sin(a), z + 0.001)
        c.ellipse([p[0] - 2, p[1] - 1.6, p[0] + 2, p[1] + 1.6],
                  fill=rgba("#5A2E12", 0.85, "#2B3231"))
    label_wall(c, cam, (0.24, 0.17, 0.02), (74, 610),
               "POLYMERISED RING  ·  SILICONE SCRAPER", size=14)
    label_wall(c, cam, (0.47, 0.31, 0.03), (770, 670),
               "SURFACE < 40 °C  ·  BURNER LOCKED", size=14)


# --------------------------------------------------------------------------- #
# B-02  Tomato sauce on the splashback
# --------------------------------------------------------------------------- #
def build_b02(s, cam):
    P.counter(s)
    P.hob(s, 0.08, 0.02, 0.55, 0.26)
    P.arm_from(s, (-0.60, -0.16, 0.44), (0.315, 0.394, 0.175), elbow=(0.02, 0.10, 0.32))
    P.jaws(s, 0.315, 0.394, 0.165, 0.132)
    s.box(0.275, 0.398, 0.098, 0.355, 0.407, 0.132, "#C46A2E", outline="#8F4A1D")


def overlay_b02(c, cam):
    wall_splatter(c, cam, n=90, x0=0.10, x1=0.50, z0=0.05, z1=0.32)
    path = [cam.project((0.30, 0.414, z))[:2] for z in (0.30, 0.24, 0.17, 0.10)]
    c.dashed(path, dash=9, gap=6, fill="#6B7A78", width=1.2)
    c.arrow(path, fill="#6B7A78", width=1.2, head=7)
    label_wall(c, cam, (0.42, 0.414, 0.28), (1330, 120),
               "TOMATO SPLATTER  ·  20 s DWELL", size=14, align="right")
    label_wall(c, cam, (0.46, 0.414, 0.10), (1330, 208),
               "GROUT LINES  ·  DETAIL BRUSH", size=14, align="right")


# --------------------------------------------------------------------------- #
# B-03  Onion skins in the corner
# --------------------------------------------------------------------------- #
def build_b03(s, cam):
    P.counter(s)
    P.board(s, 0.12, 0.07, 0.28, 0.19)
    P.onion(s, 0.185, 0.175, r=0.050)
    P.knife(s, 0.25, 0.13, angle=0.15, length=0.19)
    P.compost_chute(s, 0.60, 0.28, r=0.060)
    P.arm_from(s, (-0.55, -0.10, 0.50), (0.26, 0.205, 0.12), elbow=(0.02, 0.03, 0.32))
    P.jaws(s, 0.26, 0.205, 0.11, 0.055)


def overlay_b03(c, cam):
    peels = [
        (0.30, 0.24, 0.035, 2.6, 4.4, 0.014, "#8C5A86"),
        (0.36, 0.31, 0.045, 0.2, 1.9, 0.012, "#C79A4A"),
        (0.21, 0.29, 0.032, 4.6, 6.0, 0.010, "#8C5A86"),
        (0.39, 0.22, 0.028, 3.0, 5.1, 0.011, "#B27A3C"),
        (0.17, 0.30, 0.026, 1.2, 3.2, 0.009, "#C79A4A"),
    ]
    for (x, y, r, a0, a1, w, col) in peels:
        crescent(c, cam, x, y, r, a0, a1, w, col, outline="#6F4468")
    label_wall(c, cam, (0.26, 0.205, 0.05), (74, 606),
               "PEEL 10–40 mm  ·  GRASP + DROP", size=14)
    label_wall(c, cam, (0.60, 0.28, 0.02), (770, 668),
               "COMPOST CHUTE Ø120  ·  NO KNIFE", size=14)


# --------------------------------------------------------------------------- #
# B-04  Raw egg on the sink deck
# --------------------------------------------------------------------------- #
def build_b04(s, cam):
    P.counter(s)
    P.sink(s, 0.46, 0.01, 0.26, 0.35)
    P.arm_from(s, (-0.55, -0.12, 0.48), (0.285, 0.215, 0.135), elbow=(0.02, 0.02, 0.31))
    P.jaws(s, 0.285, 0.215, 0.125, 0.090)
    s.box(0.255, 0.195, 0.056, 0.315, 0.235, 0.090, "#BFD8E6", outline="#7FA3B5")


def overlay_b04(c, cam):
    # a thin, irregular albumen with a broken yolk and shell shards
    draw_blob(c, cam, blob_pts(0.300, 0.247, 0.105, 0.064, seed=61, n=24, rough=0.32),
              rgba("#E6F0E9", 0.42, SURFACE_HI), z=0.002)
    draw_blob(c, cam, blob_pts(0.315, 0.240, 0.082, 0.048, seed=64, n=20, rough=0.28),
              rgba("#D3E1DB", 0.24, SURFACE_HI), z=0.0022)
    draw_blob(c, cam, blob_pts(0.325, 0.238, 0.030, 0.023, seed=62, n=14, rough=0.22),
              rgba("#E39A1E", 0.96, "#E6F0E9"), z=0.0025)
    draw_blob(c, cam, blob_pts(0.341, 0.248, 0.014, 0.011, seed=65, n=10, rough=0.2),
              rgba("#F0B23A", 0.95, "#E39A1E"), z=0.0027)
    # yolk run toward the front edge
    c.line([project_xy(cam, 0.330, 0.250, 0.0028), project_xy(cam, 0.360, 0.272, 0.0028)],
           fill=rgba("#E39A1E", 0.55, SURFACE_HI), width=5)
    for (x, y, r, a) in [(0.215, 0.285, 0.034, 0.3), (0.385, 0.265, 0.028, -0.5),
                         (0.265, 0.305, 0.020, 1.1)]:
        pts = [(x + r * math.cos(2 * math.pi * i / 7 + a),
                y + r * 0.65 * math.sin(2 * math.pi * i / 7 + a)) for i in range(7)]
        c.poly([project_xy(cam, px, py, 0.003) for px, py in pts],
               fill="#F8F3E8", outline="#CFC5B3", width=0.8)
    label_wall(c, cam, (0.300, 0.247, 0.01), (74, 606),
               "RAW EGG  ·  BIOHAZARD / ONE-WAY PAD", size=14)
    label_wall(c, cam, (0.46, 0.10, 0.01), (770, 668),
               "SINK DECK  ·  NO RINSE, NO REUSE", size=14)


# --------------------------------------------------------------------------- #
# B-05  Paprika spill
# --------------------------------------------------------------------------- #
def build_b05(s, cam):
    P.counter(s)
    # two upright jars and one tipped
    P.jar(s, 0.145, 0.085, r=0.036, h=0.115, fill="#B24A22", cap="#6E5B45")
    P.jar(s, 0.225, 0.085, r=0.032, h=0.105, fill="#C98A2E", cap="#8C7A5E")
    s.tube((0.195, 0.20, 0.035), (0.315, 0.24, 0.035), 0.036, "#C9B79A", n=24)
    s.cylinder(0.322, 0.243, 0.034, 0.0, 0.012, "#C9B79A", top="#6E5B45")
    P.arm_from(s, (-0.55, -0.12, 0.48), (0.30, 0.215, 0.14), elbow=(0.03, 0.02, 0.32))
    P.jaws(s, 0.30, 0.215, 0.13, 0.095)
    s.box(0.270, 0.195, 0.066, 0.330, 0.235, 0.095, "#EDE9DD", outline="#C9C2B2")


def overlay_b05(c, cam):
    draw_blob(c, cam, blob_pts(0.355, 0.26, 0.075, 0.050, seed=71, n=18, rough=0.30),
              rgba("#C25A28", 0.30, SURFACE_HI), z=0.001)
    particles(c, cam, 420, 0.355, 0.26, 0.070, 0.048, seed=72, color="#B6401C",
              rmin=0.6, rmax=2.0, z=0.002, over=SURFACE_HI, alpha=0.9)
    label_wall(c, cam, (0.30, 0.215, 0.02), (74, 606),
               "PAPRIKA 15 g  ·  DRY PICKUP FIRST", size=14)
    label_wall(c, cam, (0.40, 0.28, 0.0), (770, 668),
               "STAINING  ·  NO WATER YET", size=14)
