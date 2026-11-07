"""Kitchen props for the perspective scenes, in metres."""

from __future__ import annotations

import math
import random

from draw import CABINET, LINE, STEEL, STEEL_DARK, SURFACE, WOOD, WOOD_DARK, WOOD_LIGHT, mix, shade
from scene3d import Scene


# --------------------------------------------------------------------------- #
# Counter context
# --------------------------------------------------------------------------- #
def counter(s: Scene, backsplash=True, wall_color="#EFE9DD", tile=True, cabinet=True):
    s.set_layer(-10)
    s.box(-0.16, -0.07, -0.045, 0.74, 0.42, 0.0, SURFACE, outline="#C9C0B0")
    if cabinet:
        s.box(-0.16, -0.07, -0.37, 0.74, -0.035, -0.045, CABINET, outline=None)
        # drawer seams and a pull, so the foreground reads as cabinetry
        for z in (-0.145, -0.255):
            s.face([(-0.16, -0.0705, z), (0.74, -0.0705, z),
                    (0.74, -0.0705, z - 0.002), (-0.16, -0.0705, z - 0.002)],
                   "#C4B8A2", double=True)
        s.box(-0.16, -0.07, -0.37, 0.74, -0.035, -0.355, "#CBBFA9", outline=None)
    s.set_layer(-9)
    if backsplash:
        s.plane([(-0.16, 0.42, 0.0), (0.74, 0.42, 0.0), (0.74, 0.42, 0.34), (-0.16, 0.42, 0.34)],
                wall_color, outline="#D6CBBB")
        if tile:
            for i in range(1, 9):
                x = -0.16 + 0.90 * i / 9
                s.face([(x, 0.419, 0.0), (x + 0.003, 0.419, 0.0),
                        (x + 0.003, 0.419, 0.34), (x, 0.419, 0.34)],
                       "#DDD4C4", double=True)
            for j in range(1, 4):
                z = 0.34 * j / 4
                s.face([(-0.16, 0.419, z), (0.74, 0.419, z),
                        (0.74, 0.419, z + 0.003), (-0.16, 0.419, z + 0.003)],
                       "#DDD4C4", double=True)
    s.set_layer(0)


def sink(s: Scene, x0, y0, w, d, deck=True):
    if deck:
        s.box(x0 - 0.02, y0 - 0.02, 0.0, x0 + w + 0.02, y0 + d + 0.02, 0.004,
              STEEL, outline=STEEL_DARK)
    s.box(x0, y0, 0.004, x0 + w, y0 + d, 0.006, "#AEB9B8", outline=STEEL_DARK)
    inset = 0.012
    s.box(x0 + inset, y0 + inset, 0.006, x0 + w - inset, y0 + d - inset, 0.007,
          "#7E8C8B", outline="#6E7C7B")
    s.box(x0 + inset * 2, y0 + inset * 2, 0.007, x0 + w - inset * 2, y0 + d - inset * 2,
          0.0075, "#6E7B7A", outline=None)
    # drain
    s.prism(x0 + w / 2, y0 + d / 2, 0.022, 0.0076, 0.009, "#4E5A59", n=24)


def faucet(s: Scene, x, y, z=0.0):
    s.tube((x, y, z), (x, y, z + 0.20), 0.012, STEEL, n=20)
    s.tube((x, y, z + 0.20), (x, y - 0.09, z + 0.24), 0.010, STEEL, n=20)
    s.tube((x, y - 0.09, z + 0.24), (x, y - 0.09, z + 0.14), 0.009, STEEL_DARK, n=16)


def board(s: Scene, x, y, w, d, h=0.022, color=WOOD):
    shadow_disc(s, x + w / 2, y + d / 2, w * 0.55, d * 0.60)
    s.box(x, y, 0.0, x + w, y + d, h, color, outline=WOOD_DARK)
    s.prism(x + w * 0.82, y + d * 0.5, 0.014, h, h + 0.001, shade(color, -0.12), n=24)


def shadow_disc(s: Scene, cx, cy, rx, ry, z=0.0005, color="#D5CBBA"):
    s.set_layer(-8)
    s.face([(cx + rx * math.cos(2 * math.pi * i / 40),
             cy + ry * math.sin(2 * math.pi * i / 40), z) for i in range(40)],
           color, double=True)
    s.set_layer(0)


# --------------------------------------------------------------------------- #
# Vessels
# --------------------------------------------------------------------------- #
def egg_half(s: Scene, cx, cy, r=0.032, h=0.030, color="#F4EDDF"):
    s.cylinder(cx, cy, r, 0.004, h * 0.35, color, top=shade(color, -0.04))
    s.cone(cx, cy, r, r * 0.92, h * 0.35, h, color, top=shade(color, 0.04))
    s.face([(cx + r * 0.80 * math.cos(2 * math.pi * i / 32),
             cy + r * 0.80 * math.sin(2 * math.pi * i / 32), h + 0.0015)
            for i in range(32)], "#D8CBB4", double=True)


def shard(s: Scene, pts, z=0.006, color="#F6F1E6"):
    s.face([(x, y, z) for x, y in pts], color, outline="#CFC5B3", double=True)
    # thin side so the piece reads as a solid, not a decal
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        s.face([(a[0], a[1], z), (b[0], b[1], z),
                (b[0], b[1], z - 0.003), (a[0], a[1], z - 0.003)], "#DED3BF")


def shell_bowl(s: Scene, cx, cy, r=0.036, color="#F4EDDF"):
    """Jagged half eggshell: irregular rim, hollow interior."""
    n = 26
    rim = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rr = r * (0.92 + 0.10 * ((i * 7) % 5) / 4.0)
        z = 0.016 + 0.006 * ((i * 3) % 4) / 3.0
        rim.append((cx + rr * math.cos(a), cy + rr * math.sin(a), z))
    for i in range(n):
        a, b = rim[i], rim[(i + 1) % n]
        s.face([(a[0], a[1], 0.0), (b[0], b[1], 0.0), (b[0], b[1], b[2]), (a[0], a[1], a[2])],
               color, outline="#D6C9B2")
    s.face([(p[0] * 0.86 + cx * 0.14, p[1] * 0.86 + cy * 0.14, 0.006) for p in rim],
           "#D9CCB5", double=True)


def flour_bag(s: Scene, x, y, w=0.10, d=0.07, h=0.22):
    s.box(x, y, 0.0, x + w, y + d, h, "#D9C9AC", outline="#B8A483")
    s.box(x + 0.008, y + 0.008, h, x + w - 0.008, y + d - 0.008, h + 0.010,
          "#CBB894", outline="#B8A483")
    s.box(x + 0.018, y + 0.018, h + 0.008, x + w - 0.018, y + d - 0.018, h + 0.010,
          "#EFE4CC", outline=None)


def sieve(s: Scene, cx, cy, r=0.065, color="#C9CFCC"):
    s.cylinder(cx, cy, r, 0.0, 0.030, color, top="#DCE1DE")
    s.face([(cx + r * 0.82 * math.cos(2 * math.pi * i / 40),
             cy + r * 0.82 * math.sin(2 * math.pi * i / 40), 0.0315) for i in range(40)],
           "#AEB4B1", double=True)
    s.tube((cx + r, cy, 0.025), (cx + r + 0.10, cy, 0.028), 0.010, "#8E9693", n=12)


def peeler(s: Scene, x, y, angle=0.0):
    dx, dy = math.cos(angle), math.sin(angle)
    s.box(x - 0.05, y - 0.012, 0.006, x + 0.06, y + 0.012, 0.020, "#3D6B58",
          outline="#274839")
    s.box(x + 0.06, y - 0.015, 0.010, x + 0.10, y + 0.015, 0.014, "#B9C2C0",
          outline="#8E9997")


def honey_jar_tipped(s: Scene, x, y, length=0.14, r=0.036):
    s.tube((x, y, r * 0.85), (x + length, y, r * 0.85), r, "#C98A2E", n=24)
    s.cylinder(x + length, y, r, 0.0, 0.002, "#6B4A2C", top="#4A321C")
    s.tube((x - 0.035, y, r * 0.85), (x, y, r * 0.85), r * 1.08, "#8C7A5E", n=20)


def wine_bottle_tipped(s: Scene, x, y, length=0.18, r=0.038):
    s.tube((x, y, r * 0.9), (x + length, y, r * 0.9), r, "#2F4A34", n=24)
    s.tube((x + length, y, r * 0.9), (x + length + 0.065, y, r * 0.9), 0.017,
           "#2F4A34", n=16)
    s.cylinder(x + length + 0.065, y, 0.017, 0.0, 0.002, "#1F2E22", top="#151F17")
    s.cylinder(x - 0.002, y, r * 1.04, 0.0, 0.002, "#233A28", top="#1A2B1E")


def bowl(s: Scene, cx, cy, r=0.064, h=0.050, color="#D3C9B6", inner="#A99E89"):
    s.cone(cx, cy, r * 0.62, r, 0.0, h, color, outline="#BDB3A0", top=shade(color, 0.12))
    s.face([(cx + r * 0.80 * math.cos(2 * math.pi * i / 40),
             cy + r * 0.80 * math.sin(2 * math.pi * i / 40), h + 0.001) for i in range(40)],
           inner, double=True)


def mug(s: Scene, cx, cy, r=0.040, h=0.090, color="#E8E2D6"):
    s.cylinder(cx, cy, r, 0.0, h, color, top=shade(color, 0.08),
               inner=mix(color, "#6B5B4A", 0.55))
    # handle: two small tubes on the +x side
    s.tube((cx + r * 0.9, cy, h * 0.75), (cx + r * 1.55, cy, h * 0.60), 0.007, color, n=12)
    s.tube((cx + r * 1.55, cy, h * 0.60), (cx + r * 1.45, cy, h * 0.25), 0.007, color, n=12)


def jar(s: Scene, cx, cy, r=0.045, h=0.130, fill="#D9A441", glass="#E6E0D3", cap="#8C7A5E"):
    s.cylinder(cx, cy, r, 0.0, h * 0.82, glass, top=shade(glass, 0.05), inner=fill)
    s.cylinder(cx, cy, r * 0.92, h * 0.82, h, cap, top=shade(cap, 0.10))


def bottle(s: Scene, cx, cy, r=0.034, h=0.115, color="#6B4A2C", cap="#3B3B36"):
    s.cylinder(cx, cy, r, 0.0, h * 0.72, color, top=shade(color, 0.10))
    s.cylinder(cx, cy, r * 0.55, h * 0.72, h * 0.93, color, top=shade(color, 0.10))
    s.cylinder(cx, cy, r * 0.58, h * 0.93, h, cap, top=shade(cap, 0.10))


def wine_glass(s: Scene, cx, cy, tipped=False, color="#E8E3D8"):
    if not tipped:
        s.prism(cx, cy, 0.038, 0.0, 0.006, color, n=24)
        s.tube((cx, cy, 0.006), (cx, cy, 0.095), 0.004, color, n=10)
        s.cone(cx, cy, 0.010, 0.040, 0.095, 0.185, color, top=None)
    else:
        # lying on its side, bowl toward +x
        s.tube((cx - 0.075, cy, 0.020), (cx + 0.055, cy, 0.020), 0.010, color, n=14)
        s.cone(cx + 0.055, cy, 0.010, 0.042, 0.020, 0.020, color)
        s.prism(cx - 0.082, cy, 0.040, 0.0, 0.006, color, n=20)


def pan(s: Scene, cx, cy, r=0.115, h=0.038, color="#3E4745"):
    s.cylinder(cx, cy, r, 0.0, h, color, top=shade(color, 0.06),
               inner="#4B5452")
    s.box(cx + r, cy - 0.018, h * 0.45, cx + r + 0.16, cy + 0.018, h * 0.62, "#2C3432",
          outline="#1F2625")


def toaster(s: Scene, x, y, color="#C9CDCB"):
    s.box(x, y, 0.0, x + 0.26, y + 0.16, 0.19, color, outline="#8E9896")
    for i in range(2):
        s.box(x + 0.03 + i * 0.10, y + 0.06, 0.19, x + 0.09 + i * 0.10, y + 0.10, 0.192,
              "#3A4240", outline=None)
    s.box(x + 0.24, y + 0.05, 0.10, x + 0.265, y + 0.06, 0.14, "#6E7876", outline=None)


def grinder(s: Scene, cx, cy, color="#444B49"):
    s.cylinder(cx, cy, 0.055, 0.0, 0.20, color, top=shade(color, 0.10), inner="#2E3533")
    s.cone(cx, cy, 0.055, 0.038, 0.20, 0.255, "#B9A27E", top="#CBB58F")
    s.cylinder(cx, cy, 0.020, 0.255, 0.275, "#6B5B45")


def hob(s: Scene, x, y, w, d):
    s.box(x, y, 0.0, x + w, y + d, 0.008, "#252C2B", outline="#171D1C")
    positions = [(x + w * 0.28, y + d * 0.30), (x + w * 0.72, y + d * 0.30),
                 (x + w * 0.28, y + d * 0.72), (x + w * 0.72, y + d * 0.72)]
    for (bx, by) in positions:
        s.prism(bx, by, 0.070, 0.008, 0.011, "#3A4240", n=36, outline="#202625")
        s.prism(bx, by, 0.052, 0.011, 0.013, "#4A5350", n=36, outline=None)
        # grate bars
        for ang in (0, math.pi / 2):
            dx, dy = math.cos(ang) * 0.085, math.sin(ang) * 0.085
            s.tube((bx - dx, by - dy, 0.014), (bx + dx, by + dy, 0.014), 0.006, "#202625", n=8)
    return positions


def eggshell(s: Scene, cx, cy, z=0.0, scale=1.0, color="#F2EBDD"):
    for (dx, dy, w, d, a) in [(-0.035, 0.004, 0.045, 0.030, 0.3),
                              (0.030, -0.012, 0.038, 0.026, -0.4),
                              (0.006, 0.030, 0.030, 0.020, 1.1)]:
        pts = []
        for i in range(8):
            ang = 2 * math.pi * i / 8 + a
            pts.append((cx + dx + w * math.cos(ang) * scale,
                        cy + dy + d * math.sin(ang) * scale, z + 0.006))
        s.face(pts, color, outline="#CFC5B3", double=True)
        s.face([(p[0], p[1], z + 0.0005) for p in pts], "#D8CDBA", double=True)


def knife(s: Scene, x, y, angle=0.0, length=0.20, blade=0.14):
    dx, dy = math.cos(angle), math.sin(angle)
    s.box(x - 0.008, y - 0.012, 0.004, x + blade, y + 0.012, 0.008, "#C3CBC9",
          outline="#8E9997")
    s.box(x + blade, y - 0.010, 0.006, x + length, y + 0.010, 0.018, "#3A3028",
          outline="#241D18")


def onion(s: Scene, cx, cy, r=0.048):
    s.sphere(cx, cy, r * 0.8, r, "#B98A4E")


def compost_chute(s: Scene, cx, cy, r=0.060):
    s.cylinder(cx, cy, r + 0.012, 0.0, 0.012, "#B9C2C1", top="#D4DBD9")
    s.cylinder(cx, cy, r, 0.003, 0.013, "#3C4645", top="#2A3332")


def bread(s: Scene, x, y, w=0.18, d=0.11, h=0.075):
    # low-poly loaf
    pts_top = [(x + 0.01, y + 0.02), (x + w * 0.30, y), (x + w * 0.78, y + 0.005),
               (x + w - 0.01, y + 0.035), (x + w - 0.02, y + d - 0.02),
               (x + w * 0.62, y + d), (x + w * 0.18, y + d - 0.005), (x, y + d * 0.55)]
    s.face([(px, py, h) for px, py in pts_top], "#C98F4E", outline="#9C6832", double=True)
    for i in range(len(pts_top)):
        a = pts_top[i]
        b = pts_top[(i + 1) % len(pts_top)]
        s.face([(a[0], a[1], h), (b[0], b[1], h), (b[0], b[1], 0.0), (a[0], a[1], 0.0)],
               "#B87C3D", outline=None)


def rolling_pin(s: Scene, cx, cy, length=0.26, z=0.024, color="#D9B381"):
    s.tube((cx - length / 2, cy, z), (cx + length / 2, cy, z), 0.021, color, n=24)
    s.tube((cx - length / 2 - 0.035, cy, z), (cx - length / 2, cy, z), 0.011, "#B98E5A", n=16)
    s.tube((cx + length / 2, cy, z), (cx + length / 2 + 0.035, cy, z), 0.011, "#B98E5A", n=16)


def carrot(s: Scene, x, y, angle=0.0, length=0.16, color="#D9782E"):
    dx, dy = math.cos(angle), math.sin(angle)
    s.tube((x, y, 0.018), (x + dx * length, y + dy * length, 0.018), 0.014, color, n=14)
    for k in (-1, 0, 1):
        s.tube((x + dx * length, y + dy * length, 0.018),
               (x + dx * (length + 0.030), y + dy * (length + 0.030) + k * 0.010, 0.030),
               0.004, "#5C7D3A", n=8)


# --------------------------------------------------------------------------- #
# Tools / end effector
# --------------------------------------------------------------------------- #
def arm_from(s: Scene, start, wrist, thick=0.024, wrist_color="#596966", elbow=None):
    s.set_layer(1)
    if elbow is None:
        s.tube(start, wrist, thick, "#3A4744", n=30)
    else:
        s.tube(start, elbow, thick, "#3A4744", n=30)
        s.sphere(elbow[0], elbow[1], elbow[2], 0.040, wrist_color)
        s.tube(elbow, wrist, thick * 0.88, "#43514E", n=30)
    s.sphere(wrist[0], wrist[1], wrist[2], 0.034, wrist_color)
    s.set_layer(0)


def jaws(s: Scene, cx, cy, z_top, z_tool, jaw_color="#5B6B68"):
    s.set_layer(1)
    s.box(cx - 0.030, cy - 0.014, z_tool, cx - 0.016, cy + 0.014, z_top, jaw_color,
          outline="#1F2927")
    s.box(cx + 0.016, cy - 0.014, z_tool, cx + 0.030, cy + 0.014, z_top, jaw_color,
          outline="#1F2927")
    s.set_layer(0)


def gripper(s: Scene, cx, cy, z_top, z_tool, tool_color="#E9C84B", jaw_color="#5B6B68"):
    jaws(s, cx, cy, z_top, z_tool, jaw_color)
    s.set_layer(1)
    s.box(cx - 0.027, cy - 0.012, z_tool - 0.034, cx + 0.027, cy + 0.012, z_tool - 0.004,
          tool_color, outline="#A98B1F")
    s.set_layer(0)
    s.set_layer(0)


def sponge(s: Scene, cx, cy, z, w=0.075, d=0.048, h=0.030, color="#E9C84B"):
    s.box(cx - w / 2, cy - d / 2, z, cx + w / 2, cy + d / 2, z + h, color, outline="#A98B1F")


def brush(s: Scene, cx, cy, z, angle=0.0):
    # handle along +x, bristles at the far end
    dx, dy = math.cos(angle), math.sin(angle)
    s.tube((cx - dx * 0.08, cy - dy * 0.08, z + 0.035),
           (cx + dx * 0.04, cy + dy * 0.04, z + 0.035), 0.012, "#8A6A45", n=14)
    s.box(cx + dx * 0.04 - 0.035, cy + dy * 0.04 - 0.022, z,
          cx + dx * 0.04 + 0.035, cy + dy * 0.04 + 0.022, z + 0.030, "#3A4240",
          outline="#202625")


def scraper(s: Scene, cx, cy, z, angle=0.0):
    dx, dy = math.cos(angle), math.sin(angle)
    s.box(cx - dx * 0.07 - 0.018, cy - dy * 0.07 - 0.012, z + 0.010,
          cx + dx * 0.02 + 0.018, cy + dy * 0.02 + 0.012, z + 0.030, "#C46A2E",
          outline="#8F4A1D")
    s.box(cx + dx * 0.02 - 0.030, cy + dy * 0.02 - 0.008, z,
          cx + dx * 0.10 + 0.030, cy + dy * 0.10 + 0.008, z + 0.012, "#9AA5A3",
          outline="#6F7A78")


