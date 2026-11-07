"""Consistent card layout for the 15 mess illustrations."""

from __future__ import annotations

from pathlib import Path

from draw import Canvas, ACCENTS, INK, INK_SOFT, MUTED, PAPER, rgba
from scene3d import Camera, Scene

CARD_W, CARD_H = 1680, 1120
PANEL = (48, 182, 1632, 912)


def mess_card(path, number, title, subtitle, layout_label, accent_key, build, overlay,
              tool, motion, limit, note=None, tags=None, cam=None):
    acc = ACCENTS[accent_key]
    c = Canvas(CARD_W, CARD_H, PAPER)
    c.rect([0, 0, CARD_W - 1, CARD_H - 1], outline="#DED5C4", width=1)

    # scene panel, clipped
    PW, PH = PANEL[2] - PANEL[0], PANEL[3] - PANEL[1]
    sc = Canvas(PW, PH, "#FBF8F1")
    sc.rect([0, 0, PW - 1, PH - 1], outline="#E4DCCB", width=1, radius=14)
    if cam is None:
        cam = Camera((0.30, -0.54, 0.47), (0.30, 0.14, 0.02), PW, PH, vfov=38)
    scene = Scene(cam)
    build(scene, cam)
    scene.render(sc)
    overlay(sc, cam)
    c.img.paste(sc.img, (PANEL[0] * c.k, PANEL[1] * c.k))

    # header
    c.text(64, 42, number, 20, acc["main"], family="mono", weight="b", tracking=1.4)
    c.text(64, 72, title, 46, INK, family="head", weight="b")
    c.text(64, 132, subtitle, 15, MUTED, family="mono", tracking=1.2)
    c.text(1616, 46, layout_label, 13, acc["main"], family="mono", weight="b",
           tracking=1.1, anchor="ra")
    if tags:
        x = 1616
        for tag in reversed(tags):
            w, _h = c.chip(x - c.tw(tag, 12, "mono", "b") - 20, 74, tag, acc["deep"],
                           acc["light"], size=12)
            x -= w + 8

    # footer
    c.line([(64, 944), (1616, 944)], fill="#DED5C4", width=1)
    cols = [(("TOOL", tool)), (("MOTION", motion)), (("LIMIT", limit))]
    for i, (label, body) in enumerate(cols):
        x = 64 + i * 520
        c.text(x, 968, label, 12, acc["main"], family="mono", weight="b", tracking=1.4)
        lines = body if isinstance(body, (list, tuple)) else [body]
        for j, line in enumerate(lines[:2]):
            c.text(x, 992 + j * 30, line, 19 if j == 0 else 17,
                   INK if j == 0 else INK_SOFT)

    if note:
        c.text(64, 1082, note, 14, MUTED, family="mono")
    c.save(str(path))
    return path


# --------------------------------------------------------------------------- #
# 2D mess helpers, projected onto a surface plane
# --------------------------------------------------------------------------- #
def project_xy(cam, x, y, z=0.0):
    return cam.project((x, y, z))[:2]


def blob_pts(cx, cy, rx, ry, seed=0, n=16, rough=0.18):
    import math
    import random
    rnd = random.Random(seed)
    return [(cx + rx * (1 + rough * (rnd.random() - 0.5)) * math.cos(2 * math.pi * i / n),
             cy + ry * (1 + rough * (rnd.random() - 0.5)) * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def draw_blob(c, cam, pts, fill, outline=None, z=0.0, width=0.8):
    c.poly([project_xy(cam, x, y, z) for x, y in pts], fill=fill, outline=outline, width=width)


def particles(c, cam, n, cx, cy, sx, sy, seed, color, rmin=0.7, rmax=2.2, z=0.0,
              over="#E9E2D4", alpha=0.95, exclude=None):
    import random
    rnd = random.Random(seed)
    for _ in range(n):
        x = rnd.gauss(cx, sx)
        y = rnd.gauss(cy, sy)
        if exclude and exclude(x, y):
            continue
        p = project_xy(cam, x, y, z)
        r = rnd.uniform(rmin, rmax)
        c.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=rgba(color, alpha, over))


def label_wall(c, cam, tip_xyz, at, text, accent=INK, size=15, align="left"):
    tip = cam.project(tip_xyz)[:2]
    c.line([tip, at], fill="#8C8A80", width=0.8)
    c.ellipse([tip[0] - 2.5, tip[1] - 2.5, tip[0] + 2.5, tip[1] + 2.5], fill=accent)
    if align == "left":
        c.text(at[0] + 8, at[1] - 7, text, size=size, color=INK, family="mono")
    else:
        c.text(at[0] - 8, at[1] - 7, text, size=size, color=INK, family="mono", anchor="ra")


def scale_bar(c, cam, x=0.015, y=0.375, length=0.05, label="50 mm"):
    a = cam.project((x, y, 0.002))
    b = cam.project((x + length, y, 0.002))
    c.line([(a[0], a[1]), (b[0], b[1])], fill="#6B7A78", width=2.5)
    for p in (a, b):
        c.line([(p[0], p[1] - 4), (p[0], p[1] + 4)], fill="#6B7A78", width=1.2)
    c.text((a[0] + b[0]) / 2, a[1] + 8, label, size=11, color="#6B7A78",
           family="mono", anchor="ma")


