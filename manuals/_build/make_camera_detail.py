"""Shared detail drawing for the optional hood-edge camera module."""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from draw import *  # noqa
from plans import panel, COUNTER_FILL, COUNTER_LINE

W, H = 1500, 1000
ACC = ACCENTS["a"]
OUT = HERE.parent / "assets" / "hood-camera-detail.png"


def main():
    c = Canvas(W, H, PAPER)
    c.rect([0, 0, W, 7], fill=ACC["main"])
    c.text(50, 38, "OPTIONAL PERCEPTION MODULE", 13, ACC["main"], family="mono",
           weight="b", tracking=1.4)
    c.text(50, 66, "Hood-Edge Camera", 44, INK, family="head", weight="b")
    c.text(50, 118, "A top-down view of the hob from the cooker-hood edge. Clamp-on, no worktop footprint.",
           15, INK_SOFT)
    c.text(W - 50, 44, "B601-RS  ·  KITCHEN DEPLOYMENT", 11, ACC["main"], family="mono",
           anchor="ra", tracking=1.0)
    c.text(W - 50, 66, "rev A  ·  dimensions in mm", 11, MUTED, family="mono", anchor="ra")
    c.line([(50, 150), (W - 50, 150)], fill="#D8CEBD", width=1)

    # ---------------- elevation panel ---------------- #
    panel(c, (40, 170, 900, 760), "EDGE MOUNT  ·  SECTION", "a")
    wall_x, floor_y = 120, 720
    counter_y = floor_y - 900 * 0.30
    hood_bottom = floor_y - 1500 * 0.30
    hood_top = floor_y - 1650 * 0.30
    front_x = 120 + 600 * 0.70
    back_x = 120

    # room / wall / floor
    c.rect([40, 170, 900, 760], fill=None, outline=None)
    c.line([(wall_x, 195), (wall_x, floor_y)], fill=INK, width=2)
    for y in range(210, floor_y, 26):
        c.line([(wall_x - 10, y), (wall_x, y + 10)], fill=INK, width=0.8)
    c.line([(60, floor_y), (880, floor_y)], fill=INK, width=2)
    c.text(60, floor_y + 8, "FLOOR 0", size=9, color=MUTED, family="mono")

    # counter
    c.rect([wall_x, counter_y, 880, floor_y], fill="#F2EBDD", outline="#C9BEA9", width=1)
    for x in (260, 420, 580, 740):
        c.line([(x, counter_y + 10), (x, floor_y)], fill="#DDD3C1", width=0.8)
    c.rect([wall_x, counter_y - 12, 760, counter_y + 2], fill=COUNTER_FILL, outline=COUNTER_LINE, width=1)
    c.line([(wall_x, counter_y - 12), (760, counter_y - 12)], fill="#FBF6EC", width=2)

    # hob
    c.rect([back_x, counter_y - 14, front_x, counter_y - 6], fill="#2B3231", outline="#171D1C", width=1)
    for x in (240, 390):
        c.ellipse([x - 30, counter_y - 40, x + 30, counter_y - 12], outline="#6C7774", width=1)
        c.line([(x - 30, counter_y - 26), (x + 30, counter_y - 26)], fill="#6C7774", width=0.8)
        c.line([(x, counter_y - 40), (x, counter_y - 12)], fill="#6C7774", width=0.8)

    # hood
    c.rect([back_x, hood_top, front_x, hood_bottom], fill="#E6DDCC", outline="#8E8576", width=1)
    c.line([(back_x, hood_top + 8), (front_x, hood_top + 8)], fill="#D5CBBB", width=0.8)
    c.text(back_x + 12, hood_top + 14, "COOKER HOOD", size=9, color=INK_SOFT, family="mono")
    c.text(front_x - 44, hood_top + 14, "EDGE", size=9, color=INK_SOFT, family="mono",
           anchor="ra")

    # camera body hanging on the edge
    body_w, body_h = 52, 16
    bx0, by0 = front_x - body_w / 2, hood_bottom - body_h
    c.rect([bx0, by0, bx0 + body_w, by0 + body_h], fill=ACC["deep"], outline=INK,
           width=0.8, radius=3)
    c.ellipse([front_x - 5, hood_bottom - 5, front_x + 5, hood_bottom + 5],
              fill="#DCE8E5", outline=INK)
    c.text(front_x + 30, 250, "CAM 62 × 48", size=9, color=ACC["deep"], family="mono")
    c.line([(front_x - 8, hood_bottom), (front_x + 8, hood_bottom)], fill=ACC["main"], width=2)

    # FOV cone
    c.dashed([(front_x, hood_bottom), (back_x, counter_y - 12)], dash=8, gap=6,
             fill=ACC["main"], width=1.1)
    c.dashed([(front_x, hood_bottom), (760, counter_y - 12)], dash=8, gap=6,
             fill=ACC["main"], width=1.1)
    c.arc([front_x - 190, counter_y - 36, front_x + 190, counter_y + 36], 180, 360,
          fill=ACC["main"], width=1.0)
    c.text(front_x, counter_y - 44, "1.66 × 1.25 m at worktop", size=9,
           color=ACC["deep"], family="mono", anchor="ma")

    # cable
    c.dashed([(front_x, by0), (front_x, 258), (back_x + 8, 258)],
             dash=5, gap=4, fill="#8C8A80", width=0.9)
    c.text(back_x + 12, 262, "USB-C / 5 V 2 A", size=8, color=MUTED, family="mono")

    # dimensions
    c.line([(790, hood_bottom), (790, counter_y - 12)], fill=MUTED, width=0.8)
    for y in (hood_bottom, counter_y - 12):
        c.line([(784, y), (796, y)], fill=MUTED, width=0.8)
    c.text(798, (hood_bottom + counter_y) / 2, "600", size=10, color=MUTED, family="mono")

    c.line([(820, counter_y - 12), (820, floor_y)], fill=MUTED, width=0.8)
    for y in (counter_y - 12, floor_y):
        c.line([(814, y), (826, y)], fill=MUTED, width=0.8)
    c.text(828, (counter_y + floor_y) / 2, "900", size=10, color=MUTED, family="mono")

    c.line([(86, floor_y), (86, hood_bottom)], fill=MUTED, width=0.8)
    for y in (floor_y, hood_bottom):
        c.line([(80, y), (92, y)], fill=MUTED, width=0.8)
    c.text(76, (floor_y + hood_bottom) / 2, "1500 AFF", size=10, color=MUTED,
           family="mono", anchor="rm")

    c.line([(back_x, 742), (front_x, 742)], fill=MUTED, width=0.8)
    for x in (back_x, front_x):
        c.line([(x, 736), (x, 748)], fill=MUTED, width=0.8)
    c.text((back_x + front_x) / 2, 750, "600 hood depth", size=9, color=MUTED,
           family="mono", anchor="ma")

    c.text(back_x + 12, counter_y + 34, "HOB", size=9, color=INK_SOFT, family="mono")
    c.text(740, counter_y - 34, "WORKTOP 900", size=9, color=INK_SOFT, family="mono",
           anchor="ra")

    # ---------------- spec panel ---------------- #
    panel(c, (930, 170, 1460, 760), "MODULE  ·  MOUNT  ·  LIMITS", "a")
    specs = [
        ("Sensor", "12 MP, 1/1.7\""),
        ("Field of view", "120° diagonal (≈108° H × 92° V)"),
        ("Frame rate", "30 fps, colour"),
        ("Interface", "USB 3.0 Type-C, UVC"),
        ("Rating", "IP65, anodised housing"),
        ("Size / mass", "62 × 62 × 48 mm / 190 g"),
        ("Power", "5 V / 2 A, 6 W"),
        ("Mount", "10–40 mm edge clamp, 1/4\"-20 ball head"),
        ("Lens height", "1500 AFF (600 mm above worktop)"),
        ("Footprint", "1.66 × 1.25 m at the worktop"),
        ("Resolution", "≈2.4 px/mm; 5 mm ≈ 12 px"),
    ]
    y = 210
    for k, v in specs:
        c.text(948, y, k, size=10, color=MUTED, family="mono")
        c.text(1080, y, v, size=11, color=INK, family="body")
        y += 28

    c.line([(946, 548), (1444, 548)], fill="#DED5C4", width=1)
    c.text(948, 562, "MOUNT", size=10, color=ACC["main"], family="mono", weight="b",
           tracking=1.2)
    steps = [
        "1. Clamp to the hood front edge, 100 mm from a corner.",
        "2. Set the lens axis vertical; check with a bubble level.",
        "3. Route USB-C along the hood to the wall; drip loop.",
        "4. Calibrate intrinsic + hand-eye with a checkerboard.",
    ]
    y = 584
    for s in steps:
        c.text(948, y, s, size=10, color=INK_SOFT, family="body")
        y += 24

    # ---------------- coverage strip ---------------- #
    panel(c, (40, 790, 1460, 960), "COVERAGE IN THE THREE LAYOUTS", "a")
    rows = [
        ("A", "Opposite-run hood edge (1200, 1865) — sees the hob and aisle approach; not the sink."),
        ("B", "Hob-leg hood edge (640, 1000) — sees the hob, corner, and sink edge."),
        ("C", "Back-wall hood edge (650, 640) — sees the hob and cooking wall; not the island."),
    ]
    y = 830
    for r in rows:
        c.text(60, y, r[0], size=11, color=ACC["main"], family="mono", weight="b")
        c.text(92, y, r[1], size=10, color=INK, family="body")
        y += 28
    c.text(60, 918, "Optional: may be omitted. The arm's wrist camera remains primary for contact tasks;",
           size=9, color=MUTED, family="mono")
    c.text(60, 934, "this module is a context/safety view, not a metrology camera.",
           size=9, color=MUTED, family="mono")

    c.save(str(OUT))
    print("wrote", OUT)


if __name__ == "__main__":
    main()




