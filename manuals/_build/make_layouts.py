"""Compose the three layout sheets."""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from draw import *  # noqa
from plans import (
    W, H, Plan, Sec, sheet_header, panel, notes_panel, footer, FLOOR, COUNTER_FILL,
    COUNTER_LINE, APPLIANCE, KEEPOUT,
)

OUT = HERE.parent / "assets"
OUT.mkdir(parents=True, exist_ok=True)


# --------------------------------------------------------------------------- #
# Layout A
# --------------------------------------------------------------------------- #
def layout_a():
    acc = "a"
    c = Canvas(W, H, PAPER)
    sheet_header(c, "LAYOUT A  ·  GALLEY KITCHEN", "End-of-Run Sink Station",
                 "A fixed base at the wet end of a single counter run. The cooktop stays outside the envelope.",
                 acc, 1)

    panel(c, (60, 205, 1480, 1240), "PLAN  ·  SCALE SHOWN", acc)
    p = Plan(c, 170, 250, 0.39, ACCENTS[acc])

    # shell
    p.room(0, 0, 3000, 2400)
    p.wall(0, 0, 3000, 0)
    p.wall(0, 0, 0, 2400)
    p.wall(3000, 0, 3000, 2400)
    p.wall(0, 2400, 2200, 2400)
    p.wall(2900, 2400, 3000, 2400)
    cx, cy = p.p(2200, 2400)
    c.arc([cx - 700 * p.s, cy - 700 * p.s, cx, cy], 180, 270, fill="#B8AE9A", width=1)

    # counters, then reach tint, then symbols on top
    p.counter(0, 0, 2400, 600)
    p.counter(0, 1700, 2400, 600)
    p.reach(1550, 350, 600, 755, clip=(0, 0, 3000, 2400))
    p.fridge(2500, 0, 500, 700)
    p.sink(700, 100, 500, 400)
    p.board(1750, 90, 400, 300)
    p.grinder(1880, 500)
    p.knife(2260, 150, 120, 200)
    p.hob(900, 1800, 600, 500)
    p.hood_camera(1200, 1865, fov=(750, 1650, 1650, 2300), label_at=(1260, 1600),
                  clip=(0, 0, 3000, 2400))
    p.line([(1200, 1865), (1200, 2400)], fill="#8C8A80", width=0.9, dash=True)
    p.hood(900, 1900, 600, 500, label="HOOD")

    p.cabinet(0, 0, 700, 350)
    p.cabinet(2200, 0, 200, 350)
    c.text(p.p(350, 168)[0], p.p(350, 168)[1], "UPPER CABINETS", size=10, color=INK_SOFT,
           family="mono", anchor="mm", tracking=0.6)
    c.text(p.p(350, 168)[0], p.p(350, 168)[1] + 14, "END 850 mm FROM BASE", size=9,
           color=MUTED, family="mono", anchor="mm", tracking=0.4)

    # arm, dock, cable
    p.arm(1550, 350, dead_center=-90)
    p.dock(1250, 510, 160, 75)
    p.line([(1550, 350), (1550, 15)], fill="#8C8A80", width=1.1, dash=True)
    c.text(p.p(1600, 55)[0], p.p(1600, 55)[1], "48V / CAN", size=9, color=MUTED, family="mono")

    # dimensions
    p.dim_v(0, 600, 2460, "600")
    p.dim_v(600, 1700, 2460, "1100")
    p.dim_h(700, 1200, 560, "500")
    p.label((950, 300), (520, 690), "SINK 500 × 400  ·  600 mm TO CENTRE", size=11)
    p.label((1950, 240), (1500, 720), "PREP BOARD  ·  400 mm", size=11)
    p.label((1550, 350), (1500, 1000), "B601-RS BASE  ·  180 mm", size=11)
    p.label((1880, 500), (1700, 1080), "GRINDER  ·  DO NOT MOVE", size=11)
    p.label((2260, 150), (2050, 760), "KNIFE BLOCK  ·  HUMAN ONLY", size=11)
    p.label((1200, 2050), (200, 1500), "COOKTOP  ·  1.45 m AWAY, OUTSIDE ENVELOPE", size=11)
    p.label((1550, 140), (1650, 150), "SERVICE WEDGE 40°", size=10)

    p.section_mark(1250, 350, 1850, 350, "A")
    p.scalebar(180, 1210, 1000, "1 m")
    p.north(2800, 2150)

    # ------------------------------------------------------------------ #
    # section
    # ------------------------------------------------------------------ #
    panel(c, (1510, 205, 2140, 900), "SECTION A–A  ·  SCHEMATIC", acc)
    s = Sec(c, 1540, 860, 0.25, ACCENTS[acc])
    s.line([(0, 0), (2400, 0)], fill=INK, width=2)
    s.rect(0, 0, 2400, 860, fill=None, outline="#C9BEA9", width=1)
    s.line([(0, 100), (2400, 100)], fill="#D5CBBB", width=0.8)
    for x in (20, 620, 1220, 1820):
        s.line([(x, 100), (x, 860)], fill="#D5CBBB", width=0.8)
    # sink basin and faucet
    s.rect(700, 700, 500, 160, fill="#B6C1BE", outline="#8E9997")
    s.line([(700, 860), (700, 900)], fill=INK, width=1)
    s.line([(1200, 860), (1200, 900)], fill=INK, width=1)
    s.rect(0, 860, 700, 40, fill=COUNTER_FILL, outline=COUNTER_LINE)
    s.rect(1200, 860, 1200, 40, fill=COUNTER_FILL, outline=COUNTER_LINE)
    s.line([(950, 900), (950, 1160)], fill=STEEL_DARK, width=3)
    s.line([(950, 1160), (1030, 1160), (1030, 1080)], fill=STEEL_DARK, width=3)
    c.text(s.p(950, 1180)[0], s.p(950, 1180)[1], "FAUCET", size=8, color=MUTED, family="mono")
    # upper cabinet
    s.rect(0, 1350, 700, 900, fill="#EFE7D8", outline="#A79C88", width=1)
    s.line([(350, 1350), (350, 2250)], fill="#D5CBBB", width=0.8)
    s.line([(175, 1430), (175, 1500)], fill="#A79C88", width=2)
    s.line([(525, 1430), (525, 1500)], fill="#A79C88", width=2)
    # PSU and base labels
    s.rect(800, 140, 220, 190, fill="#E4DCCB", outline="#A79C88", width=1)
    c.text(s.p(910, 350)[0], s.p(910, 350)[1], "48V PSU", size=8, color=INK_SOFT,
           family="mono", anchor="ma")
    c.text(s.p(350, 1270)[0], s.p(350, 1270)[1], "UPPER CABINET", size=8, color=MUTED,
           family="mono", anchor="ma")
    c.text(s.p(1550, 840)[0], s.p(1550, 840)[1], "BASE", size=8, color=MUTED,
           family="mono", anchor="ma")
    # 600 mm working limit
    s.line([(2150, 0), (2150, 900)], fill=ACCENTS[acc]["main"], width=1, dash=True)
    c.text(s.p(2150, 930)[0], s.p(2150, 930)[1], "600", size=8, color=ACCENTS[acc]["main"],
           family="mono", anchor="ma")

    s.reach(1550, 900, 600, 755)
    s.arm([(1550, 900), (1550, 990), (1720, 1230), (1420, 1170), (1200, 940)],
          base_x=1550, base_z=900)
    # tool / sponge at the sink rim
    p0 = s.p(1200, 940)
    c.rect([p0[0] - 6, p0[1] + 2, p0[0] + 14, p0[1] + 10], fill="#E9C84B", outline="#A98B1F")

    s.dim_h(700, 1550, 1290, "850 clear")
    s.dim_v(1320, 900, 1350, "450", side="right")
    s.dim_v(2050, 0, 900, "900", side="left")
    c.text(1560, 238, "CEILING 2400", size=9, color=MUTED, family="mono")
    c.text(1770, 330, "no cabinet inside", size=9, color=ACCENTS[acc]["deep"], family="mono")
    c.text(1770, 344, "700 mm of base", size=9, color=ACCENTS[acc]["deep"], family="mono")
    c.text(1540, 890, "FLOOR", size=9, color=MUTED, family="mono")
    c.text(s.p(1200, 700)[0] + 8, s.p(1200, 700)[1] - 4, "SINK", size=8, color=MUTED,
           family="mono")

    notes_panel(c, (1510, 925, 2140, 1430), "DEPLOYMENT NOTES", [
        ("Base position", "350 mm to the sink rim (260 mm clear of the foot); 600 mm to the centre."),
        ("Clear height", "Upper run stops 850 mm from the base; 1500 mm to ceiling (≥700 required)."),
        ("Dead wedge", "40° sector faces the wall. Service access is from the front."),
        ("Cooktop", "Opposite run, 1.45 m away — outside the 755 mm reach."),
        ("Services", "48V PSU in the sink cabinet; cable via a grommet; GFCI."),
        ("Human zone", "Board beyond x = 2150 is human-only; the envelope ends at 2150."),
    ], acc)

    footer(c, "Layout A  ·  plan to scale, section schematic  ·  all dimensions in mm", acc)
    c.save(str(OUT / "layout-a-plan.png"))
    print("wrote layout-a-plan.png")


# --------------------------------------------------------------------------- #
# Layout B
# --------------------------------------------------------------------------- #
def layout_b():
    acc = "b"
    c = Canvas(W, H, PAPER)
    sheet_header(c, "LAYOUT B  ·  L-SHAPED KITCHEN", "Corner Bay",
                 "The dead corner becomes the robot bay. One base covers the sink leg and the near hob burners.",
                 acc, 2)

    panel(c, (60, 205, 1480, 1240), "PLAN  ·  SCALE SHOWN", acc)
    p = Plan(c, 150, 240, 0.36, ACCENTS[acc])

    p.room(0, 0, 3200, 2600)
    p.wall(0, 0, 3200, 0)
    p.wall(0, 0, 0, 2600)
    p.wall(3200, 0, 3200, 2600)
    p.wall(0, 2600, 2400, 2600)
    p.wall(3100, 2600, 3200, 2600)

    p.counter(0, 0, 2400, 600)
    p.counter(0, 0, 600, 2400)
    p.reach(430, 430, 600, 755, clip=(0, 0, 3200, 2600))
    p.fridge(2600, 0, 600, 700)
    p.sink(650, 100, 600, 400)
    p.hob(100, 750, 400, 500)
    p.hood_camera(640, 1000, fov=(0, 400, 1000, 1500), label_at=(1150, 1300),
                  clip=(0, 0, 3200, 2600))
    p.line([(640, 1000), (0, 1000)], fill="#8C8A80", width=0.9, dash=True)
    p.hood(0, 850, 600, 500, label="HOOD")
    p.circle(560, 120, 55, fill="#EDE8DC", outline="#8E8576", width=1)
    c.text(p.p(560, 120)[0], p.p(560, 120)[1], "SPICE", size=8, color=INK_SOFT,
           family="mono", anchor="mm")
    p.circle(600, 560, 55, fill="#3C4645", outline="#8E9997", width=1)
    p.circle(600, 560, 40, fill="#2A3332", outline=None)
    c.text(p.p(600, 560)[0], p.p(600, 560)[1] + 3, "BIN", size=8, color="#D4DBD9",
           family="mono", anchor="mm")
    p.dock(820, 525, 160, 75)

    p.cabinet(1300, 0, 1100, 350)
    p.cabinet(0, 1300, 350, 1100)
    c.text(p.p(1850, 175)[0], p.p(1850, 175)[1], "UPPER CABINETS", size=10,
           color=INK_SOFT, family="mono", anchor="mm", tracking=0.6)
    c.text(p.p(175, 1850)[0], p.p(175, 1850)[1], "UPPER", size=10, color=INK_SOFT,
           family="mono", anchor="mm")

    p.arm(430, 430, dead_center=-135)
    p.line([(430, 430), (20, 20)], fill="#8C8A80", width=1.1, dash=True)
    p.line([(0, 650), (600, 650), (600, 1350), (0, 1350)], fill=KEEPOUT, width=1.2, dash=True)

    p.dim_v(0, 600, 2550, "600")
    p.dim_h(0, 600, 2500, "600")
    p.dim_v(750, 1250, 700, "500")
    p.label((950, 300), (700, 800), "SINK  ·  536 mm TO CENTRE", size=11)
    p.label((430, 430), (860, 980), "B601-RS BASE  ·  DEAD CORNER", size=11)
    p.label((300, 1000), (600, 1400), "HOB  ·  NEAR BURNERS 462–509 mm", size=11)
    p.label((560, 120), (430, 240), "SPICE RACK", size=10)
    p.label((600, 560), (720, 600), "BIN Ø110", size=10)
    p.label((900, 560), (1000, 680), "TOOL DOCK", size=10)
    p.label((1850, 175), (1500, 820), "UPPER RUN STARTS 870 mm FROM BASE", size=10)

    p.section_mark(430, 650, 430, 1350, "B")
    p.scalebar(170, 1195, 1000, "1 m")
    p.north(2950, 2350)

    # section along the hob leg
    panel(c, (1510, 205, 2140, 900), "SECTION B–B  ·  SCHEMATIC", acc)
    s = Sec(c, 1540, 860, 0.25, ACCENTS[acc])
    s.line([(0, 0), (2400, 0)], fill=INK, width=2)
    s.rect(0, 0, 2400, 860, fill=None, outline="#C9BEA9", width=1)
    s.line([(0, 100), (2400, 100)], fill="#D5CBBB", width=0.8)
    s.rect(0, 860, 2400, 40, fill=COUNTER_FILL, outline=COUNTER_LINE)
    # hob slab and pan
    s.rect(750, 900, 500, 30, fill="#2B3231", outline="#171D1C")
    s.rect(860, 930, 220, 40, fill="#3E4745", outline="#1F2625")
    s.line([(1080, 950), (1180, 950)], fill="#1F2625", width=5)
    # upper cabinet
    s.rect(1300, 1350, 1100, 900, fill="#EFE7D8", outline="#A79C88", width=1)
    s.line([(1850, 1350), (1850, 2250)], fill="#D5CBBB", width=0.8)

    s.reach(430, 900, 600, 755)
    s.arm([(430, 900), (430, 990), (620, 1230), (900, 1140), (820, 950)],
          base_x=430, base_z=900)
    # heat exclusion line
    s.line([(750, 0), (750, 2400)], fill=KEEPOUT, width=1.2, dash=True)
    c.text(s.p(750, 2320)[0] + 6, s.p(750, 2320)[1], "COLD CLEAN ONLY", size=9,
           color=KEEPOUT, family="mono")

    s.dim_v(1320, 900, 1350, "450", side="right")
    s.dim_v(2100, 0, 900, "900", side="left")
    c.text(1560, 238, "CEILING 2400", size=9, color=MUTED, family="mono")
    c.text(s.p(1650, 700)[0], s.p(1650, 700)[1], "SINK ON OTHER LEG", size=8, color=MUTED,
           family="mono", anchor="ma")
    c.text(1540, 890, "FLOOR", size=9, color=MUTED, family="mono")

    notes_panel(c, (1510, 925, 2140, 1430), "DEPLOYMENT NOTES", [
        ("Corner bay", "Base sits in the 600 × 600 dead corner; 220 mm from the sink rim."),
        ("Dual coverage", "Sink centre 536 mm; near burners 462–509 mm."),
        ("Cabinet line", "Upper runs start 870 mm from the base; 1500 mm to ceiling."),
        ("Heat interlock", "Hob surface < 40 °C, gas off, burner lockout before entry."),
        ("Waste + tools", "Compost chute 214 mm; tool dock 401 mm (inside the 450 mm rule)."),
        ("Human only", "Far burners and the upper cabinets stay outside the envelope."),
    ], acc)

    footer(c, "Layout B  ·  plan to scale, section schematic  ·  all dimensions in mm", acc)
    c.save(str(OUT / "layout-b-plan.png"))
    print("wrote layout-b-plan.png")


# --------------------------------------------------------------------------- #
# Layout C
# --------------------------------------------------------------------------- #
def layout_c():
    acc = "c"
    c = Canvas(W, H, PAPER)
    sheet_header(c, "LAYOUT C  ·  OPEN-PLAN ISLAND", "Overhead Island Boom",
                 "A ceiling-mounted base looks straight down. The worktop stays clear; the cooking wall stays human.",
                 acc, 3)

    panel(c, (60, 205, 1480, 1240), "PLAN  ·  SCALE SHOWN", acc)
    p = Plan(c, 140, 240, 0.27, ACCENTS[acc])

    p.room(0, 0, 4200, 3600)
    p.wall(0, 0, 4200, 0)
    p.wall(0, 0, 0, 3600)
    p.wall(4200, 0, 4200, 3600)
    p.wall(0, 3600, 4200, 3600)

    p.counter(0, 0, 2400, 600)
    p.rect(1000, 1700, 2200, 1000, fill="#E7DFD0", outline=COUNTER_LINE, width=1.3, radius=6)
    p.reach(2100, 2200, 600, 755, clip=(1000, 1700, 3200, 2700))
    p.fridge(2800, 0, 600, 700)
    p.sink(1600, 100, 500, 400)
    p.hob(400, 100, 500, 400)
    p.hood_camera(650, 640, fov=(250, -50, 1050, 650), label_at=(1050, 950),
                  clip=(0, 0, 4200, 3600))
    p.line([(650, 640), (650, 0)], fill="#8C8A80", width=0.9, dash=True)
    p.hood(400, 0, 500, 600, label="HOOD")
    p.sink(1400, 1850, 350, 300)
    p.board(2200, 1900, 400, 300)
    p.circle(2600, 2450, 90, fill="#3C4645", outline="#8E9997", width=1)
    p.circle(2600, 2450, 65, fill="#2A3332", outline=None)
    p.dock(1800, 1700, 180, 70)
    p.mount(2100, 2200)
    for x in (1350, 1850, 2350, 2850):
        p.stool(x, 3250, r=170)

    p.dim_v(0, 600, 2550, "600")
    p.dim_v(600, 1700, 3400, "1100")
    p.dim_h(1000, 3200, 2820, "2200")
    p.dim_v(1700, 2700, 3350, "1000")
    p.label((650, 300), (220, 900), "COOKTOP  ·  2.2 m AWAY, OUTSIDE ENVELOPE", size=11)
    p.label((1850, 300), (1450, 800), "BACK-WALL COOKING ZONE  ·  HUMAN ONLY", size=11)
    p.label((1575, 2000), (1050, 2020), "PREP SINK  ·  562 mm", size=11)
    p.label((2400, 2050), (2700, 1900), "BOARD  ·  335 mm", size=11)
    p.label((2100, 2200), (1600, 1520), "CEILING MOUNT  ·  1450 AFF", size=11)
    p.label((2600, 2450), (2850, 2600), "COMPOST", size=10)
    p.label((1850, 3250), (1900, 3420), "STOOLS  ·  HUMAN ZONE", size=10)

    p.section_mark(1750, 2200, 2450, 2200, "C")
    p.scalebar(150, 1200, 1000, "1 m")
    p.north(4000, 3400)

    # section
    panel(c, (1510, 205, 2140, 900), "SECTION C–C  ·  SCHEMATIC", acc)
    s = Sec(c, 1530, 860, 0.23, ACCENTS[acc])
    s.line([(900, 0), (3000, 0)], fill=INK, width=2)
    s.rect(1000, 0, 2000, 860, fill=None, outline="#C9BEA9", width=1)
    s.line([(1000, 100), (3000, 100)], fill="#D5CBBB", width=0.8)
    s.rect(1000, 860, 2000, 40, fill=COUNTER_FILL, outline=COUNTER_LINE)
    # soffit and mount
    s.rect(1950, 1450, 300, 950, fill="#E4DACA", outline="#A79C88", width=1)
    s.line([(1950, 1450), (2250, 1450)], fill=ACCENTS[acc]["main"], width=2)

    s.reach_below(2100, 1450, 600, 755)
    s.arm([(2100, 1450), (2100, 1360), (2250, 1160), (2050, 1000), (2050, 900)],
          base_x=2100, base_z=1450)
    p0 = s.p(2050, 900)
    c.rect([p0[0] - 8, p0[1] - 4, p0[0] + 14, p0[1] + 3], fill="#E9C84B", outline="#A98B1F")

    s.dim_v(2300, 900, 1450, "550", side="right")
    s.dim_v(1600, 0, 900, "900", side="left")
    s.dim_v(2700, 0, 2400, "2400", side="left")
    c.text(1560, 238, "CEILING 2400", size=9, color=MUTED, family="mono")
    c.text(s.p(2100, 2500)[0], s.p(2100, 2500)[1], "SOFFIT / JOIST", size=8, color=MUTED,
           family="mono", anchor="ma")
    c.text(s.p(2100, 820)[0], s.p(2100, 820)[1], "600 WORK", size=8, color=ACCENTS[acc]["main"],
           family="mono", anchor="ma")
    c.text(1540, 890, "FLOOR", size=9, color=MUTED, family="mono")

    notes_panel(c, (1510, 925, 2140, 1430), "DEPLOYMENT NOTES", [
        ("Overhead mount", "300 × 300 plate into ceiling joists; 550 mm above the worktop."),
        ("Coverage", "600 mm disc clipped to the island; prep sink 562 mm, board 335 mm."),
        ("Payload", "Inverted operation: derate to 1.0 kg; gravity feedforward required."),
        ("Cooking wall", "Hob 2.2 m away — outside the 755 mm reach; human-only."),
        ("Services", "48V PSU in the island cabinet; cable through the soffit, drip loop."),
        ("Human zone", "Island ends, stools and the cooktop remain human-only."),
    ], acc)

    footer(c, "Layout C  ·  plan to scale, section schematic  ·  all dimensions in mm", acc)
    c.save(str(OUT / "layout-c-plan.png"))
    print("wrote layout-c-plan.png")


if __name__ == "__main__":
    layout_a()
    layout_b()
    layout_c()

















