"""Detail: verified 6-DOF fold-under pose below a 450 mm upper cabinet."""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from draw import *  # noqa
from plans import panel, COUNTER_FILL, COUNTER_LINE

W, H = 1400, 850
ACC = ACCENTS["a"]
OUT = HERE.parent / "assets" / "under-cabinet-reach.png"

# joint positions (x, z) in metres for the verified pose
PTS = [
    (0.000, 0.075),
    (-0.033, 0.145),
    (-0.053, 0.375),
    (-0.075, 0.140),
    (-0.029, 0.051),
    (-0.047, 0.084),
    (-0.178, 0.056),
]


def main():
    c = Canvas(W, H, PAPER)
    c.rect([0, 0, W, 7], fill=ACC["main"])
    c.text(50, 38, "REACH VERIFICATION  ·  6-DOF FOLD-UNDER", 13, ACC["main"],
           family="mono", weight="b", tracking=1.3)
    c.text(50, 66, "Working Below an Upper Cabinet", 38, INK, family="head", weight="b")
    c.text(50, 112, "A 450 mm underside is not a reach limit. The arm folds under it; the camera sits on the edge.",
           14, INK_SOFT)
    c.text(W - 50, 44, "B601-RS  ·  KITCHEN DEPLOYMENT", 10, ACC["main"], family="mono",
           anchor="ra", tracking=1.0)
    c.text(W - 50, 66, "rev A  ·  dimensions in mm", 10, MUTED, family="mono", anchor="ra")
    c.line([(50, 140), (W - 50, 140)], fill="#D8CEBD", width=1)

    panel(c, (40, 160, 860, 700), "SECTION  ·  TESTED POSE", "a")

    # geometry: x metres -> screen, z metres -> screen
    ox, oy, s = 300, 610, 560  # base at (0,0)
    def p(x, z):
        return (ox + x * s, oy - z * s)

    # wall / counter / cabinet
    wall_x = p(-0.35, 0)[0]
    front_x = p(0, 0)[0]
    counter_y = oy
    c.rect([60, counter_y, 820, counter_y + 14], fill=COUNTER_FILL, outline=COUNTER_LINE)
    c.line([(wall_x, 190), (wall_x, counter_y)], fill=INK, width=2)
    for y in range(200, counter_y, 24):
        c.line([(wall_x - 10, y), (wall_x, y + 10)], fill=INK, width=0.8)
    c.rect([wall_x, 210, front_x, p(0, 0.45)[1]],
           fill="#E6DDCC", outline="#8E8576", width=1)
    c.dashed([(wall_x, 210), (front_x, 210)], dash=8, gap=6, fill="#8E8576", width=0.9)
    c.text(wall_x + 10, 220, "UPPER CABINET", size=9, color=INK_SOFT,
           family="mono")
    c.text(front_x - 8, 220, "EDGE", size=9, color=INK_SOFT, family="mono",
           anchor="ra")

    # camera at the bottom edge
    c.rect([front_x - 14, p(0, 0.45)[1] - 4, front_x + 14, p(0, 0.45)[1] + 8],
           fill=ACC["deep"], outline=INK, width=0.7, radius=2)
    c.ellipse([front_x - 4, p(0, 0.45)[1] + 3, front_x + 4, p(0, 0.45)[1] + 11],
              fill="#DCE8E5", outline=INK)
    c.text(front_x + 20, p(0, 0.45)[1] - 18, "CABINET CAM", size=9,
           color=ACC["deep"], family="mono")

    # arm links
    sp = [p(x, z) for x, z in PTS]
    for a, b in zip(sp[:-1], sp[1:]):
        c.line([a, b], fill="#3A4744", width=9)
    for q in sp[:-1]:
        c.ellipse([q[0] - 6, q[1] - 6, q[0] + 6, q[1] + 6], fill="#596966",
                  outline="#1F2927")
    tool = sp[-1]
    c.rect([tool[0] - 10, tool[1] - 4, tool[0] + 10, tool[1] + 4],
           fill="#E9C84B", outline="#A98B1F")
    c.text(tool[0], tool[1] + 16, "TOOL", size=9, color=ACC["deep"], family="mono",
           anchor="ma")

    # FOV from camera to the tool area
    c.dashed([(front_x, p(0, 0.45)[1] + 8), (tool[0] - 20, tool[1])],
             dash=6, gap=5, fill=ACC["main"], width=0.9)
    c.dashed([(front_x, p(0, 0.45)[1] + 8), (tool[0] + 20, tool[1])],
             dash=6, gap=5, fill=ACC["main"], width=0.9)

    # dimensions
    # 450 underside
    c.line([(front_x + 50, p(0, 0.45)[1]), (front_x + 50, counter_y)], fill=MUTED, width=0.8)
    for y in (p(0, 0.45)[1], counter_y):
        c.line([(front_x + 44, y), (front_x + 56, y)], fill=MUTED, width=0.8)
    c.text(front_x + 58, (p(0, 0.45)[1] + counter_y) / 2, "450", size=10,
           color=MUTED, family="mono")

    # 350 depth
    c.line([(wall_x, counter_y + 32), (front_x, counter_y + 32)], fill=MUTED, width=0.8)
    for x in (wall_x, front_x):
        c.line([(x, counter_y + 26), (x, counter_y + 38)], fill=MUTED, width=0.8)
    c.text((wall_x + front_x) / 2, counter_y + 40, "350 deep", size=9, color=MUTED,
           family="mono", anchor="ma")

    # clearance between p2 (max link) and cabinet
    maxp = p(*PTS[2])
    c.line([(maxp[0], maxp[1]), (maxp[0], p(0, 0.45)[1])], fill=ACC["main"], width=0.8)
    c.text(maxp[0] - 10, (maxp[1] + p(0, 0.45)[1]) / 2, "75 clearance", size=9,
           color=ACC["deep"], family="mono")

    # 178 under cabinet
    c.line([(front_x, counter_y - 60), (tool[0], counter_y - 60)], fill=ACC["main"], width=0.8)
    for x in (front_x, tool[0]):
        c.line([(x, counter_y - 66), (x, counter_y - 54)], fill=ACC["main"], width=0.8)
    c.text((front_x + tool[0]) / 2, counter_y - 70, "178 under", size=9,
           color=ACC["deep"], family="mono", anchor="ma")

    c.text(60, counter_y + 50, "WORKTOP 900", size=9, color=MUTED, family="mono")

    # right panel
    panel(c, (890, 160, 1360, 700), "WHAT THE SWEEP SHOWS", "a")
    lines = [
        ("Tested pose", "tool 178 mm behind the cabinet edge"),
        ("Max link height", "375 mm above the worktop"),
        ("Cabinet underside", "450 mm above the worktop"),
        ("Clearance", "75 mm at the highest link"),
        ("IK sweep", "350k samples, 0–0.77 m tool radius"),
        ("Low-link poses", "32 at 0.70–0.80 m with links <430 mm"),
        ("Verdict", "700 mm clear-height rule is not required"),
    ]
    y = 205
    for k, v in lines:
        c.text(910, y, k, size=10, color=MUTED, family="mono")
        c.text(1040, y, v, size=11, color=INK, family="body")
        y += 42

    c.text(910, 540, "Use collision-checked fold-under poses;", size=11, color=ACC["deep"],
           family="body", weight="sb")
    c.text(910, 562, "keep the camera and cable 50 mm outside the", size=11,
           color=ACC["deep"], family="body")
    c.text(910, 584, "swept links, and do not assume a spherical", size=11,
           color=ACC["deep"], family="body")
    c.text(910, 606, "vertical envelope.", size=11, color=ACC["deep"], family="body")

    # bottom note
    c.line([(50, 730), (W - 50, 730)], fill="#D8CEBD", width=1)
    c.text(50, 750, "The 6-DOF chain can fold its links below a 450 mm cabinet while the tool reaches 178 mm behind the edge.",
           size=11, color=INK_SOFT, family="body")
    c.text(50, 774, "This is a static IK result; every production path still needs collision checking against the cabinet, camera, cable and counter.",
           size=10, color=MUTED, family="mono")

    c.save(str(OUT))
    print("wrote", OUT)


if __name__ == "__main__":
    main()


