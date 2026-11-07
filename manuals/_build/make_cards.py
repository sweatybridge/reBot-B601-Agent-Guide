"""Build the 15 mess cards and 3 layout sheets.

Run from the repo root:
    .rebot-venv\\Scripts\\python.exe manuals/_build/make_cards.py
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from draw import *  # noqa
from card import mess_card, blob_pts, draw_blob, label_wall, particles, project_xy, scale_bar
from scene3d import Camera, Scene
import props as P
import scenes_b as B
import scenes_c as C

OUT = HERE.parent / "assets"
OUT.mkdir(parents=True, exist_ok=True)

SURF = SURFACE_HI
W = WOOD_LIGHT
PANEL_W, PANEL_H = 1584, 730


def cam_a(eye=(0.30, -0.54, 0.47), target=(0.30, 0.14, 0.02), vfov=38):
    return Camera(eye, target, PANEL_W, PANEL_H, vfov=vfov)


def cam_wall(eye=(0.30, -0.36, 0.27), target=(0.30, 0.42, 0.16), vfov=37):
    return Camera(eye, target, PANEL_W, PANEL_H, vfov=vfov)


# --------------------------------------------------------------------------- #
# A-01  Flour & dough film
# --------------------------------------------------------------------------- #
def build_a01(s, cam):
    P.counter(s)
    P.board(s, 0.12, 0.08, 0.30, 0.21)
    P.rolling_pin(s, 0.55, 0.30, length=0.24)
    P.arm_from(s, (0.95, -0.12, 0.58), (0.305, 0.185, 0.155), elbow=(0.66, 0.015, 0.34))
    P.gripper(s, 0.305, 0.185, 0.145, 0.075)
    for (x, y, rx, ry) in [(0.20, 0.13, 0.028, 0.016), (0.28, 0.22, 0.019, 0.012),
                           (0.17, 0.21, 0.014, 0.010)]:
        pts = blob_pts(x, y, rx, ry, seed=int(x * 1000), n=10, rough=0.22)
        s.face([(px, py, 0.0235) for px, py in pts], "#EBD9B5", outline="#C8B084", double=True)


def overlay_a01(c, cam):
    board_area = lambda x, y: (0.12 < x < 0.42 and 0.08 < y < 0.29)
    particles(c, cam, 470, 0.27, 0.17, 0.085, 0.055, seed=9, color="#FFFDF6",
              rmin=0.6, rmax=1.9, z=0.024, over=WOOD)
    particles(c, cam, 240, 0.27, 0.17, 0.16, 0.10, seed=17, color="#FFFDF6",
              rmin=0.6, rmax=2.0, z=0.001, over=SURFACE_HI,
              exclude=board_area)
    label_wall(c, cam, (0.20, 0.13, 0.03), (80, 628), "DOUGH SCRAP  ·  GRASP → BIN")
    label_wall(c, cam, (0.34, 0.30, 0.0), (770, 676), "FLOUR FILM  ·  BRUSH, THEN WET WIPE")
    

# --------------------------------------------------------------------------- #
# A-02  Coffee grounds
# --------------------------------------------------------------------------- #
def build_a02(s, cam):
    P.counter(s)
    P.grinder(s, 0.26, 0.22)
    P.mug(s, 0.42, 0.27, r=0.038, h=0.055, color="#DED8CB")
    P.arm_from(s, (0.95, -0.12, 0.58), (0.36, 0.17, 0.14), elbow=(0.68, 0.00, 0.34))
    P.jaws(s, 0.36, 0.17, 0.13, 0.095)
    # vacuum crevice tool
    s.set_layer(1)
    s.tube((0.36, 0.17, 0.095), (0.355, 0.17, 0.020), 0.012, "#2E3533", n=16)
    s.box(0.336, 0.145, 0.006, 0.374, 0.195, 0.018, "#1F2625", outline=None)
    s.set_layer(0)


def overlay_a02(c, cam):
    particles(c, cam, 420, 0.27, 0.22, 0.085, 0.055, seed=21, color="#5A3A22",
              rmin=0.7, rmax=2.0, z=0.002, over=SURFACE_HI, alpha=0.9)
    particles(c, cam, 180, 0.41, 0.29, 0.06, 0.04, seed=31, color="#5A3A22",
              rmin=0.7, rmax=1.9, z=0.002, over=SURFACE_HI, alpha=0.85)
    # a few larger clumps
    for (x, y, r) in [(0.215, 0.245, 2.6), (0.315, 0.205, 2.2), (0.40, 0.315, 2.4)]:
        p = project_xy(cam, x, y, 0.002)
        c.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill="#4A2F1B")
    label_wall(c, cam, (0.27, 0.24, 0.0), (80, 620), "GROUNDS 0.5–2 mm  ·  VACUUM ONLY")
    label_wall(c, cam, (0.42, 0.28, 0.05), (770, 672), "GRINDER 240 mm  ·  DO NOT MOVE")
    

# --------------------------------------------------------------------------- #
# A-03  Eggshell fragments
# --------------------------------------------------------------------------- #
def build_a03(s, cam):
    P.counter(s)
    P.shell_bowl(s, 0.255, 0.215, r=0.040)
    for pts in [
        [(0.315, 0.170), (0.342, 0.158), (0.352, 0.190), (0.330, 0.205), (0.312, 0.192)],
        [(0.190, 0.285), (0.218, 0.272), (0.226, 0.296), (0.200, 0.308)],
        [(0.288, 0.305), (0.310, 0.295), (0.318, 0.318), (0.294, 0.324)],
    ]:
        P.shard(s, pts, z=0.008)
    P.arm_from(s, (0.95, -0.12, 0.52), (0.255, 0.215, 0.095), elbow=(0.60, 0.00, 0.28))
    P.jaws(s, 0.255, 0.215, 0.085, 0.032)


def overlay_a03(c, cam):
    draw_blob(c, cam, blob_pts(0.360, 0.275, 0.042, 0.026, seed=41, n=14, rough=0.35),
              rgba("#E8A21C", 0.55, SURFACE_HI), z=0.001)
    draw_blob(c, cam, blob_pts(0.365, 0.278, 0.028, 0.018, seed=42, n=12, rough=0.3),
              rgba("#F2B93C", 0.85, SURFACE_HI), z=0.0012)
    label_wall(c, cam, (0.255, 0.215, 0.04), (80, 610), "SHELL 6–14 mm  ·  3-POINT GRASP")
    label_wall(c, cam, (0.365, 0.278, 0.0), (770, 672), "YOLK FILM  ·  WET WIPE AFTER PICK")


# --------------------------------------------------------------------------- #
# A-04  Soapy puddle on the drainboard
# --------------------------------------------------------------------------- #
def build_a04(s, cam):
    P.counter(s)
    P.sink(s, 0.53, 0.01, 0.21, 0.35)
    P.faucet(s, 0.635, 0.355)
    P.bottle(s, 0.445, 0.285, r=0.028, h=0.115, color="#7FA9A2", cap="#3F6F68")
    P.sponge(s, 0.225, 0.265, 0.0, w=0.070, d=0.048, h=0.028, color="#7FB8AE")
    # drainboard grooves
    for i in range(6):
        y = 0.075 + i * 0.042
        s.box(0.30, y, 0.001, 0.515, y + 0.004, 0.0028, "#C9D0CD", outline=None)
    P.arm_from(s, (0.95, -0.12, 0.55), (0.31, 0.205, 0.155), elbow=(0.66, 0.02, 0.36))
    P.jaws(s, 0.31, 0.205, 0.145, 0.112)
    s.box(0.283, 0.188, 0.078, 0.337, 0.222, 0.112, "#EDE9DD", outline="#C9C2B2")


def overlay_a04(c, cam):
    # water ring, then foam
    draw_blob(c, cam, blob_pts(0.305, 0.215, 0.105, 0.070, seed=51, n=20, rough=0.22),
              rgba("#A9BEC0", 0.34, SURFACE_HI), z=0.003)
    draw_blob(c, cam, blob_pts(0.305, 0.215, 0.086, 0.056, seed=52, n=18, rough=0.25),
              rgba("#7FA3A6", 0.20, SURFACE_HI), z=0.0032)
    rnd = random.Random(6)
    for _ in range(64):
        ang = rnd.uniform(0, 2 * math.pi)
        rr = rnd.uniform(0, 1) ** 0.55
        x = 0.305 + 0.098 * rr * math.cos(ang)
        y = 0.215 + 0.064 * rr * math.sin(ang)
        p = project_xy(cam, x, y, 0.006)
        r = rnd.uniform(1.8, 6.2)
        c.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill="#FBF8F0",
                  outline="#CFDCDA")
        c.ellipse([p[0] - r * 0.42, p[1] - r * 0.42, p[0] + r * 0.10, p[1] + r * 0.10],
                  fill="#FFFFFF")
    label_wall(c, cam, (0.305, 0.215, 0.01), (80, 610), "FOAM + 80 mL  ·  BLOT, DO NOT WIPE")
    label_wall(c, cam, (0.53, 0.12, 0.01), (770, 672), "SINK CUTOUT  ·  KEEP 40 mm CLEAR")
    

# --------------------------------------------------------------------------- #
# A-05  Grease splatter on the backsplash
# --------------------------------------------------------------------------- #
def build_a05(s, cam):
    P.counter(s)
    P.arm_from(s, (-0.60, -0.16, 0.44), (0.305, 0.394, 0.175), elbow=(0.02, 0.10, 0.32))
    P.jaws(s, 0.305, 0.394, 0.165, 0.132)
    # squeegee blade held against the tile
    s.box(0.265, 0.398, 0.105, 0.345, 0.407, 0.132, "#9AA5A3", outline="#6F7A78")


def overlay_a05(c, cam):
    rnd = random.Random(77)
    for _ in range(85):
        x = rnd.uniform(0.14, 0.50)
        z = rnd.uniform(0.07, 0.31)
        p = cam.project((x, 0.416, z))
        r = rnd.uniform(1.8, 6.0)
        c.ellipse([p[0] - r, p[1] - r * 0.8, p[0] + r, p[1] + r * 0.8],
                  fill=rgba("#B5651D", 0.55, "#EFE9DD"))
        c.ellipse([p[0] - r * 0.30, p[1] - r * 0.50, p[0] + r * 0.18, p[1] - r * 0.12],
                  fill=rgba("#E0A25A", 0.65, "#EFE9DD"))
        if rnd.random() < 0.18:
            c.line([(p[0], p[1] + r), (p[0], p[1] + r * 2.2)],
                   fill=rgba("#B5651D", 0.35, "#EFE9DD"), width=2)
    # vertical wipe path
    path = [cam.project((0.31, 0.414, z))[:2] for z in (0.30, 0.23, 0.16, 0.09)]
    c.dashed(path, dash=10, gap=7, fill="#6B7A78", width=1.3)
    c.arrow(path, fill="#6B7A78", width=1.3, head=8)
    label_wall(c, cam, (0.39, 0.414, 0.27), (1340, 120),
               "COOLED GREASE  ·  DEGREASER CLOTH", size=14, align="right")
    label_wall(c, cam, (0.47, 0.414, 0.09), (1340, 210),
               "MAX 320 mm ABOVE COUNTER", size=14, align="right")
    a = cam.project((0.57, 0.412, 0.0)); b = cam.project((0.57, 0.412, 0.32))
    c.line([a[:2], b[:2]], fill="#8C8A80", width=0.9)
    for p in (a, b):
        c.line([(p[0] - 5, p[1]), (p[0] + 5, p[1])], fill="#8C8A80", width=0.9)
    c.text(a[0] + 8, b[1] - 4, "320", size=11, color="#6B7A78", family="mono")


SCENES = [
    ("A-01", "Flour & dough film", "BENCHTOP  ·  DRY RESIDUE / GLUTEN", "a",
     ["2-STAGE"], build_a01, overlay_a01,
     ["Soft brush → damp sponge", "6 mm overlap, 3 passes"],
     ["S-wipes, back edge → front", "Do not brush flour into the sink"],
     ["Gluten patch needs 10 s dwell", "Contact force capped at 5 N"], cam_a()),
    ("A-02", "Coffee grounds", "BENCHTOP  ·  DRY GRANULES", "a",
     ["VACUUM"], build_a02, overlay_a02,
     ["Crevice nozzle, 12 mm", "25 mm standoff, 8 s"],
     ["Perimeter → radial passes", "Do not move the grinder"],
     ["Grounds stain grout", "No wet wipe first"], cam_a(eye=(0.26, -0.55, 0.48))),
    ("A-03", "Eggshell fragments", "BENCHTOP  ·  BRITTLE SOLIDS", "a",
     ["GRASP"], build_a03, overlay_a03,
     ["Parallel jaw, 3-point", "5–15 mm shards"],
     ["Top-down pick, 40 mm lift", "Transfer to bin"],
     ["Shards < 3 mm → vacuum", "Never drag across quartz"],
     Camera((0.30, -0.37, 0.33), (0.30, 0.15, 0.04), PANEL_W, PANEL_H, vfov=33)),
    ("A-04", "Soapy puddle", "DRAINBOARD  ·  SURFACTANT / WATER", "a",
     ["BLOT"], build_a04, overlay_a04,
     ["Absorbent pad → dry cloth", "3 pads, outside-in"],
     ["Press 3 s, lift vertically", "Do not push toward cutout"],
     ["Limit 150 mL, no suction", "GFCI + drip tray required"],
     Camera((0.34, -0.36, 0.40), (0.36, 0.12, 0.02), PANEL_W, PANEL_H, vfov=32)),
    ("B-01", "Cooled grease ring", "HOB  ·  POLYMERISED FAT", "b",
     ["COOL-DOWN"], B.build_b01, B.overlay_b01,
     ["Silicone scraper + cloth", "Circular, then radial"],
     ["Light pressure, 3 N", "Burner locked out"],
     ["Surface < 40 °C only", "No blade on glass"], B.cam_b_top()),
    ("B-02", "Tomato sauce on tile", "VERTICAL TILE  ·  DRIED SUGAR / FAT", "b",
     ["DWELL"], B.build_b02, B.overlay_b02,
     ["Damp cloth + grout brush", "Top-down, 20 mm overlap"],
     ["20 s dwell on dried spots", "Protect the hob below"],
     ["Max 350 mm above counter", "No climbing, no spray"], B.cam_b_wall()),
    ("B-03", "Onion skins & peels", "CORNER PREP  ·  SOLID CLUTTER", "b",
     ["GRASP"], B.build_b03, B.overlay_b03,
     ["Parallel jaw + vacuum", "Largest first, 20 picks"],
     ["Drop into chute Ø120", "Vacuum the fragments"],
     ["Do not handle the knife", "Wet peels adhere — lift slow"], B.cam_b(eye=(0.42, -0.48, 0.48))),
    ("B-04", "Raw egg spill", "SINK DECK  ·  BIOHAZARD", "b",
     ["BIOHAZARD"], B.build_b04, B.overlay_b04,
     ["Disposable pad + tool cover", "Edge-in, blot, bag"],
     ["One-way: bin, never reuse", "Disinfect, flag for human"],
     ["No rinse, no cloth reuse", "Max 1 egg / 60 mL"], Camera((0.34, -0.37, 0.40), (0.34, 0.13, 0.02), PANEL_W, PANEL_H, vfov=33)),
    ("B-05", "Paprika spill", "SPICE STATION  ·  FINE STAINING POWDER", "b",
     ["DRY FIRST"], B.build_b05, B.overlay_b05,
     ["Dry towel → damp cloth", "Lift, do not smear"],
     ["Dry pickup first, 2 passes", "No water, no blowing"],
     ["Stains plastic and grout", "Limit 20 g per event"], B.cam_b()),
    ("A-05", "Grease on the splashback", "VERTICAL TILE  ·  COOLED FAT", "a",
     ["SCRAPE"], build_a05, overlay_a05,
     ["Degreaser cloth + squeegee", "Top-down passes"],
     ["Zig-zag, 20 mm overlap", "Collect at the bottom"],
     ["Max 320 mm above counter", "No outlets within 300 mm"], cam_wall()),
    ("C-01", "Flour dust", "ISLAND TOP  ·  DRY FILM", "c",
     ["DRY"], C.build_c01, C.overlay_c01,
     ["Microfibre dust wand", "Overlap 30 %, 4 passes"],
     ["Wide arcs, high → low", "No compressed air"],
     ["Dust migrates into joints", "Wipe arm after cycle"], C.cam_c()),
    ("C-02", "Breadcrumbs", "ISLAND TOP  ·  GRANULAR", "c",
     ["VACUUM"], C.build_c02, C.overlay_c02,
     ["Crevice nozzle + soft brush", "Board groove + seams"],
     ["Brush → vacuum, 6 passes", "Do not lift the board"],
     ["Board 300 mm — too wide for jaw", "Food-safe tool only"], C.cam_c(38)),
    ("C-03", "Honey drizzle", "ISLAND TOP  ·  VISCOUS SUGAR", "c",
     ["DWELL"], C.build_c03, C.overlay_c03,
     ["Warm damp cloth → dry", "Dwell 15 s"],
     ["Roll-lift along the run", "Do not smear across"],
     ["Streak limit 100 mm", "Sticky drag > 8 N"], C.cam_c(36)),
    ("C-04", "Red wine spill", "ISLAND TOP  ·  LIQUID / DYE", "c",
     ["LIQUID"], C.build_c04, C.overlay_c04,
     ["Absorbent pad + neutral cloth", "2 pads, edge-in"],
     ["Blot 10 s, lift vertically", "Pad dam at the island edge"],
     ["120 mL max, act < 5 min", "Test for stain afterwards"], C.cam_c(36)),
    ("C-05", "Vegetable trimmings", "ISLAND TOP  ·  SOLID CLUTTER", "c",
     ["GRASP"], C.build_c05, C.overlay_c05,
     ["Parallel jaw + vacuum", "Bowl at 450 mm"],
     ["Largest first, 15–25 picks", "Vacuum bits < 5 mm"],
     ["Wet peels slip — slow lift", "Do not pick the peeler"], C.cam_c(38)),
]


def run():
    for (num, title, sub, accent, tags, build, overlay, tool, motion, limit, cam) in SCENES:
        path = OUT / f"layout-{accent}-mess-{num[2:]}.png"
        mess_card(path, num, title, sub, {"A": "LAYOUT A  ·  SINK STATION", "B": "LAYOUT B  ·  CORNER BAY", "C": "LAYOUT C  ·  OVERHEAD ISLAND"}[num[0]], accent,
                  build, overlay, tool, motion, limit, tags=tags, cam=cam)
        print("wrote", path)


if __name__ == "__main__":
    run()







