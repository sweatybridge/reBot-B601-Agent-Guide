"""Layout sheets: plan, section and constraint notes for each deployment."""

from __future__ import annotations

import math
from pathlib import Path

from draw import (
    ACCENTS, CABINET, INK, INK_SOFT, LINE, MUTED, PAPER, WHITE, Canvas, hx, mix, shade,
)

W, H = 2200, 1500
FLOOR = "#F1EBDD"
COUNTER_FILL = "#E7DFD0"
COUNTER_LINE = "#B8AD9A"
APPLIANCE = "#F7F3EA"
WALL_FILL = "#DCD3C3"
KEEPOUT = "#B23A22"


def clip_poly(points, rect):
    """Sutherland-Hodgman clip of a polygon to an axis-aligned rectangle."""
    x0, y0, x1, y1 = rect

    def clip_edge(pts, inside, intersect):
        out = []
        if not pts:
            return out
        prev = pts[-1]
        for cur in pts:
            if inside(cur):
                if not inside(prev):
                    out.append(intersect(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(intersect(prev, cur))
            prev = cur
        return out

    def ix_vertical(a, b, x):
        t = (x - a[0]) / (b[0] - a[0]) if b[0] != a[0] else 0
        return (x, a[1] + (b[1] - a[1]) * t)

    def ix_horizontal(a, b, y):
        t = (y - a[1]) / (b[1] - a[1]) if b[1] != a[1] else 0
        return (a[0] + (b[0] - a[0]) * t, y)

    pts = points
    pts = clip_edge(pts, lambda p: p[0] >= x0, lambda a, b: ix_vertical(a, b, x0))
    pts = clip_edge(pts, lambda p: p[0] <= x1, lambda a, b: ix_vertical(a, b, x1))
    pts = clip_edge(pts, lambda p: p[1] >= y0, lambda a, b: ix_horizontal(a, b, y0))
    pts = clip_edge(pts, lambda p: p[1] <= y1, lambda a, b: ix_horizontal(a, b, y1))
    return pts


def clip_segment(a, b, rect):
    """Liang-Barsky clip of a segment to an axis-aligned rectangle."""
    x0, y0, x1, y1 = rect
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0:
            if q < 0:
                return None
        else:
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
    if t0 > t1:
        return None
    return ((ax + dx * t0, ay + dy * t0), (ax + dx * t1, ay + dy * t1))


class Plan:
    def __init__(self, c: Canvas, ox, oy, s, accent):
        self.c, self.ox, self.oy, self.s, self.acc = c, ox, oy, s, accent

    def p(self, x, y):
        return (self.ox + x * self.s, self.oy + y * self.s)

    def rect(self, x, y, w, d, fill=None, outline=None, width=1, dash=False, radius=0):
        x0, y0 = self.p(x, y)
        x1, y1 = self.p(x + w, y + d)
        self.c.rect([x0, y0, x1, y1], fill=fill, outline=None, radius=radius)
        if outline:
            if dash:
                self.c.dashed([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)],
                              dash=8, gap=6, fill=outline, width=width)
            else:
                self.c.rect([x0, y0, x1, y1], outline=outline, width=width, radius=radius)

    def circle(self, cx, cy, r, fill=None, outline=None, width=1):
        p = self.p(cx, cy)
        rr = r * self.s
        self.c.ellipse([p[0] - rr, p[1] - rr, p[0] + rr, p[1] + rr], fill=fill,
                       outline=outline, width=width)

    def line(self, pts, fill=INK, width=1, dash=False):
        sp = [self.p(x, y) for x, y in pts]
        if dash:
            self.c.dashed(sp, dash=8, gap=6, fill=fill, width=width)
        else:
            self.c.line(sp, fill=fill, width=width)

    def label(self, tip, at, text, color=INK_SOFT, size=12, align="left", dot=True):
        t = self.p(*tip)
        a = self.p(*at)
        self.c.line([t, a], fill="#8C8A80", width=0.8)
        if dot:
            self.c.ellipse([t[0] - 2.5, t[1] - 2.5, t[0] + 2.5, t[1] + 2.5], fill=self.acc["main"])
        if align == "left":
            self.c.text(a[0] + 6, a[1] - 6, text, size=size, color=color, family="mono")
        elif align == "right":
            self.c.text(a[0] - 6, a[1] - 6, text, size=size, color=color, family="mono",
                        anchor="ra")
        else:
            self.c.text(a[0], a[1] - 6, text, size=size, color=color, family="mono",
                        anchor="ma")

    def wall(self, x0, y0, x1, y1):
        self.line([(x0, y0), (x1, y1)], fill=INK, width=4.5)
        # hatch ticks on the outside (approx: perpendicular)
        length = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        nx, ny = -uy, ux
        t = 0
        while t < length:
            x, y = x0 + ux * t, y0 + uy * t
            a = self.p(x, y)
            b = self.p(x - nx * 30, y - ny * 30)
            self.c.line([a, b], fill=INK, width=0.9)
            t += 55

    def room(self, x0, y0, x1, y1):
        self.rect(x0, y0, x1 - x0, y1 - y0, fill=FLOOR, outline=None)
        self.rect(x0, y0, x1 - x0, y1 - y0, outline="#D5CBBB", width=1)

    def counter(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill=COUNTER_FILL, outline=COUNTER_LINE, width=1.3)
        # front edge highlight
        self.line([(x0, y0 + d), (x0 + w, y0 + d)], fill="#FBF6EC", width=1.4)

    def cabinet(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill=None, outline="#A79C88", width=1, dash=True)

    def appliance(self, x0, y0, w, d, label=None, fill=APPLIANCE):
        self.rect(x0, y0, w, d, fill=fill, outline="#A79C88", width=1.1, radius=2)
        if label:
            p = self.p(x0 + w / 2, y0 + d / 2)
            self.c.text(p[0], p[1], label, size=10, color=INK_SOFT, family="mono",
                        anchor="mm", tracking=0.6)

    def sink(self, x0, y0, w, d):
        self.rect(x0 - 12, y0 - 12, w + 24, d + 24, fill="#DCE1DD", outline="#A9B3B0", width=1)
        self.rect(x0, y0, w, d, fill="#C9D2CF", outline="#8E9997", width=1.2, radius=4)
        self.rect(x0 + 18, y0 + 18, w - 36, d - 36, fill="#B6C1BE", outline="#8E9997", width=0.8,
                  radius=3)
        cx, cy = x0 + w / 2, y0 + d / 2
        p = self.p(cx, cy)
        self.c.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill="#7E8C8B")
        # faucet
        f = self.p(cx, y0 - 14)
        self.c.ellipse([f[0] - 9, f[1] - 9, f[0] + 9, f[1] + 9], fill="#B9C2C1",
                       outline="#8E9997")
        self.c.arc([f[0] - 22, f[1] - 22, f[0] + 22, f[1] + 22], 200, 340, fill="#8E9997", width=2)

    def hob(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill="#2B3231", outline="#171D1C", width=1.2, radius=3)
        for fx in (0.28, 0.72):
            for fy in (0.28, 0.72):
                p = self.p(x0 + w * fx, y0 + d * fy)
                r = min(w, d) * 0.16 * self.s
                self.c.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r],
                               outline="#6C7774", width=1.2)
                self.c.ellipse([p[0] - r * 0.45, p[1] - r * 0.45,
                                p[0] + r * 0.45, p[1] + r * 0.45], fill="#3E4745")
                self.c.line([(p[0] - r, p[1]), (p[0] + r, p[1])], fill="#6C7774", width=0.8)
                self.c.line([(p[0], p[1] - r), (p[0], p[1] + r)], fill="#6C7774", width=0.8)

    def board(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill="#D8B07A", outline="#A97A45", width=1.2, radius=3)
        p = self.p(x0 + w * 0.84, y0 + d / 2)
        self.c.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill="#A97A45")
        for i in range(1, 4):
            y = y0 + d * i / 4
            self.line([(x0 + 8, y), (x0 + w - 8, y)], fill="#C99B63", width=0.6)

    def grinder(self, cx, cy, r=80):
        self.circle(cx, cy, r, fill="#EDE8DC", outline="#8E8576", width=1)
        self.circle(cx, cy, r * 0.45, fill="#C9BFA9", outline="#8E8576", width=0.8)
        p = self.p(cx, cy)
        self.c.text(p[0], p[1] + r * self.s + 12, "GRINDER", size=9, color=INK_SOFT,
                    family="mono", anchor="ma")

    def knife(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill="#C9A06A", outline="#8E6A3E", width=1.1, radius=2)
        for i in range(4):
            y = y0 + d * (i + 1) / 5
            self.line([(x0 + w * 0.2, y), (x0 + w * 0.8, y)], fill="#4A3A2A", width=1.4)

    def fridge(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill="#E9E4D8", outline="#8E8576", width=1.2, radius=3)
        self.line([(x0 + w * 0.5, y0 + 10), (x0 + w * 0.5, y0 + d - 10)], fill="#B8AE9A", width=1)
        p = self.p(x0 + w / 2, y0 + d + 18)
        self.c.text(p[0], p[1], "FRIDGE", size=10, color=INK_SOFT, family="mono", anchor="ma")

    def arm(self, cx, cy, dead_center=-90.0, dead_half=20.0):
        # dead / service wedge, explicitly built in plan coordinates
        r = 250
        pts = [(cx, cy)]
        for k in range(9):
            a = math.radians(dead_center - dead_half + 2 * dead_half * k / 8)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        self.c.poly([self.p(*q) for q in pts], fill=mix(FLOOR, "#6B6B6B", 0.22))
        for k in range(1, 7):
            a = math.radians(dead_center - dead_half + 2 * dead_half * k / 7)
            self.c.line([self.p(cx, cy), self.p(cx + r * math.cos(a), cy + r * math.sin(a))],
                        fill="#9A8F7C", width=0.6)
        p0 = self.p(cx, cy)
        # base
        self.circle(cx, cy, 90, fill=PAPER, outline=self.acc["main"], width=2)
        self.circle(cx, cy, 72, fill=self.acc["main"], outline=self.acc["deep"], width=1)
        self.circle(cx, cy, 16, fill=PAPER, outline=self.acc["deep"], width=1)
        # front arrow, opposite the dead wedge
        a = math.radians(dead_center + 180)
        self.c.arrow([(p0[0], p0[1]), (p0[0] + math.cos(a) * 60, p0[1] + math.sin(a) * 60)],
                     fill=self.acc["main"], width=1.4, head=7)

    def reach(self, cx, cy, r_work, r_max, clip=(0, 0, 3000, 2400), label=True):
        def circle(r, n=180):
            return [(cx + r * math.cos(2 * math.pi * i / n),
                     cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]

        def inside(q):
            return clip[0] <= q[0] <= clip[2] and clip[1] <= q[1] <= clip[3]

        work = clip_poly(circle(r_work), clip)
        if work:
            self.c.poly([self.p(*q) for q in work], fill=mix(FLOOR, self.acc["light"], 0.40))
        p0 = self.p(cx, cy)

        def ring(r, color, width, dash):
            pts = circle(r)
            for a, b in zip(pts, pts[1:] + pts[:1]):
                mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                if not inside(mid):
                    continue
                sa, sb = self.p(*a), self.p(*b)
                if dash:
                    # dash by subdividing each small segment
                    self.c.dashed([sa, sb], dash=5, gap=4, fill=color, width=width)
                else:
                    self.c.line([sa, sb], fill=color, width=width)

        ring(r_work, self.acc["main"], 1.6, False)
        ring(r_max, self.acc["mid"], 1.1, True)

    def keepout(self, x0, y0, w, d, margin=400):
        # rounded dashed rectangle around the hob
        x, y = x0 - margin, y0 - margin
        ww, dd = w + margin * 2, d + margin * 2
        pts = [(x, y), (x + ww, y), (x + ww, y + dd), (x, y + dd), (x, y)]
        self.line(pts, fill=KEEPOUT, width=1.2, dash=True)

    def dock(self, x0, y0, w, d):
        self.rect(x0, y0, w, d, fill="#EFE8DA", outline=self.acc["mid"], width=1, dash=True)
        p = self.p(x0 + w / 2, y0 + d / 2)
        self.c.text(p[0], p[1], "TOOLS", size=9, color=self.acc["deep"], family="mono",
                    anchor="mm", tracking=0.6)

    def mount(self, cx, cy, plate=300):
        self.rect(cx - plate / 2, cy - plate / 2, plate, plate, fill="#EFE8DA",
                  outline=self.acc["main"], width=1.2, dash=True)
        self.circle(cx, cy, 90, fill=PAPER, outline=self.acc["main"], width=2)
        self.circle(cx, cy, 70, fill=self.acc["main"], outline=self.acc["deep"], width=1)
        self.c.line([self.p(cx - 100, cy), self.p(cx + 100, cy)], fill=self.acc["deep"], width=1)
        self.c.line([self.p(cx, cy - 100), self.p(cx, cy + 100)], fill=self.acc["deep"], width=1)

    def stool(self, cx, cy, r=170):
        self.circle(cx, cy, r, fill="#E6DDCB", outline="#B8AE9A", width=1)
        self.circle(cx, cy, r * 0.55, fill="#D8CDB8", outline=None)

    def dim_h(self, x0, x1, y, label, color=MUTED, size=11, above=True):
        a, b = self.p(x0, y), self.p(x1, y)
        self.c.line([a, b], fill=color, width=0.8)
        for p in (a, b):
            self.c.line([(p[0], p[1] - 5), (p[0], p[1] + 5)], fill=color, width=0.8)
        self.c.text((a[0] + b[0]) / 2, a[1] - (10 if above else -4), label, size=size,
                    color=color, family="mono", anchor="ma")

    def dim_v(self, y0, y1, x, label, color=MUTED, size=11, left=True):
        a, b = self.p(x, y0), self.p(x, y1)
        self.c.line([a, b], fill=color, width=0.8)
        for p in (a, b):
            self.c.line([(p[0] - 5, p[1]), (p[0] + 5, p[1])], fill=color, width=0.8)
        self.c.text(a[0] - (8 if left else -8), (a[1] + b[1]) / 2, label, size=size,
                    color=color, family="mono", anchor=("rm" if left else "lm"))

    def section_mark(self, x0, y0, x1, y1, label="A"):
        a, b = self.p(x0, y0), self.p(x1, y1)
        self.c.dashed([a, b], dash=10, gap=6, fill=self.acc["deep"], width=1.2)
        for p in (a, b):
            self.c.ellipse([p[0] - 8, p[1] - 8, p[0] + 8, p[1] + 8], fill=PAPER,
                           outline=self.acc["deep"], width=1)
            self.c.text(p[0], p[1] - 5, label, size=9, color=self.acc["deep"],
                        family="mono", anchor="ma")

    def hood(self, x0, y0, w, d, label="HOOD"):
        self.rect(x0, y0, w, d, fill=None, outline="#8E8576", width=1.2)
        p = self.p(x0 + w / 2, y0 + d / 2)
        self.c.text(p[0], p[1], label, size=9, color="#8E8576", family="mono",
                    anchor="mm", tracking=0.8)

    def hood_camera(self, cx, cy, fov, label_at=None,
                    clip=(0, 0, 3000, 2400), label="OPTIONAL HOOD CAM"):
        """Top-down camera on the hood edge: useful view rectangle + module icon."""
        def inside(q):
            return clip[0] <= q[0] <= clip[2] and clip[1] <= q[1] <= clip[3]

        x0, y0, x1, y1 = fov
        rect = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        clipped = clip_poly(rect, clip)
        if clipped:
            self.c.poly([self.p(*q) for q in clipped],
                        fill=mix(FLOOR, self.acc["light"], 0.16))
        cxm, cym = (x0 + x1) / 2, (y0 + y1) / 2
        for a, b in zip(rect, rect[1:] + rect[:1]):
            seg = clip_segment(a, b, clip)
            if seg:
                self.c.dashed([self.p(*seg[0]), self.p(*seg[1])], dash=7, gap=5,
                              fill=self.acc["main"], width=1.0)
        if inside((cxm, cym)):
            seg = clip_segment((cx, cy), (cxm, cym), clip)
            if seg:
                self.c.dashed([self.p(*seg[0]), self.p(*seg[1])], dash=5, gap=6,
                              fill=self.acc["mid"], width=0.7)
        # module body at the hood edge, lens facing down
        px, py = self.p(cx, cy)
        self.c.rect([px - 16, py - 10, px + 16, py + 10], fill=self.acc["deep"],
                    outline=INK, width=0.7, radius=2)
        self.c.ellipse([px - 5, py - 5, px + 5, py + 5], fill="#DCE8E5",
                       outline=INK)
        if label_at:
            a = self.p(*label_at)
            self.c.line([(px + 16, py - 8), (a[0], a[1])], fill="#8C8A80", width=0.8)
            self.c.text(a[0] + (6 if a[0] >= px else -6), a[1] - 6, label,
                        size=11, color=self.acc["deep"], family="mono",
                        anchor=("la" if a[0] >= px else "ra"))

    def cabinet_camera(self, cx, cy, view, label_at=None,
                       clip=(0, 0, 3000, 2400), label="ALT CABINET CAM",
                       color="#6B4E8C"):
        """Alternative observer on the bottom edge of the upper cabinets."""
        def inside(q):
            return clip[0] <= q[0] <= clip[2] and clip[1] <= q[1] <= clip[3]

        x0, y0, x1, y1 = view
        rect = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        clipped = clip_poly(rect, clip)
        if clipped:
            self.c.poly([self.p(*q) for q in clipped],
                        fill=mix(FLOOR, color, 0.10))
        for a, b in zip(rect, rect[1:] + rect[:1]):
            seg = clip_segment(a, b, clip)
            if seg:
                self.c.dashed([self.p(*seg[0]), self.p(*seg[1])], dash=4, gap=5,
                              fill=color, width=0.9)
        cxm, cym = (x0 + x1) / 2, (y0 + y1) / 2
        if inside((cxm, cym)):
            seg = clip_segment((cx, cy), (cxm, cym), clip)
            if seg:
                self.c.dashed([self.p(*seg[0]), self.p(*seg[1])], dash=3, gap=5,
                              fill=color, width=0.7)
        px, py = self.p(cx, cy)
        self.c.rect([px - 14, py - 9, px + 14, py + 9], fill=color, outline=INK,
                    width=0.7, radius=2)
        self.c.ellipse([px - 4, py - 4, px + 4, py + 4], fill="#E4DCEF", outline=INK)
        # 30-degree downward tick, to distinguish it from the hood camera
        self.c.line([(px, py + 9), (px, py + 16)], fill=color, width=1.5)
        if label_at:
            a = self.p(*label_at)
            self.c.line([(px + 14, py - 7), (a[0], a[1])], fill="#8C8A80", width=0.8)
            self.c.text(a[0] + (6 if a[0] >= px else -6), a[1] - 6, label,
                        size=10, color=color, family="mono",
                        anchor=("la" if a[0] >= px else "ra"))

    def scalebar(self, x, y, length=1000, label="1 m"):
        p0 = self.p(x, y)
        p1 = self.p(x + length, y)
        self.c.line([p0, p1], fill=INK, width=2)
        for p in (p0, p1):
            self.c.line([(p[0], p[1] - 6), (p[0], p[1] + 6)], fill=INK, width=1.5)
        self.c.text((p0[0] + p1[0]) / 2, p0[1] + 8, label, size=11, color=INK,
                    family="mono", anchor="ma")

    def north(self, x, y):
        p = self.p(x, y)
        self.c.line([(p[0], p[1] - 26), (p[0], p[1] + 26)], fill=INK, width=1)
        self.c.poly([(p[0], p[1] - 34), (p[0] - 8, p[1] - 14), (p[0] + 8, p[1] - 14)], fill=INK)
        self.c.text(p[0], p[1] + 38, "N", size=11, color=INK, family="mono", anchor="ma")


class Sec:
    """Elevation: x horizontal, z vertical, screen y up."""

    def __init__(self, c: Canvas, ox, floor_y, s, accent):
        self.c, self.ox, self.floor_y, self.s, self.acc = c, ox, floor_y, s, accent

    def p(self, x, z):
        return (self.ox + x * self.s, self.floor_y - z * self.s)

    def rect(self, x, z, w, h, fill=None, outline=None, width=1, radius=0):
        p0 = self.p(x, z + h)
        p1 = self.p(x + w, z)
        self.c.rect([p0[0], p0[1], p1[0], p1[1]], fill=fill, outline=outline, width=width,
                    radius=radius)

    def line(self, pts, fill=INK, width=1, dash=False):
        sp = [self.p(*q) for q in pts]
        if dash:
            self.c.dashed(sp, dash=7, gap=5, fill=fill, width=width)
        else:
            self.c.line(sp, fill=fill, width=width)

    def dim_v(self, x, z0, z1, label, color=MUTED, size=10, side="right"):
        a = self.p(x, z0)
        b = self.p(x, z1)
        self.c.line([a, b], fill=color, width=0.8)
        for p in (a, b):
            self.c.line([(p[0] - 5, p[1]), (p[0] + 5, p[1])], fill=color, width=0.8)
        self.c.text(a[0] + (8 if side == "right" else -8), (a[1] + b[1]) / 2, label,
                    size=size, color=color, family="mono",
                    anchor=("lm" if side == "right" else "rm"))

    def arm(self, joints, base_x=1450, base_z=900, color="#4A5A57", width=9):
        pts = [self.p(*j) for j in joints]
        for a, b in zip(pts[:-1], pts[1:]):
            self.c.line([a, b], fill=color, width=width)
        for p in pts:
            self.c.ellipse([p[0] - width * 0.55, p[1] - width * 0.55,
                            p[0] + width * 0.55, p[1] + width * 0.55],
                           fill=shade(color, 0.12), outline=color)
        b = self.p(base_x, base_z)
        self.c.rect([b[0] - 14, b[1] - 22, b[0] + 14, b[1]], fill=color, outline=None)

    def reach(self, x, z, r_work, r_max):
        p = self.p(x, z)
        for r, col, w in ((r_max, self.acc["mid"], 1.0), (r_work, self.acc["main"], 1.4)):
            rr = r * self.s
            for a0, a1 in ((-90, 0), (180, 270)):
                self.c.arc([p[0] - rr, p[1] - rr, p[0] + rr, p[1] + rr],
                           a0, a1, fill=col, width=w)

    def dim_h(self, x0, x1, z, label, color=MUTED, size=10):
        a, b = self.p(x0, z), self.p(x1, z)
        self.c.line([a, b], fill=color, width=0.8)
        for p in (a, b):
            self.c.line([(p[0], p[1] - 5), (p[0], p[1] + 5)], fill=color, width=0.8)
        self.c.text((a[0] + b[0]) / 2, a[1] - 8, label, size=size, color=color,
                    family="mono", anchor="ma")

    def reach_below(self, x, z, r_work, r_max):
        p = self.p(x, z)
        for r, col, w in ((r_max, self.acc["mid"], 1.0), (r_work, self.acc["main"], 1.4)):
            rr = r * self.s
            self.c.arc([p[0] - rr, p[1] - rr, p[0] + rr, p[1] + rr],
                       180, 360, fill=col, width=w)

    def human(self, x, base_z=0, height=1700, color="#C3B9A6"):
        p = self.p(x, base_z)
        hh = height * self.s
        head_r = hh * 0.055
        self.c.ellipse([p[0] - head_r, p[1] - hh + 0, p[0] + head_r, p[1] - hh + head_r * 2],
                       fill=color, outline=None)
        self.c.line([(p[0], p[1] - hh + head_r * 2), (p[0], p[1] - hh * 0.52)],
                    fill=color, width=max(2, hh * 0.05))
        self.c.line([(p[0], p[1] - hh * 0.52), (p[0] - hh * 0.07, p[1])], fill=color,
                    width=max(2, hh * 0.045))
        self.c.line([(p[0], p[1] - hh * 0.52), (p[0] + hh * 0.07, p[1])], fill=color,
                    width=max(2, hh * 0.045))
        self.c.line([(p[0] - hh * 0.10, p[1] - hh * 0.80), (p[0] + hh * 0.10, p[1] - hh * 0.80)],
                    fill=color, width=max(2, hh * 0.04))


def sheet_header(c, kicker, title, subtitle, accent, index, total=3):
    acc = ACCENTS[accent]
    c.rect([0, 0, W, 8], fill=acc["main"])
    c.text(70, 44, kicker, 14, acc["main"], family="mono", weight="b", tracking=1.6)
    c.text(70, 76, title, 50, INK, family="head", weight="b")
    c.text(70, 142, subtitle, 17, INK_SOFT, family="body")
    c.text(W - 70, 50, f"MANUAL {index:02d} / {total:02d}", 12, MUTED, family="mono",
           anchor="ra", tracking=1.2)
    c.text(W - 70, 76, "B601-RS  ·  KITCHEN DEPLOYMENT", 12, acc["main"], family="mono",
           anchor="ra", tracking=1.0)
    c.text(W - 70, 102, "rev A  ·  dimensions in mm", 12, MUTED, family="mono",
           anchor="ra")
    c.line([(70, 180), (W - 70, 180)], fill="#D8CEBD", width=1)


def panel(c, box, title=None, accent="a"):
    x0, y0, x1, y1 = box
    c.rect([x0, y0, x1, y1], fill="#FBF8F1", outline="#DED5C4", width=1, radius=10)
    if title:
        c.text(x0 + 18, y0 + 16, title, 11, ACCENTS[accent]["main"], family="mono",
               weight="b", tracking=1.4)


def legend(c, x, y, accent):
    acc = ACCENTS[accent]
    items = [
        ("base", "arm base Ø180"),
        ("work", "600 working radius"),
        ("max", "755 max reach"),
        ("dead", "40° dead wedge"),
        ("keep", "heat keep-out"),
        ("dock", "tool dock"),
        ("cam", "hood camera"),
        ("ccam", "cabinet cam"),
        ("hood", "hood outline"),
    ]
    for i, (kind, label) in enumerate(items):
        xx = x + (i % 3) * 200
        yy = y + (i // 3) * 34
        if kind == "base":
            c.ellipse([xx, yy + 2, xx + 16, yy + 18], fill=acc["main"])
        elif kind == "work":
            c.ellipse([xx, yy + 2, xx + 16, yy + 18], outline=acc["main"], width=2)
        elif kind == "max":
            c.ellipse([xx, yy + 2, xx + 16, yy + 18], outline=acc["mid"], width=1)
        elif kind == "dead":
            c.pie([xx, yy + 2, xx + 16, yy + 18], 200, 340, fill=mix(FLOOR, "#6B6B6B", 0.3))
        elif kind == "keep":
            c.rect([xx, yy + 2, xx + 16, yy + 18], outline=KEEPOUT, width=1)
        elif kind == "dock":
            c.rect([xx, yy + 3, xx + 16, yy + 17], outline=acc["mid"], width=1)
        elif kind == "cam":
            c.rect([xx + 2, yy + 5, xx + 14, yy + 15], fill=acc["deep"], outline=INK,
                   width=0.8, radius=2)
            c.ellipse([xx + 6, yy + 8, xx + 10, yy + 12], fill="#DCE8E5", outline=INK)
        elif kind == "ccam":
            c.rect([xx + 2, yy + 5, xx + 14, yy + 15], fill="#6B4E8C", outline=INK,
                   width=0.8, radius=2)
            c.ellipse([xx + 6, yy + 8, xx + 10, yy + 12], fill="#E4DCEF", outline=INK)
        elif kind == "hood":
            c.rect([xx, yy + 3, xx + 16, yy + 17], outline="#8E8576", width=1)
        c.text(xx + 24, yy + 5, label, size=10, color=INK_SOFT, family="mono")


def notes_panel(c, box, title, items, accent, key=True):
    x0, y0, x1, y1 = box
    acc = ACCENTS[accent]
    panel(c, box, title, accent)
    y = y0 + 44
    for i, (head, body) in enumerate(items):
        c.text(x0 + 18, y, f"{i + 1:02d}", 11, acc["main"], family="mono", weight="b")
        c.text(x0 + 44, y - 2, head, 13, INK, family="body", weight="sb")
        c.text(x0 + 44, y + 18, body, 11, INK_SOFT, family="body")
        y += 52
    if key:
        c.text(x0 + 18, y + 4, "KEY", 10, acc["main"], family="mono", weight="b", tracking=1.2)
        legend(c, x0 + 18, y + 22, accent)


def footer(c, text, accent):
    c.line([(70, 1442), (W - 70, 1442)], fill="#D8CEBD", width=1)
    c.text(70, 1456, text, 11, MUTED, family="mono")
    c.text(W - 70, 1456, "Seeed reBot B601-RS  ·  6+1 DOF  ·  754.7 mm reach  ·  2.5 kg rated payload",
           11, MUTED, family="mono", anchor="ra")


