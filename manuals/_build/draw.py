"""Minimal drawing toolkit for the kitchen-deployment manuals.

Everything is drawn with Pillow at 2x and downsampled with Lanczos, so the
output reads as vector art at normal viewing size without needing a browser.
All public coordinates are in "logical" pixels (the 1x canvas).
"""

from __future__ import annotations

import math
import random

from PIL import Image, ImageDraw, ImageFilter, ImageFont

# --------------------------------------------------------------------------- #
# Palette
# --------------------------------------------------------------------------- #
INK = "#1E2A28"
INK_SOFT = "#4A5A57"
MUTED = "#7C8A86"
LINE = "#C9C0B0"
LINE_DARK = "#A79C8A"
PAPER = "#F7F3EA"
PAPER_DEEP = "#EFE8DA"
WHITE = "#FFFFFF"

SURFACE = "#E9E2D4"       # quartz counter
SURFACE_HI = "#F3EEE4"
SURFACE_LO = "#D8CDBA"
CABINET = "#DFD6C5"
CABINET_DARK = "#C9BEA9"
WALL = "#EDE7DB"
TILE = "#E4DED1"
TILE_LINE = "#CFC6B6"
WOOD = "#C99B63"
WOOD_DARK = "#A97A45"
WOOD_LIGHT = "#DDB583"
STEEL = "#B9C2C1"
STEEL_DARK = "#8D9998"
BLACK_GLASS = "#2B3231"
GAS = "#3A4240"

ACCENTS = {
    "a": {"main": "#0E6B63", "light": "#D7E8E4", "mid": "#6FA9A2", "deep": "#08443F"},
    "b": {"main": "#A8481A", "light": "#F3DFD2", "mid": "#D08A62", "deep": "#6E2D0F"},
    "c": {"main": "#3F4C8A", "light": "#DDE0EF", "mid": "#8B95C4", "deep": "#272F5C"},
}

FONT_DIR = r"C:\Windows\Fonts"
FONT_FILES = {
    ("body", "r"): "segoeui.ttf",
    ("body", "sb"): "seguisb.ttf",
    ("body", "b"): "segoeuib.ttf",
    ("body", "l"): "segoeuil.ttf",
    ("head", "r"): "bahnschrift.ttf",
    ("head", "b"): "bahnschrift.ttf",
    ("mono", "r"): "consola.ttf",
    ("mono", "b"): "consolab.ttf",
    ("serif", "r"): "georgia.ttf",
}


def hx(value: str) -> tuple[int, int, int]:
    value = value.lstrip("#")
    return tuple(int(value[i:i + 2], 16) for i in (0, 2, 4))


def mix(a, b, t: float):
    """Blend colour a toward b by t (0..1). Accepts hex or tuples."""
    ca = hx(a) if isinstance(a, str) else a
    cb = hx(b) if isinstance(b, str) else b
    t = max(0.0, min(1.0, t))
    return tuple(round(ca[i] + (cb[i] - ca[i]) * t) for i in range(3))


def shade(color, amount: float):
    """amount > 0 lightens, < 0 darkens."""
    return mix(color, "#FFFFFF", amount) if amount >= 0 else mix(color, "#000000", -amount)


def rgba(color, alpha: float, over=PAPER):
    """Pre-blend a translucent colour over a known background."""
    return mix(over, color, alpha)


# --------------------------------------------------------------------------- #
# Canvas
# --------------------------------------------------------------------------- #
class Canvas:
    def __init__(self, width: int, height: int, bg=PAPER, k: int = 2):
        self.w, self.h, self.k = width, height, k
        self.img = Image.new("RGB", (width * k, height * k), hx(bg) if isinstance(bg, str) else bg)
        self.d = ImageDraw.Draw(self.img)
        self._fonts: dict = {}

    # -- geometry ---------------------------------------------------------- #
    def _xy(self, pts):
        k = self.k
        return [(x * k, y * k) for x, y in pts]

    def line(self, pts, fill=INK, width=1, joint="curve"):
        self.d.line(self._xy(pts), fill=hx(fill) if isinstance(fill, str) else fill,
                    width=max(1, round(width * self.k)), joint=joint)

    def poly(self, pts, fill=None, outline=None, width=1):
        xy = self._xy(pts)
        self.d.polygon(xy, fill=hx(fill) if isinstance(fill, str) else fill)
        if outline:
            self.d.line(xy + [xy[0]], fill=hx(outline) if isinstance(outline, str) else outline,
                        width=max(1, round(width * self.k)), joint="curve")

    def rect(self, box, fill=None, outline=None, width=1, radius=0):
        x0, y0, x1, y1 = box
        xy = [x0 * self.k, y0 * self.k, x1 * self.k, y1 * self.k]
        f = hx(fill) if isinstance(fill, str) else fill
        o = hx(outline) if isinstance(outline, str) else outline
        wdt = max(1, round(width * self.k))
        if radius:
            self.d.rounded_rectangle(xy, radius=radius * self.k, fill=f, outline=o, width=wdt)
        else:
            self.d.rectangle(xy, fill=f, outline=o, width=wdt)

    def ellipse(self, box, fill=None, outline=None, width=1):
        x0, y0, x1, y1 = box
        self.d.ellipse([x0 * self.k, y0 * self.k, x1 * self.k, y1 * self.k],
                       fill=hx(fill) if isinstance(fill, str) else fill,
                       outline=hx(outline) if isinstance(outline, str) else outline,
                       width=max(1, round(width * self.k)))

    def pie(self, box, start, end, fill=None, outline=None, width=1):
        x0, y0, x1, y1 = box
        self.d.pieslice([x0 * self.k, y0 * self.k, x1 * self.k, y1 * self.k], start, end,
                        fill=hx(fill) if isinstance(fill, str) else fill,
                        outline=hx(outline) if isinstance(outline, str) else outline,
                        width=max(1, round(width * self.k)))

    def arc(self, box, start, end, fill=INK, width=1):
        x0, y0, x1, y1 = box
        self.d.arc([x0 * self.k, y0 * self.k, x1 * self.k, y1 * self.k], start, end,
                   fill=hx(fill) if isinstance(fill, str) else fill,
                   width=max(1, round(width * self.k)))

    # -- text -------------------------------------------------------------- #
    def font(self, size, family="body", weight="r"):
        key = (family, weight, size)
        if key not in self._fonts:
            path = FONT_FILES.get((family, weight), FONT_FILES[("body", "r")])
            self._fonts[key] = ImageFont.truetype(
                f"{FONT_DIR}\\{path}", max(1, round(size * self.k)))
        return self._fonts[key]

    def text(self, x, y, s, size=18, color=INK, family="body", weight="r",
             anchor="la", tracking=0.0, alpha=1.0):
        f = self.font(size, family, weight)
        fill = hx(color) if isinstance(color, str) else color
        if alpha < 1:
            fill = mix(PAPER, fill, alpha)
        if not tracking:
            self.d.text((x * self.k, y * self.k), s, font=f, fill=fill, anchor=anchor)
            return
        # manual tracking for small-caps labels
        total = sum(self.d.textlength(ch, font=f) for ch in s) + tracking * self.k * (len(s) - 1)
        if anchor[0] == "m":
            cx = x * self.k - total / 2
        elif anchor[0] == "r":
            cx = x * self.k - total
        else:
            cx = x * self.k
        for ch in s:
            self.d.text((cx, y * self.k), ch, font=f, fill=fill, anchor="l" + anchor[1])
            cx += self.d.textlength(ch, font=f) + tracking * self.k

    def tw(self, s, size=18, family="body", weight="r"):
        return self.d.textlength(s, font=self.font(size, family, weight)) / self.k

    # -- composite helpers -------------------------------------------------- #
    def vgrad(self, box, top, bottom):
        x0, y0, x1, y1 = box
        top_c, bot_c = hx(top), hx(bottom)
        h = max(1, round((y1 - y0) * self.k))
        for i in range(h):
            c = mix(top_c, bot_c, i / max(1, h - 1))
            self.d.line([(x0 * self.k, y0 * self.k + i), (x1 * self.k, y0 * self.k + i)], fill=c)

    def shadow(self, draw_fn, blur=6, offset=(0, 6), color="#000000", alpha=0.18):
        layer = Image.new("RGBA", self.img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        draw_fn(d, self.k, offset)
        layer = layer.filter(ImageFilter.GaussianBlur(blur * self.k))
        tint = Image.new("RGBA", self.img.size, hx(color) + (round(alpha * 255),))
        layer = Image.composite(tint, Image.new("RGBA", self.img.size, (0, 0, 0, 0)),
                                layer.split()[3])
        self.img = Image.alpha_composite(self.img.convert("RGBA"), layer).convert("RGB")
        self.d = ImageDraw.Draw(self.img)

    def dashed(self, pts, dash=8, gap=6, fill=INK, width=1):
        """Dash a polyline using arc-length parametrisation."""
        for a, b in zip(pts[:-1], pts[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            ux, uy = dx / length, dy / length
            t = 0.0
            while t < length:
                e = min(t + dash, length)
                self.line([(a[0] + ux * t, a[1] + uy * t), (a[0] + ux * e, a[1] + uy * e)],
                          fill=fill, width=width)
                t = e + gap

    def arrow(self, pts, fill=INK, width=2, head=9, dashed=False):
        if dashed:
            self.dashed(pts, dash=7, gap=5, fill=fill, width=width)
        else:
            self.line(pts, fill=fill, width=width)
        a, b = pts[-2], pts[-1]
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        p1 = (b[0] - head * math.cos(ang - 0.45), b[1] - head * math.sin(ang - 0.45))
        p2 = (b[0] - head * math.cos(ang + 0.45), b[1] - head * math.sin(ang + 0.45))
        self.poly([b, p1, p2], fill=fill)

    def chip(self, x, y, label, fg, bg, size=13, pad_x=10, pad_y=5, radius=11,
             family="mono", weight="b", tracking=0.6):
        w = self.tw(label, size, family, weight) + tracking * (len(label) - 1) + pad_x * 2
        h = size + pad_y * 2 + 2
        self.rect([x, y, x + w, y + h], fill=bg, radius=radius)
        self.text(x + pad_x, y + pad_y + 1, label, size=size, color=fg,
                  family=family, weight=weight, tracking=tracking)
        return w, h

    def save(self, path):
        out = self.img.resize((self.w, self.h), Image.LANCZOS)
        out.save(path, optimize=True)
        return path


# --------------------------------------------------------------------------- #
# Isometric projection
# --------------------------------------------------------------------------- #
class Iso:
    """2:1 dimetric projection: +x goes right-down, +y left-down, +z up."""

    def __init__(self, ox, oy, s=1.0):
        self.ox, self.oy, self.s = ox, oy, s

    def p(self, x, y, z=0.0):
        return (self.ox + (x - y) * 0.866 * self.s,
                self.oy + (x + y) * 0.5 * self.s - z * self.s)

    def line(self, c: Canvas, pts, fill=INK, width=1):
        c.line([self.p(*q) for q in pts], fill=fill, width=width)

    def poly(self, c: Canvas, pts, fill=None, outline=None, width=1):
        c.poly([self.p(*q) for q in pts], fill=fill, outline=outline, width=width)

    def box(self, c: Canvas, x, y, z, w, d, h, color, outline=None, top_boost=0.10):
        top = shade(color, top_boost)
        right = shade(color, -0.06)
        left = shade(color, -0.16)
        # top
        self.poly(c, [(x, y, z + h), (x + w, y, z + h), (x + w, y + d, z + h), (x, y + d, z + h)],
                  fill=top, outline=outline, width=0.7)
        # +x face
        self.poly(c, [(x + w, y, z + h), (x + w, y + d, z + h), (x + w, y + d, z), (x + w, y, z)],
                  fill=right, outline=outline, width=0.7)
        # +y face
        self.poly(c, [(x, y + d, z + h), (x + w, y + d, z + h), (x + w, y + d, z), (x, y + d, z)],
                  fill=left, outline=outline, width=0.7)

    def slab(self, c: Canvas, x, y, z, w, d, h, color, outline=None):
        self.box(c, x, y, z - h, w, d, h, color, outline=outline, top_boost=0.08)

    def disc(self, c: Canvas, cx, cy, r, z=0.0, fill=SURFACE, outline=None, width=0.8, n=48):
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
        self.poly(c, pts, fill=fill, outline=outline, width=width)

    def cyl(self, c: Canvas, cx, cy, r, h, z=0.0, color=STEEL, outline=None, n=48, top=None):
        side = shade(color, -0.10)
        top_c = shade(color, 0.12) if top is None else top
        # side: sample the silhouette from angle 0 to pi
        left = [(cx + r * math.cos(a), cy + r * math.sin(a), z + h)
                for a in [math.pi * i / n for i in range(n + 1)]]
        right = [(cx + r * math.cos(a), cy + r * math.sin(a), z)
                 for a in [math.pi - math.pi * i / n for i in range(n + 1)]]
        self.poly(c, left + right, fill=side, outline=outline, width=0.7)
        self.disc(c, cx, cy, r, z + h, fill=top_c, outline=outline, width=0.7, n=n)

    def wall(self, c: Canvas, x0, y0, x1, y1, z0, z1, color, outline=None, tiles=None,
             tile_color=None, tile_line=None):
        self.poly(c, [(x0, y0, z0), (x1, y1, z0), (x1, y1, z1), (x0, y0, z1)],
                  fill=color, outline=outline, width=0.8)
        if tiles:
            cols, rows = tiles
            tc = tile_color or shade(color, 0.03)
            tl = tile_line or shade(color, -0.10)
            for i in range(1, cols):
                t = i / cols
                x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
                self.line(c, [(x, y, z0), (x, y, z1)], fill=tl, width=0.6)
            for j in range(1, rows):
                t = j / rows
                z = z0 + (z1 - z0) * t
                self.line(c, [(x0, y0, z), (x1, y1, z)], fill=tl, width=0.6)

    def ellipse_plane(self, c: Canvas, cx, cy, rx, ry, z=0, fill=None, outline=None,
                      width=0.8, n=48):
        pts = [(cx + rx * math.cos(2 * math.pi * i / n), cy + ry * math.sin(2 * math.pi * i / n), z)
               for i in range(n)]
        self.poly(c, pts, fill=fill, outline=outline, width=width)


def smooth_blob(cx, cy, rx, ry, seed=0, n=14, roughness=0.18):
    """Closed, gently irregular outline in surface coordinates."""
    rnd = random.Random(seed)
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        r = 1 + roughness * (rnd.random() - 0.5)
        pts.append((cx + rx * r * math.cos(a), cy + ry * r * math.sin(a)))
    # Catmull-Rom-ish smoothing by inserting midpoints is unnecessary at this size
    return pts


def lerp_path(pts, steps=80):
    """Resample a polyline to a smooth-ish path with cosine easing between points."""
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        for i in range(steps):
            t = i / steps
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    out.append(pts[-1])
    return out


def dim_h(c: Canvas, x0, x1, y, label, color=MUTED, size=12, tick=6, above=True):
    c.line([(x0, y), (x1, y)], fill=color, width=0.8)
    c.line([(x0, y - tick), (x0, y + tick)], fill=color, width=0.8)
    c.line([(x1, y - tick), (x1, y + tick)], fill=color, width=0.8)
    c.text((x0 + x1) / 2, y - (size + 4 if above else -4), label, size=size, color=color,
           family="mono", anchor="ma")


def dim_v(c: Canvas, y0, y1, x, label, color=MUTED, size=12, tick=6, left=True):
    c.line([(x, y0), (x, y1)], fill=color, width=0.8)
    c.line([(x - tick, y0), (x + tick, y0)], fill=color, width=0.8)
    c.line([(x - tick, y1), (x + tick, y1)], fill=color, width=0.8)
    c.text(x - (6 if left else -6), (y0 + y1) / 2, label, size=size, color=color,
           family="mono", anchor=("rm" if left else "lm"))


def leader(c: Canvas, tip, label_at, text, color=INK_SOFT, size=12, dot=True,
           family="mono", align="left"):
    c.line([tip, label_at], fill=color, width=0.8)
    if dot:
        c.ellipse([tip[0] - 2, tip[1] - 2, tip[0] + 2, tip[1] + 2], fill=color)
    if align == "left":
        c.text(label_at[0] + 5, label_at[1] - 6, text, size=size, color=color, family=family)
    elif align == "right":
        c.text(label_at[0] - 5, label_at[1] - 6, text, size=size, color=color, family=family,
               anchor="ra")
    else:
        c.text(label_at[0], label_at[1] - 6, text, size=size, color=color, family=family,
               anchor="ma")
