"""Tiny flat-shaded 3D renderer (painter's algorithm) for the mess cards.

No external 3D library: a scene is a list of convex faces in metres, a camera,
and a key light. Faces are projected, back-face culled, depth sorted and shaded.
Good enough for counter-scale still lifes and keeps the whole pipeline in Pillow.
"""

from __future__ import annotations

import math

from PIL import Image, ImageDraw, ImageFilter

from draw import Canvas, hx, mix, shade


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, t):
    return (a[0] * t, a[1] * t, a[2] * t)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    length = math.sqrt(dot(a, a)) or 1.0
    return (a[0] / length, a[1] / length, a[2] / length)


class Camera:
    def __init__(self, eye, target, width, height, vfov=40.0, roll=0.0):
        self.eye = eye
        self.width, self.height = width, height
        self.f = (height / 2) / math.tan(math.radians(vfov) / 2)
        self.forward = norm(sub(target, eye))
        up = (0.0, 0.0, 1.0)
        self.right = norm(cross(self.forward, up))
        self.up = cross(self.right, self.forward)
        if roll:
            c, s = math.cos(roll), math.sin(roll)
            self.right, self.up = (
                add(mul(self.right, c), mul(self.up, s)),
                add(mul(self.up, c), mul(self.right, -s)),
            )

    def project(self, p):
        rel = sub(p, self.eye)
        z = dot(rel, self.forward)
        if z <= 0.01:
            z = 0.01
        x = dot(rel, self.right)
        y = dot(rel, self.up)
        return (self.width / 2 + self.f * x / z,
                self.height / 2 - self.f * y / z,
                z)


class Scene:
    def __init__(self, camera: Camera, bg="#FBF8F1"):
        self.cam = camera
        self.faces = []          # (points, colour, outline, double_sided, layer)
        self.bg = bg
        self._layer = 0

    def set_layer(self, layer):
        self._layer = layer

    # -- primitives -------------------------------------------------------- #
    def face(self, pts, color, outline=None, double=False):
        if len(pts) >= 3:
            self.faces.append((tuple(pts), color, outline, double, self._layer))

    def box(self, x0, y0, z0, x1, y1, z1, color, outline=None, skip=()):
        # (name, four corners in CCW order seen from outside)
        quads = {
            "top": ([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1)),
            "bottom": ([(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)], (0, 0, -1)),
            "front": ([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0)),
            "back": ([(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)], (0, 1, 0)),
            "left": ([(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)], (-1, 0, 0)),
            "right": ([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0)),
        }
        for name, (pts, _n) in quads.items():
            if name in skip or name == "bottom":
                continue
            self.face(pts, color, outline)

    def plane(self, pts, color, outline=None, double=True):
        self.face(pts, color, outline, double=double)

    def prism(self, cx, cy, r, z0, z1, color, n=28, outline=None, top=None, bottom=True):
        pts = [(cx + r * math.cos(2 * math.pi * i / n),
                cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            self.face([(a[0], a[1], z0), (b[0], b[1], z0),
                       (b[0], b[1], z1), (a[0], a[1], z1)], color, outline)
        self.face([(p[0], p[1], z1) for p in pts], top or shade(color, 0.10), outline)
        if bottom:
            pass

    def cylinder(self, cx, cy, r, z0, z1, color, top=None, inner=None, n=48):
        """Cylinder as flat-tone side quads + caps (no per-face lighting stripes)."""
        pts = [(cx + r * math.cos(2 * math.pi * i / n),
                cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
        side = shade(color, -0.04)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            self.face([(a[0], a[1], z0), (b[0], b[1], z0),
                       (b[0], b[1], z1), (a[0], a[1], z1)], side)
        self.face([(p[0], p[1], z1) for p in pts], top or shade(color, 0.10), double=True)
        if inner is not None:
            ir = r * 0.82
            self.face([(cx + ir * math.cos(2 * math.pi * i / n),
                        cy + ir * math.sin(2 * math.pi * i / n), z1 + 0.0015)
                       for i in range(n)], inner, double=True)

    def cone(self, cx, cy, r0, r1, z0, z1, color, top=None, inner=None, n=48, outline=None):
        """Truncated cone as flat-tone side quads + caps."""
        a0 = [(cx + r0 * math.cos(2 * math.pi * i / n),
               cy + r0 * math.sin(2 * math.pi * i / n)) for i in range(n)]
        a1 = [(cx + r1 * math.cos(2 * math.pi * i / n),
               cy + r1 * math.sin(2 * math.pi * i / n)) for i in range(n)]
        side = shade(color, -0.03)
        for i in range(n):
            j = (i + 1) % n
            self.face([(a0[i][0], a0[i][1], z0), (a0[j][0], a0[j][1], z0),
                       (a1[j][0], a1[j][1], z1), (a1[i][0], a1[i][1], z1)], side)
        self.face([(p[0], p[1], z1) for p in a1], top or shade(color, 0.08), outline,
                  double=True)
        if inner is not None:
            ir = r1 * 0.80
            self.face([(cx + ir * math.cos(2 * math.pi * i / n),
                        cy + ir * math.sin(2 * math.pi * i / n), z1 + 0.0015)
                       for i in range(n)], inner, double=True)

    def disc3(self, cx, cy, r, z, color, n=48, outline=None):
        self.face([(cx + r * math.cos(2 * math.pi * i / n),
                    cy + r * math.sin(2 * math.pi * i / n), z) for i in range(n)],
                  color, outline=outline, double=True)

    def sphere(self, cx, cy, cz, r, color, n=32):
        """Camera-facing billboard sphere (joint cap)."""
        center = (cx, cy, cz)
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append(add(center, add(mul(self.cam.right, r * math.cos(a)),
                                       mul(self.cam.up, r * math.sin(a)))))
        self.face(pts, color, double=True)
        hl = add(center, add(mul(self.cam.right, -r * 0.30), mul(self.cam.up, r * 0.30)))
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append(add(hl, add(mul(self.cam.right, r * 0.28 * math.cos(a)),
                                   mul(self.cam.up, r * 0.28 * math.sin(a)))))
        self.face(pts, shade(color, 0.18), double=True)

    def tube(self, p0, p1, r, color, n=12):
        axis = norm(sub(p1, p0))
        temp = (0, 0, 1) if abs(axis[2]) < 0.9 else (1, 0, 0)
        u = norm(cross(axis, temp))
        v = cross(axis, u)
        rings = []
        for p in (p0, p1):
            rings.append([add(p, add(mul(u, r * math.cos(2 * math.pi * i / n)),
                                     mul(v, r * math.sin(2 * math.pi * i / n))))
                          for i in range(n)])
        for i in range(n):
            a, b = i, (i + 1) % n
            self.face([rings[0][a], rings[0][b], rings[1][b], rings[1][a]], color)
        self.face(rings[1], shade(color, 0.15))
        self.face(list(reversed(rings[0])), shade(color, -0.12))

    # -- rendering --------------------------------------------------------- #
    def render(self, canvas: Canvas, ox=0, oy=0, mask=None):
        cam = self.cam
        light = norm((-0.45, -0.65, 0.85))
        items = []
        for pts, color, outline, double, layer in self.faces:
            normal = (0, 0, 0)
            # Newell's method for a robust normal
            nx = ny = nz = 0.0
            for i in range(len(pts)):
                xa, ya, za = pts[i]
                xb, yb, zb = pts[(i + 1) % len(pts)]
                nx += (ya - yb) * (za + zb)
                ny += (za - zb) * (xa + xb)
                nz += (xa - xb) * (ya + yb)
            nl = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normal = (nx / nl, ny / nl, nz / nl)
            center = (sum(p[0] for p in pts) / len(pts),
                      sum(p[1] for p in pts) / len(pts),
                      sum(p[2] for p in pts) / len(pts))
            if not double and dot(normal, sub(cam.eye, center)) <= 0:
                continue
            if dot(normal, sub(cam.eye, center)) < 0:
                normal = mul(normal, -1)
            depth = dot(sub(center, cam.eye), cam.forward)
            if depth <= 0.02:
                continue
            lam = max(0.0, dot(normal, light))
            c = mix(shade(color, -0.20), shade(color, 0.10), 0.25 + 0.75 * lam)
            projected = [cam.project(p) for p in pts]
            items.append((layer, depth, projected, c, outline))
        items.sort(key=lambda item: (item[0], -item[1]))
        for _layer, _depth, pts, c, outline in items:
            xy = [(ox + p[0], oy + p[1]) for p in pts]
            canvas.poly(xy, fill=c, outline=outline,
                        width=0.8 if outline else 0)
        return canvas

