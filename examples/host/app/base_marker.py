#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A printed square on the base plate: the camera pose without the arm.

The tip-sweep solves for the camera from the arm's own motion, which means it has
to *see* the tip in every frame -- and on a cluttered bench, with a gripper whose
appearance changes as the wrist turns, that is where it keeps failing. A marker
pasted flat on the base plate inverts the problem: the camera's pose relative to
the marker is a plain PnP, exact to a millimetre, and the arm never enters it.

The marker's own frame is defined so that it is the natural one for a sticker on a
horizontal plate: x across the marker, y up the marker, z out of the sheet (which
is straight up, because the plate is level). `base_from_marker` then places that
frame in the robot's base frame from four numbers the installer measures once.

Everything downstream gets better for free: with the camera known to a pixel, the
tip detector's projection lands on the gripper, so the sweep becomes a
*verification* rather than a search.
"""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
SIZE_M = 0.04                 # the standard tag: 4 cm across the black square
QUIET_M = 0.01                # white margin the detector needs around it

# The standard placement, from the RS URDF's base_link.STL: a 140 x 200 x 10 mm
# foot plate under a round column of radius ~47.5 mm that rises to z = 75 mm, with
# the J1 axis through its centre. The tag is centred on the column's +y side at
# mid-height, its face pointing +y. Radius is the surface at that height, plus the
# sticker's own thickness. An installer's tape measure is within a millimetre of
# this, and a millimetre here is a millimetre at the tap.
STANDARD_AT = (0.000, 0.047, 0.043)
STANDARD_HEADING_DEG = 90.0


class MarkerError(Exception):
    """The marker was not found, or its pose could not be solved."""


def _dictionary():
    return cv2.aruco.getPredefinedDictionary(DICTIONARY)


def detect(image, marker_id=MARKER_ID):
    """The marker's four corners in the image, or raise.

    Corner order is the library's: top-left, top-right, bottom-right, bottom-left
    as seen in the frame.
    """
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(_dictionary(), cv2.aruco.DetectorParameters())
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        raise MarkerError("no marker in the frame: is the tag facing the camera and lit?")
    if marker_id is None:
        # No id configured: the biggest marker in view is the one being aimed at.
        sizes = [float(cv2.contourArea(c)) for c in corners]
        return corners[int(np.argmax(sizes))].reshape(4, 2).astype(float)
    for index, found in enumerate(ids.flatten()):
        if int(found) == int(marker_id):
            return corners[index].reshape(4, 2).astype(float)
    raise MarkerError("found %s but not id %d" % (list(ids.flatten()), marker_id))


def object_points(size_m=SIZE_M):
    """The marker's corners in its own frame, matching `detect`'s order."""
    half = float(size_m) / 2.0
    return np.array([[-half, half, 0.0],
                     [half, half, 0.0],
                     [half, -half, 0.0],
                     [-half, -half, 0.0]], dtype=float)


def pose(image, K, dist, size_m=SIZE_M, marker_id=MARKER_ID):
    """`T_cam_marker`: the marker's pose in the camera's frame."""
    corners = detect(image, marker_id=marker_id)
    K = np.asarray(K, dtype=float)
    dist = np.asarray(dist, dtype=float)
    ok, rvec, tvec = cv2.solvePnP(object_points(size_m), corners, K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise MarkerError("the PnP did not solve: the marker is probably too oblique or too "
                          "small in the frame")
    rotation, _jacobian = cv2.Rodrigues(rvec)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return matrix


def base_from_marker(at, yaw=0.0, mode="flat"):
    """`T_base_marker` from where the installer pasted it.

    `at` is the marker's centre in the robot's base frame (metres).

    "flat": the tag lies on the plate, its z axis (out of the printed face) is the
    base's z, and `yaw` turns it about that. `yaw` is in radians.

    "vertical": the tag stands on a side face, so its normal is horizontal at the
    heading `yaw` and its +y is the base's +z -- the printed image reads upright
    when the camera faces it.
    """
    centre = np.asarray(at, dtype=float).reshape(3)
    angle = float(yaw)
    if str(mode).lower() == "vertical":
        normal = np.array([np.cos(angle), np.sin(angle), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(up, normal)
        rotation = np.column_stack([right, up, normal])
    else:
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                             [np.sin(angle), np.cos(angle), 0.0],
                             [0.0, 0.0, 1.0]], dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = centre
    return matrix


def camera_pose(image, K, dist, at, yaw=0.0, size_m=SIZE_M, marker_id=MARKER_ID,
                mode="flat"):
    """`T_base_cam` from one photograph of the marker."""
    from_marker = base_from_marker(at, yaw, mode=mode)
    camera_from_marker = pose(image, K, dist, size_m=size_m, marker_id=marker_id)
    return from_marker @ np.linalg.inv(camera_from_marker)


def parse_spec(text, size_m=SIZE_M):
    """`{"at", "yaw", "size_m"}` from "x,y,z[,yaw]", or None when empty.

    Empty means the plate has no tag, which leaves the tap path in charge. A typo
    is an error at startup rather than a wrong calibration later.
    """
    text = str(text or "").strip()
    if not text:
        return None
    parts = [piece for piece in text.replace(";", ",").split(",") if piece.strip()]
    if len(parts) not in (3, 4, 5):
        raise MarkerError("the marker position wants 'x,y,z[,yaw[,id]]' in metres and degrees, "
                          "got %r" % text)
    try:
        values = [float(piece) for piece in parts]
    except ValueError as error:
        raise MarkerError("the marker position is not a number list: %r" % text) from error
    # Degrees on the way in, radians internally: installers measure angles in
    # degrees, and the rest of the code multiplies them into a rotation.
    return {"at": (values[0], values[1], values[2]),
            "yaw": math.radians(values[3]) if len(values) >= 4 else 0.0,
            "size_m": float(size_m),
            "id": int(values[4]) if len(values) >= 5 else None}


def outer_size(inner_m, bits=4, border_modules=1):
    """The side the *detector* measures, from the printed pattern's inner side.

    An ArUco marker is `bits` modules of payload inside `border_modules` of black
    border on each side, so a 4x4 tag is 6 modules across. A tape measure across
    the pattern inside the border is therefore smaller than the square the corners
    sit on, and PnP needs the latter: a 40 mm inner pattern is a 60 mm marker.
    """
    modules = float(bits) + 2.0 * float(border_modules)
    return float(inner_m) * modules / float(bits)


def marker_grid(bits=4, border=1):
    """The marker's modules: True where a module is black.

    `bits` payload modules inside `border` modules of black border, i.e. 6 x 6 for
    a 4x4 tag. Read back from the library's own bitmap so the sheet cannot drift
    from the dictionary.
    """
    modules = int(bits) + 2 * int(border)
    bitmap = cv2.aruco.generateImageMarker(_dictionary(), int(MARKER_ID), modules * 8)
    step = 8
    return [[bool(bitmap[row * step + step // 2, column * step + step // 2] < 128)
             for column in range(modules)] for row in range(modules)]


def sheet_pdf(path, size_m=SIZE_M, margin_m=0.015, bits=4, border=1):
    """A one-page PDF with the marker at exactly `size_m` across.

    A PDF carries its own physical size, so the "printed at 100%" failure -- a tag
    that is 4 cm on screen and 3.7 cm on paper -- cannot happen. The modules are
    drawn as vector squares, so it prints clean at any resolution.
    """
    points = 72.0 / 25.4                     # millimetres to PDF points
    grid = marker_grid(bits, border)
    modules = len(grid)
    marker_mm = float(size_m) * 1000.0
    margin = float(margin_m) * 1000.0
    bar_area = 12.0
    page_w = marker_mm + 2.0 * margin
    page_h = marker_mm + 2.0 * margin + bar_area
    cell = marker_mm / modules
    left = margin
    bottom = margin + bar_area

    commands = ["0 g"]
    for row, line in enumerate(grid):
        for column, black in enumerate(line):
            if not black:
                continue
            x = left + column * cell
            y = bottom + (modules - 1 - row) * cell
            commands.append("%.4f %.4f %.4f %.4f re f" % (x * points, y * points,
                                                          cell * points, cell * points))
    # A printed scale bar, so a wrong print is visible without a ruler.
    commands.append("%.4f %.4f %.4f %.4f re f" % (left * points, (margin * 0.45) * points,
                                                  marker_mm * points, 0.8 * points))
    commands.append("BT /F1 7 Tf %.4f %.4f Td (%g cm) Tj ET"
                    % (left * points, (margin * 0.45 - 4.0) * points, float(size_m) * 100.0))

    content = "\n".join(commands).encode("ascii")
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        ("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] /Contents 4 0 R "
         "/Resources << /Font << /F1 5 0 R >> >> >>" % (page_w * points, page_h * points)
         ).encode("ascii"),
        b"<< /Length " + str(len(content)).encode("ascii") + b" >>\nstream\n" + content
        + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]

    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for index, body in enumerate(objects, start=1):
        offsets.append(len(out))
        out += ("%d 0 obj\n" % index).encode("ascii") + body + b"\nendobj\n"
    xref_at = len(out)
    out += ("xref\n0 %d\n" % (len(objects) + 1)).encode("ascii")
    out += b"0000000000 65535 f \n"
    for offset in offsets:
        out += ("%010d 00000 n \n" % offset).encode("ascii")
    out += ("trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
            % (len(objects) + 1, xref_at)).encode("ascii")
    Path(path).write_bytes(bytes(out))
    return {"path": str(path), "page_mm": (page_w, page_h), "marker_mm": marker_mm}


def marker_bitmap(size_m=SIZE_M, pixels_per_m=4000.0):
    """The marker as an image whose printed size is exactly `size_m`."""
    pixels = int(round(float(size_m) * float(pixels_per_m)))
    if pixels < 16:
        raise MarkerError("a %g mm marker is too small to render" % (1000.0 * size_m))
    return cv2.aruco.generateImageMarker(_dictionary(), int(MARKER_ID), pixels)
