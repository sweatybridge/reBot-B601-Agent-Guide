#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render an ArUco tag at an exact physical size, for printing.

    python make_tag.py --id 0 --size 80 --out tags

The number that goes into `locate_target.py --tag-size` is the side of the
**black square, border included** -- not the sheet, and not the quiet zone. Print
at 100 % scale (no "fit to page"), then measure the black square with calipers
and pass the measured value. A 3 % size error is a 3 % depth error, which at
0.8 m is 24 mm.

The quiet zone is white at 10 % of the side on each edge. Tape the tag flat: a
tag that is not flat is a tag whose pose has a tilt the solver cannot see.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2


def main(argv=None):
    parser = argparse.ArgumentParser(description="Write a printable ArUco tag.")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--size", type=float, default=80.0, help="black square side, mm")
    parser.add_argument("--dpi", type=int, default=300, help="render resolution for printing")
    parser.add_argument("--border", type=float, default=10.0, help="white quiet zone, mm")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--out", default="tags", help="output directory")
    args = parser.parse_args(argv)

    if args.size <= 0 or args.border < 0:
        raise SystemExit("--size must be positive and --border not negative")

    mm_per_px = 25.4 / args.dpi
    side_px = int(round(args.size / mm_per_px))
    border_px = int(round(args.border / mm_per_px))

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    marker = dictionary.generateImageMarker(args.id, side_px)
    sheet = cv2.copyMakeBorder(marker, border_px, border_px, border_px, border_px,
                               cv2.BORDER_CONSTANT, value=255)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / ("tag_%s_%d_%03dmm.png" % (args.dictionary.lower(), args.id, round(args.size)))
    path.write_bytes(cv2.imencode(".png", sheet)[1].tobytes())

    print("tag        : id %d, %s" % (args.id, args.dictionary))
    print("black square: %.1f mm  (%d px at %d dpi)" % (args.size, side_px, args.dpi))
    print("quiet zone : %.1f mm on each edge" % args.border)
    print("sheet      : %.1f x %.1f mm" % (args.size + 2 * args.border, args.size + 2 * args.border))
    print("wrote      : %s" % path)
    print("print at 100 % scale, then measure the black square and pass that to --tag-size")
    return 0


if __name__ == "__main__":
    sys.exit(main())
