#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fetch the RS CAD visual meshes the whole-pose hillclimb renders.

One-off asset fetch (the STLs are ~37 MB and stay out of git): downloads every
visual mesh the URDF references from Seeed's reBotArmController_ROS2, and
writes visuals.json -- the per-link manifest identify_cad.CadModel reads.

    python examples/host/app/cad/fetch.py

Re-running is cheap: meshes already on disk are kept.
"""
import json
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

CAD = Path(__file__).resolve().parent
BASE = ("https://raw.githubusercontent.com/Seeed-Projects/reBotArmController_ROS2/"
        "main/src/rebotarm_bringup/description/RS/")


def origin(el):
    o = el.find("origin")
    if o is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    return xyz, rpy


def main():
    urdf = CAD / "ReBot_Arm_RS.urdf"
    if not urdf.exists():
        with urllib.request.urlopen(BASE + "urdf/ReBot_Arm_RS.urdf", timeout=60) as r:
            urdf.write_bytes(r.read())
    root = ET.parse(urdf).getroot()

    visuals, meshes = [], set()
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            rel = mesh.get("filename").replace(
                "package://rebotarm_bringup/description/RS/", "")
            xyz, rpy = origin(vis)
            visuals.append({"link": link.get("name"), "mesh": rel, "xyz": xyz, "rpy": rpy})
            meshes.add(rel)

    joints = []
    for j in root.findall("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints.append({
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0, 0, 0],
            "rpy": [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0],
            "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
            "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
        })

    mesh_dir = CAD / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    ok, failed = [], []
    for rel in sorted(meshes):
        dest = mesh_dir / Path(rel).name
        if dest.exists() and dest.stat().st_size > 84:
            ok.append(rel)
            continue
        try:
            with urllib.request.urlopen(BASE + rel, timeout=120) as r:
                data = r.read()
            dest.write_bytes(data)
            ok.append(rel)
            print("ok  %-60s %8d bytes" % (rel, len(data)))
        except Exception as e:
            failed.append((rel, str(e)))
            print("FAIL %-60s %s" % (rel, e))

    (CAD / "visuals.json").write_text(json.dumps(
        {"source": BASE, "visuals": visuals, "joints": joints,
         "mesh_dir": "meshes", "failed": failed}, indent=1))
    print("\n%d visuals, %d unique meshes, %d present, %d failed"
          % (len(visuals), len(meshes), len(ok), len(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())