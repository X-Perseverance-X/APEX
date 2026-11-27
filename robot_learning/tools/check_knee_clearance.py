"""Check the MARK_2 knee crossbar against the moving tibia plate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from import_mark2_leg import ASSETS, STL_TRIANGLE


def section(triangles, y):
    ends = triangles[:, [1, 2, 0], :]
    crossing = (triangles[:, :, 1] < y) != (ends[:, :, 1] < y)
    start, end = triangles[crossing], ends[crossing]
    t = (y - start[:, 1]) / (end[:, 1] - start[:, 1])
    return (start + t[:, None] * (end - start))[:, [0, 2]].reshape(-1, 2, 2)


def inside(x, z, segments):
    result = np.zeros(len(x), dtype=bool)
    for (ax, az), (bx, bz) in segments:
        if bz != az:
            result ^= ((az <= z) != (bz <= z)) & (x < ax + (z - az) * (bx - ax) / (bz - az))
    return result


def read_component(path):
    # Generated link-local meshes already use metres.
    return np.frombuffer(path.read_bytes(), dtype=STL_TRIANGLE, offset=84)["vertices"].astype(float)


def scan(components, spacing):
    crossbar = read_component(components / "femur_Tibia_Holder_3-1.stl")
    plate = read_component(components / "tibia_Tibia_Head-1.stl")
    config = json.loads((ASSETS.parent / "meta1.json").read_text(encoding="utf-8"))
    crossbar[:, :, 0] -= config["links_m"]["femur"]
    lower, upper = crossbar.min(axis=(0, 1)), crossbar.max(axis=(0, 1))
    lower[1] = max(lower[1], plate[:, :, 1].min())
    upper[1] = min(upper[1], plate[:, :, 1].max())
    grid = [np.arange(lo + spacing / 2, hi, spacing) for lo, hi in zip(lower, upper, strict=True)]
    x, z = [v.ravel() for v in np.meshgrid(grid[0], grid[2], indexing="ij")]
    sections = []
    for y in grid[1]:
        occupied = inside(x, z, section(crossbar, y))
        sections.append((x[occupied], z[occupied], section(plate, y)))
    result = {}
    for degrees in range(-175, -4):
        c, s = np.cos(np.deg2rad(degrees)), np.sin(np.deg2rad(degrees))
        result[degrees] = sum(
            int(inside(c * x + s * z, -s * x + c * z, edges).sum()) for x, z, edges in sections
        )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("components", type=Path)
    args = parser.parse_args()
    samples = scan(args.components, spacing=0.001)
    print(json.dumps({"spacing_mm": 1, "overlap_samples": samples}, indent=2))
