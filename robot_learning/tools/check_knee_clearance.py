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


def scan(components, spacing, angles=range(-175, -4), moving_part="Tibia_Head"):
    crossbar = read_component(components / "femur_Tibia_Holder_3-1.stl")
    plate = read_component(components / f"tibia_{moving_part}-1.stl")
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
    for degrees in angles:
        c, s = np.cos(np.deg2rad(degrees)), np.sin(np.deg2rad(degrees))
        result[float(degrees)] = sum(
            int(inside(c * x + s * z, -s * x + c * z, edges).sum()) for x, z, edges in sections
        )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("components", type=Path)
    parser.add_argument("--spacing-mm", type=float, default=1.0)
    parser.add_argument("--min-deg", type=float, default=-175.0)
    parser.add_argument("--max-deg", type=float, default=-5.0)
    parser.add_argument("--angle-step", type=float, default=1.0)
    parser.add_argument("--moving-part", choices=("Tibia_Head", "Tibia_1"), default="Tibia_Head")
    args = parser.parse_args()
    if args.spacing_mm <= 0 or args.angle_step <= 0 or args.max_deg < args.min_deg:
        parser.error(
            "spacing and angle step must be positive, and max-deg must be at least min-deg"
        )
    angles = np.arange(args.min_deg, args.max_deg + 1e-9, args.angle_step)
    samples = scan(args.components, args.spacing_mm / 1000, angles, args.moving_part)
    print(json.dumps({"spacing_mm": args.spacing_mm, "overlap_samples": samples}, indent=2))
