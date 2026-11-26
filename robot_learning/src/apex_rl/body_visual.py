"""Visual assembly from the supplied body STLs and photographed hardware."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ASSET_DIR = Path(__file__).resolve().parent / "assets"
VISUAL_ONLY = {"mass": "0", "contype": "0", "conaffinity": "0", "group": "2"}
BODY_COLORS = {"gray": "0.39 0.43 0.47 1", "dark_gray": "0.16 0.18 0.20 1"}


def fmt(values):
    return " ".join(f"{float(value):.9g}" for value in values)


def load_body_layout():
    return json.loads((ASSET_DIR / "body_layout.json").read_text(encoding="utf-8"))["parts"]


def place_body_cad(parent, mount, parts, anchors):
    for part in parts:
        if part["parent"] != mount:
            continue
        anchor = anchors[part["anchor"]] if "anchor" in part else (0, 0, 0)
        orientation = (
            {"xyaxes": fmt(part["xyaxes"])}
            if "xyaxes" in part
            else {"euler": fmt(map(math.radians, part.get("rpy", (0, 0, 0))))}
        )
        ET.SubElement(
            parent,
            "geom",
            name=part["name"],
            type="mesh",
            mesh=f"body_{part['mesh']}",
            pos=fmt(
                [base + offset * 0.001 for base, offset in zip(anchor, part["xyz"], strict=True)]
            ),
            rgba=BODY_COLORS[part["material"]],
            **orientation,
            **VISUAL_ONLY,
        )


def add_underbody_details(torso):
    for side, y in (("left", 0.011), ("right", -0.011)):
        ET.SubElement(
            torso,
            "geom",
            name=f"main_pack_{side}",
            type="cylinder",
            pos=fmt([-0.105, y, -0.013]),
            size="0.0105 0.0335",
            euler="0 1.570796327 0",
            rgba="0.12 0.27 0.52 1",
            **VISUAL_ONLY,
        )
    for name, pos, size, color in (
        ("middle_pack", (-0.019, 0, -0.011), (0.024, 0.021, 0.010), "0.38 0.39 0.40 1"),
        ("fan_board", (0.106, 0, -0.029), (0.0015, 0.023, 0.025), "0.075 0.19 0.12 1"),
        ("fan_left_frame", (0.129, 0.028, -0.031), (0.0125, 0.002, 0.030), "0.29 0.33 0.36 1"),
        ("fan_right_frame", (0.129, -0.028, -0.031), (0.0125, 0.002, 0.030), "0.29 0.33 0.36 1"),
        ("fan_upper_frame", (0.129, 0, -0.003), (0.0125, 0.026, 0.002), "0.29 0.33 0.36 1"),
        ("fan_lower_frame", (0.129, 0, -0.059), (0.0125, 0.026, 0.002), "0.29 0.33 0.36 1"),
    ):
        ET.SubElement(
            torso,
            "geom",
            name=name,
            type="box",
            pos=fmt(pos),
            size=fmt(size),
            rgba=color,
            **VISUAL_ONLY,
        )
    ET.SubElement(
        torso,
        "geom",
        name="fan_hub",
        type="cylinder",
        pos="0.129 0 -0.031",
        euler="0 1.570796327 0",
        size="0.010 0.008",
        rgba="0.10 0.11 0.12 1",
        **VISUAL_ONLY,
    )
    for i in range(7):
        angle = 2 * math.pi * i / 7
        y, z = 0.017 * math.cos(angle), -0.031 + 0.017 * math.sin(angle)
        ET.SubElement(
            torso,
            "geom",
            name=f"fan_blade_{i}",
            type="ellipsoid",
            pos=fmt([0.130, y, z]),
            euler=fmt([angle + 0.25, 0, 0.3]),
            size="0.002 0.009 0.004",
            rgba="0.12 0.14 0.15 1",
            **VISUAL_ONLY,
        )
