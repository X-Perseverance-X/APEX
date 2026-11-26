"""Visual assembly from the supplied body STLs and photographed hardware."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ASSET_DIR = Path(__file__).resolve().parent / "assets"
VISUAL_ONLY = {"mass": "0", "contype": "0", "conaffinity": "0", "group": "2"}
BODY_COLORS = {
    "gray": "0.39 0.43 0.47 1",
    "dark_gray": "0.16 0.18 0.20 1",
    "black": "0.065 0.07 0.08 1",
}


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


def box(torso, name, pos, size, rgba):
    ET.SubElement(
        torso, "geom", name=name, type="box", pos=fmt(pos), size=fmt(size), rgba=rgba, **VISUAL_ONLY
    )


def cylinder(torso, name, pos, size, rgba):
    ET.SubElement(
        torso,
        "geom",
        name=name,
        type="cylinder",
        pos=fmt(pos),
        size=fmt(size),
        euler="0 1.570796327 0",
        rgba=rgba,
        **VISUAL_ONLY,
    )


def add_body_hardware(torso):
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
        ("speaker_basket", (0.141, 0, -0.025), (0.006, 0.042, 0.022), "0.04 0.05 0.06 1"),
        ("speaker_amp_pcb", (0.113, 0, -0.025), (0.001, 0.025, 0.018), "0.07 0.20 0.12 1"),
        ("speaker_amp_heatsink", (0.110, 0.005, -0.022), (0.002, 0.010, 0.008), "0.11 0.13 0.15 1"),
        (
            "rear_roof_extension",
            (-0.131, 0, 0.0865),
            (0.019, 0.040, 0.0015),
            BODY_COLORS["gray"],
        ),
        ("fan_left_frame", (0.116, 0.028, 0.043), (0.0125, 0.002, 0.030), BODY_COLORS["black"]),
        ("fan_right_frame", (0.116, -0.028, 0.043), (0.0125, 0.002, 0.030), BODY_COLORS["black"]),
        ("fan_upper_frame", (0.116, 0, 0.071), (0.0125, 0.026, 0.002), BODY_COLORS["black"]),
        ("fan_lower_frame", (0.116, 0, 0.015), (0.0125, 0.026, 0.002), BODY_COLORS["black"]),
        ("fan_front_spacer", (0.144, 0, 0.070), (0.006, 0.039, 0.005), BODY_COLORS["black"]),
        (
            "fan_spacer_left_rail",
            (0.144, 0.034, 0.042),
            (0.006, 0.005, 0.027),
            BODY_COLORS["black"],
        ),
        (
            "fan_spacer_right_rail",
            (0.144, -0.034, 0.042),
            (0.006, 0.005, 0.027),
            BODY_COLORS["black"],
        ),
        ("led_bar_body", (0.187, 0, 0.0745), (0.004, 0.041, 0.0075), BODY_COLORS["black"]),
        ("led_bar_diffuser", (0.192, 0, 0.0745), (0.0018, 0.0345, 0.006), "0.51 0.55 0.57 1"),
    ):
        box(torso, name, pos, size, color)
    for i, y in enumerate((-0.027, -0.0162, -0.0054, 0.0054, 0.0162, 0.027)):
        box(
            torso,
            f"front_light_led_{i}",
            (0.194, y, 0.0745),
            (0.0004, 0.0021, 0.0016),
            "0.85 0.79 0.42 1",
        )
    for y in (-0.0375, 0.0375):
        side = "left" if y > 0 else "right"
        cylinder(
            torso,
            f"front_light_{side}_screw",
            (0.191, y, 0.0745),
            (0.0021, 0.0007),
            "0.50 0.52 0.54 1",
        )
    cylinder(torso, "rear_power_bezel", (-0.151, 0, 0.062), (0.0105, 0.0014), "0.59 0.61 0.63 1")
    cylinder(torso, "rear_power_button", (-0.1526, 0, 0.062), (0.007, 0.0008), "0.39 0.44 0.48 1")
    cylinder(torso, "fan_hub", (0.116, 0, 0.043), (0.010, 0.008), "0.10 0.11 0.12 1")
    for y in (-0.018, 0.018):
        side = "left" if y > 0 else "right"
        cylinder(
            torso, f"speaker_{side}_cone", (0.148, y, -0.025), (0.016, 0.0013), "0.09 0.09 0.10 1"
        )
    for i in range(7):
        angle = 2 * math.pi * i / 7
        y, z = 0.017 * math.cos(angle), 0.043 + 0.017 * math.sin(angle)
        ET.SubElement(
            torso,
            "geom",
            name=f"fan_blade_{i}",
            type="ellipsoid",
            pos=fmt([0.116, y, z]),
            euler=fmt([angle + 0.25, 0, 0.3]),
            size="0.002 0.009 0.004",
            rgba="0.12 0.14 0.15 1",
            **VISUAL_ONLY,
        )
