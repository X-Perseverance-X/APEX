"""Body attitude and MARK_2 knee clearance for offline ascent."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from .contract import LEG_ORDER

# Section checks of MARK_2_LEG place the tibia against Tibia_Holder_3
# between -153 and -156 degrees. Leave room for assembly and solver error.
KNEE_STOP = math.radians(-150)
KNEE_COMMAND_MIN = math.radians(-149)
KNEE_CLEARANCE_LIMIT = math.radians(-153)


def apply_knee_stops(xml: str) -> str:
    root = ET.fromstring(xml)
    for leg in LEG_ORDER:
        prefix = leg.lower()
        joint = root.find(f".//joint[@name='{prefix}_tibia_joint']")
        servo = root.find(f"actuator/position[@name='{prefix}_tibia_servo']")
        for element, attribute, minimum in (
            (joint, "range", KNEE_STOP),
            (servo, "ctrlrange", KNEE_COMMAND_MIN),
        ):
            upper = element.get(attribute).split()[1]
            element.set(attribute, f"{minimum:.12g} {upper}")
    return ET.tostring(root, encoding="unicode")


def attitude_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return body-to-world rotation, with positive pitch lifting the nose."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, -sy * cr - cy * sp * sr, sy * sr - cy * sp * cr],
            [sy * cp, cy * cr - sy * sp * sr, -cy * sr - sy * sp * cr],
            [sp, cp * sr, cp * cr],
        ]
    )
