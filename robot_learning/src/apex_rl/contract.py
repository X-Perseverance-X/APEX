"""Versioned, unit-checked geometry contract for the offline twin."""

from __future__ import annotations

import json
import math
from importlib.resources import files

LEG_ORDER = ("RF", "RM", "RR", "LF", "LM", "LR")


def load_contract() -> dict:
    path = files("apex_rl").joinpath("assets/meta1.json")
    data = json.loads(path.read_text(encoding="utf-8"))
    validate_contract(data)
    return data


def validate_contract(data: dict) -> None:
    if data.get("schema") != 1 or data.get("frame") != "X_FORWARD_Y_LEFT_Z_UP":
        raise ValueError("unsupported META-1 geometry schema or coordinate frame")
    if data.get("units") != "m_kg_rad_s" or tuple(data.get("leg_order", ())) != LEG_ORDER:
        raise ValueError("unexpected units or leg order")
    links = data["links_m"]
    if not all(0.04 < float(links[name]) < 0.25 for name in ("coxa", "femur", "tibia")):
        raise ValueError("implausible leg link length")
    for leg in LEG_ORDER:
        xyz = data["hip_xyz_m"][leg]
        if len(xyz) != 3 or any(not math.isfinite(float(v)) for v in xyz):
            raise ValueError(f"invalid hip position: {leg}")
        if (leg[0] == "R") != (xyz[1] < 0):
            raise ValueError(f"mirrored Y sign is wrong: {leg}")
        if not -math.pi / 2 < float(data["stand_coxa_rad"][leg]) < math.pi / 2:
            raise ValueError(f"invalid stand coxa angle: {leg}")
    if data["body"]["status"] != "provisional" or data["actuator"]["status"] != "provisional":
        raise ValueError("unmeasured dynamics must remain explicitly provisional")
