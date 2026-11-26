"""Physical contact for the CAD links, separate from sole contact switches."""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET

import numpy as np

from .contract import LEG_ORDER

OVERLAP_LIMIT_M = 0.002


def leg_geoms(leg: str) -> tuple[str, ...]:
    prefix = leg.lower()
    return (
        f"{prefix}_coxa_cad",
        f"{prefix}_femur_cad",
        f"{prefix}_tibia_cad",
        f"{prefix}_foot_geom",
    )


def add_leg_contacts(xml: str, risers: int) -> str:
    root = ET.fromstring(xml)
    contact = ET.SubElement(root, "contact")
    geoms = {leg: leg_geoms(leg) for leg in LEG_ORDER}
    for first, second in itertools.combinations(LEG_ORDER, 2):
        if first[0] != second[0]:
            continue
        for a, b in itertools.product(geoms[first], geoms[second]):
            ET.SubElement(
                contact, "pair", geom1=a, geom2=b, condim="1", solref="0.009 1", margin="0.001"
            )
    terrain = ("floor", *(f"course_step_{i + 1}" for i in range(risers)))
    for leg in LEG_ORDER:
        for a, b in itertools.product(geoms[leg][:-1], terrain):
            ET.SubElement(
                contact,
                "pair",
                geom1=a,
                geom2=b,
                condim="3",
                solref="0.009 1",
                friction="0.42 0.42 0.003 0.0001 0.0001",
                margin="0.001",
            )
    return ET.tostring(root, encoding="unicode")


class LinkContactMetrics:
    def __init__(self, model, terrain_ids):
        self.leg = np.full(model.ngeom, -1, dtype=np.int8)
        self.rear_link = np.zeros(model.ngeom, dtype=bool)
        self.terrain = np.zeros(model.ngeom, dtype=bool)
        self.terrain[terrain_ids] = True
        for i, leg in enumerate(LEG_ORDER):
            ids = [model.geom(name).id for name in leg_geoms(leg)]
            self.leg[ids] = i
            self.rear_link[ids[:-1]] = leg.endswith("R")
        self.reset()

    def reset(self):
        self.interleg_peak_m = 0.0
        self.rear_step_peak_m = 0.0
        self.interleg_samples = 0
        self.rear_step_samples = 0

    def observe(self, contacts):
        a, b = contacts.geom.T
        if not len(a):
            return
        interleg = (self.leg[a] >= 0) & (self.leg[b] >= 0) & (self.leg[a] != self.leg[b])
        rear_step = (self.rear_link[a] & self.terrain[b]) | (self.rear_link[b] & self.terrain[a])
        depth = -contacts.dist
        self.interleg_peak_m = max(self.interleg_peak_m, float(depth[interleg].max(initial=0)))
        self.rear_step_peak_m = max(self.rear_step_peak_m, float(depth[rear_step].max(initial=0)))
        self.interleg_samples += int(np.any(depth[interleg] > OVERLAP_LIMIT_M))
        self.rear_step_samples += int(np.any(depth[rear_step] > OVERLAP_LIMIT_M))

    def info(self):
        return {
            "interleg_peak_m": self.interleg_peak_m,
            "rear_step_peak_m": self.rear_step_peak_m,
            "interleg_overlap_samples": self.interleg_samples,
            "rear_step_overlap_samples": self.rear_step_samples,
        }
