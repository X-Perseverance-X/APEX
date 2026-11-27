"""Geometry of the photographed stair flight, with explicit assumed dimensions."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from .body_visual import fmt


@dataclass(frozen=True)
class HomeStairGeometry:
    first_tread_m: float = 0.30
    tread_m: float = 0.27
    width_m: float = 0.90  # Assumed from the photographs, pending a tape measurement.
    corner: bool = False

    def edges(self, start: float, count: int, descending: bool) -> tuple[float, ...]:
        gaps = [self.first_tread_m] + [self.tread_m] * max(0, count - 2)
        if descending:
            gaps.reverse()
        return tuple(start + sum(gaps[:i]) for i in range(count))

    def add_corner(self, root, start_x: float, start_z: float, rise: float) -> None:
        """Add three solid 30-degree left winders to the static preview."""
        pivot = (start_x, self.width_m / 2)
        inner, outer = 0.03, self.width_m + 0.03
        for i in range(3):
            lower, upper = [math.radians(-90 + 30 * a) for a in (i, i + 1)]
            outline = [
                [pivot[0] + r * math.cos(a), pivot[1] + r * math.sin(a)]
                for r, a in ((inner, lower), (outer, lower), (outer, upper), (inner, upper))
            ]
            top = start_z + (i + 1) * rise
            vertices = [(*point, z) for z in (0.0, top) for point in outline]
            faces = (
                (0, 2, 1),
                (0, 3, 2),
                (4, 5, 6),
                (4, 6, 7),
                (0, 1, 5),
                (0, 5, 4),
                (1, 2, 6),
                (1, 6, 5),
                (2, 3, 7),
                (2, 7, 6),
                (3, 0, 4),
                (3, 4, 7),
            )
            name = f"winder_{i + 1}"
            ET.SubElement(
                root.find("asset"),
                "mesh",
                name=name,
                vertex=fmt([v for vertex in vertices for v in vertex]),
                face=" ".join(str(v) for face in faces for v in face),
            )
            ET.SubElement(
                root.find("worldbody"),
                "geom",
                name=name,
                type="mesh",
                mesh=name,
                rgba="0.36 0.20 0.12 1",
            )
