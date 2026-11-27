"""The full photographed home staircase: lower flight, a left winder turn, upper flight.

Simulation only. Measured by the owner: the first step is 20 cm high with a
30 cm tread, the following straight treads are about 27 cm deep. Everything
else is estimated from the photographs and is a parameter, so it can be
replaced by tape measurements without touching the controller:

* every riser 20 cm (only the first was measured),
* 90 cm clear width between the wall (right going up) and the banister (left),
* five straight steps in the lower flight, three 30 degree winders turning left
  around the banister newel, six straight steps in the upper flight to the
  landing (14 risers, 2.80 m floor to floor),
* a 2 cm tread nosing over a 3 cm thick tread board.

The existing ``home_stairs.py`` courses are left unchanged: recorded policies are
bound to that source file. This module only adds new course names.
"""

from __future__ import annotations

import dataclasses
import math
import xml.etree.ElementTree as ET
from bisect import bisect_right
from dataclasses import dataclass

from .body_visual import fmt
from .locomotion import COURSES, Course

WOOD = "0.40 0.22 0.13 1"
WOOD_DARK = "0.33 0.17 0.10 1"
WALL = "0.86 0.84 0.82 1"
FRICTION = "1 0.005 0.0001"


@dataclass(frozen=True)
class HomeFlight:
    rise_m: float = 0.20
    first_tread_m: float = 0.30
    tread_m: float = 0.27
    width_m: float = 0.90
    lower_steps: int = 5
    winders: int = 3
    upper_steps: int = 6
    nosing_m: float = 0.02
    board_m: float = 0.03
    landing_m: float = 1.00

    def lower_edges(self, start: float) -> tuple[float, ...]:
        """Nosing front edge of every straight step in the lower flight."""
        gaps = [self.first_tread_m] + [self.tread_m] * (self.lower_steps - 1)
        return tuple(start + sum(gaps[:i]) for i in range(self.lower_steps))

    def turn_x(self, start: float) -> float:
        """Front of the winder square (back of the last straight lower tread)."""
        edges = self.lower_edges(start)
        return edges[-1] + self.tread_m

    @property
    def total_rise(self) -> float:
        return self.rise_m * (self.lower_steps + self.winders + self.upper_steps)


def _winder_polygon(pivot, half_width, x0, a0, a1):
    """Wedge between two rays from the inner pivot, clipped to the turn square."""
    px, py = pivot
    x1 = x0 + 2 * half_width
    y0 = -half_width

    def hit(angle):
        dx, dy = math.cos(angle), math.sin(angle)
        ts = []
        if dx > 1e-9:
            ts.append((x1 - px) / dx)
        if dy < -1e-9:
            ts.append((y0 - py) / dy)
        t = min(ts)
        return (px + t * dx, py + t * dy)

    corner = (x1, y0)
    corner_angle = math.atan2(y0 - py, x1 - px)
    points = [pivot, hit(a0)]
    if a0 < corner_angle < a1:
        points.append(corner)
    points.append(hit(a1))
    return points


@dataclass(frozen=True)
class HomeFlightCourse(Course):
    """Climb (or view) the full staircase; the goal lies past ``goal_steps`` straight risers."""

    flight: HomeFlight = HomeFlight()
    goal_steps: int = 5

    @property
    def step_edges(self) -> tuple[float, ...]:
        return self.flight.lower_edges(self.edge_x)[: self.goal_steps]

    @property
    def last_edge(self) -> float:
        return self.step_edges[-1]

    @property
    def goal(self) -> float:
        # Every sole beyond the last counted nosing, the body centre over its tread.
        edges = self.flight.lower_edges(self.edge_x)
        if self.goal_steps < len(edges):
            return edges[self.goal_steps - 1] + 0.5 * (
                edges[self.goal_steps] - edges[self.goal_steps - 1]
            )
        return self.flight.turn_x(self.edge_x) - 0.02

    @property
    def terrain_names(self) -> tuple[str, ...]:
        f = self.flight
        return (
            *(f"course_step_{i + 1}" for i in range(f.lower_steps)),
            *(f"course_nosing_{i + 1}" for i in range(f.lower_steps)),
            *(f"course_winder_{i + 1}" for i in range(f.winders)),
            *(f"course_upper_{i + 1}" for i in range(f.upper_steps)),
            "course_wall_lower",
            "course_wall_turn",
        )

    def height(self, x: float, y: float = 0.0) -> float:
        f = self.flight
        half = f.width_m / 2
        if abs(y) > half or x < -1.25:
            if (
                y > half
                and self.flight.turn_x(self.edge_x)
                <= x
                <= self.flight.turn_x(self.edge_x) + f.width_m
            ):
                return self._upper_height(y)
            return 0.0
        turn = f.turn_x(self.edge_x)
        if x < turn:
            level = bisect_right(f.lower_edges(self.edge_x), x + 1e-9)
            return f.rise_m * level
        if x <= turn + f.width_m:
            angle = math.atan2(y - half, x - turn)  # -90 deg at the lower flight, 0 at the upper
            share = (angle + math.pi / 2) / (math.pi / 2)
            index = min(f.winders - 1, max(0, int(share * f.winders)))
            return f.rise_m * (f.lower_steps + index + 1)
        return 0.0

    def _upper_height(self, y: float) -> float:
        f = self.flight
        start = f.width_m / 2
        level = min(f.upper_steps, 1 + max(0, math.floor((y - start) / f.tread_m + 1e-9)))
        return f.rise_m * (f.lower_steps + f.winders + level)

    def scene(self, config: dict) -> str:
        # The base scene adds the robot, the chassis contact envelopes and the
        # goal marker; with no risers it adds no steps of its own.
        base = Course.scene(dataclasses.replace(self, risers=0), config)
        root = ET.fromstring(base)
        world = root.find("worldbody")
        asset = root.find("asset")
        f = self.flight
        half = f.width_m / 2
        edges = f.lower_edges(self.edge_x)
        turn = f.turn_x(self.edge_x)

        def box(name, lo, hi, rgba=WOOD, collide=True):
            centre = [(a + b) / 2 for a, b in zip(lo, hi)]
            size = [(b - a) / 2 for a, b in zip(lo, hi)]
            extra = {} if collide else {"contype": "0", "conaffinity": "0"}
            ET.SubElement(
                world, "geom", name=name, type="box", pos=fmt(centre), size=fmt(size),
                friction=FRICTION, rgba=rgba, **extra,
            )  # fmt: skip

        # Lower flight: a solid block behind each riser face, plus the nosing board.
        for i, edge in enumerate(edges):
            top = f.rise_m * (i + 1)
            back = edges[i + 1] if i + 1 < len(edges) else turn
            box(f"course_step_{i + 1}", (edge + f.nosing_m, -half, 0.0), (back, half, top))
            box(
                f"course_nosing_{i + 1}",
                (edge, -half, top - f.board_m),
                (edge + f.nosing_m, half, top),
                WOOD_DARK,
            )
        # Winders: wedge prisms around the inner (banister) corner, turning left.
        pivot = (turn, half)
        for i in range(f.winders):
            a0 = -math.pi / 2 + i * (math.pi / 2) / f.winders
            a1 = a0 + (math.pi / 2) / f.winders
            outline = _winder_polygon(pivot, half, turn, a0, a1)
            top = f.rise_m * (f.lower_steps + i + 1)
            vertices = [(px, py, z) for z in (0.0, top) for px, py in outline]
            n = len(outline)
            faces = []
            for k in range(1, n - 1):
                faces += [(0, k + 1, k), (n, n + k, n + k + 1)]
            for k in range(n):
                j = (k + 1) % n
                faces += [(k, j, n + j), (k, n + j, n + k)]
            name = f"course_winder_{i + 1}"
            ET.SubElement(
                asset, "mesh", name=name,
                vertex=fmt([v for p in vertices for v in p]),
                face=" ".join(str(v) for face in faces for v in face),
            )  # fmt: skip
            ET.SubElement(
                world, "geom", name=name, type="mesh", mesh=name, friction=FRICTION,
                rgba=WOOD if i % 2 == 0 else WOOD_DARK,
            )  # fmt: skip
        # Upper flight runs towards +y over the turn square's width.
        for j in range(f.upper_steps):
            start = half + j * f.tread_m
            end = start + (f.tread_m if j < f.upper_steps - 1 else f.landing_m)
            top = f.rise_m * (f.lower_steps + f.winders + j + 1)
            box(f"course_upper_{j + 1}", (turn, start, 0.0), (turn + f.width_m, end, top))
        # Wall on the right going up, wrapping round the outside of the turn.
        height = f.total_rise + 1.0
        box(
            "course_wall_lower",
            (self.edge_x - 1.2, -half - 0.10, 0.0),
            (turn + f.width_m, -half, height),
            WALL,
        )
        box(
            "course_wall_turn",
            (turn + f.width_m, -half - 0.10, 0.0),
            (turn + f.width_m + 0.10, half + (f.upper_steps - 1) * f.tread_m + f.landing_m, height),
            WALL,
        )

        # Banister on the inner (left) side: balusters about 12 cm apart.
        def baluster(name, x, y, z0):
            ET.SubElement(
                world, "geom", name=name, type="cylinder",
                pos=fmt([x, y, z0 + 0.45]), size="0.018 0.45", rgba=WOOD_DARK,
            )  # fmt: skip

        k = 0
        for i, edge in enumerate(edges):
            back = edges[i + 1] if i + 1 < len(edges) else turn
            for x in (edge + 0.06, (edge + back) / 2 + 0.06):
                if x < back - 0.02:
                    k += 1
                    baluster(f"banister_{k}", x, half + 0.02, f.rise_m * (i + 1))
        ET.SubElement(
            world, "geom", name="banister_newel", type="box",
            pos=fmt([turn, half + 0.02, (f.rise_m * f.lower_steps + 1.0) / 2]),
            size=fmt([0.045, 0.045, (f.rise_m * f.lower_steps + 1.0) / 2]), rgba=WOOD_DARK,
        )  # fmt: skip
        for j in range(f.upper_steps - 1):
            for y in (half + j * f.tread_m + 0.07, half + j * f.tread_m + 0.20):
                k += 1
                baluster(
                    f"banister_{k}", turn - 0.02, y, f.rise_m * (f.lower_steps + f.winders + j + 1)
                )
        return ET.tostring(root, encoding="unicode")


HOME_FLIGHT = HomeFlight()
_COMMON = {
    "edge_x": 1.05,
    "link_contacts": True,
    "attitude_control": True,
    "steep_approach": True,
    "ledge_transfer": True,
    "fore_probe": True,
    "body_support": True,
}
# Full staircase to look at with ``course-view --static``; plain course flags so it
# opens without --body-control. The robot stands on the floor before the first riser.
COURSES["home-flight-preview"] = HomeFlightCourse(
    0.20, 5, edge_x=1.05, flight=HOME_FLIGHT, goal_steps=5
)
# Climb the first two risers (30 cm then 27 cm tread) and the whole lower flight.
for _n in (1, 2, 5):
    COURSES[f"home-flight-up-{_n}"] = HomeFlightCourse(
        0.20, _n, flight=HOME_FLIGHT, goal_steps=_n, **_COMMON
    )
