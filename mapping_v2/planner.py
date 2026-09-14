"""Deterministic A* planner used by preview and guarded route execution."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from .occupancy import OccupancyGrid, bresenham


@dataclass(frozen=True)
class PlanResult:
    ok: bool
    status: str
    message: str
    path_world: list[tuple[float, float]]
    length_m: float
    unknown_ratio: float
    expanded_nodes: int


def inflate_obstacles(states: np.ndarray, radius_cells: int) -> np.ndarray:
    occupied = np.argwhere(states >= 100)
    blocked = np.zeros_like(states, dtype=np.bool_)
    h, w = states.shape
    offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy <= radius_cells * radius_cells
    ]
    for gy, gx in occupied:
        for dx, dy in offsets:
            nx, ny = int(gx + dx), int(gy + dy)
            if 0 <= nx < w and 0 <= ny < h:
                blocked[ny, nx] = True
    return blocked


def _clear_start_footprint(
    states: np.ndarray,
    start: tuple[int, int],
    radius_cells: int,
    preserve: tuple[int, int] | None = None,
) -> np.ndarray:
    """Clear self-returns under the robot before obstacle inflation.

    A local LiDAR costmap can contain platform, chassis, or leg returns inside
    the robot's current footprint. Inflating those returns traps the start
    cell even though the robot already occupies that space. Only the current
    footprint is cleared; the selected goal and all external obstacles remain.
    """
    planning_states = states.copy()
    sx, sy = start
    h, w = planning_states.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            gx, gy = sx + dx, sy + dy
            if 0 <= gx < w and 0 <= gy < h and (gx, gy) != preserve:
                planning_states[gy, gx] = 0
    return planning_states


def _line_is_free(a: tuple[int, int], b: tuple[int, int], blocked: np.ndarray) -> bool:
    h, w = blocked.shape
    for x, y in bresenham(a[0], a[1], b[0], b[1]):
        if not (0 <= x < w and 0 <= y < h) or blocked[y, x]:
            return False
    return True


def _simplify(path: list[tuple[int, int]], blocked: np.ndarray) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        farthest = anchor + 1
        for candidate in range(anchor + 2, len(path)):
            if not _line_is_free(path[anchor], path[candidate], blocked):
                break
            farthest = candidate
        result.append(path[farthest])
        anchor = farthest
    return result


def plan_route(
    grid: OccupancyGrid,
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    footprint_radius_m: float = 0.38,
    unknown_cost: float = 3.5,
    max_expansions: int = 60000,
) -> PlanResult:
    states = grid.states()
    start, goal = grid.world_to_grid(*start_world), grid.world_to_grid(*goal_world)
    if not grid.in_bounds(*goal):
        return PlanResult(False, "GOAL_OUT_OF_MAP", "Hedef yerel haritanın dışında", [], 0.0, 1.0, 0)
    if not grid.in_bounds(*start):
        return PlanResult(False, "START_OUT_OF_MAP", "Robot pozu haritanın dışında", [], 0.0, 1.0, 0)

    radius_cells = max(1, int(math.ceil(footprint_radius_m / grid.config.resolution_m)))
    planning_states = _clear_start_footprint(states, start, radius_cells, preserve=goal)
    blocked = inflate_obstacles(planning_states, radius_cells)
    if blocked[goal[1], goal[0]]:
        return PlanResult(False, "GOAL_BLOCKED", "Hedef robot ayak izi güvenlik payıyla engel içinde", [], 0.0, 1.0, 0)

    neighbours = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    )
    frontier: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    best_cost = {start: 0.0}
    expanded = 0

    while frontier and expanded < max_expansions:
        _, cost, current = heapq.heappop(frontier)
        if cost != best_cost.get(current):
            continue
        expanded += 1
        if current == goal:
            break
        for dx, dy, step_cost in neighbours:
            nx, ny = current[0] + dx, current[1] + dy
            if not grid.in_bounds(nx, ny) or blocked[ny, nx]:
                continue
            if dx and dy and (blocked[current[1], nx] or blocked[ny, current[0]]):
                continue
            penalty = unknown_cost if states[ny, nx] < 0 else 1.0
            new_cost = cost + step_cost * penalty
            nxt = (nx, ny)
            if new_cost >= best_cost.get(nxt, math.inf):
                continue
            best_cost[nxt] = new_cost
            came_from[nxt] = current
            heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
            heapq.heappush(frontier, (new_cost + heuristic, new_cost, nxt))

    if goal not in best_cost:
        return PlanResult(False, "NO_ROUTE", "Güvenli rota bulunamadı", [], 0.0, 1.0, expanded)

    cells = [goal]
    while cells[-1] != start:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    cells = _simplify(cells, blocked)
    path_world = [grid.grid_to_world(x, y) for x, y in cells]
    length_m = sum(math.dist(a, b) for a, b in zip(path_world, path_world[1:]))
    unknown = sum(states[y, x] < 0 for x, y in cells)
    return PlanResult(
        True,
        "PLANNED",
        "Güvenli rota hesaplandı",
        path_world,
        round(length_m, 3),
        round(unknown / max(1, len(cells)), 3),
        expanded,
    )
