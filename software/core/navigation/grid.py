"""Occupancy grid with obstacle inflation and Theta*/A* planning.

Theta* is A* over an 8-connected grid with any-angle parent shortcutting
(line-of-sight check, no corner cutting). Falls back to plain grid A* by
construction (whenever the shortcut is blocked, the normal edge is used).

File:    software/core/navigation/grid.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import heapq
import itertools
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .polygon import distance_to_polygon

Point = Tuple[float, float]


class NoPathError(Exception):
    """No collision-free route exists between the requested points."""


class OccupancyGrid:
    """Rasterized play area; cells are blocked when covered by an inflated
    obstacle. Coordinates are millimetres, origin at the field corner."""

    def __init__(
        self,
        width_mm: float,
        height_mm: float,
        res_mm: float = 50.0,
        inflate_mm: float = 180.0,
        border_mm: float = 50.0,
    ) -> None:
        """Create the grid for a field with the given cell size."""
        self.width = float(width_mm)
        self.height = float(height_mm)
        self.res = max(10.0, float(res_mm))
        self.inflate = max(0.0, float(inflate_mm))
        self.border = max(0.0, min(float(border_mm), min(self.width, self.height) * 0.49))
        self.cols = max(1, int(math.ceil(self.width / self.res)))
        self.rows = max(1, int(math.ceil(self.height / self.res)))
        self.blocked = bytearray(self.cols * self.rows)
        # Field border: keep the robot body inside the play area
        self._border()

    # ------------------------------------------------------------ building
    def _border(self) -> None:
        """Block every cell whose centre is closer than ``border`` (mm) to an
        edge - the clearance is the robot's turning radius (half-diagonal)."""
        if self.border <= 0.0:
            return
        for row in range(self.rows):
            y = (row + 0.5) * self.res
            for col in range(self.cols):
                x = (col + 0.5) * self.res
                if (
                    x < self.border
                    or y < self.border
                    or x > self.width - self.border
                    or y > self.height - self.border
                ):
                    self.blocked[row * self.cols + col] = 1

    def add_rect(self, x: float, y: float, w: float, h: float) -> None:
        """Block a rectangle (position = corner) grown by the inflation radius.

        The distance test is Euclidean (Minkowski): the plain axis-aligned
        expansion wrongly blocked the corner cells up to a factor sqrt(2)
        beyond the configured clearance, which pushed paths far around boxes.
        """
        pad = self.inflate
        c0 = int((x - pad) / self.res) - 1
        c1 = int(math.ceil((x + w + pad) / self.res)) + 1
        r0 = int((y - pad) / self.res) - 1
        r1 = int(math.ceil((y + h + pad) / self.res)) + 1
        x1, y1 = x + w, y + h
        pad2 = pad * pad
        for row in range(max(0, r0), min(self.rows, r1 + 1)):
            for col in range(max(0, c0), min(self.cols, c1 + 1)):
                cx, cy = self.cell_center(col, row)
                dx = max(x - cx, 0.0, cx - x1)
                dy = max(y - cy, 0.0, cy - y1)
                if dx * dx + dy * dy <= pad2:
                    self.blocked[row * self.cols + col] = 1

    def add_circle(self, x: float, y: float, r: float) -> None:
        """Block a circle grown by the inflation radius."""
        rr = r + self.inflate
        c0 = int((x - rr) / self.res)
        c1 = int(math.ceil((x + rr) / self.res))
        r0 = int((y - rr) / self.res)
        r1 = int(math.ceil((y + rr) / self.res))
        rr2 = rr * rr
        for row in range(max(0, r0), min(self.rows, r1 + 1)):
            for col in range(max(0, c0), min(self.cols, c1 + 1)):
                cx, cy = self.cell_center(col, row)
                if (cx - x) ** 2 + (cy - y) ** 2 <= rr2:
                    self.blocked[row * self.cols + col] = 1

    def add_polygon(self, points: Sequence[Point]) -> None:
        """Block a custom shape grown by the inflation radius.

        Cell centres are tested against the real outline (Euclidean, inside
        test included), so a triangle or any hand-drawn shape blocks exactly
        what it covers plus the clearance.
        """
        pts = [(float(p[0]), float(p[1])) for p in points]
        if len(pts) < 3:
            return
        pad = self.inflate
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        c0 = int((min(xs) - pad) / self.res) - 1
        c1 = int(math.ceil((max(xs) + pad) / self.res)) + 1
        r0 = int((min(ys) - pad) / self.res) - 1
        r1 = int(math.ceil((max(ys) + pad) / self.res)) + 1
        for row in range(max(0, r0), min(self.rows, r1 + 1)):
            for col in range(max(0, c0), min(self.cols, c1 + 1)):
                cx, cy = self.cell_center(col, row)
                if distance_to_polygon(cx, cy, pts) <= pad:
                    self.blocked[row * self.cols + col] = 1

    # ------------------------------------------------------------- queries
    def cell_center(self, col: int, row: int) -> Point:
        """Millimetre centre of cell (cx, cy)."""
        return ((col + 0.5) * self.res, (row + 0.5) * self.res)

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Cell index holding the point (x, y) in mm."""
        col = min(self.cols - 1, max(0, int(x / self.res)))
        row = min(self.rows - 1, max(0, int(y / self.res)))
        return col, row

    def is_free_cell(self, col: int, row: int) -> bool:
        """True when the cell is inside the field and not blocked."""
        return self.blocked[row * self.cols + col] == 0

    def is_free_mm(self, x: float, y: float) -> bool:
        """True when the point (x, y) in mm is traversable."""
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        col, row = self.to_cell(x, y)
        return self.is_free_cell(col, row)

    def los(self, a: Point, b: Point) -> bool:
        """Conservative line-of-sight: sample the segment every res/3 mm.

        The bounds/``to_cell``/``is_free_cell`` helpers are inlined: Theta*
        calls this once per neighbour expansion and the method-call overhead
        was the planner's single biggest cost (~0.5 s of a Pi re-plan).
        """
        dx = float(b[0]) - float(a[0])
        dy = float(b[1]) - float(a[1])
        steps = max(2, int(math.hypot(dx, dy) / (self.res / 3.0)))
        cols, rows = self.cols, self.rows
        blocked = self.blocked
        width, height, res = self.width, self.height, self.res
        ax, ay = float(a[0]), float(a[1])
        for i in range(steps + 1):
            x = ax + dx * (i / steps)
            y = ay + dy * (i / steps)
            if x < 0.0 or y < 0.0 or x >= width or y >= height:
                return False
            col = int(x / res)
            if col >= cols:
                col = cols - 1
            row = int(y / res)
            if row >= rows:
                row = rows - 1
            if blocked[row * cols + col]:
                return False
        return True

    def nearest_free(self, x: float, y: float, max_radius_mm: float = 500.0) -> Point:
        """Closest free cell centre (handles start/goal inside an inflated zone).

        The Chebyshev rings are scanned for neighbourhood correctness, but the
        FREE cell **nearest to the requested point** wins: returning the first
        free cell in scan order snapped up to a ring-diagonal away from the
        closest one (a waypoint near the field border used to snap ~175 mm off
        along the border, which bent the approach leg sideways).
        """
        if self.is_free_mm(x, y):
            return (x, y)
        col0, row0 = self.to_cell(x, y)
        max_ring = int(max_radius_mm / self.res) + 1
        best: Optional[Tuple[float, Point]] = None

        def consider(col: int, row: int) -> None:
            """Offer a neighbour to the open set (Theta* relaxation)."""
            nonlocal best
            if not (0 <= col < self.cols and 0 <= row < self.rows):
                return
            if not self.is_free_cell(col, row):
                return
            centre = self.cell_center(col, row)
            distance = math.dist(centre, (x, y))
            if best is None or distance < best[0]:
                best = (distance, centre)

        for ring in range(1, max_ring):
            if best is not None and best[0] <= (ring + 0.5) * self.res:
                break  # no cell in a farther ring can be closer
            for dcol in range(-ring, ring + 1):
                for drow in (-ring, ring):
                    consider(col0 + dcol, row0 + drow)
            for drow in range(-ring + 1, ring):
                for dcol in (-ring, ring):
                    consider(col0 + dcol, row0 + drow)
        if best is None:
            raise NoPathError("no free cell near the requested point")
        return best[1]


_NEIGHBORS = [
    (-1, -1), (0, -1), (1, -1),
    (-1, 0), (1, 0),
    (-1, 1), (0, 1), (1, 1),
]


def theta_star(grid: OccupancyGrid, start_mm: Point, goal_mm: Point, max_expansions: int = 200_000) -> List[Point]:
    """Any-angle route between two millimetre points (Theta*)."""
    start = grid.nearest_free(*start_mm)
    goal = grid.nearest_free(*goal_mm)
    start_cell = grid.to_cell(*start)
    goal_cell = grid.to_cell(*goal)

    def cell_point(cell: Tuple[int, int]) -> Point:
        """Millimetre coordinates of a cell's centre."""
        return grid.cell_center(*cell)

    counter = itertools.count()
    open_heap: List[Tuple[float, int, Tuple[int, int]]] = [(0.0, next(counter), start_cell)]
    g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start_cell: None}
    closed: set = set()
    expansions = 0

    def heuristic(cell: Tuple[int, int]) -> float:
        """Octile distance heuristic between two cells."""
        dx = abs(cell[0] - goal_cell[0]) * grid.res
        dy = abs(cell[1] - goal_cell[1]) * grid.res
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)  # octile

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1
        if expansions > max_expansions:
            raise NoPathError("exploration limit exceeded")
        if current == goal_cell:
            break
        current_pt = cell_point(current)
        parent_cell = parent[current]
        parent_pt = cell_point(parent_cell) if parent_cell is not None else None
        for dcol, drow in _NEIGHBORS:
            col, row = current[0] + dcol, current[1] + drow
            if not (0 <= col < grid.cols and 0 <= row < grid.rows):
                continue
            neighbor = (col, row)
            if neighbor in closed or not grid.is_free_cell(col, row):
                continue
            neighbor_pt = cell_point(neighbor)
            if parent_pt is not None and grid.los(parent_pt, neighbor_pt):
                tentative = g_score[parent_cell] + math.dist(parent_pt, neighbor_pt)  # type: ignore[index]
                new_parent = parent_cell
            else:
                tentative = g_score[current] + math.dist(current_pt, neighbor_pt)
                new_parent = current
            if tentative < g_score.get(neighbor, math.inf) - 1e-9:
                g_score[neighbor] = tentative
                parent[neighbor] = new_parent
                heapq.heappush(open_heap, (tentative + heuristic(neighbor), next(counter), neighbor))

    if goal_cell not in parent:
        raise NoPathError("goal unreachable")

    cells: List[Tuple[int, int]] = []
    cell: Optional[Tuple[int, int]] = goal_cell
    while cell is not None:
        cells.append(cell)
        cell = parent[cell]
    cells.reverse()
    path = [cell_point(c) for c in cells]
    # Replace endpoints by the exact requested coordinates when they are free
    if grid.is_free_mm(*start_mm):
        path[0] = start_mm
    if grid.is_free_mm(*goal_mm):
        path[-1] = goal_mm
    return path
