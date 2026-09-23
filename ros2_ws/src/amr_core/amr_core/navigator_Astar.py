"""
navigator.py

Pure mathematical engine for Sub-Phase 2: the Global Navigator (A*).
Knows nothing about pixels, canvases, or Tkinter. Given a 2D grid
(0 = free, 1 = obstacle), a start coordinate, and a target coordinate,
it returns the optimal list of (x, y) waypoints using an 8-directional
A* search backed by a Min-Heap priority queue.
"""

import heapq
import json
import math


class Node:
    """
    Represents a single cell in the search space.

    x, y   : grid coordinates
    g      : cost from start to this node
    h      : heuristic (estimated) cost from this node to the target
    f      : total cost = g + h
    parent : reference to the Node this one was reached from (for backtracking)
    """

    def __init__(self, x, y, g=0.0, h=0.0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        # Required so heapq can compare Nodes directly when pushed as
        # (f, tie_breaker, node) tuples is avoided by comparing f here.
        return self.f < other.f

    def __eq__(self, other):
        return isinstance(other, Node) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class AStarPlanner:
    """
    Encapsulates the A* search over a static grid.

    Usage:
        planner = AStarPlanner(grid)
        path = planner.find_path(start, target)
    """

    # 8-way movement offsets: N, S, E, W, NE, NW, SE, SW
    NEIGHBOR_OFFSETS = [
        (0, -1), (0, 1), (1, 0), (-1, 0),
        (1, -1), (-1, -1), (1, 1), (-1, 1)
    ]

    STRAIGHT_COST = 1.0
    DIAGONAL_COST = math.sqrt(2)
    TIE_BREAKER = 1.001

    def __init__(self, grid, resolution=1.0, origin=(0.0, 0.0, 0.0)):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.resolution = resolution
        self.origin = origin

    @classmethod
    def from_costmap(cls, path):
        """
        Builds a planner directly from a costmap.json file
        (keys: "resolution", "origin", "grid"). Missing metadata falls
        back to resolution=1.0 / origin=(0,0,0), which makes world and
        grid coordinates identical for a plain index-based grid.
        """
        with open(path, "r") as f:
            data = json.load(f)
        return cls(
            data["grid"],
            resolution=data.get("resolution", 1.0),
            origin=data.get("origin", (0.0, 0.0, 0.0)),
        )

    def world_to_grid(self, wx, wy):
        """Converts world-frame meters to (col, row) grid indices."""
        col = int(round((wx - self.origin[0]) / self.resolution))
        # row = int(round((wy - self.origin[1]) / self.resolution))
        # Calculate physical offset, then invert against total height
        raw_row = int(round((wy - self.origin[1]) / self.resolution))
        row = (self.rows - 1) - raw_row    
        return col, row

    def grid_to_world(self, col, row):
        """Converts (col, row) grid indices back to world-frame meters."""
        wx = self.origin[0] + col * self.resolution
        # wy = self.origin[1] + row * self.resolution
        # Invert the row index back to physical space
        wy = self.origin[1] + ((self.rows - 1 - row) * self.resolution)
        return wx, wy

    def _in_bounds(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def _is_walkable(self, x, y):
        return self.grid[y][x] == 0

    def is_valid_cell(self, x, y):
        """Public bounds+obstacle check for callers outside this class."""
        return self._in_bounds(x, y) and self._is_walkable(x, y)

    def _heuristic(self, x, y, target_x, target_y):
        dist = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        return dist * self.TIE_BREAKER

    def find_path(self, start, target):
        """
        start, target: (x, y) tuples
        Returns: list of (x, y) tuples representing the optimal path,
                 or an empty list if no path exists.
        """
        start_x, start_y = start
        target_x, target_y = target

        if not self._in_bounds(start_x, start_y) or not self._in_bounds(target_x, target_y):
            return []
        if not self._is_walkable(start_x, start_y) or not self._is_walkable(target_x, target_y):
            return []

        start_node = Node(
            start_x, start_y,
            g=0.0,
            h=self._heuristic(start_x, start_y, target_x, target_y)
        )

        open_list = []
        heapq.heappush(open_list, (start_node.f, (start_node.x, start_node.y), start_node))

        # Tracks the best known g-cost for each visited coordinate,
        # so we can decide if a newly found route to that cell is cheaper.
        best_g = {(start_x, start_y): 0.0}

        closed_set = set()

        while open_list:
            _, _, current_node = heapq.heappop(open_list)

            # A node can be pushed multiple times with stale costs;
            # skip if we've already locked in a better path to it.
            if (current_node.x, current_node.y) in closed_set:
                continue

            # Victory check
            if current_node.x == target_x and current_node.y == target_y:
                return self._backtrack(current_node)

            closed_set.add((current_node.x, current_node.y))

            for dx, dy in self.NEIGHBOR_OFFSETS:
                nx, ny = current_node.x + dx, current_node.y + dy

                if not self._in_bounds(nx, ny):
                    continue
                if not self._is_walkable(nx, ny):
                    continue
                if (nx, ny) in closed_set:
                    continue

                step_cost = self.DIAGONAL_COST if dx != 0 and dy != 0 else self.STRAIGHT_COST
                tentative_g = current_node.g + step_cost

                if (nx, ny) not in best_g or tentative_g < best_g[(nx, ny)]:
                    best_g[(nx, ny)] = tentative_g
                    h_cost = self._heuristic(nx, ny, target_x, target_y)
                    neighbor_node = Node(nx, ny, g=tentative_g, h=h_cost, parent=current_node)
                    heapq.heappush(
                        open_list,
                        (neighbor_node.f, (neighbor_node.x, neighbor_node.y), neighbor_node)
                    )

        # Open list exhausted without reaching the target
        return []

    @staticmethod
    def _backtrack(node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path


if __name__ == "__main__":
    # Quick standalone sanity check (no Tkinter needed)
    test_grid = [
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0],
        [0, 1, 0, 0],
    ]
    planner = AStarPlanner(test_grid)
    result = planner.find_path((0, 0), (3, 3))
    print("Path found:", result)