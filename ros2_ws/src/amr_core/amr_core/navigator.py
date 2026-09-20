"""
navigator.py

Pure mathematical engine for Sub-Phase 2: the Global Navigator (D* Lite).
Knows nothing about pixels, canvases, or Tkinter. Given a 2D grid
(0 = free, 1 = obstacle), a start coordinate, and a target coordinate,
it returns the optimal list of (x, y) waypoints using an 8-directional
D* Lite search backed by a Min-Heap priority queue.

D* Lite searches backward from the goal, so a single search tree (the
g/rhs values below) covers the whole map relative to that one goal.
As long as the goal doesn't change, a local obstacle change only needs
to repair the handful of vertices actually touched by that change,
instead of re-running the search from scratch the way plain A* would.
That property is what this module is built around: compute_path()
does the (expensive) full backward search once per goal, and
update_obstacles() does the (cheap) local repair every time after that.

This replaces the earlier AStarPlanner. A* is still exactly what runs
on the very first call for a given goal (rhs(goal)=0, everything else
at infinity, searching backward toward the start is mathematically the
same as a backward A*); the difference only shows up once the map
changes mid-transit toward a goal this planner has already seen.
"""

import heapq
import json
import math


class DStarLitePlanner:
    """
    Incremental replanning search over a static-sized, dynamically
    updated occupancy grid, using the D* Lite algorithm (Koenig &
    Likhachev, 2002).

    Usage:
        planner = DStarLitePlanner(grid)
        path = planner.compute_path(start, goal)           # first plan
        ...
        path = planner.update_obstacles(changed_cells, s)   # after a sensor hit

    One instance tracks one (start, goal) search tree at a time. If the
    goal changes -- which happens often in a fleet like this one, every
    time CBBA hands a robot a new task -- compute_path() detects that
    and reinitializes, so callers don't need to build a fresh planner
    per task. What DOES carry over cheaply across calls is repeated
    obstacle changes against the SAME goal, e.g. an aisle blocking and
    clearing again while a robot is still en route to one drop-off.
    """

    # 8-way movement offsets: N, S, E, W, NE, NW, SE, SW
    NEIGHBOR_OFFSETS = [
        (0, -1), (0, 1), (1, 0), (-1, 0),
        (1, -1), (-1, -1), (1, 1), (-1, 1)
    ]

    STRAIGHT_COST = 1.0
    DIAGONAL_COST = math.sqrt(2)
    # The old AStarPlanner inflated h by 1.001 as a tie-breaking trick.
    # That's safe for a one-shot search but NOT safe here: D* Lite's
    # early-termination shortcut (stop once s_start is consistent,
    # without draining the rest of the queue) is only provably correct
    # when h is strictly admissible and consistent. Any inflation at
    # all can leave an off-path vertex sitting on a stale, not-yet-
    # repaired value that _extract_path then has no way to distinguish
    # from a settled one -- verified experimentally: with 1.001 this
    # occasionally produced an empty path after update_obstacles() even
    # though a valid route existed. Must stay exactly 1.0.
    TIE_BREAKER = 1.0

    def __init__(self, grid, resolution=1.0, origin=(0.0, 0.0, 0.0), robot_radius=0.0):
        self.resolution = resolution
        self.origin = origin
        
        # 1. Calculate inflation in terms of grid cells
        if robot_radius > 0.0 and resolution > 0.0:
            self.inflation_cells = int(math.ceil(robot_radius / resolution))
        else:
            self.inflation_cells = 0

        # 2. Inflate the static obstacles (Minkowski Sum)
        self.grid = self._inflate_grid(grid)
        self.rows = len(self.grid)
        self.cols = len(self.grid[0]) if self.rows > 0 else 0

        # Search state -- populated by _initialize() on the first compute_path() call
        self.g = {}       
        self.rhs = {}     
        self.U = []               
        self.entry_finder = {}    
        self.km = 0.0
        self.s_start = None
        self.s_goal = None
        self.s_last = None

    def _inflate_grid(self, raw_grid):
        """Expands 1s (obstacles) into surrounding 0s by the robot's physical radius."""
        if self.inflation_cells <= 0:
            return [row[:] for row in raw_grid]
            
        rows = len(raw_grid)
        cols = len(raw_grid[0]) if rows > 0 else 0
        inflated = [row[:] for row in raw_grid]  # Start with a copy of the raw grid
        
        # 1. Precompute the inflation circle (kernel) exactly once
        r = self.inflation_cells
        r_sq = r ** 2
        kernel = [
            (dx, dy) 
            for dy in range(-r, r + 1) 
            for dx in range(-r, r + 1) 
            if dx*dx + dy*dy <= r_sq
        ]
        
        # 2. Apply the precomputed kernel ONLY to existing obstacle cells
        for y in range(rows):
            for x in range(cols):
                if raw_grid[y][x] != 0:
                    for dx, dy in kernel:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < cols and 0 <= ny < rows:
                            inflated[ny][nx] = 1
                            
        return inflated

    @classmethod
    def from_costmap(cls, path, robot_radius=0.0):
        """
        Builds a planner directly from a costmap.json file.
        Pass robot_radius (in meters) to automatically inflate static obstacles.
        """
        with open(path, "r") as f:
            data = json.load(f)
        return cls(
            data["grid"],
            resolution=data.get("resolution", 1.0),
            origin=data.get("origin", (0.0, 0.0, 0.0)),
            robot_radius=robot_radius
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

    # Graph primitives
    def _neighbors(self, u):
        """
        In-bounds 8-connected neighbors of u, regardless of whether they
        are currently walkable. Obstacle cells stay in the graph rather
        than being excluded, so that when an obstacle clears, the edges
        touching it recover a finite cost instead of never having existed.
        """
        x, y = u
        result = []
        for dx, dy in self.NEIGHBOR_OFFSETS:
            nx, ny = x + dx, y + dy
            if self._in_bounds(nx, ny):
                result.append((nx, ny))
        return result

    def _cost(self, u, v):
        """c(u, v): movement cost between adjacent cells, infinite if
        either endpoint is out of bounds or currently an obstacle."""
        if not self._in_bounds(*u) or not self._in_bounds(*v):
            return math.inf
        if not self._is_walkable(*u) or not self._is_walkable(*v):
            return math.inf
        dx = v[0] - u[0]
        dy = v[1] - u[1]

        # Prevent diagonal clipping through corners
        if dx != 0 and dy != 0:
            if not self._is_walkable(u[0] + dx, u[1]) or not self._is_walkable(u[0], u[1] + dy):
                return math.inf
            return self.DIAGONAL_COST
        return self.DIAGONAL_COST if dx != 0 and dy != 0 else self.STRAIGHT_COST

    def _heuristic(self, a, b):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        return dist * self.TIE_BREAKER

    # Priority queue (lazy deletion: stale entries are discarded when
    # they surface at the top rather than hunted down up front, the
    # same tolerance-for-duplicates approach the old A* implementation
    # used via its closed_set check)
    def _queue_push(self, node, key):
        heapq.heappush(self.U, (key, node))
        self.entry_finder[node] = key

    def _queue_contains(self, node):
        return node in self.entry_finder

    def _queue_remove(self, node):
        self.entry_finder.pop(node, None)

    def _queue_top_key(self):
        while self.U and self.entry_finder.get(self.U[0][1]) != self.U[0][0]:
            heapq.heappop(self.U)
        if not self.U:
            return (math.inf, math.inf)
        return self.U[0][0]

    def _queue_pop_min(self):
        while self.U:
            key, node = heapq.heappop(self.U)
            if self.entry_finder.get(node) == key:
                del self.entry_finder[node]
                return key, node
        return None

    # Core D* Lite functions
    def _calculate_key(self, s):
        g_rhs_min = min(self.g.get(s, math.inf), self.rhs.get(s, math.inf))
        return (g_rhs_min + self._heuristic(s, self.s_start) + self.km, g_rhs_min)

    def _initialize(self, start, goal):
        self.s_start = start
        self.s_goal = goal
        self.s_last = start
        self.km = 0.0
        self.g = {}
        self.rhs = {goal: 0.0}
        self.U = []
        self.entry_finder = {}
        self._queue_push(goal, self._calculate_key(goal))

    def _update_vertex(self, u):
        if u != self.s_goal:
            best = math.inf
            for v in self._neighbors(u):
                cost = self._cost(u, v) + self.g.get(v, math.inf)
                if cost < best:
                    best = cost
            self.rhs[u] = best
        if self._queue_contains(u):
            self._queue_remove(u)
        if self.g.get(u, math.inf) != self.rhs.get(u, math.inf):
            self._queue_push(u, self._calculate_key(u))

    def _compute_shortest_path(self):
        while True:
            top_key = self._queue_top_key()
            start_key = self._calculate_key(self.s_start)
            start_consistent = self.rhs.get(self.s_start, math.inf) == self.g.get(self.s_start, math.inf)
            if not (top_key < start_key) and start_consistent:
                break

            popped = self._queue_pop_min()
            if popped is None:
                break  # queue exhausted; guards the lazy-deletion queue
            k_old, u = popped
            k_new = self._calculate_key(u)

            if k_old < k_new:
                self._queue_push(u, k_new)
            elif self.g.get(u, math.inf) > self.rhs.get(u, math.inf):
                # Overconsistent: this vertex just found a cheaper route.
                self.g[u] = self.rhs.get(u, math.inf)
                for s in self._neighbors(u):
                    self._update_vertex(s)
            else:
                # Underconsistent: this vertex's old route just got worse
                # (e.g. an obstacle appeared on it) and nothing cheaper
                # has been found yet -- force it back to infinity and
                # let its neighbors re-derive a new rhs.
                self.g[u] = math.inf
                for s in self._neighbors(u) + [u]:
                    self._update_vertex(s)

    def _extract_path(self, max_steps=None):
        """
        Greedily walks from s_start to s_goal, at each step taking
        whichever neighbor minimizes c(current, neighbor) + g(neighbor).
        Guards against the two ways this can go wrong: no finite route
        existing (returns []) and a cycle from an inconsistent partial
        g field (also returns [] rather than looping forever).
        """
        if self.g.get(self.s_start, math.inf) == math.inf:
            return []

        path = [self.s_start]
        current = self.s_start
        visited = {current}
        limit = max_steps or (self.rows * self.cols)

        while current != self.s_goal:
            best_next = None
            best_cost = math.inf
            for v in self._neighbors(current):
                cost = self._cost(current, v) + self.g.get(v, math.inf)
                if cost < best_cost:
                    best_cost = cost
                    best_next = v

            if best_next is None or best_cost == math.inf:
                return []

            path.append(best_next)
            current = best_next
            if current in visited or len(path) > limit:
                return []
            visited.add(current)

        return path

    # Public API
    def compute_path(self, start, target):
        """
        Drop-in replacement for the old AStarPlanner.find_path(start, target).

        Reinitializes the whole search tree whenever `target` differs from
        whatever goal this planner last searched for -- a changed goal
        invalidates essentially every cached g/rhs value anyway, since
        they were all computed relative to the old goal, so there is
        nothing worth preserving. If `target` is the same goal as last
        time (the common case: same task, robot has just moved or an
        obstacle changed), the existing search tree is reused as-is.
        """
        if not self._in_bounds(*start) or not self._in_bounds(*target):
            return []
        if not self._is_walkable(*start) or not self._is_walkable(*target):
            return []

        if target != self.s_goal:
            self._initialize(start, target)
        else:
            self.s_start = start

        self._compute_shortest_path()
        return self._extract_path()

    def update_obstacles(self, changed_cells, start):
        """
        Call when the outer layer's sensor scan finds cells that changed
        occupancy since the last plan. `changed_cells` is an iterable of
        (x, y, new_value) with new_value 0 (now free) or 1 (now blocked).
        `start` is the robot's current grid cell.

        This is the whole point of D* Lite over plain A*: only the
        vertices touched by the change get pushed back onto the queue
        and repaired, instead of the entire map being re-searched.
        """
        if self.s_goal is None:
            raise RuntimeError("update_obstacles() called before an initial compute_path()")

        self.km += self._heuristic(self.s_last, start)
        self.s_last = start
        self.s_start = start

        touched = set()
        for x, y, new_value in changed_cells:
            self.grid[y][x] = new_value
            touched.add((x, y))
            touched.update(self._neighbors((x, y)))

        for s in touched:
            self._update_vertex(s)

        self._compute_shortest_path()
        return self._extract_path()


if __name__ == "__main__":
    # Quick standalone sanity check (no Tkinter needed): plan once, then
    # drop an obstacle onto the path ahead of the robot and replan.
    test_grid = [
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0],
        [0, 1, 0, 0],
    ]
    planner = DStarLitePlanner(test_grid)

    path1 = planner.compute_path((0, 0), (3, 3))
    print("Initial path:", path1)

    if len(path1) > 3:
        current = path1[1]    # robot has taken one step
        blocked = path1[-2]   # a cell further along gets blocked
        print(f"Robot at {current}, blocking {blocked}, replanning...")
        path2 = planner.update_obstacles([(blocked[0], blocked[1], 1)], start=current)
        print("Repaired path:", path2)