import json
import math
import os
from ament_index_python.packages import get_package_share_directory

# Load master configuration

_SHARE_DIR = get_package_share_directory('amr_core')
_CONFIG_PATH = os.path.join(_SHARE_DIR, 'config', 'data_models.json')

with open(_CONFIG_PATH, "r") as _f:
    _CONFIG = json.load(_f)

SYSTEM_CONSTANTS = _CONFIG["system_constants"]


V_LINEAR = SYSTEM_CONSTANTS["v_linear"]
V_ANGULAR = SYSTEM_CONSTANTS["v_angular"]
E_RATE = SYSTEM_CONSTANTS["e_rate"]
C_RATE = SYSTEM_CONSTANTS["c_rate"]
LAMBDA_VAL = SYSTEM_CONSTANTS["lambda_val"]
BATTERY_SAFETY_THRESHOLD = SYSTEM_CONSTANTS["battery_safety_threshold"]
CHARGE_STATION = SYSTEM_CONSTANTS["charging_station"]


class CBBANode:
    """
    Represents a single AMR's decentralized decision-making agent.

    Physical state and ledgers are seeded from fleet_state (DS1). Tasks
    are not attached to individual robots at boot time — open_tasks (DS2)
    is the shared pool every robot bids on. The one exception is charging:
    each robot generates its own exclusive charging task internally, on
    the fly, once its battery crosses a safety threshold.
    """

    def __init__(self, robot_dict, num_tasks, num_robots, navigator=None):
        # Physical State
        self.id = robot_dict["id"]
        self.x = robot_dict["x"]
        self.y = robot_dict["y"]
        self.theta = robot_dict["theta"]
        self.battery = robot_dict["battery"]

        # The Pocket — only ever holds this robot's own exclusive tasks
        # (i.e. a self-generated charging task). Starts empty.
        self.local_tasks = []

        # The Bundle — ordered sequence of tasks this robot plans to run
        self.bundle = []

        # The Memory Matrices (Ledgers)
        self.Y = {}  # highest known bid per task (task_id -> bid)
        self.Z = {}  # current claimed winner per task (task_id -> robot_id)
        self.T = {}  # last-updated timestamp per robot (robot_id -> timestamp)

        # Live execution tracking — which leg of bundle[0] we're on.
        # False = still driving to pickup. Flips to True once pickup
        # is reached, then advance_leg() pops the task on drop.
        self._pickup_done = False

        # Global path planner for obstacle-aware bidding
        self.navigator = navigator

    # Kinematic Helpers
    def _calc_rotation_time(self, target_x, target_y, origin_x=None, origin_y=None, origin_theta=None):
        """
        Time to rotate in place to face (target_x, target_y).

        Defaults to the robot's actual current position/heading. Pass
        origin_x/origin_y/origin_theta explicitly when chaining a second
        leg that starts somewhere other than the robot's real location
        (e.g. pickup -> drop, after the robot has already reached pickup).
        """
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y
        otheta = self.theta if origin_theta is None else origin_theta

        target_angle = math.atan2(target_y - oy, target_x - ox)
        angle_diff = abs(target_angle - otheta)
        if angle_diff > math.pi:
            angle_diff = (2 * math.pi) - angle_diff
        return angle_diff / V_ANGULAR

    # def _calc_travel_time(self, target_x, target_y, origin_x=None, origin_y=None):
    #     """
    #     Time to drive in a straight line to (target_x, target_y).

    #     Same origin-override behavior as _calc_rotation_time, for the
    #     same reason: chained legs don't start from the robot's real x/y.
    #     """
    #     ox = self.x if origin_x is None else origin_x
    #     oy = self.y if origin_y is None else origin_y

    #     distance = math.hypot(target_x - ox, target_y - oy)
    #     acceleration_penalty = 1.0 if distance > 0 else 0.0
    #     return (distance / V_LINEAR) + acceleration_penalty


    def _calc_travel_time(self, target_x, target_y, origin_x=None, origin_y=None):
        """
        Time to drive to (target_x, target_y) utilizing the A* global navigator.
        """
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y

        if self.navigator is None:
            # Fallback if no navigator is provided
            distance = math.hypot(target_x - ox, target_y - oy)
        else:
            # Convert world meters to costmap grid pixels
            start_col, start_row = self.navigator.world_to_grid(ox, oy)
            target_col, target_row = self.navigator.world_to_grid(target_x, target_y)
            
            # Generate optimal path avoiding obstacles
            grid_path = self.navigator.find_path((start_col, start_row), (target_col, target_row))
            
            if not grid_path:
                # If target is completely unreachable, return infinite time so the bid drops to 0
                return float('inf')
                
            # Convert pixel path back to world meters to measure exact real-world distance
            world_path = [self.navigator.grid_to_world(c, r) for c, r in grid_path]
            
            # Sum the distance between consecutive waypoints
            distance = 0.0
            for i in range(1, len(world_path)):
                prev_x, prev_y = world_path[i-1]
                curr_x, curr_y = world_path[i]
                distance += math.hypot(curr_x - prev_x, curr_y - prev_y)

        acceleration_penalty = 1.0 if distance > 0 else 0.0
        return (distance / V_LINEAR) + acceleration_penalty

    # Task Time Accumulation (tau)
    def _calc_task_time(self, task, origin_x=None, origin_y=None, origin_theta=None):
        """
        Total execution time (tau) for a task:
          rotate+drive to pickup -> load -> rotate+drive to drop -> unload

        origin_x/origin_y/origin_theta let build_bundle pass a VIRTUAL state
        instead of the robot's real position, so leg 1 can be evaluated from
        wherever the bundle's previous task left off. Default (None) falls
        back to the robot's actual x/y/theta, so calling this with just
        `task` behaves exactly as before.

        Leg 2 (pickup -> drop) is computed FROM the pickup point, not the
        robot's actual current position, since the robot has already
        travelled to pickup by the time leg 2 starts. The heading used
        for leg 2's rotation cost is whatever heading the robot ends leg 1
        facing (i.e. the direction it just drove).
        """
    
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y
        otheta = self.theta if origin_theta is None else origin_theta

        # Leg 1: origin (real or virtual) -> pickup
        leg1_rotation = self._calc_rotation_time(
            task["pick_x"], task["pick_y"], origin_x=ox, origin_y=oy, origin_theta=otheta,
        )
        leg1_travel = self._calc_travel_time(task["pick_x"], task["pick_y"], origin_x=ox, origin_y=oy)

        # Heading after arriving at pickup = the direction just travelled
        heading_after_leg1 = math.atan2(task["pick_y"] - oy, task["pick_x"] - ox)

        # Leg 2: pickup -> drop
        leg2_rotation = self._calc_rotation_time(
            task["drop_x"], task["drop_y"],
            origin_x=task["pick_x"], origin_y=task["pick_y"], origin_theta=heading_after_leg1,
        )
        leg2_travel = self._calc_travel_time(
            task["drop_x"], task["drop_y"],
            origin_x=task["pick_x"], origin_y=task["pick_y"],
        )

        return (
            leg1_rotation
            + leg1_travel
            + task["t_load"]
            + leg2_rotation
            + leg2_travel
            + task["t_unload"]
        )

    def _final_heading_after_task(self, task):
        """
        Heading the robot ends facing after completing a task: the
        direction of travel for leg 2 (pickup -> drop). Same convention
        as heading_after_leg1 in _calc_task_time, applied to the second leg.
        """
        return math.atan2(task["drop_y"] - task["pick_y"], task["drop_x"] - task["pick_x"])
        
    def update_consensus(self, sender_id, recv_Y, recv_Z, recv_T):
        """
        Ingests broadcasted matrices and resolves conflicts deterministically.
        """
        import time
        self.T[sender_id] = time.time()

        for task_id, sender_bid in recv_Y.items():
            # If the task is entirely new to us, initialize it in our ledgers
            if task_id not in self.Y:
                self.Y[task_id] = 0.0
                self.Z[task_id] = ""

            local_bid = self.Y[task_id]
            sender_winner = recv_Z.get(task_id, "")
            
            # CBBA Rule: If broadcasted bid is higher, overwrite local memory
            if sender_bid > local_bid:
                self.Y[task_id] = sender_bid
                self.Z[task_id] = sender_winner
                
            # Tie-breaker: If bids are identical, sort IDs alphabetically to prevent deadlocks
            elif sender_bid == local_bid and sender_bid > 0:
                if sender_winner < self.Z[task_id]:
                    self.Z[task_id] = sender_winner

    # Bidding
    def calculate_bid_from_tau(self, task, tau, battery = None):
        """
        Energy-gated, exponentially-decaying bid for a task.

        Returns 0.0 if the robot cannot physically survive the trip on
        its current battery. Otherwise returns R * e^(-lambda * tau),
        so distant/slow tasks are worth less than close/fast ones even
        when the raw reward is identical.

        Split out from calculate_bid so callers with a better tau estimate
        (e.g. one derived from an actual A* path instead of straight-line
        distance) can reuse the same gate+decay formula without duplicating it.
        """
        b = self.battery if battery is None else battery
        energy_required = tau * E_RATE
        if energy_required > self.battery:
            return 0.0

        return task["reward"] * math.exp(-LAMBDA_VAL * tau)

    def calculate_bid(self, task, origin_x=None, origin_y=None, origin_theta=None, battery=None):
        """
        Default bid using straight-line kinematic tau (_calc_task_time).
        Ignores obstacles — see calculate_bid_from_tau for a version that
        accepts an obstacle-aware tau instead.

        origin_x/origin_y/origin_theta/battery let build_bundle evaluate
        this from a virtual state. All default to None, which falls back
        to the robot's real x/y/theta/battery, so calling this with just
        `task` behaves exactly as before.
        """
        tau = self._calc_task_time(task, origin_x=origin_x, origin_y=origin_y, origin_theta=origin_theta)
        return self.calculate_bid_from_tau(task, tau, battery=battery)

    # Dynamic Charging Trigger
    def _project_battery_after(self, elapsed_time):
        """Battery level remaining after `elapsed_time` seconds of operation."""
        return self.battery - (elapsed_time * E_RATE)

    def _needs_charging(self, candidate_task):
        """
        Single-task lookahead check: would completing candidate_task alone
        drop battery below the safety threshold?

        NOTE: this only checks one task in isolation. Once the Level 2
        Sequencer exists, this needs to run cumulatively against the
        robot's full bundle, not just the next candidate — a bundle of
        several "safe" tasks can still drain the robot past the threshold
        if each one is only checked against the current, not projected,
        battery level.
        """
        tau = self._calc_task_time(candidate_task)
        projected_battery = self._project_battery_after(tau)
        return projected_battery < BATTERY_SAFETY_THRESHOLD

    def _generate_charging_task(self):
        """
        Auto-generates this robot's exclusive charging task. The bid is
        locked at infinity so it always wins any comparison in Y/Z, and
        is_exclusive=True marks it as never up for broadcast to the fleet.
        """
        return {
            "id": f"CHG-{self.id}",
            "pick_x": CHARGE_STATION["x"],
            "pick_y": CHARGE_STATION["y"],
            "drop_x": CHARGE_STATION["x"],
            "drop_y": CHARGE_STATION["y"],
            "t_load": 0.0,
            "t_unload": 0.0,
            "reward": float("inf"),
            "is_exclusive": True,
        }

    # Level 2 Sequencer — Bundle Building
    def build_bundle(self, open_tasks=None, max_bundle_size=3):
        """
        Greedily builds self.bundle by repeatedly picking whichever open
        task scores the highest bid from the robot's current VIRTUAL
        state, then advancing that virtual state to the winning task's
        drop-off point before evaluating the next slot.
 
        Stops when the bundle hits max_bundle_size, or when no remaining
        open task returns a nonzero bid from the current virtual state
        (i.e. every remaining task is either already claimed, already in
        the bundle, or unsurvivable on the projected battery).
 
        NOTE: "unsurvivable" here only means calculate_bid's energy gate
        (tau * E_RATE > battery). It does NOT check BATTERY_SAFETY_THRESHOLD,
        so this loop can walk the virtual battery down near zero without
        ever triggering a charging need. _needs_charging isn't consulted
        here yet — wiring it in is a separate step.
        """
        if open_tasks is None:
            open_tasks = OPEN_TASKS
 
        v_x, v_y, v_theta = self.x, self.y, self.theta
        v_battery = self.battery
        cumulative_tau = 0.0
 
        while len(self.bundle) < max_bundle_size:
            best_task = None
            best_bid = 0.0
            best_tau = None
 
            for task in open_tasks:
                task_id = task["id"]
 
                # Already in our own bundle
                if any(t["id"] == task_id for t in self.bundle):
                    continue
 
                # Someone else already won this task per our ledger
                if self.Z[task_id] != -1 and self.Z[task_id] != self.id:
                    continue
 
                bid = self.calculate_bid(
                    task, origin_x=v_x, origin_y=v_y, origin_theta=v_theta, battery=v_battery,
                )
 
                if bid > best_bid:
                    best_bid = bid
                    best_task = task
                    best_tau = self._calc_task_time(task, origin_x=v_x, origin_y=v_y, origin_theta=v_theta)
 
            if best_task is None:
                break  # no remaining task returns a positive bid from here
 
            # Commit the winner and advance the virtual state
            self.bundle.append(best_task)
            cumulative_tau += best_tau
            v_battery -= best_tau * E_RATE
            v_x = best_task["drop_x"]
            v_y = best_task["drop_y"]
            v_theta = self._final_heading_after_task(best_task)
 
        return self.bundle

        # Live Execution Tracking
    def current_target(self):
        """
        (x, y, is_pickup) for wherever this robot should drive next,
        or None if the bundle is empty. is_pickup is True while still
        headed to pick_x/pick_y, False once redirected to the drop point.
        """
        if not self.bundle:
            return None
        task = self.bundle[0]
        if not self._pickup_done:
            return task["pick_x"], task["pick_y"], True
        return task["drop_x"], task["drop_y"], False

    def advance_leg(self):
        """
        Call when LocalPlanner's on_goal_reached fires. First call
        flips pickup -> drop on the current task. Second call pops
        the finished task off the bundle and resets for the next one.

        Signature matches on_goal_reached exactly (no args), so it
        can be passed straight in:
            LocalPlanner(..., on_goal_reached=agent.advance_leg)
        """
        if not self.bundle:
            return
        if not self._pickup_done:
            self._pickup_done = True
        else:
            self.bundle.pop(0)
            self._pickup_done = False

    def abandon_current_task(self):
        """
        Drops bundle[0] outright without treating it as completed —
        for when the controller finds the target unreachable (e.g.
        is_valid_cell fails) rather than actually having driven there.
        Keeps that reset out of the controller's hands, which would
        otherwise have to poke _pickup_done directly.
        """
        if not self.bundle:
            return
        self.bundle.pop(0)
        self._pickup_done = False