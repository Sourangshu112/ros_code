# import json
# import os
# _CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_models.json")
# with open(_CONFIG_PATH, "r") as _f:
#     _CONFIG = json.load(_f)
# SYSTEM_CONSTANTS = _CONFIG["system_constants"]
# OPEN_TASKS = _CONFIG["open_tasks"]
# V_LINEAR = SYSTEM_CONSTANTS["v_linear"]
# V_ANGULAR = SYSTEM_CONSTANTS["v_angular"]
# E_RATE = SYSTEM_CONSTANTS["e_rate"]
# C_RATE = SYSTEM_CONSTANTS["c_rate"]
# LAMBDA_VAL = SYSTEM_CONSTANTS["lambda_val"]
# BATTERY_SAFETY_THRESHOLD = SYSTEM_CONSTANTS["battery_safety_threshold"]
# BATTERY_FULL = SYSTEM_CONSTANTS.get("battery_full", 100.0)
import math

class CBBANode:
    def __init__(self, node):
        # Object injection & memory linking
        self.node = node
        self.id = node.get_name()
        
        self.bundle = []
        self.Y = {}
        self.Z = {}
        self.T = {}
        
        self._pickup_done = False

    # --- KINEMATICS ---
    def _calc_rotation_time(self, target_x, target_y, origin_x=None, origin_y=None, origin_theta=None):
        ox = self.node.current_x if origin_x is None else origin_x
        oy = self.node.current_y if origin_y is None else origin_y
        otheta = self.node.current_yaw if origin_theta is None else origin_theta

        target_angle = math.atan2(target_y - oy, target_x - ox)
        angle_diff = abs(target_angle - otheta)
        if angle_diff > math.pi:
            angle_diff = (2 * math.pi) - angle_diff
        return angle_diff / self.node.v_angular

    # def _calc_travel_time(self, target_x, target_y, origin_x=None, origin_y=None):
    #     ox = self.node.current_x if origin_x is None else origin_x
    #     oy = self.node.current_y if origin_y is None else origin_y
    #     distance = math.hypot(target_x - ox, target_y - oy)
    #     return (distance / self.node.v_linear) + (1.0 if distance > 0 else 0.0)
    def _calc_travel_time(self, target_x, target_y, origin_x=None, origin_y=None):
        ox = self.node.current_x if origin_x is None else origin_x
        oy = self.node.current_y if origin_y is None else origin_y
        
        navigator = self.node.global_navigator
        start_col, start_row = navigator.world_to_grid(ox, oy)
        target_col, target_row = navigator.world_to_grid(target_x, target_y)
        
        # Calculate obstacle-aware path using D* Lite
        grid_path = navigator.compute_path((start_col, start_row), (target_col, target_row))
        
        # If the target is unreachable (e.g., inside an obstacle), reject the bid
        if not grid_path:
            return float('inf')
            
        # Convert grid coordinates back to world coordinates and sum the segments
        distance = 0.0
        world_path = [navigator.grid_to_world(c, r) for c, r in grid_path]
        for i in range(1, len(world_path)):
            prev_x, prev_y = world_path[i-1]
            curr_x, curr_y = world_path[i]
            distance += math.hypot(curr_x - prev_x, curr_y - prev_y)
            
        return (distance / self.node.v_linear) + (1.0 if distance > 0 else 0.0)
    '''

*   **World-to-Grid Translation:** The physical `ox` and `oy` coordinates are first mapped to the discrete costmap via 
    `navigator.world_to_grid` so the `DStarLitePlanner` can process the graph.
*   **Path Unreachability:** If the D* Lite search returns an empty array `[]` (meaning the target is walled off or invalid), 
    the function returns `float('inf')`. This safely cascades into your bidding logic, as `tau = inf` will 
    instantly fail the battery check (`inf * e_rate > battery`) and force a bid of `0.0`.
*   **True Distance Aggregation:** The planner returns a list of grid coordinates. 
    To maintain exact physical distance accuracy, the list is projected back into world 
    coordinates and the Euclidean distance of every intermediate segment is summed. 

    **Architectural Warning for CBBA Bidding** 
    The CBBA bundle builder evaluates every open task repeatedly during its consensus loop. 
    By swapping Euclidean distance for a full D* Lite path search inside `_calc_travel_time`, 
    the `build_bundle` function will now trigger hundreds of graph searches per second. 
    If the node begins to drop ROS 2 messages or lock up due to CPU bottlenecking, 
    you may need to implement a cached distance matrix (an offline lookup table of distances between all known pickup/drop-off nodes) 
    rather than running D* Lite on the fly during an auction.
    '''
    def _calc_task_time(self, task, origin_x=None, origin_y=None, origin_theta=None):
        ox = self.node.current_x if origin_x is None else origin_x
        oy = self.node.current_y if origin_y is None else origin_y
        otheta = self.node.current_yaw if origin_theta is None else origin_theta

        leg1_rot = self._calc_rotation_time(task["pick_x"], task["pick_y"], ox, oy, otheta)
        leg1_trav = self._calc_travel_time(task["pick_x"], task["pick_y"], ox, oy)
        heading_after_leg1 = math.atan2(task["pick_y"] - oy, task["pick_x"] - ox)

        leg2_rot = self._calc_rotation_time(task["drop_x"], task["drop_y"], task["pick_x"], task["pick_y"], heading_after_leg1)
        leg2_trav = self._calc_travel_time(task["drop_x"], task["drop_y"], task["pick_x"], task["pick_y"])

        return leg1_rot + leg1_trav + task["t_load"] + leg2_rot + leg2_trav + task["t_unload"]

    def _final_heading_after_task(self, task):
        return math.atan2(task["drop_y"] - task["pick_y"], task["drop_x"] - task["pick_x"])

    # --- BIDDING ---
    def calculate_bid_from_tau(self, task, tau, battery=None):
        b = self.node.battery if battery is None else battery
        if (tau * self.node.e_rate) > b:
            return 0.0
        return task["reward"] * math.exp(-self.node.lambda_val * tau)

    def calculate_bid(self, task, origin_x=None, origin_y=None, origin_theta=None, battery=None):
        tau = self._calc_task_time(task, origin_x, origin_y, origin_theta)
        return self.calculate_bid_from_tau(task, tau, battery)

    # --- CHARGING (EXCLUSIVE TASKS) ---
    def _generate_charging_task(self):
        return {
            "id": f"CHG-{self.id}",
            "pick_x": self.node.offset_x, "pick_y": self.node.offset_y,
            "drop_x": self.node.offset_x, "drop_y": self.node.offset_y,
            "t_load": 0.0, "t_unload": 0.0,
            "reward": float("inf"),
            "is_exclusive": True,
        }

    def is_charging_task_active(self):
        return bool(self.bundle) and self.bundle[0].get("is_exclusive", False)

    # --- PHASE 1: BUNDLE BUILDING ---
    def build_bundle(self, open_tasks=None, max_bundle_size=3):
        if open_tasks is None:
            open_tasks = []
            
        v_x, v_y, v_theta = self.node.current_x, self.node.current_y, self.node.current_yaw
        v_battery = self.node.battery
        
        for task in self.bundle:
            if not task.get("is_exclusive"):
                tau = self._calc_task_time(task, v_x, v_y, v_theta)
                v_battery -= tau * self.node.e_rate
                v_x, v_y = task["drop_x"], task["drop_y"]
                v_theta = self._final_heading_after_task(task)

        while len(self.bundle) < max_bundle_size:
            best_task, best_bid, best_tau = None, 0.0, None

            for task in open_tasks:
                tid = task["id"]
                if any(t["id"] == tid for t in self.bundle):
                    continue
                
                current_winner = self.Z.get(tid, -1)
                if current_winner != -1 and current_winner != self.id:
                    continue

                bid = self.calculate_bid(task, v_x, v_y, v_theta, v_battery)
                
                if bid > best_bid and bid > self.Y.get(tid, 0.0):
                    best_bid = bid
                    best_task = task
                    best_tau = self._calc_task_time(task, v_x, v_y, v_theta)

            if best_task is None:
                break 

            self.bundle.append(best_task)
            self.Y[best_task["id"]] = best_bid
            self.Z[best_task["id"]] = self.id
            
            v_battery -= best_tau * self.node.e_rate
            v_x, v_y = best_task["drop_x"], best_task["drop_y"]
            v_theta = self._final_heading_after_task(best_task)

        return self.bundle

    # --- PHASE 2: CONSENSUS & SYNC ---
    def make_payload(self, v_x, v_y):
        self.T[self.id] = self.T.get(self.id, 0) + 1
        return {
            "id": self.id, 
            "x": self.node.current_x, 
            "y": self.node.current_y,
            "v_x": v_x, "v_y": v_y, 
            "battery": self.node.battery,
            "Z_ledger": dict(self.Z), 
            "Y_ledger": dict(self.Y), 
            "T": dict(self.T),
        }

    def receive_broadcast(self, payload):
        sid = payload["id"]
        if sid == self.id:
            return False
        
        incoming_T = payload["T"].get(sid, 0)
        self.T[sid] = max(self.T.get(sid, 0), incoming_T)
        changed = False

        for tid, z_o in payload["Z_ledger"].items():
            y_o = payload["Y_ledger"].get(tid, 0.0)
            
            mine = self.Z.get(tid, -1)
            my_bid = self.Y.get(tid, 0.0)

            if z_o == -2:
                if mine != -2:
                    self.Z[tid] = -2
                    changed = True
            elif z_o == sid:
                wins = (mine == -1 or mine == sid or y_o > my_bid or (y_o == my_bid and sid < mine))
                if wins and (mine, my_bid) != (sid, y_o):
                    self.Z[tid] = sid
                    self.Y[tid] = y_o
                    changed = True
            elif z_o == -1 and mine == sid:
                self.Z[tid] = -1
                self.Y[tid] = 0.0
                changed = True

        if changed:
            self._apply_cbba_drop_rule()
        return changed

    def _apply_cbba_drop_rule(self):
        drop_index = -1
        for i, t in enumerate(self.bundle):
            if not t.get("is_exclusive") and self.Z.get(t["id"], -1) != self.id:
                drop_index = i
                break
        
        if drop_index != -1:
            dropped_tasks = self.bundle[drop_index:]
            self.bundle = self.bundle[:drop_index]
            if drop_index == 0:
                self._pickup_done = False
                
            for t in dropped_tasks:
                tid = t["id"]
                if not t.get("is_exclusive") and self.Z.get(tid) == self.id:
                    self.Z[tid] = -1
                    self.Y[tid] = 0.0

    def close_task(self, task):
        task_id = task["id"] if isinstance(task, dict) else task

        self.bundle = [t for t in self.bundle if t["id"] != task_id]
        
        if hasattr(self, 'path'):
            self.path = [p for p in self.path if p.get("id") != task_id]
             
        if hasattr(self, 'task_status'):
            self.task_status[task_id] = 'COMPLETED'

    # --- EXECUTION ---
    def current_target(self):
        if not self.bundle:
            return None
        task = self.bundle[0]
        if not self._pickup_done:
            return task["pick_x"], task["pick_y"], True
        return task["drop_x"], task["drop_y"], False

    def advance_leg(self):
        if not self.bundle: return
        if not self._pickup_done:
            self._pickup_done = True
        else:
            done = self.bundle.pop(0)
            if not done.get("is_exclusive"):
                self.Z[done["id"]] = -2
            self._pickup_done = False

    def abandon_current_task(self):
        if not self.bundle: return
        t = self.bundle.pop(0)
        if not t.get("is_exclusive"):
            self.Z[t["id"]] = -1
            self.Y[t["id"]] = 0.0
        self._pickup_done = False

    def check_battery_status(self):
        """
        Monitors live hardware battery. Triggers emergency preemption 
        if the threshold is breached.
        """
        # Check if the hardware battery is critically low and we aren't already charging
        if self.node.battery < self.node.battery_threshold and not self.is_charging_task_active():

            # Relinquish the current fleet task back to the network
            if self.bundle:
                t = self.bundle[0]
                if not t.get("is_exclusive"):
                    self.Z[t["id"]] = -1
                    self.Y[t["id"]] = 0.0
                self.abandon_current_task()

            # Force the charging sequence to the front of the execution queue
            self.bundle.insert(0, self._generate_charging_task())
            self._pickup_done = False
        return self.node.battery