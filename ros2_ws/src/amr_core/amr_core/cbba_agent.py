import json
import math
import os

# Load master configuration
_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_models.json")

with open(_CONFIG_PATH, "r") as _f:
    _CONFIG = json.load(_f)

SYSTEM_CONSTANTS = _CONFIG["system_constants"]
FLEET_STATE = _CONFIG["fleet_state"]  # Restored for fleet_sim.py
OPEN_TASKS = _CONFIG["open_tasks"]    # Restored for fleet_sim.py
CHARGE_STATION = SYSTEM_CONSTANTS["charging_station"]
V_LINEAR = SYSTEM_CONSTANTS["v_linear"]
V_ANGULAR = SYSTEM_CONSTANTS["v_angular"]
E_RATE = SYSTEM_CONSTANTS["e_rate"]
C_RATE = SYSTEM_CONSTANTS["c_rate"]
LAMBDA_VAL = SYSTEM_CONSTANTS["lambda_val"]
BATTERY_SAFETY_THRESHOLD = SYSTEM_CONSTANTS["battery_safety_threshold"]
BATTERY_FULL = SYSTEM_CONSTANTS.get("battery_full", 100.0)

class CBBANode:
    def __init__(self, robot_dict, num_robots):
        # Physical State
        self.id = robot_dict["id"]
        self.x = robot_dict["x"]
        self.y = robot_dict["y"]
        self.theta = robot_dict["theta"]
        self.battery = robot_dict["battery"]

        self.bundle = []

        # Dynamic Memory Matrices (Ledgers) using dictionaries to allow infinite/dynamic tasks
        self.Y = {}  # task_id -> highest known bid 
        self.Z = {}  # task_id -> current claimed winner 
        self.T = [0 for _ in range(num_robots)] # last-updated timestamp per robot

        self._pickup_done = False

    # --- KINEMATICS ---
    def _calc_rotation_time(self, target_x, target_y, origin_x=None, origin_y=None, origin_theta=None):
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y
        otheta = self.theta if origin_theta is None else origin_theta

        target_angle = math.atan2(target_y - oy, target_x - ox)
        angle_diff = abs(target_angle - otheta)
        if angle_diff > math.pi:
            angle_diff = (2 * math.pi) - angle_diff
        return angle_diff / V_ANGULAR

    def _calc_travel_time(self, target_x, target_y, origin_x=None, origin_y=None):
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y
        distance = math.hypot(target_x - ox, target_y - oy)
        return (distance / V_LINEAR) + (1.0 if distance > 0 else 0.0)

    def _calc_task_time(self, task, origin_x=None, origin_y=None, origin_theta=None):
        ox = self.x if origin_x is None else origin_x
        oy = self.y if origin_y is None else origin_y
        otheta = self.theta if origin_theta is None else origin_theta

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
        b = self.battery if battery is None else battery
        if (tau * E_RATE) > b:
            return 0.0
        return task["reward"] * math.exp(-LAMBDA_VAL * tau)

    def calculate_bid(self, task, origin_x=None, origin_y=None, origin_theta=None, battery=None):
        tau = self._calc_task_time(task, origin_x, origin_y, origin_theta)
        return self.calculate_bid_from_tau(task, tau, battery)

    # --- CHARGING (EXCLUSIVE TASKS) ---
    def _generate_charging_task(self):
        return {
            "id": f"CHG-{self.id}",
            "pick_x": CHARGE_STATION["x"], "pick_y": CHARGE_STATION["y"],
            "drop_x": CHARGE_STATION["x"], "drop_y": CHARGE_STATION["y"],
            "t_load": 0.0, "t_unload": 0.0,
            "reward": float("inf"),
            "is_exclusive": True,
        }

    def is_charging_task_active(self):
        return bool(self.bundle) and self.bundle[0].get("is_exclusive", False)

    # --- PHASE 1: BUNDLE BUILDING ---
    def build_bundle(self, open_tasks=None, max_bundle_size=3):
        """Continually bids on newly added open_tasks until the bundle is full or no profitable bids remain."""
        if open_tasks is None:
            open_tasks = OPEN_TASKS
            
        # Guarantee it's an iterable list even if OPEN_TASKS is None/null
        if open_tasks is None:
            open_tasks = []
        v_x, v_y, v_theta = self.x, self.y, self.theta
        v_battery = self.battery
        
        # Fast-forward virtual state through the currently locked bundle
        for task in self.bundle:
            if not task.get("is_exclusive"):
                tau = self._calc_task_time(task, v_x, v_y, v_theta)
                v_battery -= tau * E_RATE
                v_x, v_y = task["drop_x"], task["drop_y"]
                v_theta = self._final_heading_after_task(task)

        while len(self.bundle) < max_bundle_size:
            best_task, best_bid, best_tau = None, 0.0, None

            for task in open_tasks:
                tid = task["id"]
                if any(t["id"] == tid for t in self.bundle):
                    continue
                
                # Check dynamic ledger; default to unclaimed (-1)
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
            
            v_battery -= best_tau * E_RATE
            v_x, v_y = best_task["drop_x"], best_task["drop_y"]
            v_theta = self._final_heading_after_task(best_task)

        return self.bundle

    # --- PHASE 2: CONSENSUS & SYNC ---
    def make_payload(self, v_x, v_y):
        self.T[self.id] += 1
        return {
            "id": self.id, "x": self.x, "y": self.y,
            "v_x": v_x, "v_y": v_y, "battery": self.battery,
            "Z_ledger": dict(self.Z), "Y_ledger": dict(self.Y), "T": list(self.T),
        }

    def receive_broadcast(self, payload):
        sid = payload["id"]
        if sid == self.id:
            return False
        
        self.T[sid] = max(self.T[sid], payload["T"][sid])
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
        """CBBA strict rule: If a task is lost to an outbid, drop it AND all subsequent tasks in the bundle."""
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
        """Removes a physically completed task from the agent's execution queue."""
        task_id = task["id"] if isinstance(task, dict) else task

        # 1. Remove the completed task from the bundle
        self.bundle = [t for t in self.bundle if t["id"] != task_id]
        
        # 2. Update physical path queue if it exists
        if hasattr(self, 'path'):
            self.path = [p for p in self.path if p.get("id") != task_id]
             
        # 3. Mark completed so it is excluded from future auctions
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

    def update_state(self, dt, is_charging=False, check_threshold=True):
        if is_charging:
            self.battery = min(BATTERY_FULL, self.battery + C_RATE * dt)
            return self.battery

        if self.bundle:
            self.battery = max(0.0, self.battery - E_RATE * dt)

        if check_threshold and self.battery < BATTERY_SAFETY_THRESHOLD and not self.is_charging_task_active():
            if self.bundle:
                t = self.bundle[0]
                if not t.get("is_exclusive"):
                    self.Z[t["id"]] = -1
                    self.Y[t["id"]] = 0.0
                self.abandon_current_task()
            self.bundle.insert(0, self._generate_charging_task())
            self._pickup_done = False
            
        return self.battery