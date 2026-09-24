import rclpy
import math
import threading
import time
import json
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import zenoh

# External module imports
from fleet_interfaces.msg import AMRTelemetry, FleetVelocity
from amr_core.local_planner import LocalPlanner
from amr_core.orca_filter import ORCAFilter, Neighbor

class ROSHardwareInterface:
    def __init__(self, node):
        self.node = node
        self.peer_states = {}
        self.peer_lock = threading.Lock()

        '''
        Static-obstacle mailbox: a plain list of (x, y) world-frame coordinates.
        Lives on self.node (not self) so any other module with a reference to
        the node — e.g. a teammate's LiDAR/obstacle-detection node — can write
        to it directly: self.node.static_obstacles.append((x, y)) or replace
        the whole list each scan. This class only ever reads it.
                
        '''
        self.node.static_obstacles.append((x, y, obstacle_radius))

        # Publishers
        self.cmd_pub = self.node.create_publisher(Twist, f'/{self.node.get_name()}/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.node.create_subscription(Odometry, f'/{self.node.get_name()}/odom', self.odom_callback, qos_profile_sensor_data)
    
        # Timers
        self.state_timer = self.node.create_timer(10.0, self.check_battery_status_loop)

    def update_peer_state(self, msg: FleetVelocity):
        """Thread-safe update of neighboring robots from Zenoh."""
        with self.peer_lock:
            self.peer_states[msg.robot_id] = (msg, time.time())

    def odom_callback(self, msg: Odometry):
        self.node.current_x = msg.pose.pose.position.x
        self.node.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.node.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        v_local = msg.twist.twist.linear.x
        v_x = v_local * math.cos(self.node.current_yaw)
        v_y = v_local * math.sin(self.node.current_yaw)

        self.node.consensus.broadcast_orca_state(v_x, v_y)

    def trigger_hardware_thread(self):
        if not self.node.is_driving:
            self.node.is_driving = True
            threading.Thread(target=self.execute_physical_tasks, daemon=True).start()

    def check_battery_status_loop(self):
        self.node.agent.check_battery_status()

    # --- PHYSICAL EXECUTION BLOCK ---

    def get_world_path(self, start_x, start_y, target_x, target_y):
        if math.hypot(target_x - start_x, target_y - start_y) < 0.5:
            return [(target_x, target_y)]
        navigator = self.node.global_navigator
        start_col, start_row = navigator.world_to_grid(start_x, start_y)
        target_col, target_row = navigator.world_to_grid(target_x, target_y)
        
        grid_path = navigator.compute_path((start_col, start_row), (target_col, target_row))
        if not grid_path:
            return []
            
        return [navigator.grid_to_world(col, row) for col, row in grid_path]

    def execute_physical_tasks(self):
        
        while self.node.agent.bundle and rclpy.ok():
            task_dict = self.node.agent.bundle[0]
            task_id = task_dict['id']
            
            # Fetch coordinates directly from the bundle memory
            pick_x, pick_y = task_dict['pick_x'], task_dict['pick_y']
            drop_x, drop_y = task_dict['drop_x'], task_dict['drop_y']
            
            # 1. Pre-calculate D* Lite paths for both legs
            path_to_pickup = self.get_world_path(self.node.current_x, self.node.current_y, pick_x, pick_y)
            path_to_drop = self.get_world_path(pick_x, pick_y, drop_x, drop_y)
            
            if not path_to_pickup or not path_to_drop:
                self.node.get_logger().error(f"[{task_id}] Path blocked. Abandoning task.")
                self.node.agent.abandon_current_task()
                continue

            self.node.active_trajectory = path_to_pickup + path_to_drop
            
            # 2. Leg 1: Move to Pickup
            self.node.get_logger().info(f"[{task_id}] Moving to pickup...")
            if not self.follow_path(path_to_pickup):
                if getattr(self.node, 'cancel_current_path', False):
                    self.node.cancel_current_path = False
                    continue  # Safely loop back to pick up the new charging task
                self.node.agent.abandon_current_task()
                self.node.active_trajectory = []
                continue

            self.node.get_logger().info(f"[{task_id}] Arrived at pickup. Loading...")
            time.sleep(task_dict.get('t_load', 2.0))

            if task_id in self.node.consensus.ledger_data:
                self.node.consensus.ledger_data[task_id]["Task_starting_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.node.consensus.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.node.consensus.ledger_data[task_id]))

            self.node.agent.advance_leg()

            # 3. Leg 2: Drop-off
            self.node.active_trajectory = path_to_drop
            self.node.get_logger().info(f"[{task_id}] Moving to drop-off...")
            if not self.follow_path(path_to_drop):
                if getattr(self.node, 'cancel_current_path', False):
                    self.node.cancel_current_path = False
                    continue
                self.node.agent.abandon_current_task()
                self.node.active_trajectory = []
                continue
                
            self.node.get_logger().info(f"[{task_id}] Arrived at drop-off. Unloading...")
            # --- NEW: Block thread if charging, otherwise normal sleep ---
            if task_dict.get('is_exclusive'):
                while self.node.battery < 100.0 and rclpy.ok():
                    time.sleep(1.0)
            else:
                time.sleep(task_dict.get('t_unload', 2.0))
            
            self.node.agent.advance_leg()
            self.node.consensus.finish_task(task_id)
            self.node.active_trajectory = []
            
        self.node.is_driving = False
        self.node.active_trajectory = []
        self.node.get_logger().info("Bundle empty. Idling.")

    def follow_path(self, world_path):
        """The 20 Hz ORCA Execution Loop"""
        local_driver = LocalPlanner(v_max=self.node.v_linear,
                                    yield_check=lambda: self.node.agent.yield_flag,)
        local_driver.on_path(world_path)

        orca = ORCAFilter(epsilon=0.1, radius=0.3, tau=2.0, v_max=self.node.v_linear)
        vel_msg = Twist()

        while rclpy.ok() and local_driver.is_active:
            if getattr(self.node, 'cancel_current_path', False):
                break
            
            local_driver.on_odometry(self.node.current_x, self.node.current_y, theta=self.node.current_yaw)
            vx_pref, vy_pref = local_driver.step()

            current_time = time.time()
            active_neighbors = []

            with self.peer_lock:
                for peer_id, state_tuple in list(self.peer_states.items()):
                    peer_msg, recv_time = state_tuple

                    if current_time - recv_time > 0.5:
                        del self.peer_states[peer_id]
                    else:
                        active_neighbors.append(Neighbor(
                            x=peer_msg.x, y=peer_msg.y,
                            vx=peer_msg.vx, vy=peer_msg.vy, radius=peer_msg.radius
                        ))
                        
                # --- DEBUG PRINT ---
                if active_neighbors:
                    neighbor_info = ", ".join([f"{peer_id} at ({n.x:.2f}, {n.y:.2f})" for peer_id, (peer_msg, _) in self.peer_states.items() for n in active_neighbors if n.x == peer_msg.x])
                    self.node.get_logger().info(f"ORCA tracking {len(active_neighbors)} neighbors: {neighbor_info}")
                # -------------------
            '''
            # Fold static obstacles into the same neighbor list ORCA sees.
            # vx=vy=0 because they don't move. responsibility=1.0 because a
            # static obstacle can't take its share of avoidance the way a
            # reciprocating peer does — the ego robot has to do 100% of the
            # steering around it instead of the usual 50/50 split.
            '''
            for obs_x, obs_y, obs_radius in self.node.static_obstacles:
                active_neighbors.append(Neighbor(
                    x=obs_x, y=obs_y,
                    vx=0.0, vy=0.0,
                    radius=obs_radius,
                    responsibility=1.0,
                ))

            v_safe, omega_safe = orca.step(
                x=self.node.current_x, y=self.node.current_y, theta=self.node.current_yaw, 
                v_pref=(vx_pref, vy_pref), neighbors=active_neighbors
            )

            vel_msg.linear.x = float(v_safe)
            vel_msg.angular.z = float(omega_safe)
            self.cmd_pub.publish(vel_msg)

            time.sleep(0.05)

        # GUARANTEE MOTORS STOP FIRST
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_pub.publish(vel_msg)

        # THEN CHECK IF WE ABORTED
        if getattr(self.node, 'cancel_current_path', False):
            return False

        return True