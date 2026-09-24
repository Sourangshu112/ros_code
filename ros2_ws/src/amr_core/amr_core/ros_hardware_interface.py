import rclpy
import math
import threading
import time
import json
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from tf2_ros import Buffer, TransformListener

# External module imports
from fleet_interfaces.msg import AMRTelemetry, FleetVelocity
from amr_core.local_planner import LocalPlanner
from amr_core.orca_filter import ORCAFilter, Neighbor

class ROSHardwareInterface:
    def __init__(self, node):
        self.node = node
        self.peer_states = {}
        self.peer_lock = threading.Lock()

        # Ensure mailbox is initialized on the node
        if not hasattr(self.node, 'static_obstacles'):
            self.node.static_obstacles = []

        # TF2 Setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Publishers
        self.cmd_pub = self.node.create_publisher(Twist, f'/{self.node.get_name()}/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.node.create_subscription(
            Odometry, f'/{self.node.get_name()}/odom', self.odom_callback, qos_profile_sensor_data
        )
        self.lidar_sub = self.node.create_subscription(
            LaserScan, f'/{self.node.get_name()}/scan', self.lidar_callback, qos_profile_sensor_data
        )

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
        # print("odom_callback is fired")
        self.node.consensus.broadcast_orca_state(v_x, v_y)

    def lidar_callback(self, msg: LaserScan):
        """
        Filters LiDAR ranges <= 1.5m, transforms points to world coordinates,
        clusters nearby points, and updates self.node.static_obstacles (max 6-8 clusters).
        """
        # 1. Obtain TF2 Transform from sensor frame to world frame (with odometry fallback)
        try:
            # Target frame is odom or map (world frame)
            t = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            # Fallback to internal odometry tracking if TF2 tree is unlinked
            tx = self.node.current_x
            ty = self.node.current_y
            yaw = self.node.current_yaw

        # 2. Downsample and project valid returns into world coordinates
        # Skip rays if array is dense (step=3 for ~360-720 rays) to reduce CPU load
        step = 3 if len(msg.ranges) > 180 else 1
        points = []

        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if 0.95 <= r <= 2.0 and math.isfinite(r):
                angle = msg.angle_min + (i * msg.angle_increment)
                # Polar to sensor Cartesian
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                # Sensor Cartesian to World coordinates
                gx = tx + (lx * math.cos(yaw) - ly * math.sin(yaw))
                gy = ty + (lx * math.sin(yaw) + ly * math.cos(yaw))
                points.append((gx, gy))

        if not points:
            self.node.static_obstacles = []
            return

        # 3. Successive Distance Clustering (O(N) segmentation along the scan sweep)
        clusters = []
        current_cluster = [points[0]]

        for pt in points[1:]:
            prev_pt = current_cluster[-1]
            dist = math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])
            if dist < 0.35:  # Spatial continuity threshold
                current_cluster.append(pt)
            else:
                if len(current_cluster) >= 2:  # Suppress single-ray noise spikes
                    clusters.append(current_cluster)
                current_cluster = [pt]

        if len(current_cluster) >= 2:
            clusters.append(current_cluster)

        # 4. Compute Centroid and Bounding Radius for each cluster
        cluster_data = []
        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            # Bounding circle covering the cluster points + safety padding
            max_dist = max(math.hypot(p[0] - cx, p[1] - cy) for p in cluster)
            radius = max(max_dist, 0.15)  # Enforce minimum radius of 0.15m

            dist_to_robot = math.hypot(cx - tx, cy - ty)
            cluster_data.append((dist_to_robot, (cx, cy, radius)))

        # 5. Restrict to nearest 6 clusters to preserve ORCA LP solvability at 20 Hz
        cluster_data.sort(key=lambda item: item[0])
        bounded_clusters = [data[1] for data in cluster_data[:6]]

        # Atomic overwrite of the shared mailbox
        self.node.static_obstacles = bounded_clusters

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
            
            pick_x, pick_y = task_dict['pick_x'], task_dict['pick_y']
            drop_x, drop_y = task_dict['drop_x'], task_dict['drop_y']
            
            path_to_pickup = self.get_world_path(self.node.current_x, self.node.current_y, pick_x, pick_y)
            path_to_drop = self.get_world_path(pick_x, pick_y, drop_x, drop_y)
            
            if not path_to_pickup or not path_to_drop:
                self.node.get_logger().error(f"[{task_id}] Path blocked. Abandoning task.")
                self.node.agent.abandon_current_task()
                continue

            self.node.active_trajectory = path_to_pickup + path_to_drop
            
            # Leg 1: Pickup
            self.node.get_logger().info(f"[{task_id}] Moving to pickup...")
            if not self.follow_path(path_to_pickup):
                if getattr(self.node, 'cancel_current_path', False):
                    self.node.cancel_current_path = False
                    continue
                self.node.agent.abandon_current_task()
                self.node.active_trajectory = []
                continue

            self.node.get_logger().info(f"[{task_id}] Arrived at pickup. Loading...")
            time.sleep(task_dict.get('t_load', 2.0))

            if task_id in self.node.consensus.ledger_data:
                self.node.consensus.ledger_data[task_id]["Task_starting_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.node.consensus.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.node.consensus.ledger_data[task_id]))

            self.node.agent.advance_leg()

            # Leg 2: Drop-off
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
<<<<<<< HEAD
        # The 20 Hz ORCA Execution Loop
        local_driver = LocalPlanner(v_max=self.node.v_linear,
                                    yield_check=lambda: self.node.agent.yield_flag,)
=======
        """The 20 Hz ORCA Execution Loop"""
        local_driver = LocalPlanner(
            v_max=self.node.v_linear,
            yield_check=lambda: self.node.agent.yield_flag
        )
>>>>>>> a09329cf4bb1051c9cdb15f84221130bdd8b1d68
        local_driver.on_path(world_path)

        orca = ORCAFilter(epsilon=0.15, radius=0.6, tau=2.0, v_max=self.node.v_linear, omega_max=self.node.v_angular)
        vel_msg = Twist()

        debug_tick = 0

        while rclpy.ok() and local_driver.is_active:
            if getattr(self.node, 'cancel_current_path', False):
                break
            
            local_driver.on_odometry(self.node.current_x, self.node.current_y, theta=self.node.current_yaw)
            vx_pref, vy_pref = local_driver.step()

            current_time = time.time()
            active_neighbors = []

            # Add dynamic peer AMRs (responsibility=0.5 reciprocal)
            with self.peer_lock:
                for peer_id, state_tuple in list(self.peer_states.items()):
                    peer_msg, recv_time = state_tuple

                    if current_time - recv_time > 0.5:
                        del self.peer_states[peer_id]
                    else:
                        active_neighbors.append(Neighbor(
                            x=peer_msg.x, y=peer_msg.y,
                            vx=peer_msg.vx, vy=peer_msg.vy, radius=peer_msg.radius,
                            responsibility=0.5
                        ))

            # Add static LiDAR obstacle clusters (responsibility=1.0 non-reciprocal)
            for obs_x, obs_y, obs_radius in self.node.static_obstacles:
                active_neighbors.append(Neighbor(
                    x=obs_x, y=obs_y,
                    vx=0.0, vy=0.0,
                    radius=obs_radius,
                    responsibility=1.0
                ))

            v_safe, omega_safe = orca.step(
                x=self.node.current_x, y=self.node.current_y, theta=self.node.current_yaw, 
                v_pref=(vx_pref, vy_pref), neighbors=active_neighbors
            )

            # omega_clamped = max(-self.node.v_angular, min(self.node.v_angular, float(omega_safe)))

            # --- DEBUG PROBES (Prints every 20 ticks / 1 second) ---
            if debug_tick % 20 == 0:
                target_wp = local_driver.waypoint_array[local_driver.current_target_index]
                self.node.get_logger().info(
                    f"\n--- DEBUG TICK ---\n"
                    f"1. POSE   : x={self.node.current_x:.2f}, y={self.node.current_y:.2f}, yaw={self.node.current_yaw:.2f}\n"
                    f"2. TARGET : x={target_wp[0]:.2f}, y={target_wp[1]:.2f} (Index {local_driver.current_target_index}/{len(local_driver.waypoint_array)})\n"
                    f"3. V_PREF : vx={vx_pref:.2f}, vy={vy_pref:.2f} (Pure Pursuit output)\n"
                    f"4. ORCA   : {len(active_neighbors)} total obstacles mapped.\n"
                    f"5. COMMAND: linear={v_safe:.2f}, angular={omega_safe:.2f}\n"
                    f"------------------"
                )
                if len(self.node.static_obstacles) > 0:
                    self.node.get_logger().info(f"CLOSEST LIDAR OBS: {self.node.static_obstacles[0]}")
            
            debug_tick += 1
            # --------------------------------------------------------

            vel_msg.linear.x = float(v_safe)
            vel_msg.angular.z = float(omega_safe)
            self.cmd_pub.publish(vel_msg)

            time.sleep(0.05)

        # Stop motors when exiting loop
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_pub.publish(vel_msg)

        if getattr(self.node, 'cancel_current_path', False):
            return False

        return True