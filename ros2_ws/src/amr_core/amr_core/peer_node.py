import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
import threading
import time
import json
import math
from datetime import datetime
import zenoh
from ament_index_python.packages import get_package_share_directory
import os

# Import custom messages
from fleet_interfaces.msg import AMRTelemetry, DispatchTask, TaskBid
from fleet_interfaces.msg import Pose2D
from fleet_interfaces.msg import LocalTrajectory
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

# External scripts
from amr_core.amr_controller import execute_physical_tasks
from amr_core.cbba_agent import CBBANode
from amr_core.navigator import AStarPlanner

class PeerNode(Node):
    def __init__(self):
        super().__init__('peer_node')

        # 1. Declare ROS 2 parameters for absolute spawn coordinates
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_theta', 0.0)



        # 2. Fetch the values
        self.offset_x = self.get_parameter('spawn_x').value
        self.offset_y = self.get_parameter('spawn_y').value
        self.offset_theta = self.get_parameter('spawn_theta').value

        # print(f"x: {self.offset_x}, y: {self.offset_y}", flush=True)
        
        self.is_busy = False
        self.current_task_id = ""

        # 3. Initialize current coordinates to the offset instead of 0.0
        self.current_x = self.offset_x
        self.current_y = self.offset_y
        self.current_yaw = self.offset_theta

        self.is_driving = False
        self.active_trajectory = []
        
        # Internal memory
        self.task_registry = {}  
        self.ledger_data = {}
        self.pending_bids = {} 

        # Initialize the Global Navigator using the costmap file
        share_dir = get_package_share_directory('amr_core')
        costmap_path = os.path.join(share_dir, 'config', 'costmap.json')
        self.global_navigator = AStarPlanner.from_costmap(costmap_path)   
        
        # Initialize CBBA Agent
        robot_dict = {
            "id": self.get_name(), 
            "x": self.offset_x, 
            "y": self.offset_y, 
            "theta": self.offset_theta, 
            "battery": 100.0
        }
        self.agent = CBBANode(robot_dict, num_tasks=0, num_robots=3, navigator=self.global_navigator)

        self.z_session = zenoh.open(zenoh.Config())
        
        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # Publishers and Subscribers
        self.telemetry_pub = self.create_publisher(AMRTelemetry, '/fleet_status', 10)
        self.bid_pub = self.create_publisher(TaskBid, '/fleet_tasks_bids', 10) 
        self.traj_pub = self.create_publisher(LocalTrajectory, '/fleet_trajectories', 10)
        self.traj_timer = self.create_timer(1.5, self.publish_trajectory)
        self.task_sub = self.create_subscription(DispatchTask, '/fleet_tasks', self.on_new_task_received, task_qos)
        self.bid_sub = self.create_subscription(TaskBid, '/fleet_tasks_bids', self.on_bid_received, 10)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.get_name()}/odom', self.odom_callback, qos_profile_sensor_data)
        
        self.timer = self.create_timer(1.0, self.publish_telemetry)
        self.get_logger().info(f"[{self.get_name()}] Peer Node Online. Ready for tasks.")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update CBBA Agent's physical state so future bids are accurate
        self.agent.x = self.current_x
        self.agent.y = self.current_y
        self.agent.theta = self.current_yaw

    def publish_telemetry(self):
        msg = AMRTelemetry()
        msg.robot_id = self.get_name()
        msg.pose = Pose2D(x=self.current_x, y=self.current_y, theta=self.current_yaw)
        msg.battery_percent = self.agent.battery
        msg.system_status = 1 if self.is_driving else 0
        msg.is_busy = self.is_busy
        if self.agent.bundle:
            msg.current_task_id = self.agent.bundle[0]['id']
        else:
            msg.current_task_id = "IDLE"
        self.telemetry_pub.publish(msg)

    def publish_trajectory(self):
        # Only publish if driving and a valid path sequence exists
        if self.is_driving and self.active_trajectory:
            traj_msg = LocalTrajectory()
            traj_msg.robot_id = self.get_name()
            
            waypoints = []
            for (wx, wy) in self.active_trajectory:
                pt = Point32()
                pt.x = float(wx)
                pt.y = float(wy)
                pt.z = 0.0
                waypoints.append(pt)
                
            traj_msg.future_waypoints = waypoints
            self.traj_pub.publish(traj_msg)
            # print(waypoints, flush=True);

    def on_new_task_received(self, msg: DispatchTask):
        self.get_logger().info(f"New task broadcast received: {msg.task_id}")
        
        # Max Bundle Size Limit
        if len(self.agent.bundle) >= 3:
            self.get_logger().info(f"Bundle full. Ignoring task {msg.task_id}.")
            return
            
        self.task_registry[msg.task_id] = msg
        
        # 1. Determine Virtual Origin (Where will the robot be when it finishes its current bundle?)
        if not self.agent.bundle:
            v_x, v_y, v_theta = self.current_x, self.current_y, self.current_yaw
        else:
            last_task = self.agent.bundle[-1]
            v_x = last_task["drop_x"]
            v_y = last_task["drop_y"]
            v_theta = self.agent._final_heading_after_task(last_task)
    
        task_dict = {
            "id": msg.task_id,
            "pick_x": msg.pickup_coordinates.x,
            "pick_y": msg.pickup_coordinates.y,
            "drop_x": msg.drop_coordinates.x,
            "drop_y": msg.drop_coordinates.y,
            "t_load": 2.0,
            "t_unload": 2.0,
            "reward": 1000 if msg.is_priority else 100
        }
        
        self.pending_bids[msg.task_id] = task_dict
    
        # 2. Calculate bid from the virtual state
        bid_value = self.agent.calculate_bid(
            task_dict, origin_x=v_x, origin_y=v_y, origin_theta=v_theta
        )
    
        if bid_value <= 0.0:
            self.get_logger().warning(f"Task {msg.task_id} unreachable from bundle end. Rejecting.")
            del self.task_registry[msg.task_id]
            del self.pending_bids[msg.task_id]
            return
            
        self.agent.Y[msg.task_id] = bid_value
        self.agent.Z[msg.task_id] = self.get_name()
        
        self.broadcast_matrices()
        self.get_logger().info(f"Submitted bid for {msg.task_id} with score: {bid_value:.2f}")
        import threading
        threading.Timer(2.0, self.evaluate_consensus, args=[msg.task_id]).start()

    def broadcast_matrices(self):
        """Serializes and broadcasts the memory matrices over ROS 2."""
        bid_msg = TaskBid()
        bid_msg.robot_id = self.get_name()
        bid_msg.y_matrix = json.dumps(self.agent.Y)
        bid_msg.z_matrix = json.dumps(self.agent.Z)
        bid_msg.t_matrix = json.dumps(self.agent.T)
        self.bid_pub.publish(bid_msg)

    def on_bid_received(self, msg: TaskBid):
        # Ignore our own broadcast loops
        if msg.robot_id == self.get_name():
            return
            
        # Deserialize matrices
        recv_Y = json.loads(msg.y_matrix)
        recv_Z = json.loads(msg.z_matrix)
        recv_T = json.loads(msg.t_matrix)
        
        # Feed into mathematical consensus
        self.agent.update_consensus(msg.robot_id, recv_Y, recv_Z, recv_T)


    def evaluate_consensus(self, task_id: str):
        if task_id not in self.agent.Z:
            return

        winning_robot = self.agent.Z[task_id]

        if winning_robot == self.get_name():
            self.get_logger().info(f"WON TASK {task_id}! Adding to bundle.")

            # 1. Append task to the sequence
            task_dict = self.pending_bids.pop(task_id)
            self.agent.bundle.append(task_dict)

            # 2. Push Zenoh Log
            self.ledger_data[task_id] = {
                "Task_id": task_id,
                "Amr_completed": self.get_name(),
                "Bid_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))

            # 3. Start hardware thread ONLY if the robot is idling
            if not self.is_driving:
                threading.Thread(target=execute_physical_tasks, args=(self,), daemon=True).start()

        else:
            self.get_logger().info(f"Lost task {task_id} to {winning_robot}.")
            if task_id in self.task_registry:
                del self.task_registry[task_id]
            if task_id in self.pending_bids:
                del self.pending_bids[task_id]

    def finish_task(self, task_id: str):
        self.get_logger().info(f"COMPLETED TASK {task_id}. Broadcasting final ledger update.")

        self.ledger_data[task_id]["Task_completion_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))

        del self.task_registry[task_id]
        del self.ledger_data[task_id]


def main(args=None):
    rclpy.init(args=args)
    node = PeerNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.z_session.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()