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
from nav_msgs.msg import Odometry

# External scripts
from amr_core.amr_controller import execute_physical_task
from amr_core.cbba_agent import CBBANode
from amr_core.navigator import AStarPlanner

class PeerNode(Node):
    def __init__(self):
        super().__init__('peer_node')
        
        self.is_busy = False 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Internal memory
        self.task_registry = {}  
        self.ledger_data = {} 

        # Initialize the Global Navigator using the costmap file
        share_dir = get_package_share_directory('amr_core')
        costmap_path = os.path.join(share_dir, 'config', 'costmap.json')
        self.global_navigator = AStarPlanner.from_costmap(costmap_path)   
        
        # Initialize CBBA Agent
        robot_dict = {
            "id": self.get_name(), 
            "x": 0.0, 
            "y": 0.0, 
            "theta": 0.0, 
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
        msg.system_status = 1 if self.is_busy else 0
        self.telemetry_pub.publish(msg)

    def on_new_task_received(self, msg: DispatchTask):
        self.get_logger().info(f"New task broadcast received: {msg.task_id}")
        
        if self.is_busy:
            self.get_logger().info(f"Busy. Ignoring task {msg.task_id}.")
            return
            
        self.task_registry[msg.task_id] = msg
            
        # 1. Calculate Bid (CBBA utilizes a reward-based system, highest bid wins)
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
        bid_value = self.agent.calculate_bid(
            task_dict, origin_x=self.current_x, origin_y=self.current_y, origin_theta=self.current_yaw
        )
        
        # 2. Update local memory to claim the task initially
        self.agent.Y[msg.task_id] = bid_value
        self.agent.Z[msg.task_id] = self.get_name()
        
        # 3. Broadcast entire memory matrix state
        self.broadcast_matrices()
        
        self.get_logger().info(f"Submitted bid for {msg.task_id} with score: {bid_value:.2f}")
        
        # 4. Localized consensus countdown starts
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
        # The localized timer pops. We do not calculate anything here, we just check the ledger.
        if self.is_busy or task_id not in self.agent.Z:
            return
            
        winning_robot = self.agent.Z[task_id]
        
        if winning_robot == self.get_name():
            self.get_logger().info(f"WON TASK {task_id}! Logging bid completion...")
            self.is_busy = True
            
            self.ledger_data[task_id] = {
                "Task_id": task_id,
                "Amr_completed": self.get_name(),
                "Bid_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))
            
            task_msg = self.task_registry[task_id]
            threading.Thread(target=execute_physical_task, args=(self, task_msg), daemon=True).start()
            
        else:
            self.get_logger().info(f"Lost task {task_id} to {winning_robot}.")
            if task_id in self.task_registry:
                del self.task_registry[task_id]

    def finish_task(self, task_id: str):
        self.get_logger().info(f"COMPLETED TASK {task_id}. Broadcasting final ledger update.")
        
        self.ledger_data[task_id]["Task_completion_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))
        
        del self.task_registry[task_id]
        del self.ledger_data[task_id]
        self.is_busy = False

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