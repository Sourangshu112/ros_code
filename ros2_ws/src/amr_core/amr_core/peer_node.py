import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
import random
import threading
import time
import json
import math
from datetime import datetime
import zenoh

# Import custom messages
from fleet_interfaces.msg import AMRTelemetry, DispatchTask, TaskBid
from fleet_interfaces.msg import Pose2D
from nav_msgs.msg import Odometry

# External scripts
from amr_core.amr_controller import execute_physical_task
# from bidding_algo import calculate_bid_cost

class PeerNode(Node):
    def __init__(self):
        super().__init__('peer_node')
        
        # Current state of the AMR (now initialized at 0, updated by Gazebo)
        self.is_busy = False 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Internal memory
        self.active_bids = {}
        self.task_registry = {}  # Stores full task payloads to retrieve coordinates after winning
        self.ledger_data = {}    # Stores Zenoh payloads incrementally to prevent overwriting
        
        # Initialize Zenoh session
        self.z_session = zenoh.open(zenoh.Config())
        
        # QoS Profiles
        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # Publishers
        self.telemetry_pub = self.create_publisher(AMRTelemetry, '/fleet_status', 10)
        self.bid_pub = self.create_publisher(TaskBid, '/fleet_tasks_bids', 10) 
        
        # Subscribers
        self.task_sub = self.create_subscription(DispatchTask, '/fleet_tasks', self.on_new_task_received, task_qos)
        self.bid_sub = self.create_subscription(TaskBid, '/fleet_tasks_bids', self.on_bid_received, 10)
        
        # Gazebo Odometry Subscriber (Matches launch file bridge: /robot_name/odom)
        self.odom_sub = self.create_subscription(
            Odometry, 
            f'/{self.get_name()}/odom', 
            self.odom_callback, 
            qos_profile_sensor_data
        )
        
        # Timers
        self.timer = self.create_timer(1.0, self.publish_telemetry)
        
        self.get_logger().info(f"[{self.get_name()}] Peer Node Online. Ready for tasks.")

    def odom_callback(self, msg: Odometry):
        """Updates internal coordinates dynamically from Gazebo."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        print(self.current_x , flush=True)
        print(self.current_y , flush=True)
        
        # Extract yaw from quaternion for physical turning control
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_telemetry(self):
        msg = AMRTelemetry()
        msg.robot_id = self.get_name()
        msg.pose = Pose2D(x=self.current_x, y=self.current_y, theta=self.current_yaw)
        msg.battery_percent = 90
        msg.system_status = 1 if self.is_busy else 0
        self.telemetry_pub.publish(msg)

    def on_new_task_received(self, msg: DispatchTask):
        self.get_logger().info(f"New task broadcast received: {msg.task_id}")
        
        if self.is_busy:
            self.get_logger().info(f"Busy. Ignoring task {msg.task_id}.")
            return
            
        # Register the full task message so we can access coordinates later if we win
        self.task_registry[msg.task_id] = msg
            
        if msg.task_id not in self.active_bids:
            self.active_bids[msg.task_id] = {}
            
        cost = self.calculate_bid_cost(msg)
        
        self.active_bids[msg.task_id][self.get_name()] = cost
        
        bid_msg = TaskBid()
        bid_msg.task_id = msg.task_id
        bid_msg.robot_id = self.get_name()
        bid_msg.bid_cost = float(cost)
        
        self.bid_pub.publish(bid_msg)
        self.get_logger().info(f"Submitted bid for {msg.task_id} with cost: {cost}")
        
        threading.Timer(2.0, self.evaluate_consensus, args=[msg.task_id]).start()

    def on_bid_received(self, msg: TaskBid):
        if msg.task_id not in self.active_bids:
            self.active_bids[msg.task_id] = {}
        self.active_bids[msg.task_id][msg.robot_id] = msg.bid_cost

    def evaluate_consensus(self, task_id: str):
        if task_id not in self.active_bids or self.is_busy:
            return
            
        bids = self.active_bids[task_id]
        winning_robot = min(bids, key=bids.get)
        
        if winning_robot == self.get_name():
            self.get_logger().info(f"WON TASK {task_id}! Logging bid completion...")
            self.is_busy = True
            
            # Step 1: Log Bid Completion immediately 
            self.ledger_data[task_id] = {
                "Task_id": task_id,
                "Amr_completed": self.get_name(),
                "Bid_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))
            
            # Step 2: Fetch the task coordinates and launch the physical execution thread
            task_msg = self.task_registry[task_id]
            threading.Thread(target=execute_physical_task, args=(self, task_msg), daemon=True).start()
            
        else:
            self.get_logger().info(f"Lost task {task_id} to {winning_robot}.")
            # Clean up memory if lost
            if task_id in self.task_registry:
                del self.task_registry[task_id]
            
        del self.active_bids[task_id]

    def finish_task(self, task_id: str):
        self.get_logger().info(f"COMPLETED TASK {task_id}. Broadcasting final ledger update.")
        
        # Step 3: Append Task Completion Time and push final ledger update
        self.ledger_data[task_id]["Task_completion_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))
        
        # Free up memory and state
        del self.task_registry[task_id]
        del self.ledger_data[task_id]
        self.is_busy = False

    def calculate_bid_cost(self, task_msg: DispatchTask) -> float:
        return round(random.uniform(1.0, 20.0), 2)
    # calculate_bid_cost(self, task_msg: DispatchTask)


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