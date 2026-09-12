import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import random
import threading
import time
import json
from datetime import datetime
import zenoh

# Import custom messages
from fleet_interfaces.msg import AMRTelemetry, DispatchTask, TaskBid
from fleet_interfaces.msg import Pose2D

class PeerNode(Node):
    def __init__(self):
        super().__init__('peer_node')
        
        # Current state of the AMR
        self.is_busy = False 
        self.current_x = round(random.uniform(0.0, 10.0), 2)
        self.current_y = round(random.uniform(0.0, 10.0), 2)
        
        # Internal memory for peer-to-peer consensus
        self.active_bids = {}
        
        # Initialize Zenoh session for distributed storage commands
        self.z_session = zenoh.open(zenoh.Config())
        
        # QoS Profile to match the Dashboard's Transient Local task publishing
        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # Publishers
        self.telemetry_pub = self.create_publisher(AMRTelemetry, '/fleet_status', 10)
        self.bid_pub = self.create_publisher(TaskBid, '/fleet_tasks_bids', 10) # Fixed discrepancy
        
        # Subscribers
        self.task_sub = self.create_subscription(DispatchTask, '/fleet_tasks', self.on_new_task_received, task_qos)
        self.bid_sub = self.create_subscription(TaskBid, '/fleet_tasks_bids', self.on_bid_received, 10)
        
        # Timers
        self.timer = self.create_timer(1.0, self.publish_telemetry)
        
        self.get_logger().info(f"[{self.get_name()}] Peer Node Online. Ready for tasks.")

    def publish_telemetry(self):
        msg = AMRTelemetry()
        msg.robot_id = self.get_name()
        msg.pose = Pose2D(x=self.current_x, y=self.current_y, theta=0.0)
        msg.battery_percent = 90
        msg.system_status = 1 if self.is_busy else 0
        self.telemetry_pub.publish(msg)

    def on_new_task_received(self, msg: DispatchTask):
        self.get_logger().info(f"New task broadcast received: {msg.task_id}")
        
        # Reject the bid immediately if the robot is already executing a task
        if self.is_busy:
            self.get_logger().info(f"Busy. Ignoring task {msg.task_id}.")
            return
            
        # Initialize internal ledger for this specific task
        if msg.task_id not in self.active_bids:
            self.active_bids[msg.task_id] = {}
            
        # Execute the calculation function
        cost = self.calculate_bid_cost(msg)
        
        # Store own bid in internal memory
        self.active_bids[msg.task_id][self.get_name()] = cost
        
        # Publish the bid to the mesh network
        bid_msg = TaskBid()
        bid_msg.task_id = msg.task_id
        bid_msg.robot_id = self.get_name()
        bid_msg.bid_cost = float(cost)
        
        self.bid_pub.publish(bid_msg)
        self.get_logger().info(f"Submitted bid for {msg.task_id} with cost: {cost}")
        
        # Trigger the consensus evaluation after a 2-second bidding window
        threading.Timer(2.0, self.evaluate_consensus, args=[msg.task_id]).start()

    def on_bid_received(self, msg: TaskBid):
        """Records bids from other peers on the network to establish consensus."""
        if msg.task_id not in self.active_bids:
            self.active_bids[msg.task_id] = {}
        
        # Update internal memory with the peer's bid
        self.active_bids[msg.task_id][msg.robot_id] = msg.bid_cost

    def evaluate_consensus(self, task_id: str):
        """Evaluates all collected bids to determine the winner without a central server."""
        if task_id not in self.active_bids or self.is_busy:
            return
            
        bids = self.active_bids[task_id]
        
        # Find the robot with the lowest cost
        winning_robot = min(bids, key=bids.get)
        
        if winning_robot == self.get_name():
            self.get_logger().info(f"WON TASK {task_id}! Updating ledger and starting work...")
            self.is_busy = True
            
            # Execute Zenoh put command to update the decentralized ledger
            payload = {
                "Task_id": task_id,
                "Amr_completed": self.get_name(),
                "Bid_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Task_starting_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(payload))
            
            # Simulate the physical work of moving the pallet (e.g., 5 seconds)
            threading.Timer(5.0, self.finish_task, args=[task_id]).start()
        else:
            self.get_logger().info(f"Lost task {task_id} to {winning_robot}.")
            
        # Clean up internal memory for this task
        del self.active_bids[task_id]

    def finish_task(self, task_id: str):
        """Marks the task as complete and logs the final timestamp."""
        self.get_logger().info(f"COMPLETED TASK {task_id}. Broadcasting final ledger update.")
        
        payload = {
            "Task_id": task_id,
            "Task_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(payload))
        
        # Free up the AMR for the next task
        self.is_busy = False

    def calculate_bid_cost(self, task_msg: DispatchTask) -> float:
        """
        PLACEHOLDER FOR TEAMMATE (Dipam/Akash/Arani):
        Write the custom bid calculation logic here. 
        Lower cost = better bid.
        
        Available inputs:
        - self.current_x, self.current_y (Robot's current position)
        - task_msg.pickup_coordinates.x, task_msg.pickup_coordinates.y
        - task_msg.drop_coordinates.x, task_msg.drop_coordinates.y
        - task_msg.is_priority (Boolean)
        """
        # Returns a random number between 1 and 20 as requested
        return round(random.uniform(1.0, 20.0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = PeerNode()
    
    # Use a MultiThreadedExecutor so Timer callbacks don't block subscriptions
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