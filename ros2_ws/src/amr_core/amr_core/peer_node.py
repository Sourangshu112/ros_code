import rclpy
from rclpy.node import Node
import random

# Import your custom messages
from fleet_interfaces.msg import AMRTelemetry, DispatchTask, TaskBid
from geometry_msgs.msg import Pose2D

class PeerNode(Node):
    def __init__(self):
        super().__init__('peer_node')
        
        # Current state of the AMR
        self.is_busy = False 
        self.current_x = round(random.uniform(0.0, 10.0), 2)
        self.current_y = round(random.uniform(0.0, 10.0), 2)
        
        # Publishers
        self.telemetry_pub = self.create_publisher(AMRTelemetry, 'fleet_status', 10)
        self.bid_pub = self.create_publisher(TaskBid, 'fleet_bids', 10)
        
        # Subscribers
        self.task_sub = self.create_subscription(DispatchTask, 'fleet_tasks', self.on_new_task_received, 10)
        
        # Timers
        self.timer = self.create_timer(1.0, self.publish_telemetry)

    def publish_telemetry(self):
        msg = AMRTelemetry()
        msg.robot_id = self.get_name()
        msg.pose = Pose2D(x=self.current_x, y=self.current_y, theta=0.0)
        msg.battery_percent = 90
        msg.system_status = 0
        self.telemetry_pub.publish(msg)

    def on_new_task_received(self, msg: DispatchTask):
        print(f"[{self.get_name()}] New task broadcast received: {msg.task_id}", flush=True)
        
        # Reject the bid immediately if the robot is already executing a task
        if self.is_busy:
            print(f"[{self.get_name()}] Busy. Ignoring task {msg.task_id}.", flush=True)
            return
            
        # Execute the calculation function
        cost = self.calculate_bid_cost(msg)
        
        # Publish the bid to the mesh network
        bid_msg = TaskBid()
        bid_msg.task_id = msg.task_id
        bid_msg.robot_id = self.get_name()
        bid_msg.bid_cost = float(cost)
        
        self.bid_pub.publish(bid_msg)
        print(f"[{self.get_name()}] Submitted bid for {msg.task_id} with cost: {cost}", flush=True)

    def calculate_bid_cost(self, task_msg: DispatchTask) -> float:
        """
        PLACEHOLDER FOR TEAMMATE:
        Write the custom bid calculation logic here. 
        Lower cost = better bid.
        
        Available inputs:
        - self.current_x, self.current_y (Robot's current position)
        - task_msg.pickup_coordinates.x, task_msg.pickup_coordinates.y
        - task_msg.drop_coordinates.x, task_msg.drop_coordinates.y
        - task_msg.is_priority (Boolean)
        """
        # Temporary random distance simulation
        simulated_distance = random.uniform(1.0, 15.0)
        return round(simulated_distance, 2)


def main(args=None):
    rclpy.init(args=args)
    node = PeerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()