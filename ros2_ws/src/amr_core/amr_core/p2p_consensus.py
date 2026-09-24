import rclpy
from rclpy.serialization import serialize_message, deserialize_message
import zenoh, json, threading, time
from datetime import datetime
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from fleet_interfaces.msg import DispatchTask, TaskBid, AMRTelemetry, LocalTrajectory, Pose2D, FleetVelocity
from geometry_msgs.msg import Point32



class P2PConsensus:
    def __init__(self, node, hw_interface):
        self.node = node
        self.hw = hw_interface
        
        self.task_registry = {}
        self.ledger_data = {}
        
        # Zenoh Session Management
        self.z_session = zenoh.open(zenoh.Config())
        
        task_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,depth=100)
        
        # ROS Communication
        #Publisher
        self.bid_pub = self.z_session.declare_publisher('fleet_tasks_bids')
        self.telemetry_pub = self.z_session.declare_publisher('fleet_status')
        self.traj_pub = self.z_session.declare_publisher('fleet_trajectories')
        self.orca_pub = self.z_session.declare_publisher(f"fleet/orca/{self.node.get_name()}")

        #subscriber
        self.task_sub = self.z_session.declare_subscriber('fleet_tasks', self.on_new_task_received)
        self.bid_sub = self.z_session.declare_subscriber('fleet_tasks_bids', self.on_bid_received)
        self.orca_sub = self.z_session.declare_subscriber("fleet/orca/*", self.on_zenoh_orca_received)

        # Timers
        self.telem_timer = self.node.create_timer(1.0, self.publish_telemetry)
        self.traj_timer = self.node.create_timer(1.5, self.publish_trajectory)
        
    def broadcast_orca_state(self, v_x: float=0, v_y: float=0):
        """Serializes the ROS 2 message to binary and publishes over Zenoh."""
        orca_msg = FleetVelocity()
        orca_msg.robot_id = self.node.get_name()
        orca_msg.x = float(self.node.current_x)
        orca_msg.y = float(self.node.current_y)
        orca_msg.vx = float(v_x)
        orca_msg.vy = float(v_y)
        orca_msg.radius = 0.3
        orca_msg.timestamp = float(time.time())
        serialized_orca_msg = serialize_message(orca_msg)
        self.orca_pub.put(serialized_orca_msg)

    def on_zenoh_orca_received(self, sample):
        """Deserializes incoming binary Zenoh payload back to a ROS 2 message."""
        try:
            msg = deserialize_message(sample.payload.to_bytes(), FleetVelocity)
            if msg.robot_id != self.node.get_name():
                self.hw.update_peer_state(msg)
        except Exception as e:
            self.node.get_logger().error(f"Failed to deserialize ORCA message: {e}")

    def publish_telemetry(self):
            msg = AMRTelemetry()
            msg.robot_id = self.node.get_name()
            msg.pose = Pose2D(x=self.node.current_x, y=self.node.current_y, theta=self.node.current_yaw)
            msg.battery_percent = float(self.node.battery)
            msg.system_status = 1 if self.node.is_driving else 0
            msg.is_busy = self.node.is_busy
            
            if self.node.agent.bundle:
                msg.current_task_id = self.node.agent.bundle[0]['id']
            else:
                msg.current_task_id = "IDLE"

            serialised_telemetry_payload = serialize_message(msg)
            self.telemetry_pub.put(serialised_telemetry_payload)
    
    def publish_trajectory(self):
        if self.node.is_driving and self.node.active_trajectory:
            traj_msg = LocalTrajectory()
            traj_msg.robot_id = self.node.get_name()
            
            waypoints = []
            for (wx, wy) in self.node.active_trajectory:
                pt = Point32(x=float(wx), y=float(wy), z=0.0)
                waypoints.append(pt)
                
            traj_msg.future_waypoints = waypoints
            serialize_traj_payload = serialize_message(traj_msg)
            self.traj_pub.put(serialize_traj_payload)

    def broadcast_matrices(self):
        if not self.node.agent.bundle:
            v_x, v_y = self.node.current_x, self.node.current_y
        else:
            last_task = self.node.agent.bundle[-1]
            v_x = last_task["drop_x"]
            v_y = last_task["drop_y"]
            
        payload = self.node.agent.make_payload(v_x, v_y)
        
        bid_msg = TaskBid()
        bid_msg.robot_id = self.node.get_name()
        bid_msg.y_matrix = json.dumps(payload["Y_ledger"])
        bid_msg.z_matrix = json.dumps(payload["Z_ledger"])
        extended_t_data = {
            "T": payload["T"],
            "path_segment": payload.get("path_segment", []),
            "task_reward": payload.get("task_reward", 0.0)
        }
        bid_msg.t_matrix = json.dumps(extended_t_data)
        
        # Serialize the ROS 2 object to raw C-struct bytes
        binary_payload = serialize_message(bid_msg)
        
        # Send over Zenoh peer-to-peer
        self.bid_pub.put(binary_payload)

    def on_new_task_received(self, sample: zenoh.Sample):
        # Extract binary payload from Zenoh sample and reconstruct the ROS 2 object
        msg = deserialize_message(sample.payload.to_bytes(), DispatchTask)
        
        self.node.get_logger().info(f"New task broadcast received: {msg.task_id}")
        
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
        
        self.task_registry[msg.task_id] = task_dict
        
        # Pass tasks directly to the new bundle builder
        self.node.agent.build_bundle(open_tasks=list(self.task_registry.values()))
        self.broadcast_matrices()
        
        threading.Timer(2.0, self.evaluate_consensus, args=[msg.task_id]).start()

    def on_bid_received(self, sample: zenoh.Sample):
        # Extract binary payload from Zenoh sample and reconstruct the ROS 2 object
        msg = deserialize_message(sample.payload.to_bytes(), TaskBid)

        if msg.robot_id == self.node.get_name():
            return
        extended_t_data = json.loads(msg.t_matrix)
            
        payload = {
            "id": msg.robot_id,
            "Y_ledger": json.loads(msg.y_matrix),
            "Z_ledger": json.loads(msg.z_matrix),
            "T": extended_t_data.get("T", {}),
            "path_segment": extended_t_data.get("path_segment", []),
            "task_reward": extended_t_data.get("task_reward", 0.0)
        }
        
        changed = self.node.agent.receive_broadcast(payload)
        if changed:
            self.broadcast_matrices()
        
    def evaluate_consensus(self, task_id: str):
        if task_id not in self.node.agent.Z:
            return

        winning_robot = self.node.agent.Z[task_id]

        if winning_robot == self.node.get_name():
            self.node.get_logger().info(f"WON TASK {task_id}! Adding to bundle.")

            self.ledger_data[task_id] = {
                "Task_id": task_id,
                "Amr_completed": self.node.get_name(),
                "Bid_completion_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))

            self.hw.trigger_hardware_thread()

        else:
            self.node.get_logger().info(f"Lost task {task_id} to {winning_robot}.")
            if task_id in self.task_registry:
                del self.task_registry[task_id]

    def finish_task(self, task_id: str):
        self.node.get_logger().info(f"COMPLETED TASK {task_id}. Broadcasting final ledger update.")

        if task_id in self.ledger_data:
            self.ledger_data[task_id]["Task_completion_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(self.ledger_data[task_id]))
            del self.ledger_data[task_id]

        if task_id in self.task_registry:
            del self.task_registry[task_id]

        self.node.agent.close_task(task_id)

    def shutdown(self):
        self.z_session.close()