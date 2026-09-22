import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import json, os

from amr_core.battery_manager import BatteryManager
from amr_core.cbba_agent import CBBANode
from amr_core.ros_hardware_interface import ROSHardwareInterface
from amr_core.p2p_consensus import P2PConsensus
from amr_core.navigator import DStarLitePlanner

_SHARE_DIR = get_package_share_directory('amr_core')
_CONFIG_PATH = os.path.join(_SHARE_DIR, 'config', 'data_models.json')

costmap_path = os.path.join(_SHARE_DIR, 'config', 'costmap.json')

with open(_CONFIG_PATH, "r") as _f:
    _CONFIG = json.load(_f)

SYSTEM_CONSTANTS = _CONFIG["system_constants"]
OPEN_TASKS = _CONFIG["open_tasks"]
V_LINEAR = SYSTEM_CONSTANTS["v_linear"]
V_ANGULAR = SYSTEM_CONSTANTS["v_angular"]
E_RATE = SYSTEM_CONSTANTS["e_rate"]
C_RATE = SYSTEM_CONSTANTS["c_rate"]
LAMBDA_VAL = SYSTEM_CONSTANTS["lambda_val"]
BATTERY_SAFETY_THRESHOLD = SYSTEM_CONSTANTS["battery_safety_threshold"]
BATTERY_FULL = SYSTEM_CONSTANTS.get("battery_full", 100.0)

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

        # 3. Initialize current coordinates to the absolute offset
        self.current_x = self.offset_x
        self.current_y = self.offset_y
        self.current_yaw = self.offset_theta

        self.is_driving = False
        self.is_busy = False
        self.active_trajectory = []

        #5 System constants
        self.v_linear = V_LINEAR
        self.v_angular = V_ANGULAR
        self.e_rate = E_RATE
        self.c_rate = C_RATE
        self.lambda_val = LAMBDA_VAL
        self.battery_threshold = BATTERY_SAFETY_THRESHOLD
        self.battery = BATTERY_FULL

        # 4. Dependency Injection & Module Instantiation
        self.agent = CBBANode(self)
        self.hw_interface = ROSHardwareInterface(self)
        self.consensus = P2PConsensus(self, self.hw_interface)
        self.battery_manager = BatteryManager(self, driving_multiplier=3.0)

        #5 Other classes
        self.global_navigator = DStarLitePlanner.from_costmap(costmap_path, robot_radius=0.3)

        self.get_logger().info(f"[{self.get_name()}] Peer Node Online. Ready for tasks.")

def main(args=None):
    rclpy.init(args=args)
    node = PeerNode()
    
    # 5. MultiThreadedExecutor Setup
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.consensus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()