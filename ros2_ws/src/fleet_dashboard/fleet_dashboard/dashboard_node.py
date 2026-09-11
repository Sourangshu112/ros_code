import rclpy
from rclpy.node import Node
import threading
from flask import Flask
from flask_socketio import SocketIO

from fleet_interfaces.msg import AMRTelemetry, DispatchTask
from geometry_msgs.msg import Pose2D

# Initialize Flask and SocketIO
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global reference to allow Flask to interact with the ROS 2 node
ros_node_instance = None

class DashboardNode(Node):
    def __init__(self):
        super().__init__('fleet_dashboard')
        self.subscription = self.create_subscription(AMRTelemetry, 'fleet_status', self.listener_callback, 10)
        
        # Updated publisher using the custom DispatchTask message
        self.task_publisher = self.create_publisher(DispatchTask, 'fleet_tasks', 10)
        
        print("[Dashboard] WebSocket Server Active. Listening for mesh data...", flush=True)

    def listener_callback(self, msg):
        data = {
            "id": msg.robot_id,
            "x": msg.pose.x,
            "y": msg.pose.y,
            "battery": msg.battery_percent,
            "status" : msg.system_status
        }
        # print(f"[Backend] Got ROS data: {data['id']} at X:{data['x']}", flush=True)
        socketio.emit('fleet_update', data)

# Socket.IO Listener for frontend task dispatch
@socketio.on('issue_task')
def handle_issue_task(payload):
    if ros_node_instance is None:
        print("[Backend] Warning: ROS node not ready to publish tasks.", flush=True)
        return

    try:
        # Construct the custom message from the incoming JSON payload
        msg = DispatchTask()
        msg.task_id = payload.get('task_id', 'UNKNOWN_TASK')
        msg.is_priority = payload.get('priority', False)

        msg.pickup_coordinates = Pose2D(
            x=float(payload['pickup'][0]),
            y=float(payload['pickup'][1]),
            theta=0.0
        )
        msg.drop_coordinates = Pose2D(
            x=float(payload['drop'][0]),
            y=float(payload['drop'][1]),
            theta=0.0
        )

        ros_node_instance.task_publisher.publish(msg)
        print(f"[Dashboard] Issued task {msg.task_id} via Socket.IO", flush=True)
        
    except KeyError as e:
        print(f"[Backend] Malformed task payload missing key: {e}", flush=True)

def ros_spin_thread():
    global ros_node_instance
    rclpy.init(args=None)
    ros_node_instance = DashboardNode()
    
    rclpy.spin(ros_node_instance)
    
    ros_node_instance.destroy_node()
    rclpy.shutdown()

def main(args=None):
    # 1. Start ROS 2 node in a background thread
    threading.Thread(target=ros_spin_thread, daemon=True).start()
    
    # 2. Start the Flask WebSocket server on the main thread
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()