import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import threading
from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS
import sqlite3
import json
from datetime import datetime
from fleet_dashboard.storage_manager import init_db, listener
import zenoh
from rclpy.serialization import serialize_message, deserialize_message

from fleet_interfaces.msg import AMRTelemetry, DispatchTask, TaskBid, LocalTrajectory
from fleet_interfaces.msg import Pose2D

# Initialize Flask and SocketIO
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global reference to allow Flask to interact with the ROS 2 node
ros_node_instance = None
DB_FILE = "/app/data/fleet_ledger.db"

# --- SQLite Database Helper Functions ---
def db_upsert_task(task_data):
    """Inserts the new task into the database immediately upon creation."""
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO tasks_ledger (
            Task_id, Task, Task_issue_time
        ) VALUES (?, ?, ?)
        ON CONFLICT(Task_id) DO UPDATE SET
            Task = COALESCE(excluded.Task, tasks_ledger.Task),
            Task_issue_time = COALESCE(excluded.Task_issue_time, tasks_ledger.Task_issue_time)
    ''', (
        task_data['Task_id'],
        json.dumps(task_data['Task']),
        task_data['Task_issue_time']
    ))
    conn.commit()
    conn.close()

def get_all_tasks_from_db():
    """Fetches all tasks to populate the frontend on reload."""
    conn = sqlite3.connect(DB_FILE)
    conn.row_factory = sqlite3.Row  # Enables column access by name
    cursor = conn.cursor()
    
    # Optional: Filter for only active tasks if you add a status column later
    cursor.execute("SELECT * FROM tasks_ledger")
    rows = cursor.fetchall()
    conn.close()
    
    # Convert SQLite rows to a list of dicts, parsing the JSON payload back out
    tasks = []
    for row in rows:
        task_dict = dict(row)
        if task_dict.get('Task'):
            task_dict['Task'] = json.loads(task_dict['Task'])
        tasks.append(task_dict)
    return tasks
# ----------------------------------------

class DashboardNode(Node):
    def __init__(self):
        super().__init__('fleet_dashboard')
        
        # 1. Mesh QoS Profile: Transient Local is REQUIRED so robots connecting 
        # later will still receive the active tasks.
        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.z_session = zenoh.open(zenoh.Config())
        # Zenoh subscribers
        self.subscription = self.z_session.declare_subscriber('fleet_status', self.listener_callback)
        self.bid_subscription = self.z_session.declare_subscriber('fleet_tasks_bids', self.bid_callback)
        self.traj_sub = self.z_session.declare_subscriber('fleet_trajectories', self.traj_callback)

        #Zenoh publisher
        self.task_publisher = self.z_session.declare_publisher('fleet_tasks')
        
        print("[Dashboard] WebSocket Server Active. Listening for mesh data...", flush=True)

    def listener_callback(self, sample: zenoh.Sample):
        msg = deserialize_message(sample.payload.to_bytes(), AMRTelemetry)
        data = {
            "id": msg.robot_id,
            "x": msg.pose.x,
            "y": msg.pose.y,
            "theta": msg.pose.theta,
            "battery": msg.battery_percent,
            "status": msg.system_status,
            "is_busy": msg.is_busy,
            "current_task_id": msg.current_task_id
        }
        # print(data, flush=True)
        socketio.emit('fleet_update', data)

    def traj_callback(self, sample: zenoh.Sample):
        # Flatten the Point32 objects into simple [x, y] arrays for JSON
        msg = deserialize_message(sample.payload.to_bytes(), LocalTrajectory)
        path_data = [[pt.x, pt.y] for pt in msg.future_waypoints]
        socketio.emit('trajectory_update', {
            "id": msg.robot_id,
            "path": path_data
        })

    def bid_callback(self, sample: zenoh.Sample):
        # Forward the decentralized bid up to the frontend UI
        msg = deserialize_message(sample.payload.to_bytes(), TaskBid)
        try:
            # Deserialize the CBBA matrices
            y_matrix = json.loads(msg.y_matrix)
            z_matrix = json.loads(msg.z_matrix)
            
            # Forward the decentralized bid landscape to the frontend UI
            bid_data = {
                "robot_id": msg.robot_id,
                "bids": y_matrix,       # Dictionary of {task_id: bid_score}
                "winners": z_matrix     # Dictionary of {task_id: winning_robot_id}
            }
            
            # print(f"[Dashboard] Received matrix update from {msg.robot_id}", flush=True)
            socketio.emit('task_bid', bid_data)
            
        except json.JSONDecodeError as e:
            print(f"[Dashboard] Failed to parse bid matrix: {e}", flush=True)

# Socket.IO Listener for frontend task dispatch
@socketio.on('issue_task')
def handle_issue_task(payload):
    if ros_node_instance is None:
        # print("[Backend] Warning: ROS node not ready to publish tasks.", flush=True)
        return

    try:
        task_id = payload.get('task_id', f"TASK_{int(datetime.now().timestamp())}")
        
        # 1. Update Database FIRST
        task_data = {
            "Task_id": task_id,
            "Task": payload,
            "Task_issue_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        db_upsert_task(task_data)
        print(f"[Dashboard] Task {task_id} saved to SQLite.", flush=True)

        # 2. Start Broadcast
        msg = DispatchTask()
        msg.task_id = task_id
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
        task_payload = serialize_message(msg)
        ros_node_instance.task_publisher.put(task_payload)
        print(f"[Dashboard] Issued task {msg.task_id} via Socket.IO", flush=True)
        
    except KeyError as e:
        print(f"[Backend] Malformed task payload missing key: {e}", flush=True)
    except Exception as e:
        print(f"[Backend] DB/ROS Error: {e}", flush=True)

# 3. Listen on the frontend reload to repopulate the UI
@socketio.on('request_initial_state')
def handle_initial_state():
    print("[Dashboard] Frontend connected. Serving historical state from DB...", flush=True)
    tasks = get_all_tasks_from_db()
    
    # Emit all past/current tasks directly to the client that just connected
    socketio.emit('initial_state_response', {"tasks": tasks})

def zenoh_mesh_callback(sample: zenoh.Sample):
    # 1. Update SQLite using your existing storage_manager logic
    listener(sample)
    
    # 2. Fetch the newly updated tasks and push them to the frontend
    tasks = get_all_tasks_from_db()
    socketio.emit('initial_state_response', {"tasks": tasks})

def ros_spin_thread():
    global ros_node_instance
    rclpy.init(args=None)
    ros_node_instance = DashboardNode()
    
    rclpy.spin(ros_node_instance)
    
    ros_node_instance.destroy_node()
    rclpy.shutdown()

def main(args=None):
    init_db()

    z_session = zenoh.open(zenoh.Config())
    z_session.declare_subscriber("fleet/tasks/ledger/**", zenoh_mesh_callback)

    # 1. Start ROS 2 node in a background thread
    threading.Thread(target=ros_spin_thread, daemon=True).start()
    
    # 2. Start the Flask WebSocket server on the main thread
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()