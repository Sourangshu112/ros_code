import sqlite3
import json
import time
import zenoh

DB_FILE = "/app/data/fleet_ledger.db"
TOPIC_PATH = "fleet/tasks/ledger/**"

def init_db():
    """Initializes the SQLite database and creates the schema if it doesn't exist."""
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS tasks_ledger (
            Task_id TEXT PRIMARY KEY,
            Task TEXT,
            Amr_completed TEXT,
            Task_issue_time TIMESTAMP,
            Bid_completion_time TIMESTAMP,
            Task_starting_time TIMESTAMP,
            Task_completion_time TIMESTAMP
        )
    ''')
    conn.commit()
    conn.close()

def db_upsert(task_data):
    """Inserts a new task or updates an existing task without overwriting old data."""
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()
    
    task_id = task_data.get("Task_id")
    if not task_id:
        return # Skip if no Task_id is present
        
    # Ensure nested task JSON is stringified for SQLite TEXT column
    task_payload = task_data.get("Task")
    if isinstance(task_payload, dict):
        task_payload = json.dumps(task_payload)
    
    # Using SQLite UPSERT (ON CONFLICT). 
    # COALESCE ensures we only update fields the robot actually sent in this payload,
    # preserving previously recorded timestamps (like Task_issue_time).
    cursor.execute('''
        INSERT INTO tasks_ledger (
            Task_id, Task, Amr_completed, Task_issue_time, 
            Bid_completion_time, Task_starting_time, Task_completion_time
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(Task_id) DO UPDATE SET
            Task = COALESCE(excluded.Task, tasks_ledger.Task),
            Amr_completed = COALESCE(excluded.Amr_completed, tasks_ledger.Amr_completed),
            Task_issue_time = COALESCE(excluded.Task_issue_time, tasks_ledger.Task_issue_time),
            Bid_completion_time = COALESCE(excluded.Bid_completion_time, tasks_ledger.Bid_completion_time),
            Task_starting_time = COALESCE(excluded.Task_starting_time, tasks_ledger.Task_starting_time),
            Task_completion_time = COALESCE(excluded.Task_completion_time, tasks_ledger.Task_completion_time)
    ''', (
        task_id,
        task_payload,
        task_data.get("Amr_completed"),
        task_data.get("Task_issue_time"),
        task_data.get("Bid_completion_time"),
        task_data.get("Task_starting_time"),
        task_data.get("Task_completion_time")
    ))
    
    conn.commit()
    conn.close()

def listener(sample: zenoh.Sample):
    """Callback fired immediately whenever any AMR updates a task on the mesh."""
    try:
        # Extract the string payload from the Zenoh sample
        # Compatibility check: to_string() is used in Zenoh API 1.0+
        if hasattr(sample.payload, 'to_string'):
            payload_str = sample.payload.to_string()
        else:
            payload_str = sample.payload.decode('utf-8')
            
        data = json.loads(payload_str)
        
        # Failsafe: If the robot just sent the JSON without the ID inside the payload,
        # extract the Task_id directly from the Zenoh key expression (e.g., fleet/tasks/ledger/TASK_904)
        if "Task_id" not in data:
            data["Task_id"] = str(sample.key_expr).split("/")[-1]
            
        print(f"[{time.strftime('%H:%M:%S')}] Received update for {data['Task_id']} from mesh. Writing to SQLite...")
        db_upsert(data)
        
    except json.JSONDecodeError:
        print(f"Dropped invalid JSON payload from {sample.key_expr}")
    except Exception as e:
        print(f"Failed to process sample: {e}")

if __name__ == "__main__":
    init_db()
    print(f"Database initialized at ./{DB_FILE}")
    
    # Open Zenoh Session using the default configuration (listens on all network interfaces)
    conf = zenoh.Config()
    
    with zenoh.open(conf) as session:
        print(f"Zenoh node online. Subscribing to '{TOPIC_PATH}'...")
        
        # background=True keeps the subscriber running asynchronously
        session.declare_subscriber(TOPIC_PATH, listener, background=True)
        
        try:
            # Keep the main thread alive indefinitely while the background listener does the work
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down Dashboard Storage node...")