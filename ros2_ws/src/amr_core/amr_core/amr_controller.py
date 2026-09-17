import rclpy
import time
import json
from datetime import datetime
from geometry_msgs.msg import Twist
from amr_core.local_planner import LocalPlanner

def get_world_path(node, start_x, start_y, target_x, target_y):
    """Helper to generate an exact array of world coordinates using A*."""
    navigator = node.global_navigator
    start_col, start_row = navigator.world_to_grid(start_x, start_y)
    target_col, target_row = navigator.world_to_grid(target_x, target_y)
    
    grid_path = navigator.find_path((start_col, start_row), (target_col, target_row))
    # print(grid_path, flush=True)
    if not grid_path:
        return []
        
    return [navigator.grid_to_world(col, row) for col, row in grid_path]

def execute_physical_tasks(node):
    """Continuously processes tasks in the AMR's bundle."""
    node.is_driving = True
    cmd_pub = node.create_publisher(Twist, f'/{node.get_name()}/cmd_vel', 10)
    
    while node.agent.bundle and rclpy.ok():
        task_dict = node.agent.bundle[0]
        task_id = task_dict['id']
        task_msg = node.task_registry[task_id]
        
        # 1. Pre-calculate the exact physical paths for BOTH legs
        path_to_pickup = get_world_path(
            node, node.current_x, node.current_y, 
            task_msg.pickup_coordinates.x, task_msg.pickup_coordinates.y
        )
        path_to_drop = get_world_path(
            node, task_msg.pickup_coordinates.x, task_msg.pickup_coordinates.y, 
            task_msg.drop_coordinates.x, task_msg.drop_coordinates.y
        )
        
        # Abort if either leg is entirely blocked
        if not path_to_pickup or not path_to_drop:
            node.get_logger().error(f"[{task_id}] Path blocked. Abandoning task.")
            node.agent.abandon_current_task()
            continue

        # 2. Expose the stitched trajectory to the telemetry publisher
        node.active_trajectory = path_to_pickup + path_to_drop
        
        # 3. Leg 1: Move to Pickup
        node.get_logger().info(f"[{task_id}] Moving to pickup...")
        success = follow_path(node, cmd_pub, path_to_pickup)
        if not success:
            node.agent.abandon_current_task()
            node.active_trajectory = []
            continue
            
        node.get_logger().info(f"[{task_id}] Arrived at pickup. Loading...")
        time.sleep(2.0)  
        
        node.ledger_data[task_id]["Task_starting_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        node.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(node.ledger_data[task_id]))
        node.agent.advance_leg()
        
        # 4. Leg 2: Update trajectory to drop-off only, then move
        node.active_trajectory = path_to_drop
        
        node.get_logger().info(f"[{task_id}] Moving to drop-off...")
        success = follow_path(node, cmd_pub, path_to_drop)
        if not success:
            node.agent.abandon_current_task()
            node.active_trajectory = []
            continue
            
        node.get_logger().info(f"[{task_id}] Arrived at drop-off. Unloading...")
        time.sleep(2.0)  
        
        node.agent.advance_leg()
        node.finish_task(task_id)
        node.active_trajectory = []
        
    # Bundle exhausted
    node.is_driving = False
    node.active_trajectory = []
    node.get_logger().info("Bundle empty. Idling.")

def follow_path(node, cmd_pub, world_path):
    """Feeds a pre-calculated path to the local proportional driver."""
    local_driver = LocalPlanner(v_max=2.5, omega_max=1.0)
    local_driver.on_path(world_path)
    
    vel_msg = Twist()
    while rclpy.ok() and local_driver.is_active:
        local_driver.on_odometry(node.current_x, node.current_y, theta=node.current_yaw)
        v, omega = local_driver.step()
        
        vel_msg.linear.x = v
        vel_msg.angular.z = omega
        cmd_pub.publish(vel_msg)
        time.sleep(0.1)  
        
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    cmd_pub.publish(vel_msg)
    return True