import rclpy
import time
import math
import json
from datetime import datetime
from geometry_msgs.msg import Twist

from amr_core.local_planner import LocalPlanner

def execute_physical_task(node, task_msg):
    """
    Executes the physical movement for the AMR in a separate thread.
    Communicates with Gazebo via cmd_vel and reads state from node.current_x/y.
    """
    task_id = task_msg.task_id
    
    # Set up Publisher for Gazebo (matches launch file bridge: /robot_name/cmd_vel)
    cmd_pub = node.create_publisher(Twist, f'/{node.get_name()}/cmd_vel', 10)

    # 1. Move to Pickup Coordinates
    node.get_logger().info(f"[{task_id}] Moving to pickup coordinates...")
    move_to_point(node, cmd_pub, task_msg.pickup_coordinates.x, task_msg.pickup_coordinates.y)
    
    node.get_logger().info(f"[{task_id}] Arrived at pickup. Loading pallet...")
    time.sleep(2.0)  # Simulate the physical time it takes to lift the pallet
    
    # 2. Log Task Start Time to Zenoh (Task officially begins once pallet is loaded)
    node.ledger_data[task_id]["Task_starting_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    node.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(node.ledger_data[task_id]))
    node.get_logger().info(f"[{task_id}] Task officially started. Moving to drop-off...")
    
    # 3. Move to Drop-off Coordinates
    move_to_point(node, cmd_pub, task_msg.drop_coordinates.x, task_msg.drop_coordinates.y)

    node.get_logger().info(f"[{task_id}] Arrived at drop-off. Unloading...")
    time.sleep(2.0)  # Simulate the physical time it takes to drop the pallet
    
    # 4. Trigger Task Completion
    node.finish_task(task_id)

def move_to_point(node, cmd_pub, target_x, target_y):
    """Integrates A* global navigation with local proportional waypoint following."""
    # Use the node's global navigator and initialize a fresh local driver
    navigator = node.global_navigator
    local_driver = LocalPlanner(v_max=0.5, omega_max=1.0)
    
    # 1. Convert Gazebo world meters to costmap grid pixels
    start_col, start_row = navigator.world_to_grid(node.current_x, node.current_y)
    target_col, target_row = navigator.world_to_grid(target_x, target_y)
    
    # 2. Generate optimal path avoiding obstacles
    grid_path = navigator.find_path((start_col, start_row), (target_col, target_row))
    if not grid_path:
        node.get_logger().error(f"No valid path to target ({target_x}, {target_y}) found!")
        return
        
    # 3. Convert grid pixel path back to Gazebo world meters for the hardware driver
    world_path = [navigator.grid_to_world(col, row) for col, row in grid_path]
    
    # 4. Hand physical path to the local hardware controller
    local_driver.on_path(world_path)
    
    vel_msg = Twist()
    
    # 5. Step the controller at a fixed 10Hz rate
    while rclpy.ok() and local_driver.is_active:
        # Sync real-time Gazebo coordinates into the planner
        local_driver.on_odometry(node.current_x, node.current_y, theta=node.current_yaw)
        
        # Fetch computed velocities
        v, omega = local_driver.step()
        
        vel_msg.linear.x = v
        vel_msg.angular.z = omega
        cmd_pub.publish(vel_msg)
        
        time.sleep(0.1)  
        
    # Stop the AMR when the target is reached or ROS shuts down
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    cmd_pub.publish(vel_msg)
    node.get_logger().info("Waypoint sequence completed.")