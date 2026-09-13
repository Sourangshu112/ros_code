import rclpy
import time
import math
import json
from datetime import datetime
from geometry_msgs.msg import Twist

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
    #time.sleep(2.0)  # Simulate the time it takes to lift the pallet
    
    # 2. Log Task Start Time to Zenoh (Task officially begins once pallet is loaded)
    node.ledger_data[task_id]["Task_starting_time"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    node.z_session.put(f"fleet/tasks/ledger/{task_id}", json.dumps(node.ledger_data[task_id]))
    node.get_logger().info(f"[{task_id}] Task officially started. Moving to drop-off...")
    
    # 3. Move to Drop-off Coordinates
    move_to_point(node, cmd_pub, task_msg.drop_coordinates.x, task_msg.drop_coordinates.y)

    node.get_logger().info(f"[{task_id}] Arrived at drop-off. Unloading...")
    #time.sleep(2.0)  # Simulate dropping the pallet
    
    # 4. Trigger Task Completion
    node.finish_task(task_id)
def move_to_point(node, cmd_pub, target_x, target_y):
    """Basic Proportional (P) controller for differential drive navigation."""
    vel_msg = Twist()
    loop_count = 0
    
    # Prevents crash on Ctrl+C by stopping the loop when ROS shuts down
    while rclpy.ok():
        # Calculate distance and angle to target
        dx = target_x - node.current_x
        dy = target_y - node.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Print diagnostic data every ~2 seconds (20 iterations at 0.1s sleep)
        if loop_count % 20 == 0:
            node.get_logger().info(f"Tracking... Current: ({node.current_x:.2f}, {node.current_y:.2f}) | Target: ({target_x:.2f}, {target_y:.2f}) | Distance: {distance:.2f}m")
        loop_count += 1
        
        # Stop condition
        if distance < 0.2:  
            node.get_logger().info("Target reached!")
            break
            
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - node.current_yaw
        
        # Normalize the angle between -pi and pi
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Control Logic: Rotate first if heavily misaligned, otherwise drive and correct
        if abs(heading_error) > 0.2:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 if heading_error > 0 else -0.5
        else:
            vel_msg.linear.x = min(0.5, distance)  # Cap max speed at 0.5 m/s
            vel_msg.angular.z = 0.5 * heading_error
            
        cmd_pub.publish(vel_msg)
        time.sleep(0.1) # Run loop at roughly 10Hz
        
    # Stop the robot when target is reached or ROS shuts down
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    cmd_pub.publish(vel_msg)