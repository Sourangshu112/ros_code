import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Resolve the absolute path to your 3D model
    pkg_share = get_package_share_directory('package_sim')
    model_path = os.path.join(pkg_share, 'models', 'amr_model.sdf')

    # 1. Start the Gazebo Jetty simulation world natively on the host
    # start_gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', 'empty.sdf'], 
    #     output='screen'
    # )

    # 2. Define the starting positions for the 3 AMRs
    robots = [
        {'name': 'robot_1', 'x': '0.0', 'y': '0.0'},
        {'name': 'robot_2', 'x': '2.0', 'y': '0.0'},
        {'name': 'robot_3', 'x': '0.0', 'y': '2.0'},
    ]

    # nodes = [start_gazebo]
    nodes = []

    for robot in robots:
        # Spawn the 3D model into Gazebo
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot['name'],
                '-file', model_path,
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.1',
            ],
            output='screen'
        )
        
        # Bridge the Gazebo topics to ROS 2 topics under the robot's specific namespace
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f"/{robot['name']}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/{robot['name']}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/{robot['name']}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
            ],
            output='screen'
        )
        
        nodes.append(spawn_node)
        nodes.append(bridge_node)

    return LaunchDescription(nodes)