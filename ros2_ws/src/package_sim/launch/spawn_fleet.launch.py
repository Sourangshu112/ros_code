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
        {'name': 'robot_1', 'x': '-13.0', 'y': '-1.0',  'z': '0.8'},
        {'name': 'robot_2', 'x': '-13.0', 'y': '-6.0', 'z': '0.8'},
        {'name': 'robot_3', 'x': '-13.0', 'y': '-11.0', 'z': '0.8'},
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
                '-z', robot['z'],
            ],
            output='screen'
        )
        
        # Bridge the Gazebo topics to ROS 2 topics under the robot's specific namespace
        bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Target the exact topics your gz topic -l command revealed
            f"/model/{robot['name']}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/model/{robot['name']}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        ],
        remappings=[
            # Translate the Gazebo topics into the ROS topics peer_node expects
            (f"/model/{robot['name']}/cmd_vel", f"/{robot['name']}/cmd_vel"),
            (f"/model/{robot['name']}/odometry", f"/{robot['name']}/odom")
        ],
        output='screen'
    )
        
        nodes.append(spawn_node)
        nodes.append(bridge_node)

    return LaunchDescription(nodes)