from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

from typing import List
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the automation robot simulation.
    """
    # Paths to package shares
    automation_robot_description_pkg_share = get_package_share_directory('automation_robot_description_pkg')
    automation_world_pkg_share = get_package_share_directory('automation_world')
    ros_gz_sim_pkg_share = get_package_share_directory('ros_gz_sim')

    # Paths to necessary files
    robot_description_xacro = os.path.join(automation_robot_description_pkg_share, 'urdf', 'automation_robot_description.urdf.xacro')
    empty_world_file = os.path.join(automation_world_pkg_share, 'worlds', 'porao.sdf')

    # Generate robot description from xacro
    robot_description = xacro.process_file(robot_description_xacro).toxml()
    urdf_file = os.path.join(automation_robot_description_pkg_share, 'urdf', 'automation_robot_description.urdf')
    with open(urdf_file, 'w') as f:
        f.write(robot_description)

    # Declare launch arguments
    declared_arguments = [
        # World for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("magnetic_adhesion_plugin"), "worlds", "world.sdf"]),
            description="Name or filepath of world to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            'gz_ros_bridge_file',
            default_value="magnetic_adhesion_plugin_gz_ign_bridge.yaml",
            description='Parameter file to gz_ros_bridge pkg'
        ),
    ]

    # Define nodes and processes
    nodes = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
            ],
            output='screen'
        ),

        # Nó de cálculo de inclinação
        Node(
            package='robot_inclination_pkg',  # Substitua pelo nome do seu pacote
            executable='robot_inclination',  # Nome do executável do nó
            name='robot_inclination',
            output='screen'
        ),
    ]

    # Include Gazebo Simulator
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-v 4 {empty_world_file}'}.items()
    )

    # Spawn Robot in Gazebo
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-string', robot_description],
        output='screen'
    )

    # Combine all actions into the launch description
    return LaunchDescription(declared_arguments + nodes + [gazebo_launch, spawn_robot])
