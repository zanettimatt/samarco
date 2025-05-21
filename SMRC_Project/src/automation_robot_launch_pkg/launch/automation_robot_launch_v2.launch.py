from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Paths to package shares
    automation_robot_description_pkg_share = get_package_share_directory('automation_robot_description_pkg')
    automation_world_pkg_share = get_package_share_directory('automation_world')
    ros_gz_sim_pkg_share = get_package_share_directory('ros_gz_sim')

    # Paths to necessary files
    robot_description_xacro = os.path.join(automation_robot_description_pkg_share, 'urdf', 'automation_robot_description.urdf.xacro')
    rviz_config_file = os.path.join(automation_robot_description_pkg_share, 'rviz', 'automation_robot_rviz.rviz')
    empty_world_file = os.path.join(automation_world_pkg_share, 'worlds', 'porao.sdf')

    # Generate robot description from xacro
    robot_description = os.popen(f"xacro {robot_description_xacro}").read()

    # Percorso al file .xacro
    package_dir = get_package_share_directory('automation_robot_description_pkg')
    #xacro_file = os.path.join(package_dir, 'urdf', 'automation_robot_description.urdf.xacro')
    xacro_file = os.path.join(package_dir, 'urdf', 'automation_robot_description.urdf')
    urdf_file = os.path.join(package_dir, 'urdf', 'automation_robot_description.urdf')

    # Processa il file xacro
    doc = xacro.process_file(xacro_file)
    with open(urdf_file, 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

    # Force package
    #package_dir_force = get_package_share_directory('gazebo_plugins')

    return LaunchDescription([
        # Launch RViz2
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    arguments=['-d', rviz_config_file],
        #    output='screen'
        #),

        # Launch robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            #parameters=[{'robot_description': robot_description}]
            parameters=[{'robot_description': doc.toxml()}]
        ),

        # Launch joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Include Gazebo empty world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-v 4 {empty_world_file}'}.items()
        ),

        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', robot_description
            ],
            output='screen'
        ),

        # Create velocity command bridge
        Node(
				 package='ros_gz_bridge',
				 executable='parameter_bridge',
				 arguments=[
					  '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
					  '/world/porao_samarco/model/isi_robot/link/base_footprint/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
				 ],
				 remappings=[
					  ('/world/porao_samarco/model/isi_robot/link/base_footprint/sensor/lidar_sensor/scan', '/lidar')
				 ],
				 output='screen'
			),
        
    ])
