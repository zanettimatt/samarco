from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            #remappings=[
            #    ('/cmd_vel', '/your_robot_cmd_vel')  # Optional remap if needed
            #]
        )
    ])
