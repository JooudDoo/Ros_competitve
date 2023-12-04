from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_path_code',
            executable='follow_node',
            name='follow_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'Kp': 1.0},
                {'Ki': 0.0},
                {'Kd': 0.0}
            ]
        )
    ])
