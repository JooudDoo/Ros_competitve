from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    follow_node = Node(
            package='autorace_core_TheRosBoss',
            executable='follow_node',
            name='follow_node',
            output='screen',
            emulate_tty=True,
            #parameters=[
            #   {'Kp': 3.0},
            #   {'Ki': 1.0},
            #   {'Kd': 0.25}
            #]
        )
    detect_node = Node(
            package='autorace_core_TheRosBoss',
            executable='detect_signs',
            name='detect_signs',
            output='screen',
            emulate_tty=True,
        )
    return LaunchDescription([
        follow_node,
        detect_node
    ])
