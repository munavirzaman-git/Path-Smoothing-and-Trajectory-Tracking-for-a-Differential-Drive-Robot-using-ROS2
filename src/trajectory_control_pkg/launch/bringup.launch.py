from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='trajectory_control_pkg',
            executable='path_smoother_node',
            output='screen'
        ),

        Node(
            package='trajectory_control_pkg',
            executable='trajectory_generator_node',
            output='screen'
        ),

        Node(
            package='trajectory_control_pkg',
            executable='trajectory_tracker_node',
            output='screen'
        ),
    ])

