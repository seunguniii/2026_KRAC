from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flight_control',
            executable='landing_test_vel',  # 또는 landing_test/landing_test_vel
            name='fc_node',
            output='screen',
        ),
        Node(
            package='imagery_processing',
            executable='marker_recognition',
            name='cv_node',
            output='screen',
        ),
    ])
