from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
      package='stack_cpp',
      executable='master',
      name='master',
      output='screen',
      emulate_tty=True
    ),
    
    
    Node(
      package='stack_cpp',
      executable='logger',
      name='logger',
      output='screen',
      emulate_tty=True
    ),
  ])
