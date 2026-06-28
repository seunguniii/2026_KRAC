from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
      package='stack_cpp',
      executable='mission',
      name='mission',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_cpp',
      executable='flight',
      name='flight',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_py',
      executable='vision',
      name='vision',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_py',
      executable='marker',
      name='marker',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_cpp',
      executable='target',
      name='target',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_cpp',
      executable='gripper',
      name='gripper',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_py',
      executable='yolo',
      name='yolo',
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
