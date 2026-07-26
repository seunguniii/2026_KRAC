from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory("mission_launch")

flight = os.path.join(pkg_share, "config", "flight.yaml")
trajectory = os.path.join(pkg_share, "config", "trajectory.yaml")
marker_recog = os.path.join(pkg_share, "config", "marker_recog.yaml")

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
      parameters=[
        {"trajectory_dir": trajectory},
      ],
      name='flight',
      output='screen',
      emulate_tty=True
    ),
    
    Node(
      package='stack_py',
      executable='marker_recog',
      parameters=[
        marker_recog,
      ]
      name='marker_recog',
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
      executable='logger',
      name='logger',
      output='screen',
      emulate_tty=True
    ),
  ])
