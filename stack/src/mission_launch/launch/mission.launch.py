import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


pkg_share = get_package_share_directory("mission_launch")

flight = os.path.join(pkg_share, "config", "flight.yaml")
trajectory = os.path.join(pkg_share, "config", "trajectory.yaml")
yolo = os.path.join(pkg_share, "config", "yolo.yaml")


################
#<yolo-settings>
stack_py_prefix = get_package_prefix('stack_py')
stack_py_site_packages = os.path.join(
    stack_py_prefix, 'lib', 'python3.10', 'site-packages'
)

#TODO: check python version
ros_pythonpath = '/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages'
current_pythonpath = os.environ.get('PYTHONPATH', '')
full_pythonpath = f"{stack_py_site_packages}:{ros_pythonpath}:{current_pythonpath}"

yolo_env = os.environ.copy()
yolo_env['PYTHONNOUSERSITE'] = '1'
yolo_env['PYTHONPATH'] = full_pythonpath

default_yolo_venv = os.path.expanduser('~/venvs/yolo/bin/yolo')
#</yolo-settings>
#################


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
                flight,
            ],
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
            package='stack_py',
            executable='gripper',
            name='gripper',
            output='screen',
            emulate_tty=True
        ),
        #Node(
        #    package='ros_gz_bridge',
        #    executable='parameter_bridge',
        #    name='gripper_gz_bridge',
        #    arguments=[
        #        '/gripper/left/horizontal@std_msgs/msg/Float64]gz.msgs.Double',
        #        '/gripper/right/horizontal@std_msgs/msg/Float64]gz.msgs.Double',
        #        '/gripper/left/vertical@std_msgs/msg/Float64]gz.msgs.Double',
        #        '/gripper/right/vertical@std_msgs/msg/Float64]gz.msgs.Double',
        #    ],
        #    output='screen',
        #    emulate_tty=True
        #),

        DeclareLaunchArgument(
            'yolo_venv',
            default_value=default_yolo_venv,
            description='YOLO venv path'
        ),
        Node(
            package='stack_py',
            executable='yolo',
            name='yolo',
            prefix=[LaunchConfiguration('yolo_venv')],
            env=yolo_env,
            #parameters = yolo,
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
