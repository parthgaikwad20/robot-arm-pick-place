import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('arm_pick_place')
    with open(os.path.join(pkg, 'urdf', 'arm.urdf')) as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2'),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='arm_pick_place',
             executable='pick_place',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'params.yaml')]),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'demo.rviz')],
             condition=IfCondition(use_rviz)),
    ])
