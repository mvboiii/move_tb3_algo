"""
Author - Manav Nagda (manav19@umd.edu)
Launch file to initiate the walker algorithm
for the turtlebot3 in gazebo and record the bag file
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Method to launch the nodes in the package with bag record flag"""
    bag_record = LaunchConfiguration('bag_record')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_record',
            default_value='False'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),

        Node(
            package='move_tb3_algo',
            executable='move_tb3',
        ),

        ExecuteProcess(
            condition=IfCondition(bag_record),
            cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
            shell=True
        )

    ])
