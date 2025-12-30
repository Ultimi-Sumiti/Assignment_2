from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml


def generate_launch_description():
    
    assignment2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ir_launch'),
                'launch',
                'assignment_2.launch.py'
            ])
        ])
    )

    moveit_config_pkg = "ir_movit_config"
    moveit_config = MoveItConfigsBuilder("ir_gripper", package_name=moveit_config_pkg).to_moveit_configs()
    test_node = Node(
        package="group_24_assignment_2",
        executable="planner_action_server",
        name="planner_action_server_test",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    # Create launch descriptor.
    ld = LaunchDescription()
    
    ld.add_action(assignment2_launch)
    ld.add_action(test_node)

    return ld
