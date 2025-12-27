from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml


def generate_launch_description():

    moveit_config_pkg = "ir_movit_config"

    moveit_config = MoveItConfigsBuilder("ir_gripper", package_name=moveit_config_pkg).to_moveit_configs()

    # Node
    test_node = Node(
        package="hello_moveit",
        executable="hello_moveit",
        name="hello_movit_test",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    return LaunchDescription([test_node])