from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart # <--- Added this import
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml

def generate_launch_description():
    pkg_name = 'group_24_assignment_2'
    
    assignment2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ir_launch'),
                'launch',
                'assignment_2.launch.py'
            ])
        ])
    )

    apriltag_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'launch_AprilTag_node.yml'
            ])
        )
    )

    moveit_config_pkg = "ir_movit_config"
    moveit_config = MoveItConfigsBuilder("ir_gripper", package_name=moveit_config_pkg).to_moveit_configs()
    planner_node = Node(
        package=pkg_name,
        executable="planner_action_server",
        name="planner_action_server",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict(),
        ],
        output="screen",
        emulate_tty=True
    )

    gripper_node = Node(
        package= pkg_name,
        executable="gripper",
        name="gripper",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    coordinator_node = Node(
        package=pkg_name,
        executable="coordinator_action_client",
        name="coordinator_action_client",
        output="screen",
        emulate_tty=True
    )

    start_coordinator_after_planner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=planner_node,
            on_start=[
                LogInfo(msg='Planner started, now launching Coordinator...'),
                coordinator_node
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(assignment2_launch)
    ld.add_action(apriltag_launch)
    ld.add_action(gripper_node)
    ld.add_action(planner_node)
    ld.add_action(start_coordinator_after_planner)

    return ld
