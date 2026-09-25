#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    share = get_package_share_directory("fast_livo")

    params_default = os.path.join(
        share, "config", "ugv_v1_livo.yaml"
    )

    rviz_default = os.path.join(
        share, "rviz_cfg", "fast_livo2_ugv_livo.rviz"
    )

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"),
        value_type=bool,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_default,
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_default,
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": use_sim_time},
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="fast_livo2_ugv_rviz",
            arguments=[
                "-d",
                LaunchConfiguration("rviz_config"),
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(
                LaunchConfiguration("use_rviz")
            ),
            output="screen",
        ),
    ])
