#!/usr/bin/env python3

# Launch file for ROSbot XL with autonomy configuration (LiDAR + Camera) in empty Gazebo world

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    # Use ros_gz_sim to launch Gazebo properly
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # Bridge for clock topic
    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_gazebo"), "config", "gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    # Spawn ROSbot XL with autonomy configuration (LiDAR + Camera)
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_gazebo"),
                    "launch",
                    "spawn_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_model": "rosbot_xl",
            "configuration": "autonomy",
            "x": "0.0",
            "y": "0.0",
            "z": "0.2",
        }.items(),
    )

    # Optional RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={"namespace": ""}.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            declare_rviz_arg,
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            spawn_robot,
            rviz_launch,
        ]
    )
