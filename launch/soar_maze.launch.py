from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
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

    # Set GZ_SIM_RESOURCE_PATH to include our models directory
    models_path = PathJoinSubstitution(
        [FindPackageShare("soar_rosbot_ros"), "models"]
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            models_path
        ]
    )

    # Get path to custom maze world
    maze_world_path = PathJoinSubstitution(
        [FindPackageShare("soar_rosbot_ros"), "worlds", "maze_world.sdf"]
    )

    # Use husarion_gz_worlds wrapper to launch Gazebo with custom maze world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            'gz_world': maze_world_path,
            'gz_log_level': '1'
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
            "y": "4.0",
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
            set_gazebo_model_path,  # Set GZ_SIM_RESOURCE_PATH before launching Gazebo
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
