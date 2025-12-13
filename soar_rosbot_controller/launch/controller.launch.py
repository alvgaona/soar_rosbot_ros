from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare log level argument
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Log level for nodes'
    )

    log_level = LaunchConfiguration('log_level')

    # Soar maze controller node
    soar_controller = Node(
        package="soar_rosbot_controller",
        executable="soar_maze_controller",
        name="soar_maze_controller",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Motion controller node
    motion_controller = Node(
        package="soar_rosbot_controller",
        executable="motion_controller",
        name="motion_controller",
        parameters=[
            {"holonomic": False},  # False for differential drive
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Yaw observer node
    yaw_observer = Node(
        package="soar_rosbot_controller",
        executable="yaw_observer",
        name="yaw_observer",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription(
        [
            declare_log_level,
            soar_controller,
            motion_controller,
            yaw_observer,
        ]
    )
