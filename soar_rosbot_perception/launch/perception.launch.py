from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Wall detection node
    wall_detector = Node(
        package="soar_rosbot_perception",
        executable="wall_detector",
        name="wall_detector",
        parameters=[
            {"wall_threshold": 0.5},  # 0.5m threshold for wall detection
            {"cone_angle": 10.0},     # 10° cone for each direction
        ],
        output="screen",
    )

    # ArUco detection node
    aruco_detector = Node(
        package="soar_rosbot_perception",
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[
            {"marker_size": 0.15},          # 15cm marker
            {"aruco_dict": "DICT_4X4_50"},  # ArUco dictionary
            {"camera_fx": 615.0},           # Focal length x (adjust if calibrated)
            {"camera_fy": 615.0},           # Focal length y
            {"camera_cx": 640.0},           # Principal point x
            {"camera_cy": 360.0},           # Principal point y
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            wall_detector,
            aruco_detector,
        ]
    )
