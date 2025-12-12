from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Launch configuration
    cone_angle = LaunchConfiguration("cone_angle")
    wall_threshold = LaunchConfiguration("wall_threshold")
    marker_size = LaunchConfiguration("marker_size")
    aruco_dict = LaunchConfiguration("aruco_dict")
    image_topic = LaunchConfiguration("image_topic")
    camera_fx = LaunchConfiguration("camera_fx")
    camera_fy = LaunchConfiguration("camera_fy")
    camera_cx = LaunchConfiguration("camera_cx")
    camera_cy = LaunchConfiguration("camera_cy")
    
    # Declare launch arguments
    declare_cone_angle_arg = DeclareLaunchArgument(
        "cone_angle", default_value="10.0", description="Cone angle for wall detection"
    )
    declare_wall_threshold_arg = DeclareLaunchArgument(
        "wall_threshold", default_value="2.15", description="Wall detection distance threshold in"
    )

    declare_marker_size_arg = DeclareLaunchArgument(
        "marker_size", default_value="0.75", description="ArUco marker size in meters"
    )

    declare_aruco_dict_arg = DeclareLaunchArgument(
        "aruco_dict", default_value="DICT_6X6_1000", description="ArUco dictionary to use"
    )

    declare_image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/oak/rgb/color", description="Camera image topic"
    )

    declare_camera_fx_arg = DeclareLaunchArgument(
        "camera_fx", default_value="1108.51", description="Camera focal length x"
    )

    declare_camera_fy_arg = DeclareLaunchArgument(
        "camera_fy", default_value="1108.51", description="Camera focal length y"
    )

    declare_camera_cx_arg = DeclareLaunchArgument(
        "camera_cx", default_value="640.0", description="Camera principal point x"
    )

    declare_camera_cy_arg = DeclareLaunchArgument(
        "camera_cy", default_value="360.0", description="Camera principal point y"
    )

    # Wall detection node
    wall_detector = Node(
        package="soar_rosbot_perception",
        executable="wall_detector",
        name="wall_detector",
        parameters=[
            {"wall_threshold": wall_threshold},  # Wall detection threshold
            {"cone_angle": cone_angle},          # Cone angle for wall detection
        ],
        output="screen",
    )

    # ArUco detection node
    aruco_detector = Node(
        package="soar_rosbot_perception",
        executable="aruco_detector",
        name="aruco_detector",
        parameters=[
            {"marker_size": marker_size},          # ArUco marker size
            {"aruco_dict": aruco_dict},            # ArUco dictionary
            {"camera_fx": camera_fx},              # Camera focal length x
            {"camera_fy": camera_fy},              # Camera focal length y
            {"camera_cx": camera_cx},              # Camera principal point x
            {"camera_cy": camera_cy},              # Camera principal point y
            {"image_topic": image_topic},          # Camera image topic
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_cone_angle_arg,
            declare_wall_threshold_arg,
            declare_marker_size_arg,
            declare_aruco_dict_arg,
            declare_image_topic_arg,
            declare_camera_fx_arg,
            declare_camera_fy_arg,
            declare_camera_cx_arg,
            declare_camera_cy_arg,
            wall_detector,
            aruco_detector,
        ]
    )
