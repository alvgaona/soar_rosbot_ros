#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from soar_rosbot_msgs.msg import WallDetection
import math


class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')

        # Parameters
        self.declare_parameter('front_wall_threshold', 2.5)
        self.declare_parameter('side_wall_threshold', 4.0)
        self.declare_parameter('cone_angle', 5.0)  # Cone angle in degrees (±5° from center)

        self.front_wall_threshold = self.get_parameter('front_wall_threshold').value
        self.side_wall_threshold = self.get_parameter('side_wall_threshold').value
        self.cone_angle = self.get_parameter('cone_angle').value

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher
        self.wall_pub = self.create_publisher(WallDetection, '/wall/detection', 10)

        self.get_logger().info(
            f'Wall detector initialized - Front threshold: {self.front_wall_threshold}m, '
            f'Side threshold: {self.side_wall_threshold}m, Cone: {self.cone_angle}°'
        )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_cone_readings(self, scan_msg, center_angle_deg):
        """
        Get all range readings within a cone centered at `center_angle_deg`.

        Args:
            scan_msg: `LaserScan` message
            center_angle_deg: Center angle in degrees (0=front, 90=left, -90=right, 180=back)

        Returns:
            List of valid range readings within the cone
        """
        center_angle = math.radians(center_angle_deg)
        half_cone = math.radians(self.cone_angle / 2.0)

        readings = []

        for i, range_val in enumerate(scan_msg.ranges):
            # Calculate angle for this reading
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Normalize the difference between current angle and center angle
            angle_diff = abs(self.normalize_angle(angle - center_angle))

            # Check if this reading is within the cone
            if angle_diff <= half_cone:
                # Only include valid readings (not inf, not NaN, within min/max range)
                if (range_val >= scan_msg.range_min and
                    range_val <= scan_msg.range_max and
                    not math.isnan(range_val) and
                    not math.isinf(range_val)):
                    readings.append(range_val)

        return readings

    def detect_wall(self, readings, threshold):
        """
        Determine if a wall is present based on cone readings

        Args:
            readings: List of range values
            threshold: Distance threshold in meters for wall detection

        Returns:
            Tuple of (wall_detected: bool, distance: float)
            - wall_detected: True if wall detected, False otherwise
            - distance: Distance to wall in meters (0.0 if no wall detected)
        """
        if not readings:
            return (False, 0.0)

        # Use minimum distance in the cone
        min_distance = min(readings)

        # Detect wall if threshold is crossed and enough readings are present
        wall_detected = (min_distance < threshold and len(readings) > 3)

        return (wall_detected, min_distance if wall_detected else 0.0)

    def scan_callback(self, msg):
        """Process laser scan and detect walls in all four directions"""

        # LiDAR frame on the robot is 180° rotated from the base link (robot) frame.
        front_readings = self.get_cone_readings(msg, 180.0)   # Front (Robot) is 180° (LiDAR)
        left_readings = self.get_cone_readings(msg, -90.0)    # Left (Robot) is -90° (LiDAR)
        right_readings = self.get_cone_readings(msg, 90.0)    # Right (Robot) is 90° (LiDAR)

        # Detect walls with appropriate thresholds
        front_wall, front_distance = self.detect_wall(front_readings, self.front_wall_threshold)
        left_wall, left_distance = self.detect_wall(left_readings, self.side_wall_threshold)
        right_wall, right_distance = self.detect_wall(right_readings, self.side_wall_threshold)

        # Publish unified wall detection message
        detection_msg = WallDetection()
        detection_msg.front = front_wall
        detection_msg.left = left_wall
        detection_msg.right = right_wall
        detection_msg.back = False  # Not currently detecting back wall
        detection_msg.front_distance = front_distance
        detection_msg.left_distance = left_distance
        detection_msg.right_distance = right_distance
        detection_msg.back_distance = 0.0
        self.wall_pub.publish(detection_msg)

        # Log for debugging (can be commented out later)
        self.get_logger().debug(
            f'Walls - Front: {front_wall} ({front_distance:.2f}m), '
            f'Left: {left_wall} ({left_distance:.2f}m), '
            f'Right: {right_wall} ({right_distance:.2f}m)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
