#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')

        # Parameters
        self.declare_parameter('wall_threshold', 2.15)  # Distance in meters to consider a wall
        self.declare_parameter('cone_angle', 5.0)  # Cone angle in degrees (±5° from center)

        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.cone_angle = self.get_parameter('cone_angle').value

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers for each wall direction
        self.front_wall_pub = self.create_publisher(Bool, '/wall/front', 10)
        self.left_wall_pub = self.create_publisher(Bool, '/wall/left', 10)
        self.right_wall_pub = self.create_publisher(Bool, '/wall/right', 10)
        self.back_wall_pub = self.create_publisher(Bool, '/wall/back', 10)

        self.get_logger().info(f'Wall detector initialized with threshold: {self.wall_threshold}m, cone: {self.cone_angle}°')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_cone_readings(self, scan_msg, center_angle_deg):
        """
        Get all range readings within a cone centered at center_angle_deg

        Args:
            scan_msg: LaserScan message
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
                # Only include valid readings (not inf, not nan, within min/max range)
                if (range_val >= scan_msg.range_min and
                    range_val <= scan_msg.range_max and
                    not math.isnan(range_val) and
                    not math.isinf(range_val)):
                    readings.append(range_val)

        return readings

    def detect_wall(self, readings):
        """
        Determine if a wall is present based on cone readings

        Args:
            readings: List of range values

        Returns:
            True if wall detected, False otherwise
        """
        if not readings:
            return False

        # Use minimum distance in the cone
        min_distance = min(readings)
        
        # Detect wall if threshold is crossed and enough readings are present
        wall_detected = (min_distance < self.wall_threshold and len(readings) > 3)

        return wall_detected

    def scan_callback(self, msg):
        """Process laser scan and detect walls in all four directions"""

        # LiDAR frame on the robot is 180° rotated from the base link (robot) frame.
        front_readings = self.get_cone_readings(msg, 180.0)   # Front (Robot) is 180° (LiDAR)
        left_readings = self.get_cone_readings(msg, -90.0)    # Left (Robot) is -90° (LiDAR)
        right_readings = self.get_cone_readings(msg, 90.0)    # Right (Robot) is 90° (LiDAR)

        # Detect walls
        front_wall = self.detect_wall(front_readings)
        left_wall = self.detect_wall(left_readings)
        right_wall = self.detect_wall(right_readings)

        # Publish results
        self.front_wall_pub.publish(Bool(data=front_wall))
        self.left_wall_pub.publish(Bool(data=left_wall))
        self.right_wall_pub.publish(Bool(data=right_wall))

        # Log for debugging (can be commented out later)
        self.get_logger().debug(
            f'Walls - Front: {front_wall}, Left: {left_wall}, '
            f'Right: {right_wall}'
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
