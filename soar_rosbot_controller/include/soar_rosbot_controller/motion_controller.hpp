#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "soar_rosbot_msgs/msg/wall_detection.hpp"

namespace soar_rosbot_controller
{

/**
 * @brief Motion controller that converts high-level commands to velocity values
 *
 * Subscribes to command strings from Soar and publishes Twist messages to /cmd_vel.
 * Supports both holonomic (mecanum/omni) and non-holonomic (differential drive) kinematics.
 */
class MotionController : public rclcpp::Node
{
public:
  MotionController();
  ~MotionController();

private:
  /**
   * @brief Callback for command messages from Soar
   */
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Convert command string to velocity values
   */
  geometry_msgs::msg::Twist commandToVelocity(const std::string & command);

  /**
   * @brief Initialize command mappings based on kinematics type
   */
  void initializeCommandMappings();

  /**
   * @brief Callback for yaw updates from yaw_observer
   */
  void yawCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback for wall detection updates
   */
  void wallCallback(const soar_rosbot_msgs::msg::WallDetection::SharedPtr msg);

  /**
   * @brief Start a rotation to reach target yaw
   */
  void startRotation(double target_yaw, double angular_velocity);

  /**
   * @brief Stop current rotation
   */
  void stopRotation();

  /**
   * @brief Normalize angle to [-π, π]
   */
  double normalizeAngle(double angle);

  /**
   * @brief Publish velocity command
   */
  void publishVelocity(double linear_x, double linear_y, double angular_z);

  // Subscriber for commands
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  // Subscriber for yaw from yaw_observer
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;

  // Subscriber for wall detection
  rclcpp::Subscription<soar_rosbot_msgs::msg::WallDetection>::SharedPtr wall_sub_;

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

  // Track last command sent to avoid sending duplicates
  std::string last_command_;

  // Current yaw from yaw_observer
  double current_yaw_;
  bool yaw_received_;

  // Safety: Front wall distance
  double front_wall_distance_;
  bool wall_data_received_;
  static constexpr double SAFETY_DISTANCE_THRESHOLD = 2.0;  // meters

  // Rotation state
  struct RotationState {
    bool active;
    double target_yaw;
    double angular_velocity;
    double tolerance;  // radians (~2.9 degrees)
  };
  RotationState rotation_state_;

  // Kinematics type
  bool is_holonomic_;

  // Command to velocity mapping
  struct VelocityCommand {
    double linear_x;
    double linear_y;  // Only used for holonomic
    double angular_z;
  };

  std::unordered_map<std::string, VelocityCommand> command_map_;
};

}  // namespace soar_rosbot_controller
