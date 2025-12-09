#ifndef SOAR_ROSBOT_CONTROLLER__MOTION_CONTROLLER_HPP_
#define SOAR_ROSBOT_CONTROLLER__MOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

  // Subscriber for commands
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

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

#endif  // SOAR_ROSBOT_CONTROLLER__MOTION_CONTROLLER_HPP_
