#include "soar_rosbot_controller/motion_controller.hpp"

namespace soar_rosbot_controller
{

MotionController::MotionController()
: Node("motion_controller")
{
  // Declare and get kinematics type parameter
  this->declare_parameter("holonomic", false);
  is_holonomic_ = this->get_parameter("holonomic").as_bool();

  // Initialize command mappings
  initializeCommandMappings();

  // Create subscriber for command messages using lambda
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/soar/command", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { commandCallback(msg); });

  // Create publisher for velocity commands
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Motion controller initialized - Kinematics: %s",
    is_holonomic_ ? "Holonomic" : "Non-holonomic");
  RCLCPP_INFO(this->get_logger(), "Listening for commands on /soar/command");
  RCLCPP_INFO(this->get_logger(), "Publishing velocities to /cmd_vel");
}

MotionController::~MotionController()
{
}

void MotionController::initializeCommandMappings()
{
  if (is_holonomic_) {
    // Holonomic (mecanum/omni) kinematics - can move in any direction
    command_map_["move-forward"] = {0.2, 0.0, 0.0};
    command_map_["move-backward"] = {-0.2, 0.0, 0.0};
    command_map_["strafe-left"] = {0.0, 0.2, 0.0};
    command_map_["strafe-right"] = {0.0, -0.2, 0.0};
    command_map_["turn-right"] = {0.0, 0.0, -0.5};
    command_map_["turn-left"] = {0.0, 0.0, 0.5};
    command_map_["turn-around"] = {0.0, 0.0, 1.0};
    command_map_["approach-goal"] = {0.15, 0.0, 0.0};
    command_map_["stop"] = {0.0, 0.0, 0.0};
    command_map_["stop-at-goal"] = {0.0, 0.0, 0.0};
  } else {
    // Non-holonomic (differential drive) kinematics - can only move forward/backward and turn
    command_map_["move-forward"] = {0.2, 0.0, 0.0};
    command_map_["move-backward"] = {-0.2, 0.0, 0.0};
    command_map_["turn-right"] = {0.0, 0.0, -0.5};
    command_map_["turn-left"] = {0.0, 0.0, 0.5};
    command_map_["turn-around"] = {0.0, 0.0, 1.0};
    command_map_["approach-goal"] = {0.15, 0.0, 0.0};
    command_map_["stop"] = {0.0, 0.0, 0.0};
    command_map_["stop-at-goal"] = {0.0, 0.0, 0.0};
  }
}

void MotionController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & command = msg->data;

  RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

  // Convert command to velocity
  geometry_msgs::msg::Twist velocity = commandToVelocity(command);

  // Publish velocity
  velocity_pub_->publish(velocity);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing velocity: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
    velocity.linear.x, velocity.linear.y, velocity.angular.z);
}

geometry_msgs::msg::Twist MotionController::commandToVelocity(const std::string & command)
{
  geometry_msgs::msg::Twist velocity;

  // Look up command in map
  auto it = command_map_.find(command);
  if (it != command_map_.end()) {
    velocity.linear.x = it->second.linear_x;
    velocity.linear.y = it->second.linear_y;
    velocity.angular.z = it->second.angular_z;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown command '%s', defaulting to stop", command.c_str());
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.angular.z = 0.0;
  }

  return velocity;
}

}  // namespace soar_rosbot_controller
