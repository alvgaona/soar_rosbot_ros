#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "soar_rosbot_controller/motion_controller.hpp"

namespace soar_rosbot_controller
{

MotionController::MotionController()
: Node("motion_controller"), last_command_("")
{
  // Declare and get kinematics type parameter
  this->declare_parameter("holonomic", true);
  is_holonomic_ = this->get_parameter("holonomic").as_bool();

  // Declare velocity parameters
  this->declare_parameter("linear_velocity_forward", 1.0);
  this->declare_parameter("linear_velocity_approach", 0.15);
  this->declare_parameter("angular_velocity_turn", 0.5);
  this->declare_parameter("angular_velocity_turn_around", 1.0);

  // Initialize command mappings
  initializeCommandMappings();

  // Create subscriber for command messages using lambda
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/soar/command", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { commandCallback(msg); });

  // Create publisher for velocity commands
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Motion controller initialized - Kinematics: %s",
    is_holonomic_ ? "Holonomic" : "Non-holonomic");
  RCLCPP_INFO(this->get_logger(), "Listening for commands on /soar/command");
  RCLCPP_INFO(this->get_logger(), "Publishing velocities to /cmd_vel");
}

MotionController::~MotionController()
{
  // Send stop command before shutting down
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = this->now();
  stop_cmd.header.frame_id = "base_link";
  stop_cmd.twist.linear.x = 0.0;
  stop_cmd.twist.linear.y = 0.0;
  stop_cmd.twist.angular.z = 0.0;

  velocity_pub_->publish(stop_cmd);

  RCLCPP_INFO(this->get_logger(), "Motion controller shutting down - stop command sent");
}

void MotionController::initializeCommandMappings()
{
  // Get velocity parameters
  double linear_vel_forward = this->get_parameter("linear_velocity_forward").as_double();
  double linear_vel_approach = this->get_parameter("linear_velocity_approach").as_double();
  double angular_vel_turn = this->get_parameter("angular_velocity_turn").as_double();
  double angular_vel_turn_around = this->get_parameter("angular_velocity_turn_around").as_double();

  if (is_holonomic_) {
    command_map_["move-forward"] = {linear_vel_forward, 0.0, 0.0};
    command_map_["move-backward"] = {-linear_vel_forward, 0.0, 0.0};
    command_map_["strafe-left"] = {0.0, linear_vel_forward, 0.0};
    command_map_["strafe-right"] = {0.0, -linear_vel_forward, 0.0};
    command_map_["turn-cw"] = {0.0, 0.0, -angular_vel_turn};
    command_map_["turn-ccw"] = {0.0, 0.0, angular_vel_turn};
    command_map_["stop"] = {0.0, 0.0, 0.0};
  } else {
    command_map_["move-forward"] = {linear_vel_forward, 0.0, 0.0};
    command_map_["move-backward"] = {-linear_vel_forward, 0.0, 0.0};
    command_map_["turn-cw"] = {0.0, 0.0, -angular_vel_turn};
    command_map_["turn-ccw"] = {0.0, 0.0, angular_vel_turn};
    command_map_["stop"] = {0.0, 0.0, 0.0};
  }
}

void MotionController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & command = msg->data;

  RCLCPP_DEBUG(this->get_logger(), "Received command: %s", command.c_str());

  // Convert command to velocity
  geometry_msgs::msg::Twist velocity = commandToVelocity(command);

  // Create TwistStamped message with timestamp
  geometry_msgs::msg::TwistStamped velocity_stamped;
  velocity_stamped.header.stamp = this->now();
  velocity_stamped.header.frame_id = "base_link";
  velocity_stamped.twist = velocity;

  velocity_pub_->publish(velocity_stamped);

  // Log only when command changes
  if (command != last_command_) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Command changed to: %s - Publishing velocity: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
      command.c_str(), velocity.linear.x, velocity.linear.y, velocity.angular.z);
    last_command_ = command;
  }
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<soar_rosbot_controller::MotionController>();

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Motion Controller...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
