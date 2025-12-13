#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "soar_rosbot_controller/motion_controller.hpp"

namespace soar_rosbot_controller
{

MotionController::MotionController()
: Node("motion_controller"), last_command_(""), current_yaw_(0.0), yaw_received_(false),
  front_wall_distance_(0.0), wall_data_received_(false)
{
  // Declare and get kinematics type parameter
  this->declare_parameter("holonomic", true);
  is_holonomic_ = this->get_parameter("holonomic").as_bool();

  // Declare velocity parameters
  this->declare_parameter("linear_velocity_forward", 1.0);
  this->declare_parameter("linear_velocity_approach", 0.15);
  this->declare_parameter("angular_velocity_turn", 0.5);
  this->declare_parameter("angular_velocity_turn_around", 1.0);

  // Initialize rotation state
  rotation_state_.active = false;
  rotation_state_.target_yaw = 0.0;
  rotation_state_.angular_velocity = 0.0;
  rotation_state_.tolerance = 0.05;  // ~2.9 degrees

  // Initialize command mappings
  initializeCommandMappings();

  // Create subscriber for command messages using lambda
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/soar/command", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { commandCallback(msg); });

  // Create subscriber for yaw from yaw_observer
  yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/rosbot/yaw", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) { yawCallback(msg); });

  // Create subscriber for wall detection
  wall_sub_ = this->create_subscription<soar_rosbot_msgs::msg::WallDetection>(
    "/wall/detection", 10,
    [this](const soar_rosbot_msgs::msg::WallDetection::SharedPtr msg) { wallCallback(msg); });

  // Create publisher for velocity commands
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Motion controller initialized - Kinematics: %s",
    is_holonomic_ ? "Holonomic" : "Non-holonomic");
  RCLCPP_INFO(this->get_logger(), "Listening for commands on /soar/command");
  RCLCPP_INFO(this->get_logger(), "Subscribing to /rosbot/yaw for rotation control");
  RCLCPP_INFO(this->get_logger(), "Subscribing to /wall/detection for safety monitoring");
  RCLCPP_INFO(this->get_logger(), "Safety policy: Stop if front wall < %.2fm", SAFETY_DISTANCE_THRESHOLD);
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
  double angular_vel_turn = this->get_parameter("angular_velocity_turn").as_double();

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

  // Reject any command if rotation is already in progress
  if (rotation_state_.active) {
    RCLCPP_WARN(this->get_logger(), "Cannot execute '%s': rotation in progress", command.c_str());
    return;
  }

  // Handle rotation commands specially
  if (command == "rotate-right") {
    if (!yaw_received_) {
      RCLCPP_DEBUG(this->get_logger(), "Cannot rotate-right: yaw not available yet");
      return;
    }
    double target_yaw = normalizeAngle(current_yaw_ - M_PI / 2.0);  // -90 degrees
    double angular_vel = -this->get_parameter("angular_velocity_turn").as_double();
    startRotation(target_yaw, angular_vel);
    RCLCPP_INFO(this->get_logger(), "Starting rotate-right: current=%.2f, target=%.2f",
                current_yaw_, target_yaw);
    last_command_ = command;
    return;
  }

  if (command == "rotate-left") {
    if (!yaw_received_) {
      RCLCPP_WARN(this->get_logger(), "Cannot rotate-left: yaw not available yet");
      return;
    }
    double target_yaw = normalizeAngle(current_yaw_ + M_PI / 2.0);  // +90 degrees
    double angular_vel = this->get_parameter("angular_velocity_turn").as_double();
    startRotation(target_yaw, angular_vel);
    RCLCPP_INFO(this->get_logger(), "Starting rotate-left: current=%.2f, target=%.2f",
                current_yaw_, target_yaw);
    last_command_ = command;
    return;
  }

  // SAFETY POLICY: Stop if trying to move forward and front wall is too close
  if (command == "move-forward" && wall_data_received_) {
    if (front_wall_distance_ > 0.0 && front_wall_distance_ < SAFETY_DISTANCE_THRESHOLD) {
      RCLCPP_WARN(
        this->get_logger(),
        "SAFETY: Rejecting move-forward command - front wall too close (%.2fm < %.2fm threshold)",
        front_wall_distance_, SAFETY_DISTANCE_THRESHOLD);
      // Send stop command instead
      geometry_msgs::msg::TwistStamped stop_cmd;
      stop_cmd.header.stamp = this->now();
      stop_cmd.header.frame_id = "base_link";
      stop_cmd.twist.linear.x = 0.0;
      stop_cmd.twist.linear.y = 0.0;
      stop_cmd.twist.angular.z = 0.0;
      velocity_pub_->publish(stop_cmd);
      return;
    }
  }

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

void MotionController::yawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_yaw_ = msg->data;
  yaw_received_ = true;

  // Check if we're in an active rotation
  if (rotation_state_.active) {
    double error = normalizeAngle(rotation_state_.target_yaw - current_yaw_);

    // Check if we've reached the target
    if (std::abs(error) < rotation_state_.tolerance) {
      stopRotation();
      RCLCPP_INFO(this->get_logger(), "Rotation complete! Final yaw: %.2f", current_yaw_);
    } else {
      // Continue rotating
      publishVelocity(0.0, 0.0, rotation_state_.angular_velocity);
    }
  }
}

void MotionController::wallCallback(const soar_rosbot_msgs::msg::WallDetection::SharedPtr msg)
{
  front_wall_distance_ = msg->front_distance;
  wall_data_received_ = true;

  RCLCPP_DEBUG(this->get_logger(), "Wall detection: front_distance=%.2fm", front_wall_distance_);
}

void MotionController::startRotation(double target_yaw, double angular_velocity)
{
  rotation_state_.active = true;
  rotation_state_.target_yaw = target_yaw;
  rotation_state_.angular_velocity = angular_velocity;
}

void MotionController::stopRotation()
{
  if (rotation_state_.active) {
    rotation_state_.active = false;
    publishVelocity(0.0, 0.0, 0.0);
  }
}

double MotionController::normalizeAngle(double angle)
{
  // Normalize to [-π, π]
  return std::atan2(std::sin(angle), std::cos(angle));
}

void MotionController::publishVelocity(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::TwistStamped velocity_stamped;
  velocity_stamped.header.stamp = this->now();
  velocity_stamped.header.frame_id = "base_link";
  velocity_stamped.twist.linear.x = linear_x;
  velocity_stamped.twist.linear.y = linear_y;
  velocity_stamped.twist.angular.z = angular_z;

  velocity_pub_->publish(velocity_stamped);
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
