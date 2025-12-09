#ifndef SOAR_ROSBOT_CONTROLLER__SOAR_MAZE_CONTROLLER_HPP_
#define SOAR_ROSBOT_CONTROLLER__SOAR_MAZE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "soar_ros/SoarRunner.hpp"
#include "soar_ros/Publisher.hpp"

namespace soar_rosbot_controller
{

/**
 * @brief Publisher for command strings from Soar output-link to ROS
 */
class CommandPublisher : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
  CommandPublisher(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic);

  ~CommandPublisher() {}

  std_msgs::msg::String parse(sml::Identifier * id) override;
};

/**
 * @brief Main Soar controller node for maze solving
 *
 * This node:
 * - Subscribes to wall detection topics (/wall/front, /wall/left, /wall/right, /wall/back)
 * - Subscribes to ArUco detection topics (/aruco/detected, /aruco/distance)
 * - Feeds perception data into Soar's input-link
 * - Publishes command strings from Soar's output-link to /soar/command
 */
class SoarMazeController : public soar_ros::SoarRunner
{
public:
  SoarMazeController(
    const std::string & agent_name,
    const std::string & soar_rules_path);

  ~SoarMazeController();

private:
  // Wall detection callbacks
  void frontWallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void leftWallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void rightWallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void backWallCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // ArUco detection callbacks
  void arucoDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void arucoDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // Update Soar input-link with perception data
  void updateSoarInputLink();

  // Subscribers for perception data
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_wall_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_wall_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_wall_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr back_wall_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aruco_distance_sub_;

  // Publisher for command strings
  std::shared_ptr<CommandPublisher> command_publisher_;

  // Current perception state
  bool front_wall_;
  bool left_wall_;
  bool right_wall_;
  bool back_wall_;
  bool aruco_detected_;
  float aruco_distance_;

  // Soar input-link identifiers
  sml::Identifier * walls_wme_;
  sml::Identifier * aruco_wme_;

  // Timer for updating Soar input-link
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace soar_rosbot_controller

#endif  // SOAR_ROSBOT_CONTROLLER__SOAR_MAZE_CONTROLLER_HPP_
