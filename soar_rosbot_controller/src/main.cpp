#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "soar_ros/SoarRunner.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "soar_rosbot_controller/perception_subscribers.hpp"
#include "soar_rosbot_controller/command_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string package_share_directory;
  try {
    package_share_directory =
      ament_index_cpp::get_package_share_directory("soar_rosbot_controller");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("main"),
      "Failed to find package 'soar_rosbot_controller': %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  std::filesystem::path soar_rules_path = package_share_directory + "/soar_rules";

  if (!std::filesystem::exists(soar_rules_path)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("main"),
      "Soar rules directory does not exist: %s", soar_rules_path.string().c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::filesystem::path main_soar_file = soar_rules_path / "main.soar";
  if (!std::filesystem::exists(main_soar_file)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("main"),
      "Main Soar file does not exist: %s", main_soar_file.string().c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Loading Soar rules from: %s", main_soar_file.string().c_str());

  auto node = std::make_shared<soar_ros::SoarRunner>("MazeSolver", main_soar_file);

  auto command_publisher = std::make_shared<soar_rosbot_controller::CommandPublisher>(
    node->getAgent(),
    node,
    "/soar/command");
  // Register with Soar command name "command" to match (<ol> ^command ...)
  node->addPublisher<std_msgs::msg::String>(command_publisher, "command");

  // Create ArUco subscribers (share WME pointers for coordinated updates)
  auto aruco_detected_sub = std::make_shared<soar_rosbot_controller::ArUcoDetectedSubscriber>(
    node->getAgent(),
    node,
    "/aruco/detected");
  node->addSubscriber<std_msgs::msg::Bool>(aruco_detected_sub);

  auto aruco_distance_sub = std::make_shared<soar_rosbot_controller::ArUcoDistanceSubscriber>(
    node->getAgent(),
    node,
    "/aruco/distance");
  node->addSubscriber<std_msgs::msg::Float32>(aruco_distance_sub);

  // Start Soar kernel thread
  node->startThread();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Soar Maze Controller...");

  // Spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
