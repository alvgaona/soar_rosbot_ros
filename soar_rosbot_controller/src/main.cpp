#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "soar_rosbot_controller/soar_maze_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Get path to Soar rules
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("soar_rosbot_controller");
  std::string soar_rules_path = package_share_directory + "/soar_rules";

  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Loading Soar rules from: %s", soar_rules_path.c_str());

  // Create Soar controller node
  auto node = std::make_shared<soar_rosbot_controller::SoarMazeController>(
    "MazeSolver",
    soar_rules_path);

  // Start Soar thread
  node->startThread();

  // Create multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Soar Maze Controller...");

  // Spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
