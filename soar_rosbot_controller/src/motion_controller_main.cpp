#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "soar_rosbot_controller/motion_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<soar_rosbot_controller::MotionController>();

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Motion Controller...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
