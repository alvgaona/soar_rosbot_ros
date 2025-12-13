#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace soar_rosbot_controller
{

class YawObserver : public rclcpp::Node
{
public:
  YawObserver();
  ~YawObserver() = default;

private:
  void publishYaw();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace soar_rosbot_controller
