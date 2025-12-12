#ifndef SOAR_ROSBOT_CONTROLLER_PERCEPTION_SUBSCRIBERS_HPP_
#define SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_

#include <memory>
#include <string>
#include <optional>

#include "soar_ros/Subscriber.hpp"
#include "std_msgs/msg/bool.hpp"
#include "soar_rosbot_msgs/msg/aruco_detection.hpp"
#include "soar_rosbot_msgs/msg/wall_detection.hpp"

namespace soar_rosbot_controller
{

class ArUcoSubscriber : public soar_ros::Subscriber<soar_rosbot_msgs::msg::ArucoDetection>
{
public:
  ArUcoSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<soar_rosbot_msgs::msg::ArucoDetection>(agent, node, topic),
    aruco_wme_(nullptr),
    detected_wme_(nullptr),
    distance_wme_(nullptr)
  {}

  // Override process_r2s to drain queue and use only the latest value
  void process_r2s() override
  {
    // Drain the queue, keeping only the latest message
    std::optional<soar_rosbot_msgs::msg::ArucoDetection> latest;
    auto res = this->m_r2sQueue.tryPop();
    while (res.has_value()) {
      latest = res;
      res = this->m_r2sQueue.tryPop();
    }

    if (!latest.has_value()) {
      return;  // No messages
    }

    bool detected = latest.value().detected;
    float distance = latest.value().distance;

    sml::Identifier * il = this->m_pAgent->GetInputLink();

    RCLCPP_INFO(
      this->m_node->get_logger(),
      "ArUcoSubscriber: detected=%d, distance=%.3f",
      detected ? 1 : 0,
      distance
    );

    // Remove old WMEs if they exist
    if (aruco_wme_ != nullptr) {
      aruco_wme_->DestroyWME();
      aruco_wme_ = nullptr;
      detected_wme_ = nullptr;
      distance_wme_ = nullptr;
    }

    // Create new WMEs
    aruco_wme_ = il->CreateIdWME("aruco");
    detected_wme_ = aruco_wme_->CreateIntWME("detected", detected ? 1 : 0);
    distance_wme_ = aruco_wme_->CreateFloatWME("distance", distance);
  }

  void parse(soar_rosbot_msgs::msg::ArucoDetection msg) override
  {
    // Not used - we override process_r2s directly
    (void)msg;
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* detected_wme_;
  sml::WMElement* distance_wme_;
};

class WallSubscriber : public soar_ros::Subscriber<soar_rosbot_msgs::msg::WallDetection>
{
public:
  WallSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<soar_rosbot_msgs::msg::WallDetection>(agent, node, topic),
    walls_wme_(nullptr),
    front_wme_(nullptr),
    left_wme_(nullptr),
    right_wme_(nullptr),
    back_wme_(nullptr)
  {}

  // Override process_r2s to drain queue and use only the latest value
  void process_r2s() override
  {
    // Drain the queue, keeping only the latest message
    std::optional<soar_rosbot_msgs::msg::WallDetection> latest;
    auto res = this->m_r2sQueue.tryPop();
    while (res.has_value()) {
      latest = res;
      res = this->m_r2sQueue.tryPop();
    }

    if (!latest.has_value()) {
      return;  // No messages
    }

    bool front = latest.value().front;
    bool left = latest.value().left;
    bool right = latest.value().right;
    bool back = latest.value().back;

    sml::Identifier * il = this->m_pAgent->GetInputLink();

    RCLCPP_DEBUG(
      this->m_node->get_logger(),
      "WallSubscriber: front=%d, left=%d, right=%d, back=%d",
      front ? 1 : 0,
      left ? 1 : 0,
      right ? 1 : 0,
      back ? 1 : 0
    );

    // Remove old WMEs if they exist
    if (walls_wme_ != nullptr) {
      walls_wme_->DestroyWME();
      walls_wme_ = nullptr;
      front_wme_ = nullptr;
      left_wme_ = nullptr;
      right_wme_ = nullptr;
      back_wme_ = nullptr;
    }

    // Create new WMEs
    walls_wme_ = il->CreateIdWME("walls");
    front_wme_ = walls_wme_->CreateIntWME("front", front ? 1 : 0);
    left_wme_ = walls_wme_->CreateIntWME("left", left ? 1 : 0);
    right_wme_ = walls_wme_->CreateIntWME("right", right ? 1 : 0);
    back_wme_ = walls_wme_->CreateIntWME("back", back ? 1 : 0);
  }

  void parse(soar_rosbot_msgs::msg::WallDetection msg) override
  {
    // Not used - we override process_r2s directly
    (void)msg;
  }

private:
  sml::Identifier* walls_wme_;
  sml::WMElement* front_wme_;
  sml::WMElement* left_wme_;
  sml::WMElement* right_wme_;
  sml::WMElement* back_wme_;
};

}  // namespace soar_rosbot_controller

#endif  // SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_
