#ifndef SOAR_ROSBOT_CONTROLLER_PERCEPTION_SUBSCRIBERS_HPP_
#define SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_

#include <memory>
#include <string>

#include "soar_ros/Subscriber.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace soar_rosbot_controller
{

class ArUcoDetectedSubscriber : public soar_ros::Subscriber<std_msgs::msg::Bool>
{
public:
  ArUcoDetectedSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::Bool>(agent, node, topic),
    aruco_wme_(nullptr)
  {}

  void parse(std_msgs::msg::Bool msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();

    aruco_wme_ = il->CreateIdWME("aruco");
    aruco_wme_->CreateIntWME("detected", msg.data ? 1 : 0);
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* detected_wme_;
};

class ArUcoDistanceSubscriber : public soar_ros::Subscriber<std_msgs::msg::Float32>
{
public:
  ArUcoDistanceSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::Float32>(agent, node, topic) {}

  void parse(std_msgs::msg::Float32 msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();
    aruco_wme_ = il->CreateIdWME("aruco");
    aruco_wme_->CreateFloatWME("distance", msg.data);
  }

private:
  sml::Identifier* aruco_wme_;
};

class WallSubscriber : public soar_ros::Subscriber<std_msgs::msg::Bool>
{
public:
  WallSubscriber(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const std::string & direction)
  : Subscriber<std_msgs::msg::Bool>(agent, node, topic),
    direction_(direction),
    walls_wme_(nullptr)
  {}

  void parse(std_msgs::msg::Bool msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();

    walls_wme_ = il->CreateIdWME("walls");
    walls_wme_->CreateIntWME(direction_.c_str(), msg.data ? 1 : 0);
  }

private:
  std::string direction_;
  sml::Identifier * walls_wme_;
};

}  // namespace soar_rosbot_controller

#endif  // SOAR_ROSBOT_CONTROLLER__PERCEPTION_SUBSCRIBERS_HPP_
