#ifndef SOAR_ROSBOT_CONTROLLER__COMMAND_PUBLISHER_HPP_
#define SOAR_ROSBOT_CONTROLLER__COMMAND_PUBLISHER_HPP_

#include "soar_ros/Publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace soar_rosbot_controller {

class CommandPublisher : public soar_ros::Publisher<std_msgs::msg::String> {
public:
    CommandPublisher(
        sml::Agent * agent,
        rclcpp::Node::SharedPtr node,
        const std::string & topic
    ) : Publisher<std_msgs::msg::String>(agent, node, topic) {}

    ~CommandPublisher() = default;

    std_msgs::msg::String parse(sml::Identifier * id) override {
        std_msgs::msg::String msg;

        // Get command name from Soar output-link
        const char * command_name = id->GetParameterValue("name");
        
        // TODO: if command_name is nullptr, ignore or handle error
        msg.data = command_name;

        RCLCPP_INFO(
            m_node->get_logger(),
            "[SOAR->ROS] Publishing command to /soar/command: '%s'", msg.data.c_str());

        return msg;
    }
};

} // namespace soar_rosbot_controller

#endif // SOAR_ROSBOT_CONTROLLER__COMMAND_PUBLISHER_HPP_