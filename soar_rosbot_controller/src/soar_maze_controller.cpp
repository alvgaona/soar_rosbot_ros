#include "soar_rosbot_controller/soar_maze_controller.hpp"

namespace soar_rosbot_controller
{

CommandPublisher::CommandPublisher(
  sml::Agent * agent,
  rclcpp::Node::SharedPtr node,
  const std::string & topic)
: Publisher<std_msgs::msg::String>(agent, node, topic)
{
}

std_msgs::msg::String CommandPublisher::parse(sml::Identifier * id)
{
  std_msgs::msg::String msg;

  // Get command name from Soar output-link
  const char * command_name = id->GetParameterValue("name");
  if (command_name != nullptr) {
    msg.data = command_name;
  } else {
    msg.data = "stop";  // Default to stop if no command specified
  }

  RCLCPP_INFO(
    m_node->get_logger(),
    "Publishing command: %s", msg.data.c_str());

  return msg;
}

SoarMazeController::SoarMazeController(
  const std::string & agent_name,
  const std::string & soar_rules_path)
: SoarRunner(agent_name, soar_rules_path),
  front_wall_(false),
  left_wall_(false),
  right_wall_(false),
  back_wall_(false),
  aruco_detected_(false),
  aruco_distance_(0.0f),
  walls_wme_(nullptr),
  aruco_wme_(nullptr)
{
  // Create subscribers for wall detection
  front_wall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wall/front", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { frontWallCallback(msg); });

  left_wall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wall/left", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { leftWallCallback(msg); });

  right_wall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wall/right", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { rightWallCallback(msg); });

  back_wall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wall/back", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { backWallCallback(msg); });

  // Create subscribers for ArUco detection
  aruco_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/aruco/detected", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { arucoDetectedCallback(msg); });

  aruco_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/aruco/distance", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) { arucoDistanceCallback(msg); });

  // Create command publisher
  command_publisher_ = std::make_shared<CommandPublisher>(
    this->getAgent(),
    this->shared_from_this(),
    "/soar/command");
  this->addPublisher(command_publisher_);

  // Create timer to update Soar input-link (10 Hz)
  update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { updateSoarInputLink(); });

  RCLCPP_INFO(this->get_logger(), "Soar Maze Controller initialized");
}

SoarMazeController::~SoarMazeController()
{
}

void SoarMazeController::frontWallCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  front_wall_ = msg->data;
}

void SoarMazeController::leftWallCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  left_wall_ = msg->data;
}

void SoarMazeController::rightWallCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  right_wall_ = msg->data;
}

void SoarMazeController::backWallCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  back_wall_ = msg->data;
}

void SoarMazeController::arucoDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  aruco_detected_ = msg->data;
}

void SoarMazeController::arucoDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  aruco_distance_ = msg->data;
}

void SoarMazeController::updateSoarInputLink()
{
  sml::Identifier * input_link = this->getAgent()->GetInputLink();

  if (input_link == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Input link is null");
    return;
  }

  // Destroy and recreate walls WME each update (simple and safe approach)
  if (walls_wme_ != nullptr) {
    input_link->DestroyWME(walls_wme_);
  }
  walls_wme_ = input_link->CreateIdWME("walls");
  walls_wme_->CreateIntWME("front", front_wall_ ? 1 : 0);
  walls_wme_->CreateIntWME("left", left_wall_ ? 1 : 0);
  walls_wme_->CreateIntWME("right", right_wall_ ? 1 : 0);
  walls_wme_->CreateIntWME("back", back_wall_ ? 1 : 0);

  // Destroy and recreate ArUco WME each update
  if (aruco_wme_ != nullptr) {
    input_link->DestroyWME(aruco_wme_);
  }
  aruco_wme_ = input_link->CreateIdWME("aruco");
  aruco_wme_->CreateIntWME("detected", aruco_detected_ ? 1 : 0);
  aruco_wme_->CreateFloatWME("distance", aruco_distance_);
}

}  // namespace soar_rosbot_controller
