#include "imgui_ros2_listener/ros_listener.hpp"

#include <utility>

namespace imgui_ros2
{

namespace
{
constexpr size_t kQueueSize = 10;
constexpr char kDefaultMessage[] = "Waiting for messages...";
}  // namespace

RosListener::RosListener(const std::string & topic_name)
: Node("imgui_listener"),
  topic_name_(topic_name),
  last_message_(kDefaultMessage)
{
  // Use reliable QoS for guaranteed message delivery
  auto qos = rclcpp::QoS(rclcpp::KeepLast(kQueueSize))
    .reliable()
    .durability_volatile();

  subscription_ = create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    [this](std_msgs::msg::String::SharedPtr msg) {
      message_callback(std::move(msg));
    });

  RCLCPP_INFO(
    get_logger(),
    "ROS2 listener node initialized, subscribing to /%s",
    topic_name_.c_str());
}

void RosListener::message_callback(std_msgs::msg::String::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(message_mutex_);
    last_message_ = std::move(msg->data);
  }
  RCLCPP_INFO(get_logger(), "Received: '%s'", last_message_.c_str());
}

std::string RosListener::get_last_message() const
{
  std::lock_guard<std::mutex> lock(message_mutex_);
  return last_message_;
}

}  // namespace imgui_ros2

