#include "imgui_ros2_listener/ros_listener.hpp"

namespace imgui_ros2 {

namespace {
constexpr std::string_view kTopicName = "chatter";
constexpr int kQueueSize = 10;
} // namespace

RosListener::RosListener() 
    : Node("imgui_listener"), 
      last_message_("Waiting for messages...") {
    
    subscription_ = create_subscription<std_msgs::msg::String>(
        std::string(kTopicName), kQueueSize,
        [this](std_msgs::msg::String::SharedPtr msg) {
            message_callback(std::move(msg));
        });
    
    RCLCPP_INFO(get_logger(), "ROS2 listener node initialized, subscribing to /%s", 
                std::string(kTopicName).c_str());
}

void RosListener::message_callback(std_msgs::msg::String::SharedPtr msg) {
    last_message_ = std::move(msg->data);
    RCLCPP_INFO(get_logger(), "Received: '%s'", last_message_.c_str());
}

} // namespace imgui_ros2

