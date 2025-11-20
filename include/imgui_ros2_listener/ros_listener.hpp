#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace imgui_ros2 {

class RosListener final : public rclcpp::Node {
public:
    RosListener();

    [[nodiscard]] const std::string& get_last_message() const noexcept { 
        return last_message_; 
    }

private:
    void message_callback(std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string last_message_;
};

} // namespace imgui_ros2

