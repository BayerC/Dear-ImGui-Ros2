#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace imgui_ros2
{

class RosListener final : public rclcpp::Node
{
public:
  static std::unique_ptr<RosListener> create(const std::string & topic_name);

  [[nodiscard]] std::string get_last_message() const;

  [[nodiscard]] const std::string & get_topic_name() const noexcept
  {
    return topic_name_;
  }

  void spin();

  [[nodiscard]] static bool is_running() noexcept;

private:
  explicit RosListener(const std::string & topic_name);

  void message_callback(std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::Node> node_handle_;
  std::string topic_name_;
  mutable std::mutex message_mutex_;
  std::string last_message_;
};

}  // namespace imgui_ros2

