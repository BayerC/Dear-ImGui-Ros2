#include "imgui_ros2_listener/application.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  imgui_ros2::Application app{};
  const int result = app.run();

  rclcpp::shutdown();
  return result;
}

