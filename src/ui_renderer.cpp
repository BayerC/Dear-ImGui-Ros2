#include "imgui_ros2_listener/ui_renderer.hpp"
#include "imgui_ros2_listener/ros_listener.hpp"

#include <imgui.h>

namespace imgui_ros2
{

void render_ui(const RosListener & node) noexcept
{
  ImGui::Begin("ROS2 Listener");

  ImGui::Text("Topic: /%s", node.get_topic_name().c_str());
  ImGui::Separator();

  ImGui::TextWrapped("Last Message:");
  ImGui::Separator();

  const std::string last_message = node.get_last_message();
  ImGui::TextWrapped("%s", last_message.c_str());
  ImGui::Separator();

  const auto & io = ImGui::GetIO();
  ImGui::Text(
    "%.3f ms/frame (%.1f FPS)",
    static_cast<double>(1000.0f / io.Framerate),
    static_cast<double>(io.Framerate));

  ImGui::End();
}

}  // namespace imgui_ros2

