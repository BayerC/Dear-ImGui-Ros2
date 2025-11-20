#include "imgui_ros2_listener/ui_renderer.hpp"
#include "imgui_ros2_listener/ros_listener.hpp"

#include <imgui.h>
#include <string>

namespace imgui_ros2 {

void render_ui(const RosListener& node, std::string_view topic_name) {
    ImGui::Begin("ROS2 Listener");
    
    ImGui::Text("Topic: /%s", std::string(topic_name).c_str());
    ImGui::Separator();
    
    ImGui::TextWrapped("Last Message:");
    ImGui::Separator();
    ImGui::TextWrapped("%s", node.get_last_message().c_str());
    ImGui::Separator();
    
    const auto& io = ImGui::GetIO();
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    
    ImGui::End();
}

} // namespace imgui_ros2

