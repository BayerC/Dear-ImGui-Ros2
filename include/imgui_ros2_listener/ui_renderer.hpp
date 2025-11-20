#pragma once

#include <string_view>

namespace imgui_ros2 {

class RosListener;

void render_ui(const RosListener& node, std::string_view topic_name);

} // namespace imgui_ros2

