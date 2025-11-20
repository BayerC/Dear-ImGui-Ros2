#include "imgui_ros2_listener/imgui_context.hpp"
#include "imgui_ros2_listener/ros_listener.hpp"
#include "imgui_ros2_listener/ui_renderer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <imgui.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace
{
constexpr char kWindowTitle[] = "ROS2 ImGui Listener";
constexpr char kDefaultTopicName[] = "chatter";
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr ImVec4 kClearColor{0.45f, 0.55f, 0.60f, 1.00f};

struct FramebufferSize
{
  int width;
  int height;
};

[[nodiscard]] FramebufferSize get_framebuffer_size(GLFWwindow * window) noexcept
{
  FramebufferSize size{};
  glfwGetFramebufferSize(window, &size.width, &size.height);
  return size;
}

[[nodiscard]] std::string get_topic_name(int argc, char ** argv)
{
  if (argc > 1) {
    return argv[1];
  }
  return kDefaultTopicName;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    const std::string topic_name = get_topic_name(argc, argv);
    auto node = std::make_shared<imgui_ros2::RosListener>(topic_name);

    // Setup GLFW and ImGui with RAII
    imgui_ros2::GLFWContext glfw_context(kWindowWidth, kWindowHeight, kWindowTitle);
    imgui_ros2::ImGuiContext imgui_context(glfw_context.get_window());

    // Main loop
    while (!glfwWindowShouldClose(glfw_context.get_window()) && rclcpp::ok()) {
      rclcpp::spin_some(node);
      glfwPollEvents();

      imgui_context.new_frame();
      imgui_ros2::render_ui(*node);

      const auto [width, height] = get_framebuffer_size(glfw_context.get_window());

      glViewport(0, 0, width, height);
      glClearColor(
        kClearColor.x * kClearColor.w,
        kClearColor.y * kClearColor.w,
        kClearColor.z * kClearColor.w,
        kClearColor.w);
      glClear(GL_COLOR_BUFFER_BIT);

      imgui_context.render();
      glfwSwapBuffers(glfw_context.get_window());
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}

