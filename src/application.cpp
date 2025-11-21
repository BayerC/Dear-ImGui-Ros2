#include "imgui_ros2_listener/application.hpp"
#include "imgui_ros2_listener/imgui_context.hpp"
#include "imgui_ros2_listener/ros_listener.hpp"
#include "imgui_ros2_listener/ui_renderer.hpp"

#include <imgui.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace
{
constexpr char kWindowTitle[] = "ROS2 ImGui Listener";
constexpr char kTopicName[] = "chatter";
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

}  // namespace

namespace imgui_ros2
{

Application::Application()
: node_{RosListener::create(kTopicName)},
  glfw_context_{std::make_unique<GLFWContext>(kWindowWidth, kWindowHeight, kWindowTitle)},
  imgui_context_{std::make_unique<ImGuiContext>(glfw_context_->get_window())}
{
}

Application::~Application() = default;

int Application::run()
{
  try {

    while (!glfwWindowShouldClose(glfw_context_->get_window()) && RosListener::is_running()) {
      node_->spin();
      glfwPollEvents();

      imgui_context_->new_frame();
      render_ui(*node_);

      const auto [width, height] = get_framebuffer_size(glfw_context_->get_window());

      glViewport(0, 0, width, height);
      glClearColor(
        kClearColor.x * kClearColor.w,
        kClearColor.y * kClearColor.w,
        kClearColor.z * kClearColor.w,
        kClearColor.w);
      glClear(GL_COLOR_BUFFER_BIT);

      imgui_context_->render();
      glfwSwapBuffers(glfw_context_->get_window());
    }

    return EXIT_SUCCESS;

  } catch (const std::exception & e) {
    std::cerr << "Application error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}

}  // namespace imgui_ros2

