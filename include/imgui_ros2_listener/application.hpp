#pragma once

#include <cstdlib>
#include <memory>

namespace imgui_ros2
{

class GLFWContext;
class ImGuiContext;
class RosListener;

class Application
{
public:
  Application();
  ~Application();

  Application(const Application &) = delete;
  Application & operator=(const Application &) = delete;
  Application(Application &&) = delete;
  Application & operator=(Application &&) = delete;

  [[nodiscard]] int run();

private:
  std::unique_ptr<RosListener> node_;
  std::unique_ptr<GLFWContext> glfw_context_;
  std::unique_ptr<ImGuiContext> imgui_context_;
};

}  // namespace imgui_ros2

