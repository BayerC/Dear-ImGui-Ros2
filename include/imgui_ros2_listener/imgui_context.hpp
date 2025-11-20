#pragma once

#include <GLFW/glfw3.h>
#include <string_view>

namespace imgui_ros2 {

class GLFWContext final {
public:
    GLFWContext(int width, int height, std::string_view title);
    ~GLFWContext();

    // Non-copyable, non-movable
    GLFWContext(const GLFWContext&) = delete;
    GLFWContext& operator=(const GLFWContext&) = delete;
    GLFWContext(GLFWContext&&) = delete;
    GLFWContext& operator=(GLFWContext&&) = delete;

    [[nodiscard]] GLFWwindow* get_window() const noexcept { return window_; }

private:
    GLFWwindow* window_ = nullptr;
};

class ImGuiContext final {
public:
    explicit ImGuiContext(GLFWwindow* window);
    ~ImGuiContext();

    // Non-copyable, non-movable
    ImGuiContext(const ImGuiContext&) = delete;
    ImGuiContext& operator=(const ImGuiContext&) = delete;
    ImGuiContext(ImGuiContext&&) = delete;
    ImGuiContext& operator=(ImGuiContext&&) = delete;

    void new_frame() const;
    void render() const;
};

} // namespace imgui_ros2

