#include "imgui_ros2_listener/imgui_context.hpp"
#include "imgui_ros2_listener/ros_listener.hpp"
#include "imgui_ros2_listener/ui_renderer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <imgui.h>
#include <GLFW/glfw3.h>

#include <memory>
#include <string_view>

namespace {
constexpr std::string_view kWindowTitle = "ROS2 ImGui Listener";
constexpr std::string_view kTopicName = "chatter";
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr ImVec4 kClearColor{0.45f, 0.55f, 0.60f, 1.00f};
} // namespace

int main(int argc, char** argv) {
    try {
        // Initialize ROS2
        rclcpp::init(argc, argv);
        auto node = std::make_shared<imgui_ros2::RosListener>();

        // Setup GLFW and ImGui with RAII
        imgui_ros2::GLFWContext glfw_context(kWindowWidth, kWindowHeight, kWindowTitle);
        imgui_ros2::ImGuiContext imgui_context(glfw_context.get_window());

        // Main loop
        while (!glfwWindowShouldClose(glfw_context.get_window()) && rclcpp::ok()) {
            rclcpp::spin_some(node);
            glfwPollEvents();

            imgui_context.new_frame();
            imgui_ros2::render_ui(*node, kTopicName);
            
            auto [width, height] = [window = glfw_context.get_window()]() {
                int w, h;
                glfwGetFramebufferSize(window, &w, &h);
                return std::pair{w, h};
            }();

            glViewport(0, 0, width, height);
            glClearColor(kClearColor.x * kClearColor.w, 
                        kClearColor.y * kClearColor.w,
                        kClearColor.z * kClearColor.w, 
                        kClearColor.w);
            glClear(GL_COLOR_BUFFER_BIT);

            imgui_context.render();
            glfwSwapBuffers(glfw_context.get_window());
        }

        rclcpp::shutdown();
        return 0;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}

