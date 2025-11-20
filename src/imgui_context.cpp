#include "imgui_ros2_listener/imgui_context.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

namespace imgui_ros2 {

namespace {
constexpr std::string_view kGlslVersion = "#version 130";
} // namespace

GLFWContext::GLFWContext(int width, int height, std::string_view title) {
    glfwSetErrorCallback([](int error, const char* description) {
        RCLCPP_ERROR(rclcpp::get_logger("glfw"), "GLFW Error %d: %s", error, description);
    });

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    
    window_ = glfwCreateWindow(width, height, std::string(title).c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync
}

GLFWContext::~GLFWContext() {
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

ImGuiContext::ImGuiContext(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(std::string(kGlslVersion).c_str());
}

ImGuiContext::~ImGuiContext() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void ImGuiContext::new_frame() const {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiContext::render() const {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

} // namespace imgui_ros2

