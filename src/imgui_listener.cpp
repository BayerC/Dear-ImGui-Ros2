#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>

class ImGuiListener : public rclcpp::Node {
public:
    ImGuiListener() : Node("imgui_listener"), last_message_("Waiting for messages...") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                last_message_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
            });
    }

    std::string get_last_message() const { return last_message_; }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string last_message_;
};

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImGuiListener>();

    // Setup GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // Create window
    GLFWwindow* window = glfwCreateWindow(800, 600, "ROS2 ImGui Listener", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
        // Poll ROS2
        rclcpp::spin_some(node);

        // Poll and handle events
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create window with message display
        {
            ImGui::Begin("ROS2 Listener");
            ImGui::Text("Topic: /chatter");
            ImGui::Separator();
            ImGui::TextWrapped("Last Message:");
            ImGui::Separator();
            ImGui::TextWrapped("%s", node->get_last_message().c_str());
            ImGui::Separator();
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                       1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, 
                     clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();
    return 0;
}

