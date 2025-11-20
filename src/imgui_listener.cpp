#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <memory>
#include <string>
#include <string_view>
#include <stdexcept>

namespace {
constexpr std::string_view kWindowTitle = "ROS2 ImGui Listener";
constexpr std::string_view kTopicName = "chatter";
constexpr std::string_view kGlslVersion = "#version 130";
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kQueueSize = 10;
} // namespace

class ImGuiListener final : public rclcpp::Node {
public:
    ImGuiListener() : Node("imgui_listener"), last_message_("Waiting for messages...") {
        using namespace std::placeholders;
        subscription_ = create_subscription<std_msgs::msg::String>(
            std::string(kTopicName), kQueueSize,
            [this](std_msgs::msg::String::SharedPtr msg) {
                last_message_ = std::move(msg->data);
                RCLCPP_INFO(get_logger(), "Received: '%s'", last_message_.c_str());
            });
    }

    [[nodiscard]] const std::string& get_last_message() const noexcept { 
        return last_message_; 
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string last_message_;
};

class GLFWContext final {
public:
    GLFWContext() {
        glfwSetErrorCallback([](int error, const char* description) {
            RCLCPP_ERROR(rclcpp::get_logger("glfw"), "GLFW Error %d: %s", error, description);
        });

        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        
        window_ = glfwCreateWindow(kWindowWidth, kWindowHeight, 
                                   std::string(kWindowTitle).c_str(), nullptr, nullptr);
        if (!window_) {
            glfwTerminate();
            throw std::runtime_error("Failed to create GLFW window");
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1); // Enable vsync
    }

    ~GLFWContext() {
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }

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
    explicit ImGuiContext(GLFWwindow* window) {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        
        ImGui::StyleColorsDark();
        
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(std::string(kGlslVersion).c_str());
    }

    ~ImGuiContext() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    // Non-copyable, non-movable
    ImGuiContext(const ImGuiContext&) = delete;
    ImGuiContext& operator=(const ImGuiContext&) = delete;
    ImGuiContext(ImGuiContext&&) = delete;
    ImGuiContext& operator=(ImGuiContext&&) = delete;

    void new_frame() const {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void render() const {
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
};

void render_ui(const ImGuiListener& node) {
    ImGui::Begin("ROS2 Listener");
    
    ImGui::Text("Topic: /%s", std::string(kTopicName).c_str());
    ImGui::Separator();
    
    ImGui::TextWrapped("Last Message:");
    ImGui::Separator();
    ImGui::TextWrapped("%s", node.get_last_message().c_str());
    ImGui::Separator();
    
    const auto& io = ImGui::GetIO();
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    
    ImGui::End();
}

int main(int argc, char** argv) {
    try {
        // Initialize ROS2
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ImGuiListener>();

        // Setup GLFW and ImGui with RAII
        GLFWContext glfw_context;
        ImGuiContext imgui_context(glfw_context.get_window());

        constexpr ImVec4 clear_color{0.45f, 0.55f, 0.60f, 1.00f};

        // Main loop
        while (!glfwWindowShouldClose(glfw_context.get_window()) && rclcpp::ok()) {
            rclcpp::spin_some(node);
            glfwPollEvents();

            imgui_context.new_frame();
            render_ui(*node);
            
            auto [width, height] = [window = glfw_context.get_window()]() {
                int w, h;
                glfwGetFramebufferSize(window, &w, &h);
                return std::pair{w, h};
            }();

            glViewport(0, 0, width, height);
            glClearColor(clear_color.x * clear_color.w, 
                        clear_color.y * clear_color.w,
                        clear_color.z * clear_color.w, 
                        clear_color.w);
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

