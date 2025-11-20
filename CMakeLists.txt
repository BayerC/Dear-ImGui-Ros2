cmake_minimum_required(VERSION 3.14)
project(imgui_ros2_listener VERSION 0.1.0 LANGUAGES CXX)

# Export compile commands for language servers (clangd, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find graphics dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

# Fetch Dear ImGui
include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.5
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)

# Create ImGui library as a proper target
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
  $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
  $<BUILD_INTERFACE:${imgui_SOURCE_DIR}/backends>
)

target_link_libraries(imgui PUBLIC
  OpenGL::GL
  glfw
)

target_compile_definitions(imgui PUBLIC
  IMGUI_IMPL_OPENGL_LOADER_GLAD=0
)

# Main executable
add_executable(imgui_listener
  src/main.cpp
  src/imgui_context.cpp
  src/ros_listener.cpp
  src/ui_renderer.cpp
)

target_include_directories(imgui_listener PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imgui_listener
  rclcpp
  std_msgs
)

target_link_libraries(imgui_listener
  imgui
)

# Installation
install(
  TARGETS imgui_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

