cmake_minimum_required(VERSION 3.8)
project(imgui_ros2_listener)

# Set OpenGL preference to GLVND
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Fetch Dear ImGui
include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.5
)
FetchContent_MakeAvailable(imgui)

# Dear ImGui sources
set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Main executable
add_executable(imgui_listener
  src/imgui_listener.cpp
  ${IMGUI_SOURCES}
)

target_include_directories(imgui_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
)

target_link_libraries(imgui_listener
  ${OPENGL_LIBRARIES}
  glfw
)

ament_target_dependencies(imgui_listener
  rclcpp
  std_msgs
)

install(TARGETS imgui_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

