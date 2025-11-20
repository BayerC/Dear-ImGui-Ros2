# ROS2 + Dear ImGui Minimal Example

A minimal, modern C++17 example of integrating Dear ImGui with ROS2 to display messages from the `/chatter` topic in a GUI window.

## Features

- **Modern C++17**: RAII, `constexpr`, structured bindings, `[[nodiscard]]`, `noexcept`
- **Modern CMake**: Target-based approach, proper library creation, generator expressions
- **Clean architecture**: Separate concerns with dedicated RAII wrapper classes
- **Exception safety**: Proper resource management and error handling
- **ROS2 integration**: Subscribes to `/chatter` topic with real-time message display
- **Dear ImGui**: Automatically fetched via CMake FetchContent

## Dependencies

- ROS2 (Humble, Iron, or Jazzy recommended)
- All system dependencies are declared in `package.xml` and managed via `rosdep`

## Building

```bash
# Source your ROS2 installation
source /opt/ros/<your-ros2-distro>/setup.bash

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Navigate to workspace root
cd /home/noxem/workspace/Dear-ImGui-Ros2

# Install dependencies using rosdep (the ROS2 way)
rosdep install --from-paths . --ignore-src -y

# Build the package
colcon build

# Source the workspace
source install/setup.bash
```

## Running

### Terminal 1: Start the talker node
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
ros2 run demo_nodes_cpp talker
```

### Terminal 2: Start the ImGui listener
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
source install/setup.bash
ros2 run imgui_ros2_listener imgui_listener
```

A window will open displaying the messages received from the `/chatter` topic.

## How It Works

1. **RAII wrappers** manage GLFW and ImGui lifecycles automatically
2. The node subscribes to the `/chatter` topic (same as `demo_nodes_cpp talker` publishes to)
3. Received messages are stored and displayed in the Dear ImGui window in real-time
4. The main loop integrates ROS2 spinning with the GLFW/ImGui render loop
5. All resources are cleaned up automatically via RAII destructors

## Modern C++ Features Used

- **RAII classes**: `GLFWContext` and `ImGuiContext` for automatic resource management
- **`constexpr`**: Compile-time constants for configuration
- **Structured bindings**: Elegant tuple unpacking for framebuffer size
- **`[[nodiscard]]`**: Prevent ignoring important return values
- **`noexcept`**: Mark non-throwing functions for optimization
- **`final`**: Prevent unintended inheritance
- **Lambda captures**: Clean callback implementations
- **Exception handling**: Proper error propagation
- **Anonymous namespace**: Internal linkage for constants

## Modern CMake Features Used

- **Target-based design**: ImGui as a proper CMake library target
- **Generator expressions**: `$<BUILD_INTERFACE>` and `$<INSTALL_INTERFACE>`
- **`PRIVATE`/`PUBLIC`**: Proper dependency propagation
- **Interface includes**: Clean include directory management
- **Version specification**: Semantic versioning in project declaration
- **Shallow clone**: Faster ImGui fetching with `GIT_SHALLOW`

## Project Structure

```
.
├── CMakeLists.txt          # Build configuration with FetchContent for ImGui
├── package.xml             # ROS2 package manifest
├── src/
│   └── imgui_listener.cpp  # Main node implementation
└── README.md
```

## License

MIT License (same as Dear ImGui)

