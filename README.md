# ROS2 + Dear ImGui Minimal Example

A minimal example of integrating Dear ImGui with ROS2 to display messages from the `/chatter` topic in a GUI window.

## Features

- ROS2 node subscribing to `/chatter` topic
- Dear ImGui window displaying received messages
- Minimal C++17 implementation
- Automatic Dear ImGui dependency fetching via CMake

## Dependencies

### System Dependencies
```bash
sudo apt update
sudo apt install -y \
  libglfw3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev
```

### ROS2 Dependencies
- ROS2 (Humble, Iron, or Jazzy recommended)
- `rclcpp`
- `std_msgs`

## Building

```bash
# Source your ROS2 installation
source /opt/ros/<your-ros2-distro>/setup.bash

# Build the package
cd /home/noxem/workspace/Dear-ImGui-Ros2
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

1. The node subscribes to the `/chatter` topic (same as `demo_nodes_cpp talker` publishes to)
2. Received messages are stored and displayed in the Dear ImGui window
3. The ImGui window updates in real-time as new messages arrive
4. The node integrates ROS2 spinning with the GLFW/ImGui render loop

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

