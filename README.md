# ROS2 bytes data transmit

## Introduction

Transmit bytes data between ROS 2 nodes using service/client.

## Getting Started

### 1. Build Custom srv package
In your ROS2 colcon workspace (e.g. colcon_ws) `src` directory, create the custom service type package (in this case, named `tutorial_interfaces`, which is refer from [tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html))

```bash
ros2 pkg create --build-type ament_cmake tutorial_interfaces

mkdir srv

cd srv
```
Create file `Ros2BytesTransmit` and Add 

```
byte[] req
---
bool resp
```

Add 
```
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Ros2BytesTransmit.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on
)
```
in `CMakeLists.txt` of `tutorial_interfaces` package(according to [link](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) )


Add 
```
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
in `package.xml`

### 2. Run server and client
> Make sure two packages are in same workspace's src directory
Clone repository in same workspace's src directory
```bash
cd ~/<your_ws>/src
git clone https://github.com/khinggan/ros2_bytes_transmit.git
```
Build 
```
cd ~/<your_ws>/
colcon build --packages-select tutorial_interfaces --symlink-install
colcon build --packages-select ros2_bytes_transmit --symlink-install
```

> !TODO, configure `absolute_path` with your project path; need to be modified to relative path later. (line 8 in `src/ros2_bytes_transmit/client.py`)

Run server and client (`cd` to `your_workspace_directory`)

```bash
# Terminal 1
source install/setup.bash
ros2 run ros2_bytes_transmit server
```

```bash
# Terminal 2
source install/setup.bash
ros2 run ros2_bytes_transmit client
```
