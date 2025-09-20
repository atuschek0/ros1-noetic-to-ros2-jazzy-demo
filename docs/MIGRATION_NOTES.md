> Note: All steps were validated on Kubuntu 24.04. ROS 1 (Noetic) was run in Docker; ROS 2 (Jazzy) ran natively.

## ROS1 (Noetic) to ROS2 (Jazzy) mapping table

| Area | ROS 1 (Noetic) | ROS 2 (Jazzy) | What I did |
|---|---|---|---|
| **Workspace build** | `catkin_make` | `colcon build` | Separate `~/ros1_ws` and `~/ros2_ws`. |
| **Env setup** | `source devel/setup.bash` | `source install/setup.bash` | Shown in README run blocks. |
| **Build system** | **catkin** + `package.xml` | **ament** + `package.xml` | `sensor_pkg` (ROS1), `my_first_pkg` & `demo_cpp_pkg` (ROS2). |
| **Manifest** | `build_depend`, `run_depend` (format `"<package format=\"2\">"`) | `build_depend`, `exec_depend` (format `"<package format=\"3\">"`) | Updated to `exec_depend` in ROS2. |
| **Python publisher API** | `rospy.init_node`, `Publisher`, `Rate`, `get_param` | `rclpy.init`, `Node.create_publisher`, `create_timer`, `declare/get_parameter` | Declared parameters; explicit QoS. |
| **C++ subscriber/listener API** | `ros::init`, `NodeHandle`, `subscribe`, callback `const Msg&` | `rclcpp::init`, class `Node`, `create_subscription`, callback `SharedPtr` | Class `Subscriber : public rclcpp::Node` with QoS. |
| **QoS** | *Non-existent in ROS 1* | **QoS profiles** (e.g., `SensorDataQoS()` ) | Use `SensorDataQoS()` for sensor streams. |
| **Launch files** | XML `.launch` with `<node/>`, `<param/>` | Python `.launch.py` via `launch` / `launch_ros` | `demo.launch` (ROS1), `demo.launch.py` (ROS2). |
| **Install/run (Python)** | Mark script `+x` and call directly | Use `entry_points` (console_scripts) | `my_first_pkg/setup.py` exposes `dht22_pub`. |
| **Headers (C++)** | `#include <std_msgs/String.h>` | `#include "std_msgs/msg/string.hpp"` | Updated includes + ament dependencies. |
| **Time & rate** | `ros::Rate`, `ros::Time::now()` | timers; `this->now()` | Timer used in Python publisher. |
| **Run commands** | `roslaunch sensor_pkg demo.launch` | `ros2 launch my_first_pkg demo.launch.py` | Both blocks in README. |

## Package and Build Differences

| Area                   | ROS 1 (catkin)                                                        | ROS 2 (ament)                                                                                                                  |
| ---------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| **Build tool**         | `catkin_make`                                                         | `colcon build`                                                                                                                 |
| **C++ deps**           | `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs …)`          | `find_package(ament_cmake REQUIRED)` + `find_package(rclcpp REQUIRED)` + `find_package(sensor_msgs REQUIRED)`                  |
| Targets | add exe, **link catkin libs** | add exe, **declare ament deps**|
| **Install**            | Usually none for scripts                                              | `install(TARGETS … DESTINATION lib/${PROJECT_NAME})` and `install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)` |
| **Python entry point** | Mark script executable (`chmod +x`)                                   | `setup.py` with `entry_points={"console_scripts": [...]}`                                                                      |
