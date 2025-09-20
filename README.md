# ROS1 Noetic → ROS2 Jazzy: DHT22 Publisher + Listener (Python + C++)

A small, runnable demo showing the **same behavior** in both ROS1 (Noetic) and ROS2 (Jazzy):

- **ROS1**: Python `rospy` DHT22 publisher → C++ `roscpp` listener  
- **ROS2**: Python `rclpy` DHT22 publisher → C++ `rclcpp` listener (via an explicit QoS definition)

By default, my publishers generate **synthesized** temperature/humidity data; a Raspberry Pi setup for a real DHT22 sensor is included.

## Repo layout
```
./
├── docs/
│   ├── ROS1_Launcher.webm
│   ├── ROS1_LiveViewer.webm
│   ├── ROS2_Launcher.webm
│   └── ROS2_LiveViewer.webm
├── .gitignore
├── media/
│   ├── ros1_demo.gif
│   ├── ros1_live.gif
│   ├── ros2_demo.gif
│   └── ros2_live.gif
├── ros1/
│   └── sensor_pkg/
│       ├── CMakeLists.txt
│       ├── launch/
│       │   └── demo.launch
│       ├── package.xml
│       ├── scripts/
│       │   └── dht22_pub.py*
│       ├── setup.py
│       └── src/
│           └── sensor_pkg/
│               └── _dht.py
└── ros2/
    ├── demo_cpp_pkg/
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── demo_cpp_pkg/
    │   │       └── Subscriber.h
    │   ├── launch/
    │   │   └── demo.launch.py
    │   ├── package.xml
    │   └── src/
    │       ├── listener_dht22.cpp
    │       ├── Subscriber.cpp
    │       └── talker.cpp
    └── my_first_pkg/
        ├── launch/
        │   └── demo.launch.py
        ├── my_first_pkg/
        │   ├── dht22_pub.py
        │   ├── _dht.py
        │   ├── __init__.py
        │   └── listener.py*
        ├── package.xml
        ├── setup.cfg
        └── setup.py

18 directories, 30 files

```

## Quick start

### ROS1 (Noetic)

```bash
# workspace
cd ~/ros1_ws
catkin_make
source devel/setup.bash

# run publisher
roslaunch sensor_pkg demo.launch

# verify (new terminal)
source ~/ros1_ws/devel/setup.bash
rostopic echo /dht22/temperature -n1
rostopic echo /dht22/humidity -n1
```

### ROS2 (Jazzy)
```bash
# workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

# run both nodes via ROS2 launch
ros2 launch my_first_pkg demo.launch.py

# verify (same shell)
ros2 topic echo /dht22/temperature --once
ros2 topic echo /dht22/humidity --once
```

## This project demonstrates

- **API migration**: `rospy/roscpp` -> `clpy/rclcpp`

- **Launch migration** :`roslaunch XML` -> `Python launch/launch_ros`

- **Parameters**: `YAML/param server` -> `ROS2 declare_parameter()/get_parameter()`

-  **QoS:** ROS1 implicit queue size -> ROS2 explicit `SensorDataQoS()`

- **Packaging**: catkin vs ament/colcon (setup.py entry points for Python)

## Raspberry Pi 

My publishers support real DHT22 on a Pi. Please enable the `gpio_pin `parameter and install the sensor library (notes in `MIGRATION_NOTES.md`). Synthesized sensor data is set to the default to keep my demo portable.