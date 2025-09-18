cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#talker
find_package(std_msgs REQUIRED)
#listener
find_package(sensor_msgs REQUIRED)

#talker
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

#listener (the one with main) and Subscriber
add_executable(listener_dht22
        src/listener_dht22.cpp
        src/Subscriber.cpp)
ament_target_dependencies(listener_dht22 rclcpp sensor_msgs)
target_include_directories(listener_dht22 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(TARGETS listener_dht22 DESTINATION lib/${PROJECT_NAME})

# headers & launch
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
