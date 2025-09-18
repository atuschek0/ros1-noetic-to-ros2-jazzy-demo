from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): # must stay snake_case for ROS
    #CLI args
    useFakes = LaunchConfiguration("use_fakes")
    frameId  = LaunchConfiguration("frame_id")
    rateHz   = LaunchConfiguration("publish_rate_hz")
    gpioPin  = LaunchConfiguration("gpio_pin")

    return LaunchDescription([
        # Wiring
        DeclareLaunchArgument("use_fakes", default_value="true"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("frame_id", default_value="dht22"),
        DeclareLaunchArgument("gpio_pin", default_value="-1"),

        Node(
            package="my_first_pkg",
            executable="dht22_pub",
            name="dht22_pub",
            output="screen",
            parameters=[{
                "use_fakes": useFakes,
                "publish_rate_hz": rateHz,
                "frame_id":  frameId,
                "gpio_pin":  gpioPin,
            }],
        ),
        Node(
            package="demo_cpp_pkg",
            executable="listener_dht22",
            name="listener_dht22",
            output="screen",
        ),
    ])
