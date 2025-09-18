from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_cpp_pkg', executable='listener_dht22', name='cpp_listener'),
    ])
