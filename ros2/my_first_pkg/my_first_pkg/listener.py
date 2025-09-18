#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from rclpy.qos import qos_profile_sensor_data


TEMP_TOPIC  = "/dht22/temperature"
HUM_TOPIC   = "/dht22/humidity"
QUEUE_SIZE  = 10


class PyListener(Node):
    def __init__(self) -> None:
        super().__init__("py_listener")  # ROS node names stay snake_case
        self.tempSub = self.create_subscription(
            Temperature, TEMP_TOPIC, self.onTemp, qos_profile_sensor_data
        )
        self.humSub = self.create_subscription(
            RelativeHumidity, HUM_TOPIC, self.onHum, qos_profile_sensor_data
        )

    def onTemp(self, msg: Temperature) -> None:
        self.get_logger().info(
            f"[TEMP] {msg.temperature:.2f} C (frame={msg.header.frame_id})"
        )

    def onHum(self, msg: RelativeHumidity) -> None:
        pct = msg.relative_humidity * 100.0  # RelativeHumidity betwee 0 1
        self.get_logger().info(
            f"[HUM ] {pct:.1f} % (frame={msg.header.frame_id})"
        )


def main() -> None:
    rclpy.init()
    node = PyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
