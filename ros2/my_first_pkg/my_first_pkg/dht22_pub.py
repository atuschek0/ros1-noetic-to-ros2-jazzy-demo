#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from std_msgs.msg import Header
from ._dht import readFakeSensor, readRealSensor, TEMP_VARIANCE, HUM_VARIANCE
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def readSensor(tick, useFakes, gpioPin, logger):
    if useFakes:
        #synthesized
        return readFakeSensor(tick)
    #region REAL_DHT_CALL
    try:
        return readRealSensor(gpioPin)
    except Exception as e:
        logger.warn(f"{e} -> using fake data")
        return readFakeSensor(tick)
    #endregion REAL_DHT_CALL

# constructing message objects by populating message fields
def populateMesseges(frameId, tempC, rhPct, stamp):
    hdr = Header(stamp=stamp, frame_id=frameId)
    tempMsg = Temperature(header=hdr, temperature=float(tempC), variance=TEMP_VARIANCE)
    humMsg  = RelativeHumidity(header=hdr, relative_humidity=float(rhPct) / 100.0, variance=HUM_VARIANCE)
    return tempMsg, humMsg

def publishMesseges(tempPub, humPub, tempMsg, humMsg, logger):
    # publishing the message on both topics
    tempPub.publish(tempMsg)
    humPub.publish(humMsg)
    # formatting the output with celsius temp and humid percentage:
    logger.info(f"[TEMP] {tempMsg.temperature:.2f} C  [HUM] {100.0 * humMsg.relative_humidity:.1f} %")


# node publishes Temperature/RelativeHHumidity
class Dht22Publisher(Node):
    def __init__(self) -> None:
        # Construction of nodes
        super().__init__("dht22_publisher")

        # declaration of ROS node parameters
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "dht22")
        self.declare_parameter("use_fakes", True)
        self.declare_parameter("gpio_pin", -1)

        # reading the ROS parameters into member variables
        self.publishRateHz = float(self.get_parameter("publish_rate_hz").value)
        self.frameId       = str(self.get_parameter("frame_id").value)
        self.useFakes      = bool(self.get_parameter("use_fakes").value)
        self.gpioPin       = int(self.get_parameter("gpio_pin").value)

        # Builds a QoS config object for sensor's data stream
        sensorQos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # QoS object passed to endpoints, not publishers themselves
        self.tempPub = self.create_publisher(Temperature, "dht22/temperature", sensorQos)
        self.humPub  = self.create_publisher(RelativeHumidity, "dht22/humidity",  sensorQos)

        # scheduling timer based on elapsed time
        self.tick = 0
        self.create_timer(1.0 / self.publishRateHz, self.publishOnce)

    # defines the reading, building, and publishing in a cycle
    def publishOnce(self) -> None:
        # reading sensor data (consolidated via helper to match ROS1 naming)
        tempC, rhPct = readSensor(self.tick, self.useFakes, self.gpioPin, logger=self.get_logger())
        self.tick += 1

        # constructing message objects by populating message fields
        now = self.get_clock().now().to_msg()
        tempMsg, humMsg = populateMesseges(self.frameId, tempC, rhPct, stamp=now)

        # publishing the message on both topics
        publishMesseges(self.tempPub, self.humPub, tempMsg, humMsg, logger=self.get_logger())

# entry
def main() -> None:
    rclpy.init()
    node = Dht22Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()