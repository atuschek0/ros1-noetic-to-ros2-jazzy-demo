#!/usr/bin/env python3
"""DHT22 publisher (ROS 1)"""
"""Note: THis code is compatible with a real RPi5 AND DHT22 sensor"""
"""Fake sensor data values are synthesized by default"""

import rospy
from sensor_msgs.msg import Temperature, RelativeHumidity
from std_msgs.msg import Header
from sensor_pkg._dht import readFakeSensor, readRealSensor, TEMP_VARIANCE, HUM_VARIANCE

def readSensor(tick, useFakes, gpioPin):
    if useFakes:
        return readFakeSensor(tick)
    else:
        #region REAL_DHT_CALL
        try:
            return readRealSensor(gpioPin)
        except Exception as e:
            rospy.logwarn("%s -> using fake data", str(e))
            return readFakeSensor(tick)
        #endregion REAL_DHT_CALL

# constructing message objects by populating message fields
def populateMessages(frameId, tempC, rhPct):
    hdr = Header(stamp=rospy.Time.now(), frame_id=frameId)
    tempMsg = Temperature(header=hdr, temperature=float(tempC), variance=TEMP_VARIANCE)
    humMsg  = RelativeHumidity(header=hdr, relative_humidity=float(rhPct) / 100.0, variance=HUM_VARIANCE)
    return tempMsg, humMsg

    # publishing the message on both topics
def publishMesseges(tempPub, humPub, tempMsg, humMsg):
    tempPub.publish(tempMsg)
    humPub.publish(humMsg)
    rospy.loginfo("[TEMP] %.2f C  [HUM] %.1f %%",
                  tempMsg.temperature, 100.0 * humMsg.relative_humidity)


def main():
    rospy.init_node("dht22_publisher")

    # reading ros parameters into variables
    publishRateHz = float(rospy.get_param("~publish_rate_hz", 5.0))
    frameId       = str(rospy.get_param("~frame_id", "dht22"))
    useFakes      = bool(rospy.get_param("~use_fakes", True))
    gpioPin       = int(rospy.get_param("~gpio_pin", -1))

    # creating publisher objects
    tempPub = rospy.Publisher("dht22/temperature", Temperature, queue_size=10)
    humPub  = rospy.Publisher("dht22/humidity",    RelativeHumidity, queue_size=10)

    rate = rospy.Rate(publishRateHz)
    tick = 0
    rospy.loginfo("DHT22: %.1fHz fakes=%s frame=%s pin=%d",
                  publishRateHz, useFakes, frameId, gpioPin)

    while not rospy.is_shutdown():
        tempC, rhPct = readSensor(tick, useFakes, gpioPin)
        tick += 1
        tempMsg, humMsg = populateMessages(frameId, tempC, rhPct)
        publishMesseges(tempPub, humPub, tempMsg, humMsg)
        rate.sleep()


if __name__ == "__main__":
    main()