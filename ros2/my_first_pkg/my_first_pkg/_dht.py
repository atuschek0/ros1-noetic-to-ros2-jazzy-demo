# _dht.py  Note: this is literally identical in ROS1 to my ROS1 version
import math

TEMP_VARIANCE = 0.0
HUM_VARIANCE  = 0.0

#sensor reading

# function to synthetically create sensor data
def readFakeSensor(tick: int):
    tempC = 22.0 + 3.0 * math.sin(0.1 * tick)
    rhPct = 45.0 + 10.0 * math.cos(0.05 * tick)
    return float(tempC), float(rhPct)

# fucntion to read sensor data from real DHT22 hardware
#region REALDHT
def readRealSensor(gpioPin: int):
    import Adafruit_DHT
    SENSOR = Adafruit_DHT.DHT22
    rhPct, tempC = Adafruit_DHT.read_retry(SENSOR, int(gpioPin), retries=1, delay_seconds=0.1)
    if rhPct is None or tempC is None:
        raise RuntimeError("DHT22 read failed")
    return float(tempC), float(rhPct)
#endregion REALDHT