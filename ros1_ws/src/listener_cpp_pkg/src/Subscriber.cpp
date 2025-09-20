//
// Created by vboxuser on 9/20/25.
//

#include "listener_cpp_pkg/Subscriber.h"

Subscriber::Subscriber(ros::NodeHandle& nodeHandler)
: nH(nodeHandler), frameId("dht22")
{
    ros::NodeHandle privateNodeHandle("~");
    privateNodeHandle.param<std::string>("frame_id", frameId, frameId);

    tempSub = nH.subscribe<sensor_msgs::Temperature>(
        "dht22/temperature", 10, &Subscriber::tempCallback, this);

    humiSub = nH.subscribe<sensor_msgs::RelativeHumidity>(
        "dht22/humidity", 10, &Subscriber::humiCallback, this);

    logInitConfig();
}

Subscriber::~Subscriber() {}

void Subscriber::begin() {
    ros::spin();
}

void Subscriber::tempCallback(const sensor_msgs::Temperature::ConstPtr& msg) {
    const std::string& fid = msg->header.frame_id.empty() ? frameId : msg->header.frame_id;
    ROS_INFO_STREAM(
        "[Temp] t=" << msg->header.stamp
        << " frame=" << fid
        << " tempC=" << msg->temperature
        << " var=" << msg->variance
    );
}

void Subscriber::humiCallback(const sensor_msgs::RelativeHumidity::ConstPtr& msg) {
    const std::string& fid = msg->header.frame_id.empty() ? frameId : msg->header.frame_id;
    ROS_INFO_STREAM(
        "[Hum ] t=" << msg->header.stamp
        << " frame=" << fid
        << " rh=" << (msg->relative_humidity * 100.0) << "%%"
        << " var=" << msg->variance
    );
}

void Subscriber::logInitConfig() const {
    ROS_INFO_STREAM(
        "DHT22 listener ready: frame_id=\"" << frameId
        << "\", topics=[dht22/temperature, dht22/humidity], queue_size=10"
    );
}
