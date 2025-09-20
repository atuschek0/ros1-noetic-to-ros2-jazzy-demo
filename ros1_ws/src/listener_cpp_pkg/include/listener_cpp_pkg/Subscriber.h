//
// Created by vboxuser on 9/20/25.
//
#ifndef LISTENER_CPP_PKG_SUBSCRIBER_H
#define LISTENER_CPP_PKG_SUBSCRIBER_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

class Subscriber {
public:
    explicit Subscriber(ros::NodeHandle& nodeHandler);
    ~Subscriber();

    void begin();

private:
    void tempCallback(const sensor_msgs::Temperature::ConstPtr& msg);
    void humiCallback(const sensor_msgs::RelativeHumidity::ConstPtr& msg);

    void logInitConfig() const;

    ros::NodeHandle nH;
    ros::Subscriber tempSub;
    ros::Subscriber humiSub;
    std::string frameId;
};

#endif // LISTENER_CPP_PKG_SUBSCRIBER_H
