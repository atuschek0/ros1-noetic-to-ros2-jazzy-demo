//
// Created by vboxuser on 9/17/25.
//

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H
#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

/// listens to dht22 temperature and humidity topics
/// also logs vlaues
class Subscriber : public rclcpp::Node
{

public:
    Subscriber(); ///creates subscription

private:
    using tempMsg = sensor_msgs::msg::Temperature;
    using humiMsg = sensor_msgs::msg::RelativeHumidity;

    ///callback lambda funct for frowarding
    void tempCallback(const tempMsg::SharedPtr msg) const;
    void humiCallback(const humiMsg::SharedPtr msg) const;

    rclcpp::Subscription<tempMsg>::SharedPtr myTempSub;
    rclcpp::Subscription<humiMsg>::SharedPtr myHumiSub;
};

#endif //SUBSCRIBER_H
