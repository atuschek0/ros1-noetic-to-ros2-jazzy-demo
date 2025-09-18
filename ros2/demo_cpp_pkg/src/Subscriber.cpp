//
// Created by vboxuser on 9/17/25.
//

#include "demo_cpp_pkg/Subscriber.h"

using std::placeholders::_1;

Subscriber::Subscriber()
    : rclcpp::Node("subscriber")
{
    auto qos = rclcpp::SensorDataQoS();

    myTempSub = this->create_subscription<tempMsg>(
                "/dht22/temperature",
                qos,
                [this](tempMsg::SharedPtr msg){ tempCallback(std::move(msg)); } );

    myHumiSub = this->create_subscription<humiMsg>(
                "/dht22/humidity",
                qos,
                [this](humiMsg::SharedPtr msg) { humiCallback(std::move(msg)); }
                );
}

void Subscriber::tempCallback(tempMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[TEMP] %.2f C", msg->temperature);
}

void Subscriber::humiCallback(humiMsg::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "[HUMI] %.2f %%", msg->relative_humidity * 100.0);
}