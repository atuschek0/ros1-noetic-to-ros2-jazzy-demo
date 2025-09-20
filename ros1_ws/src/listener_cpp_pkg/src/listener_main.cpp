//
// Created by vboxuser on 9/20/25.
//
#include <ros/ros.h>
#include "listener_cpp_pkg/Subscriber.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dht22_listener");
    ros::NodeHandle nh;

    Subscriber Dht22Listener(nh);
    Dht22Listener.begin();
    return 0;
}

