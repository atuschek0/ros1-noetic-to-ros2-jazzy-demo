//
// Created by vboxuser on 9/12/25.
//

#include <memory>
#include "demo_cpp_pkg/Subscriber.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}
