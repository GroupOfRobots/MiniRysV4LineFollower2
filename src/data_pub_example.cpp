#include "../lib/data_publisher/DataPublisher.h"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    DataPublisher data_publisher;
    rclcpp::spin(std::make_shared<DataPublisher>());
    rclcpp::shutdown();
    return 0;
}
