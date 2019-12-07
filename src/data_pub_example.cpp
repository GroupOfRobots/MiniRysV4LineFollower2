#include "../lib/data_publisher/DataPublisher.h"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DataPublisher> data_publisher = std::make_shared<DataPublisher>();
    rclcpp::spin(data_publisher);
    rclcpp::shutdown();
    return 0;
}
