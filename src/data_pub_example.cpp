#include "../lib/data_publisher/DataPublisher.h"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DataPublisher> data_publisher = std::make_shared<DataPublisher>();
    
    std::map<std::string, double> parameters_map = data_publisher->get_params();
    std::map<std::string, double>::iterator it;
    for ( it = parameters_map.begin(); it != parameters_map.end(); it++ )
    {
        std::cout << it->first  // string (key)
        << ':'
        << it->second   // string's value 
        << std::endl ;
    }

    rclcpp::spin(data_publisher);
    rclcpp::shutdown();
    return 0;
}
