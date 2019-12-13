#include "../lib/data_publisher/DataPublisher.h"
#include "../lib/detector/Detector.h"
#include "../lib/controller/Controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DataPublisher> data_publisher = std::make_shared<DataPublisher>();
    std::shared_ptr<Detector> detector = std::make_shared<Detector>(1.0/6.0, 5.0/6.0, 0.5);
    std::shared_ptr<Controller> controller = std::make_shared<Controller>(0.3, 50, 0.05, 40, -70, 60, detector);
    detector->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));//wait for detector 
    controller->run();

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
