#include "../lib/data_publisher/DataPublisher.h"
#include "../lib/detector/Detector.h"
#include "../lib/controller/Controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<DataPublisher> data_publisher = std::make_shared<DataPublisher>();
    std::map<std::string, double> params = data_publisher->get_params();
    std::map<std::string, double>::iterator it;
    for ( it = params.begin(); it != params.end(); it++ )
    {
        std::cout << it->first  // string (key)
        << ':'
        << it->second   // string's value 
        << std::endl ;
    }
    
    std::shared_ptr<Detector> detector = std::make_shared<Detector>(params["up_roi_boundary"], 
        params["down_roi_boundary"], params["resolution_factor"], params["detection_threshold"], 
        static_cast<int>(round(params["detection_period"]*1000)));//(1.0/6.0, 5.0/6.0, 0.5);
    
    std::shared_ptr<Controller> controller = std::make_shared<Controller>(params["K"], params["Ti"], params["Td"], 
        params["const_vel"], params["vel_down_lim"], params["vel_up_lim"], detector, 
        static_cast<int>(round(params["control_period"]*1000)));

    detector->run();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));//wait for detector 
    controller->run();

    data_publisher->set_controller(controller);
    data_publisher->set_detector(detector);

    rclcpp::spin(data_publisher);
    rclcpp::shutdown();
    return 0;
}
