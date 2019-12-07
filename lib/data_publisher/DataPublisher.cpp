#include "DataPublisher.h"

using namespace std::chrono_literals;

DataPublisher::DataPublisher(): Node("data_publisher"), motor1_(0), motor2_(0), line_center_(0), img_center_(0){

    std::vector<std::string> param_vect{"K", "Td", "Ti", "publish_period", "control_period", "frame_rate", 
        "line_detect_method", "points_number", "resolution_factor"};
    parameters_ = param_vect;
    
    for (auto i : param_vect) this->declare_parameter(i);

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    
    while (!parameters_client_->wait_for_service(1s)) {
        
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    control_publisher_ = this->create_publisher<std_msgs::msg::String>("control", 10);
    img_data_publisher_ = this->create_publisher<std_msgs::msg::String>("line_data", 10);
    
    timer_ = this->create_wall_timer(500ms, std::bind(&DataPublisher::timer_callback, this));
}


void DataPublisher::timer_callback()
{
    auto control_message = std_msgs::msg::String();
    control_message.data = "Control! ";
    auto img_data_message = std_msgs::msg::String();
    img_data_message.data = "Img data! ";

    control_publisher_->publish(control_message);
    img_data_publisher_->publish(img_data_message);
}


std::map <std::string, double> DataPublisher::get_params(){
    std::map<std::string, double> parameters_map;
    for (auto & parameter : parameters_client_->get_parameters(parameters_)){
        parameters_map.insert(std::pair<std::string, double>(parameter.get_name(), parameter.as_double()));
    }
    return parameters_map;
}

void DataPublisher::set_data(int motor1, int motor2, int line_center, int img_center){
    motor1_ = motor1;
    motor2_ = motor2;
    line_center_ = line_center;
    img_center_ = img_center;
}