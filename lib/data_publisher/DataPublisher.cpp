#include "DataPublisher.h"

using namespace std::chrono_literals;

DataPublisher::DataPublisher(): Node("data_publisher"){
    control_publisher_ = this->create_publisher<std_msgs::msg::String>("control", 10);
    img_data_publisher_ = this->create_publisher<std_msgs::msg::String>("line_data", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&DataPublisher::timer_callback, this));

    this->declare_parameter("K");
    this->declare_parameter("Td");
    this->declare_parameter("Ti");
    this->declare_parameter("publish_period");
    this->declare_parameter("control_period");
    this->declare_parameter("frame_rate");
    this->declare_parameter("line_detect_method");
    this->declare_parameter("points_number");
    this->declare_parameter("resolution_factor");

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    
    while (!parameters_client_->wait_for_service(1s)) {
        
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
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
    std::map<std::string, double> tmp_map;
    std::vector<std::string> tmp_vec;
    tmp_vec.push_back("K");
    tmp_vec.push_back("Ti");
    
    for (auto & parameter : parameters_client_->get_parameters(tmp_vec)){
        tmp_map.insert(std::pair<std::string, double>(parameter.get_name(), parameter.as_double()));
    }
    return tmp_map;
}
