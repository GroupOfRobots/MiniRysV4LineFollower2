#include "DataPublisher.h"

using namespace std::chrono_literals;

DataPublisher::DataPublisher(): Node("data_publisher"), count_(0){
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
}


void DataPublisher::timer_callback()
{
    auto control_message = std_msgs::msg::String();
    control_message.data = "Control! " + std::to_string(count_++);
    auto img_data_message = std_msgs::msg::String();
    img_data_message.data = "Img data! " + std::to_string(count_++);

    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  
    control_publisher_->publish(control_message);
    img_data_publisher_->publish(img_data_message);
}


std::map <std::string, double> DataPublisher::get_params()
{
    std::map < std::string, double > mapa;
    return mapa;
}
