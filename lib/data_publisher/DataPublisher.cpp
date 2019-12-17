#include "DataPublisher.h"

using namespace std::chrono_literals;

DataPublisher::DataPublisher(): Node("data_publisher") {

    std::vector<std::string> param_vect{
        "K",
        "Td", 
        "Ti",
        "const_vel",
        "vel_up_lim",
        "vel_down_lim",
        "line_detect_method", 
        "points_number", 
        "up_roi_boundary", 
        "down_roi_boundary", 
        "resolution_factor", 
        "detection_threshold", 
        "publish_period", 
        "detection_period", 
        "control_period", 
        "frame_rate"};

    parameters_ = param_vect;
    for (auto i : parameters_) this->declare_parameter(i);
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);

    while (!parameters_client_->wait_for_service(1s)) {
        
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    data_publisher_ = this->create_publisher<robot_line_follower::msg::ProcessData>("process_data");
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_data");
    
    timer_ = this->create_wall_timer(60ms, std::bind(&DataPublisher::timer_callback, this));
}


void DataPublisher::timer_callback()
{
    std::map<std::string, int> process_data = controller_->getProcessData();
    auto message = robot_line_follower::msg::ProcessData();
    message.motor_left = process_data["motor_left"];
    message.motor_right = process_data["motor_right"];
    message.line_center =process_data["line_center"];
    message.img_center = process_data["img_center"];
    data_publisher_->publish(message);

    cv::Mat frame = detector_->getFrame();
    if(!frame.empty() && frame.cols != -1 && frame.rows != -1){
        std::shared_ptr<sensor_msgs::msg::Image> image_msg;
        std_msgs::msg::Header header;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        image_msg = img_bridge.toImageMsg(); // from cv_bridge to sensor_msgs::msg::Image
        image_publisher_->publish(image_msg);
    }
}


std::map <std::string, double> DataPublisher::get_params(){
    std::map<std::string, double> parameters_map;
    for (auto & parameter : parameters_client_->get_parameters(parameters_)){
        parameters_map.insert(std::pair<std::string, double>(parameter.get_name(), parameter.as_double()));
    }
    return parameters_map;
}

void DataPublisher::set_controller(std::shared_ptr<Controller> controller){
    controller_ = controller;
}

void DataPublisher::set_detector(std::shared_ptr<Detector> detector){
    detector_ = detector;
}