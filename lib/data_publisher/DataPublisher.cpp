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
        "control_period"};

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

    double publish_period = parameters_client_->get_parameter("publish_period", 100.0);
    data_publisher_ = this->create_publisher<robot_line_follower::msg::ProcessData>("process_data");
    image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_image");
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(round(publish_period))), std::bind(&DataPublisher::timer_callback, this));
}


void DataPublisher::timer_callback()
{
    std::map<std::string, int> process_data = controller_->getProcessData();
    auto message = robot_line_follower::msg::ProcessData();
    message.left_motor_vel = process_data["motor_left"];
    message.right_motor_vel = process_data["motor_right"];
    message.line_center_x = process_data["line_center"];
    message.img_center_x = process_data["img_center"];
    rclcpp::Time timestamp = this->get_clock()->now();
    message.header.stamp = timestamp;
    message.header.frame_id = process_data_id_;
    data_publisher_->publish(message);

    cv::Mat frame = detector_->getFrame();
    if(!frame.empty() && frame.cols != -1 && frame.rows != -1){
        std_msgs::msg::Header header;
        rclcpp::Time timestamp = this->get_clock()->now();
        header.stamp = timestamp;
        header.frame_id = frame_id_;
        img_bridge_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        sensor_msgs::msg::CompressedImage::SharedPtr image_msg =img_bridge_.toCompressedImageMsg(cv_bridge::JPG);
        image_publisher_->publish(image_msg);
    }
}


std::map <std::string, double> DataPublisher::getParams(){
    std::map<std::string, double> parameters_map;
    for (auto & parameter : parameters_client_->get_parameters(parameters_)){
        parameters_map.insert(std::pair<std::string, double>(parameter.get_name(), parameter.as_double()));
    }
    return parameters_map;
}

void DataPublisher::setController(std::shared_ptr<Controller> controller){
    controller_ = controller;
}

void DataPublisher::setDetector(std::shared_ptr<Detector> detector){
    detector_ = detector;
}