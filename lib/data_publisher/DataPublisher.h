#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "robot_line_follower/msg/process_data.hpp"

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
  public:
    DataPublisher();
    std::map <std::string, double> get_params();
    void set_data(int motor1, int motor2, int line_center, int img_center);

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr img_data_publisher_;
    std::vector<std::string> parameters_;
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
    int motor1_; 
    int motor2_;
    int line_center_; 
    int img_center_;
  };