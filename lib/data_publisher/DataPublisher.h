#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
  public:
    DataPublisher();
    std::map <std::string, double> get_params();

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr img_data_publisher_;
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  };