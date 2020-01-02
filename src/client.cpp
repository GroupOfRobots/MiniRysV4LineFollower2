#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;

class Decoder : public rclcpp::Node
{
  public:
    Decoder(): Node("decoder")
	{
    	publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image");
		subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    	"compressed_image", 10, std::bind(&Decoder::topic_callback, this, _1));
    }

	private:
		void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    	{
    		cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);
		sensor_msgs::msg::Image::SharedPtr ptr = std::make_shared<sensor_msgs::msg::Image>();
		
    		cv::imshow("view", image);
			cv::waitKey(10);
    	}

    	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
		rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Decoder>());
    rclcpp::shutdown();
    return 0;
  }