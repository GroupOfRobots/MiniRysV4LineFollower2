#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <boost/endian/conversion.hpp>
#include <sensor_msgs/image_encodings.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace enc = sensor_msgs::image_encodings;

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
		void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
		{
			cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);
			sensor_msgs::msg::Image ros_image;
			rclcpp::Time timestamp = this->get_clock()->now();
			ros_image.header.stamp = timestamp;
			ros_image.header.frame_id = "image";
		
			ros_image.height = image.rows;
			ros_image.width = image.cols;
			ros_image.encoding = enc::BGR8;
			ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
			ros_image.step = image.cols * image.elemSize();
			size_t size = ros_image.step * image.rows;
			ros_image.data.resize(size);

			if (image.isContinuous()) {
				memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
			} 
			
			else {
				uchar * ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
				uchar * cv_data_ptr = image.data;
				for (int i = 0; i < image.rows; ++i) {
					memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
					ros_data_ptr += ros_image.step;
					cv_data_ptr += image.step;
				}
			}
			publisher_->publish(ros_image);
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