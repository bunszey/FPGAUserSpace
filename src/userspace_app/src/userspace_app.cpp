#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

class ImageSubscriber : public rclcpp::Node
{
	public:
		ImageSubscriber() : Node("image_subscriber") {
			RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

			RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1)
			);

			camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
					"/image_out",
					10
			);
			timer_ = this->create_wall_timer(300ms, std::bind(&MinimalPublisher::timer_callback, this));


		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;

		cv::Mat gray_;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat img = cv_ptr->image;

			cv::cvtColor(img, gray_ cv::COLOR_YUV2GRAY_YUY2 );
	
			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");
		}

		void timer_callback(){
			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", gray_).toImageMsg();
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;->publish(msg);
        	std::cout << "Published!" << std::endl;
		}

};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
