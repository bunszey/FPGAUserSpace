#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <vector>

#include <chrono>

#include "xexample.h"

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
			timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImageSubscriber::timer_callback, this));

			char instance_name[ 20 ];
			sprintf(instance_name, "example");

			int status = XExample_Initialize(&ip_inst, instance_name);
			if (status != XST_SUCCESS) {
				printf("Error: Could not initialize the IP core.\n");
				return XST_FAILURE;
			}


		}

		~ImageSubscriber(){
			// Cleanup
			XExample_DisableAutoRestart(&ip_inst);
			XExample_Release(&ip_inst);
			cleanup_platform();
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		XExample ip_inst;

		cv::Mat gray_;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat img = cv_ptr->image;

			//cv::cvtColor(img, gray_, cv::COLOR_YUV2GRAY_YUY2 );
			std::vector<cv::Mat> channels(2);
			cv::split(img, channels);

			gray_ = channels[0];

			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");
		}

		void timer_callback(){
			std::vector<unsigned char> vec_gray(gray_.begin<unsigned char>(), gray_.end<unsigned char>());
			
			XExample_Write_in_r_Bytes(&ip_inst, 0, vec_gray, DATA_SIZE);

			// Call the IP core function
			XExample_Start(&ip_inst);

			// Wait for the IP core to finish
			while (!XExample_IsDone(&ip_inst));

			std::vector<unsigned char> out;
			XExample_Read_out_r_Bytes(&ip_inst, 0, out, DATA_SIZE);
			cv::Mat res = cv::Mat(out).reshape(0, gray_.rows);

			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", res).toImageMsg();
			camera_publisher_->publish(*msg.get());
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
