#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <vector>

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  //Header file for sleep(). man 3 sleep for details.
#include <pthread.h>
#include <iostream>
#include <string>
//#include "libs/platform.h"
//#include "libs/xil_printf.h"
#include "libs/xinverter.h"
#include "BRAM-uio-driver/src/bram_uio.h"

#define BRAMSIZE 125000
#define XST_FAILURE 1L

BRAM BRAM1(0,BRAMSIZE);
BRAM BRAM2(1,BRAMSIZE);

#define DATA_SIZE 25440

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

			int status = XInverter_Initialize(&ip_inst, instance_name);
			if (status != XST_SUCCESS) {
				RCLCPP_INFO(this->get_logger(), "Error: Could not initialize the IP core.");
			}


		}

		~ImageSubscriber(){
			// Cleanup
			XInverter_DisableAutoRestart(&ip_inst);
			XInverter_Release(&ip_inst);
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		XInverter ip_inst;

		cv::Mat gray_;
		bool subscribing = false;
        int noOfPixels = 0;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat img = cv_ptr->image;

			//cv::cvtColor(img, gray_, cv::COLOR_YUV2GRAY_YUY2 );
			std::vector<cv::Mat> channels(2);
			cv::split(img, channels);

			gray_ = channels[0];

            noOfPixels = gray_.rows * gray_.cols;
            if (noOfPixels > DATA_SIZE){
                RCLCPP_INFO(this->get_logger(), "WONG SIZE!");
                subscribing = false;
                return;
            }

            for (int i = 0; i < noOfPixels; i++) {
                uint32_t data_write = *(reinterpret_cast<uint32_t*>(&gray_.data[i*4]));
                BRAM1[i] = data_write;
		    }
			
			RCLCPP_INFO(this->get_logger(), "Successfully loaded image");
			subscribing = true;
		}

		void timer_callback(){
			if(!subscribing){return;}
			XInverter_Start(&ip_inst);

			cv::Mat outMat(gray_.rows, gray_.cols, gray_.type());

            for (int i = 0; i < noOfPixels; i++) {
                int data_read = BRAM2[i];

                unsigned char* data_r_bytes = (unsigned char*)&data_read;

                outMat.data[i*4]   = data_r_bytes[0];
                outMat.data[i*4+1] = data_r_bytes[1];
                outMat.data[i*4+2] = data_r_bytes[2];
                outMat.data[i*4+3] = data_r_bytes[3];
		    }

			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", outMat).toImageMsg();
			camera_publisher_->publish(*msg.get());
        	RCLCPP_INFO(this->get_logger(), "Published!");
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
