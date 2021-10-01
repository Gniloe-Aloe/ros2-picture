#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "base64.hpp"
#include "timer.hpp"
#include "picture.hpp"


//using std::placeholders::_1;


class My_subscriber : public rclcpp::Node {
public:
    My_subscriber() : rclcpp::Node("subscriber_node") {
        counter = 0;
        //subscriber = this->create_subscription<std_msgs::msg::String>("picture_topic", 30);
        subscriber = this->create_subscription<std_msgs::msg::String>("picture_topic", 30, std::bind(&My_subscriber::get_msg, this, std::placeholders::_1));
    }

private:

    void get_msg(const std_msgs::msg::String::SharedPtr msg) {

        std::string dec_jpg = base64_decode(std::move(msg->data));
        std::vector<uchar> data(dec_jpg.begin(), dec_jpg.end());
        cv::Mat base64_image = cv::imdecode(cv::Mat(data), 1);
        //cv::imwrite("/home/dkosinov/win_home/Desktop/pic/test.png", base64_image);
        std::cout << counter++ << '\t' << "Width = " << base64_image.cols << '\t' << "Heigh = " << base64_image.rows << '\n';
    }

    size_t counter;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber"){
        subscription_ = this->create_subscription<std_msgs::msg::String>("picture_topic", 30, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        std::string dec_jpg = base64_decode(std::move(msg->data));
        std::vector<uchar> data(dec_jpg.begin(), dec_jpg.end());
        cv::Mat base64_image = cv::imdecode(cv::Mat(data), 1);
        //cv::imwrite("/home/dkosinov/win_home/Desktop/pic/test.png", base64_image);
        std::cout << '\t' << "Width = " << base64_image.cols << '\t' << "Heigh = " << base64_image.rows << '\n';
        //std::cout << counter << '\t' << "Lenth = " << msg->data.length() << '\n';

        /*Picture pic(1920, msg->data);
        pic.prepare_opencv_Mat();
        std::cout << counter << '\t' << "Width = " << pic.opencv_picture.cols << '\t' << "Heigh = " << pic.opencv_picture.rows << '\n';*/
        
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    timer t;
    rclcpp::init(argc, argv);
    std::shared_ptr<My_subscriber> picture_node = std::make_shared<My_subscriber>();
    rclcpp::spin(picture_node);
    rclcpp::shutdown();
   
    return 0;
}