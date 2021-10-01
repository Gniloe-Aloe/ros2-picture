#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
//#include <image_transport/image_transport.h>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "base64.hpp"
#include "timer.hpp"
#include "picture.hpp"

std::string encoding;

//using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class My_publisher : public rclcpp::Node {
public:
    My_publisher(const std::string& node_name) : rclcpp::Node(node_name) {
        std::cout << "[" << this->get_logger().get_name() << "] created" << std::endl;
        count = 0;
        publisher = this->create_publisher<std_msgs::msg::String>("picture_topic", 30);


        // this segment can be include in timer_callback()
        //timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&My_publisher::timer_callback, this));
        for (size_t i = 0; i < 30; ++i) {
            std::cout << std::to_string(count++) << '\t' << "[" << this->get_logger().get_name() << "] send " << encoding.length() << " bytes" << std::endl;
            std_msgs::msg::String message;
            message.set__data(encoding);
            publisher->publish(message);
            //std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
        }
        
    }


private:

    //void timer_callback() {
    //    std_msgs::msg::String message;
    //    //message.data = "Hello! " + std::to_string(count_++);
    //    message.set__data(encoding);
    //    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //    std::cout << std::to_string(count++) << '\t' << "Message send!" << std::endl;
    //    publisher->publish(message);
    //}

    size_t count;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 100);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        

        std_msgs::msg::String message;
        //message.data = "Hello! " + std::to_string(count_++);
        message.set__data(encoding);
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        std::cout << std::to_string(count_++) << '\t' << "Message send!" << std::endl;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

void test (){

}

int main(int argc, char* argv[])
{
    timer t;
    cv::Mat image = cv::imread("/home/dkosinov/win_home/Desktop/big.png");
    
    std::vector<uchar> buffer;
    buffer.resize(static_cast<size_t>(image.rows) * static_cast<size_t>(image.cols));
    cv::imencode(".png", image, buffer);
    encoding = base64_encode(buffer.data(), buffer.size());
    

    /*auto node_ = rclcpp::Node::make_shared("test_subscriber");
    image_transport::ImageTransport it(node_.get());
    auto sub = it.subscribe("in_image_base_topic", 1, imageCallback);
    auto pub = it.advertise("out_image_base_topic", 1);*/
    
    rclcpp::init(argc, argv);

    std::shared_ptr<My_publisher> picture_node = std::make_shared<My_publisher>("picture_publisher_node");

    rclcpp::spin(picture_node);
    

    rclcpp::shutdown();

    
    return 0;
}