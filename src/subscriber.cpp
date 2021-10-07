#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
//#include <image_transport/image_transport.h>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>

#include "base64.hpp"
#include "timer.hpp"
#include "picture.hpp"




void timer_thread(tms::timer& my_timer, size_t* point_on_counter, bool* p_not_end_flag) {
    size_t current_frame = *point_on_counter;
    // *p_not_end_flag = 0 when we are ending get messages
    while (*p_not_end_flag) {
        my_timer.set_checkpoint();
        // ждём, пока не прошла одна секунда
        while (!my_timer.check_checkpoint()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        uint fps = *point_on_counter - current_frame;
        current_frame = *point_on_counter;
        // завершаем вывод фпс, если перестали получать сообщения
        if (fps == 0 && *point_on_counter != 0)break;
        // выводим количество кадров за одну секунду
        std::cout << "FPS = " << fps << std::endl;
        
    }
    std::cout << *point_on_counter << " messages acepted" << '\n';
}

class My_subscriber : public rclcpp::Node {
public:
    My_subscriber(const std::string& node_name, const std::string& topic_name, size_t* point_on_counter, bool* point_not_end_flag) : rclcpp::Node(node_name) {
        std::cout << "[" << this->get_logger().get_name() << "] created" << std::endl;
        point_counter = point_on_counter;
        p_not_end_flag = point_not_end_flag;
        *point_counter = 0;
        
        // приём текста
        //subscriber = this->create_subscription<std_msgs::msg::String>(topic_name, 30, std::bind(&My_subscriber::get_msg, this, std::placeholders::_1));

        // приём изображения
        //picture_subscriber = image_transport::create_subscription(topic_name, )
        
        //!!!!!!!!
        //camera_sub_ = image_transport::create_subscription(this, "image", &MonoOdometer::imageCallback, transport, 1);
        //image_transport::create_subscription(this, "image",[&](auto& msg) { this->imageCallback(msg); }, transport, 1);
    
        picture_subscriber = image_transport::create_subscription(this, topic_name,
            [&](auto& msg) { this->get_image(msg); }, "raw", rmw_qos_profile_default);

        //= this->create_subscription<sensor_msgs::msg::Image>(topic_name, rclcpp::QoS(0), std::bind(&My_subscriber::get_image, this, std::placeholders::_1));

    }

private:
    // изображение image_transport
    void get_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        //std::cout << *point_counter << "\t [" << this->get_logger().get_name() << "] get: " << msg->width << "x" << msg->height << std::endl;
        *point_counter += 1;
        /*if (*point_counter > 9998) {
            *p_not_end_flag = false;
        }*/
    }

    void test_get(sensor_msgs::msg::Image::ConstUniquePtr& msg) {
        *point_counter += 1;
    }


    // изображение в текстовом формате
    void get_pic_msg(const std_msgs::msg::String::SharedPtr msg) {
        std::string dec_jpg = base64_decode(std::move(msg->data));
        std::vector<uchar> data(dec_jpg.begin(), dec_jpg.end());
        cv::Mat base64_image = cv::imdecode(cv::Mat(data), 1);
        //cv::imwrite("/home/dkosinov/win_home/Desktop/pic/test.png", base64_image);
        std::cout << *point_counter++ << '\t' << "Width = " << base64_image.cols << '\t' << "Heigh = " << base64_image.rows << '\n';
    }
    // текст
    void get_msg(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << *point_counter++ << '\t' << "[" << this->get_logger().get_name() << "] get: " << msg->data << std::endl;
    }

    bool* p_not_end_flag;
    size_t* point_counter;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_subscriber;
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr picture_subscriber;
    image_transport::Subscriber picture_subscriber;
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
    tms::timer my_timer;
    rclcpp::init(argc, argv);
    std::string node_name = "picture_subscriber_node";
    std::string topic_name = "picture_topic";
    
    // указатель на счётчик принятых сообщений
    size_t* p_counter = new(size_t);

    // флаг конца приёма
    bool* p_not_end_flag = new(bool);
    *p_not_end_flag = true;

    std::shared_ptr<My_subscriber> picture_node = std::make_shared<My_subscriber>(node_name, topic_name, p_counter, p_not_end_flag);

    std::thread timer_th([&my_timer, &p_counter, &p_not_end_flag]() {timer_thread(my_timer, p_counter, p_not_end_flag); });
    
    rclcpp::spin(picture_node);

    timer_th.join();
    delete p_counter;
    delete p_not_end_flag;
    rclcpp::shutdown();
   
    return 0;
}