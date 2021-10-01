#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#pragma once
#ifndef MY_PICTURE
#define MY_PICTURE

class Picture {
private:
    class Pixel {
    public:
        uint8_t red;
        uint8_t green;
        uint8_t blue;


        std::string get_string_pixel();
    };
    unsigned picture_pixel_width;
    unsigned picture_pixel_height;
    std::vector<std::vector<Pixel>> pixel_matrix;


public:
    cv::Mat opencv_picture;

    Picture();

    Picture(const unsigned& width, const std::string& message);

    Picture(cv::Mat& image);

    void set_picture(const unsigned& width, const std::string& message);

    void print_picture();

    unsigned get_picture_width();

    unsigned get_picture_height();

    std::string get_picture_message();

    // переносим данные из нашего класса в 'Mat' из openCV
    void prepare_opencv_Mat();

    // загружаем изображение из cv::Mat в наш класс
    void set_picture_from_openCV(cv::Mat& image);
};

#endif //MY_PICTURE