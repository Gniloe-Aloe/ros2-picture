#include "picture.hpp"
#include <string>
#include <iostream>
#include <vector>
#include "picture.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>


std::string Picture::Pixel::get_string_pixel() {
    std::string red_string = std::to_string(int(red));
    std::string green_string = std::to_string(int(green));
    std::string blue_string = std::to_string(int(blue));

    if (red_string.length() == 1) red_string = "00" + red_string;
    if (red_string.length() == 2) red_string = "0" + red_string;

    if (green_string.length() == 1) green_string = "00" + green_string;
    if (green_string.length() == 2) green_string = "0" + green_string;

    if (blue_string.length() == 1) blue_string = "00" + blue_string;
    if (blue_string.length() == 2) blue_string = "0" + blue_string;

    return red_string + green_string + blue_string;
}


Picture::Picture() { picture_pixel_height = picture_pixel_width = 0; }


Picture::Picture(const unsigned& width, const std::string& message) {
    this->set_picture(width, message);
}


void Picture::set_picture(const unsigned& width, const std::string& message) {

    picture_pixel_width = width;
    picture_pixel_height = message.length() / (width * 3 * 3);

    //char in int
    /*std::vector<int> int_msg(message.length());
    for (int i = 0; i < message.length(); ++i) int_msg[i] = int(message[i]) - 48;*/

    //alloc memory for picture
    pixel_matrix.resize(picture_pixel_width);
    for (int w = 0; w < picture_pixel_width; ++w) pixel_matrix[w].resize(picture_pixel_height);

    //parsing message for pixels
    //i_widtx - x coordinate | i_height - y coordinate
    // 'x' and 'y' are positive ('y' go down)
    for (unsigned i_width = 0; i_width < picture_pixel_width; ++i_width) {
        for (unsigned i_height = 0; i_height < picture_pixel_height; ++i_height) {
            // * 100 - * 10 rank coefficient
            // * 9 colour coding with help of three rgb numbers, this numbers with help of three simbols (3*3=9)
            // + 1, + 2 ... + 8 simbhol position relatively of first red simbhol
            // - 48 convert cahr in numbers
            pixel_matrix[i_width][i_height].red = uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9] - 48) * 100
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 1] - 48) * 10
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 2] - 48);

            pixel_matrix[i_width][i_height].green = uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 3] - 48) * 100
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 4] - 48) * 10
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 5] - 48);

            pixel_matrix[i_width][i_height].blue = uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 6] - 48) * 100
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 7] - 48) * 10
                + uint8_t(message[picture_pixel_width * 9 * i_height + i_width * 9 + 8] - 48);

        }
    }
}


void Picture::print_picture() {
    //print pixels in consol
    for (int h = 0; h < picture_pixel_height; ++h) {
        for (int w = 0; w < picture_pixel_width; ++w) {
            std::cout << "(" << int(pixel_matrix[w][h].red) << "; " << int(pixel_matrix[w][h].green) << "; " << int(pixel_matrix[w][h].blue) << ")" << '\t';
        }
        std::cout << '\n';
    }
}


unsigned Picture::get_picture_width() { return this->picture_pixel_width; }


unsigned Picture::get_picture_height() { return this->picture_pixel_height; }


std::string Picture::get_picture_message() {
    std::string tmp;

    for (unsigned h = 0; h < picture_pixel_height; ++h) {
        for (unsigned w = 0; w < picture_pixel_width; ++w) {
            tmp += this->pixel_matrix[w][h].get_string_pixel();
        }
    }

    return tmp;
}

void Picture::prepare_opencv_Mat() {

    opencv_picture.create(picture_pixel_width, picture_pixel_height, CV_8UC3);

    uint8_t* p_data = opencv_picture.data;

    // переносим данные из нашего класса в 'cv::Mat'
    for (int h = 0; h < picture_pixel_height; ++h) {
        for (int w = 0, i = 0; w < picture_pixel_width * 3; w += 3, ++i) {
            //возможно не работает!!!
            p_data[picture_pixel_width * 3 * h + w] = (uint8_t)pixel_matrix[i][h].blue;//blue
            p_data[picture_pixel_width * 3 * h + w + 1] = (uint8_t)pixel_matrix[i][h].green;//green
            p_data[picture_pixel_width * 3 * h + w + 2] = (uint8_t)pixel_matrix[i][h].red;//red
        }
    }
}

// загружаем изображение из cv::Mat в наш класс
void Picture::set_picture_from_openCV(cv::Mat& image) {
    picture_pixel_width = image.rows;
    picture_pixel_height = image.cols;

    // выделяем память для изображения
    pixel_matrix.resize(picture_pixel_width);
    for (int w = 0; w < picture_pixel_width; ++w) pixel_matrix[w].resize(picture_pixel_height);

    // указатель на 'матрицу'(массив) пикселей
    uint8_t* p_data = image.data;

    // заполняем наш класс изображения пикселями из cv::Mat
    for (int h = 0; h < picture_pixel_height; ++h) {
        for (int w = 0, i_width_of_our_class_pixel = 0; w < picture_pixel_width * 3; w += 3, ++i_width_of_our_class_pixel) {
            // возможно не работает!!!
            pixel_matrix[i_width_of_our_class_pixel][h].blue = (uint8_t)p_data[picture_pixel_width * 3 * h + w];//blue
            pixel_matrix[i_width_of_our_class_pixel][h].green = (uint8_t)p_data[picture_pixel_width * 3 * h + w + 1];//green
            pixel_matrix[i_width_of_our_class_pixel][h].red = (uint8_t)p_data[picture_pixel_width * 3 * h + w + 2];//red
        }
    }

}

Picture::Picture(cv::Mat& image) {
    this->set_picture_from_openCV(image);
}
