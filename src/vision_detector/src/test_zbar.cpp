#include <iostream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "zbar.h"
#include "cout_color.h"

#define WINDOW_NAME "clor"
#define WINDOW_GRAY_NAME "gray"

int main(int argc, char *argv[])
{
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_GRAY_NAME, cv::WINDOW_AUTOSIZE);

    cv::Mat image = cv::imread("../data/123+321.png");
    if (!image.data) {
        std::cerr << RED << "Error: Could not open or find the image." << RESET << std::endl;
        return -1;
    }
    std::cout << BLUE << "---------------图像参数---------------" << RESET << std::endl;
    std::cout << GREEN << "flags: " << image.flags << std::endl   //标志位
                       << "size: "  << image.size  << std::endl    //图像尺寸
                       << "clos: "  << image.cols  << std::endl    //列
                       << "rows: "  << image.rows  << std::endl    //行
                       << "dims: "  <<image.dims   << RESET << std::endl;    //维度
    std::cout << BLUE << "-------------------------------------" << RESET << std::endl;
    cv::imshow(WINDOW_NAME,image);

    //灰度转换
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_RGB2GRAY);
    cv::imshow(WINDOW_GRAY_NAME, imageGray);
    
    //获取二进制数据
    int width = imageGray.cols;
    int height = imageGray.rows;
    uchar *raw = (uchar *)imageGray.data;
    zbar::Image imageZbar = zbar::Image(width, height, "Y800", raw, width * height);

    //创建扫描器
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    scanner.scan(imageZbar);

    if(imageZbar.symbol_begin() == imageZbar.symbol_end()){
        ROS_INFO_STREAM(RED << "识别错误 " << RESET);
    }

    zbar::Image::SymbolIterator symbol = imageZbar.get_symbols();
    for(; symbol != imageZbar.symbol_end(); ++symbol){
        ROS_INFO_STREAM("类型： " << symbol->get_type_name());
        ROS_INFO_STREAM("条码： " << symbol->get_data());
    }

    //释放二进制数据
    imageZbar.set_data(NULL, 0);
    cv::waitKey(0);
    return 0;
}
