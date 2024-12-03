#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cout_color.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;
std_msgs::Header header;
cv::VideoCapture camera;
double brightness;
double contrast;
double saturation;
double hue; 
double gain; 
double exposure; 
double white_balance; 
double frame_width;
double frame_height;
double fps; 
double format;
double pos_avi_ratio; 
std::string camera_ip;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "use_camera");
    ros::NodeHandle nh;
    ros::NodeHandle pr_nh("~");
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("/camera", 100);
    ros::Rate loop_rate(1000);

    pr_nh.param<std::string>("camera/camera_ip", camera_ip, "");
    try
    {
        camera.open("/dev/video" + camera_ip, cv::CAP_V4L2);
        std::cout << GREEN << "Open camera successfully!" << RESET << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    bool is_calibration;
    pr_nh.param<bool>("camera/is_calibration", is_calibration, false);
    if(is_calibration){
        brightness = camera.get(cv::CAP_PROP_BRIGHTNESS);
	    contrast= camera.get(cv::CAP_PROP_CONTRAST);
	    saturation = camera.get(cv::CAP_PROP_SATURATION);
    	hue = camera.get(cv::CAP_PROP_HUE);
	    gain = camera.get(cv::CAP_PROP_GAIN);
	    exposure = camera.get(cv::CAP_PROP_EXPOSURE);
	    white_balance = camera.get(cv::CAP_PROP_WHITE_BALANCE_BLUE_U);
        frame_width = camera.get(cv::CAP_PROP_FRAME_WIDTH);
	    frame_height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
	    fps = camera.get(cv::CAP_PROP_FPS);
	    format = camera.get(cv::CAP_PROP_FORMAT);
	    pos_avi_ratio = camera.get(cv::CAP_PROP_POS_AVI_RATIO);
        std::cout << BLUE << "---------------------------------------------" << RESET << std::endl;
	    std::cout << GREEN << "视频帧的像素宽度:" << frame_width << std::endl
	                       << "视频帧的像素高度:" << frame_height << std::endl
	                       << "录制视频的帧速率(帧/秒):" << fps << std::endl
	                       << "视频中的相对位置(范围为0.0到1.0):" << pos_avi_ratio << std::endl
	                       << "图像的格式:" << format << std::endl
	                       << "摄像头亮度：" << brightness << std::endl
	                       << "摄像头对比度：" << contrast << std::endl
	                       << "摄像头饱和度：" << saturation << std::endl
	                       << "摄像头色调：" << hue << std::endl
	                       << "摄像头增益：" << gain << std::endl
	                       << "摄像头曝光度：" << exposure << std::endl
	                       << "摄像头白平衡：" << white_balance << RESET << std::endl;
	    std::cout << BLUE << "---------------------------------------------" << RESET << std::endl;
    }
    else{
        pr_nh.param<double>("camera/frame_width", frame_width, 640);
        pr_nh.param<double>("camera/frame_height", frame_height, 240);
        pr_nh.param<double>("camera/fps", fps, 30);
        pr_nh.param<double>("camera/exposure",exposure, 1.0);
        pr_nh.param<double>("camera/contrast", contrast, 1.0);
        pr_nh.param<double>("camera/saturation", saturation, 1.0);

        camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
        camera.set(cv::CAP_PROP_FPS, fps);
        camera.set(cv::CAP_PROP_CONTRAST, contrast);
        camera.set(cv::CAP_PROP_EXPOSURE, exposure);
        camera.set(cv::CAP_PROP_SATURATION, saturation);
        camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'));
    }


    while(ros::ok())
    {
        cv::Mat frame;
        camera >> frame;
        
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        img_bridge.toImageMsg(img_msg);
        pub_image.publish(img_msg);

        if(frame.empty()){
            std::cerr << "Error: camerad frame is empty!" << std::endl;
        }
        cv::imshow("test", frame);
        if(cv::waitKey(30) > 0)
        {
            std::cout << "frame size = " << frame.rows << "x" << frame.cols << std::endl;
            camera.release();
            frame.release();
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
