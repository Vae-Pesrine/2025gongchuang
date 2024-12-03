#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <camera/std_cout.h>

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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "use_camera");
    ros::NodeHandle nh;
    ros::NodeHandle pr_nh("~");
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("/camera", 100);
    ros::Rate loop_rate(1000);

    try
    {
        camera.open("/dev/video2", cv::CAP_V4L2);
        std::cout << GREEN << "Open camera successfully!" << RESET << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    // camera.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // camera.set(cv::CAP_PROP_FPS, 30);
    // camera.set(cv::CAP_PROP_EXPOSURE, 1.0);
    // camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'));
    if(is_calibration){
        int frame_width = camera.get(cv::CAP_PROP_FRAME_WIDTH);
	    int frame_height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
	    int fps = camera.get(cv::CAP_PROP_FPS);
	    int format = camera.get(cv::CAP_PROP_FORMAT);
	    int pos_avi_ratio = camera.get(cv::CAP_PROP_POS_AVI_RATIO);
        ROS_INFO_STREAM(BLUE << "frame_width " << frame_width << " frame_height " << frame_height
                         << " fps " << fps << " format " << format << " pos_avi_ratio " << pos_avi_ratio << RESET);
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
