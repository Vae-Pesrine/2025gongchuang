#include <ros/ros.h>
#include <camera/std_cout.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

static int iLowH = 10;
static int iHighH =  40;
static int iLowS = 90;
static int iHighS = 255;
static int iLowV = 1;
static int iHighV = 255;

void RGBCallback(sensor_msgs::Image::ConstPtr image_msg)
{
    cv_bridge::CvImage::Ptr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "cv_bridge exception:" << e.what() << std::endl;
        return;
    }
    //将RGB图片转换为HSV
    cv::Mat img_origin = cv_ptr->image;
    cv::Mat img_HSV;
    cv::cvtColor(img_origin, img_HSV, cv::COLOR_BGR2HSV);

    //在HSV空间做直方图均衡化
    std::vector<cv::Mat> hsvSplit;
    cv::split(img_HSV, hsvSplit);
    cv::equalizeHist(hsvSplit[1], hsvSplit[2]);
    cv::merge(hsvSplit, img_HSV);

    //使用上面的Hue，Saturation，Value的阈值范围对图像进行二值化
    cv::Mat img_Threshold;
    cv::inRange(img_HSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), img_Threshold);

    //开操作，去除一些噪点
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(img_Threshold, img_Threshold, cv::MORPH_OPEN, element);
    //闭操作，连接一些连通域
    morphologyEx(img_Threshold, img_Threshold, cv::MORPH_CLOSE, element);

    int nTargetX = 0;
    int nTargetY = 0;                               
    int nPixCount = 0;                              //目标占用的像素点数量
    int nImgWidth = img_Threshold.cols;             //图像分辨率
    int nImgHeight = img_Threshold.rows;
    int nImgChannels = img_Threshold.channels();    //每个像素占用几个字节
    for(int y = 0; y < nImgHeight; ++y)
    {
        for(int x = 0; x < nImgWidth; ++ x)
        {
            if(img_Threshold.data[y * nImgWidth + x] == 255){
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }
    if(nPixCount > 0)
    {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        std::cout << "颜色质心坐标" << nTargetX << " " << nTargetY << "点数为" << nPixCount << std::endl;

        //画坐标
        cv::Point line_begin = cv::Point(nTargetX - 10, nTargetY);
        cv::Point line_end = cv::Point(nTargetX + 10, nTargetY);
        line(img_origin, line_begin, line_end, cv::Scalar(255, 0, 0));
        line_begin.x = nTargetX;
        line_begin.y = nTargetY - 10;
        line_end.x = nTargetX;
        line_end.y = nTargetY + 10;
        line(img_origin, line_begin, line_end, cv::Scalar(255, 0, 0)); 
    }
    else{
        std::cout << RED << "No target!" << RESET << std::endl;
    }

    imshow("RGB", img_origin);
    imshow("HSV", img_HSV);
    imshow("Result", img_Threshold);
    cv::waitKey(5);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cv_hsv_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb = nh.subscribe("/camera", 1, RGBCallback);

    //生成图像显示和参数调节的窗口
    cv::namedWindow("Threshold", cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("LowH",  "Threshold", &iLowH,  179);
    cv::createTrackbar("HighH", "Threshold", &iHighH, 179);
    cv::createTrackbar("LowS",  "Threshold", &iLowS,  255);
    cv::createTrackbar("HighS", "Threshold", &iHighS, 255);
    cv::createTrackbar("LowV",  "Threshold", &iLowV,  255);
    cv::createTrackbar("HighV", "Threshold", &iHighV, 255);

    cv::namedWindow("RGB");
    cv::namedWindow("HSV");
    cv::namedWindow("Result");

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
