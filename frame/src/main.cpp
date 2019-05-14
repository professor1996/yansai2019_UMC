#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>

#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include <ros/ros.h>

using namespace cv;
using namespace std;
  
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "frame");
    ros::NodeHandle n; 
    ROS_INFO("Start read the frame!\n");
    ros::Time time = ros::Time::now();
    ros::Rate loop_rate(1);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("frame", 1);
    sensor_msgs::ImagePtr msg;
    // msg = boost::make_shared<cv_bridge::CvImage>();

    cv_bridge::CvImagePtr Frame;
    // /*初始化CvImage智能指针，CvImage为Mat与ROS图像之间转换的载体*/
    Frame = boost::make_shared<cv_bridge::CvImage>();
    // /*设置ROS图片为BGR且每个像素点用1个字节来表示类似与CV_8U*/
    Frame->encoding = sensor_msgs::image_encodings::BGR8;


    VideoCapture cap("/dev/video1");
    if( !cap.isOpened() )
    {
        ROS_INFO("Read Video failed!\n");
        return 0;
    }

    Mat img;
    int count = 0;
    char imgname[30];

    while(ros::ok())
    {
        count++;
        
        // cap>>Frame->image;
        // if( Frame->image.empty() )
        //     break;
        // Frame->header.stamp = ros::Time::now();  
        // pub.publish(Frame->toImageMsg()); 


        cap >> img;
        if( img.empty() )
            break;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // msg->header.stamp = ros::Time::now();  
        pub.publish(msg);

 

        ROS_INFO( "read the %dth frame successfully!", count );
        //cout << count << endl;
        //imshow("frame",img);
        //sprintf(imgname,"%d%s",count,".jpg");
        //std::string str(imgname);
        //imwrite("/home/pro/data/frame/" + str, img);
 
        // sprintf(imgname,"/home/pro/data/binfile/%d.txt",count);
        // ofstream out(imgname);
        // out.close();

        // ros::spinOnce();
    }

    cap.release();

    return 0;
 

}
