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
    ros::init(argc, argv, "raw_event_comfirm");
    ros::NodeHandle n; 
    ROS_INFO("Start read the frame!\n");
    ros::Time time = ros::Time::now();
  
    // std::vector<EventData> vecEvent;
    // cout << "vecEvent.size()" << vecEvent.size() << endl;
    cv::Mat mat = cv::Mat::zeros(cv::Size(1280, 800), CV_8UC1);
    cv::Mat flip_mat;

    int count, row, col;
    ifstream myfile("/home/pro/data/raw_event/127.txt"); 

    string s, _t="\t" , test="0";
    int _1_T = 0, _2_T = 0 ;

    string Count, Row, Col;
 

    while(getline(myfile,s))
    {
        _1_T = s.find(_t, 0);
        _2_T = s.find(_t, _1_T+1);       
        Count = s.substr(0, _1_T);

        stringstream ss1;
        ss1<<Count;
        ss1>>count;

        Row   = s.substr(_1_T+1, _2_T-_1_T-1);
        stringstream ss2;
        ss2<<Row;
        ss2>>row;

        Col   = s.substr(_2_T+1, s.length()-_2_T-1);
        stringstream ss3;
        ss3<<Col;
        ss3>>col;

        mat.at<uchar>(800 - row - 1, 1280 - col - 1) = 255;
    }

    flip(mat, flip_mat, 1);
    imshow("event", flip_mat);
    cv::waitKey(0);
    destroyAllWindows();

    return 0;
 

}
