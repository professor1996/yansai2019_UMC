#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>

#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>

#include <signal.h>

#define FPN_PATH    "../Samples/config/FPN_3.txt"

using namespace cv;
using namespace std;

CeleX5 *pCeleX = new CeleX5;

// void handler(int);

// class Camera_bin
// {
// public:
//     Camera_bin(CeleX5 *cam)
//     {
// 		cout << "Camera Initialized Successfully." << endl;
// 		m_camera = cam;
// 	}
//     ~Camera_bin()
//     {
//         cout << "Recording ended." << endl;
// 		m_camera->stopRecording();
//     }
// private:
// 	CeleX5 *m_camera;
// };

string  saveFile = "/home/pro/data/binfile";

Mat imgevent;
void event(const sensor_msgs::Image &msg_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img, "bgr8");
    imgevent = cv_ptr->image;
    imshow("infrared", imgevent);
    waitKey(1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "event");
	ros::NodeHandle n; 

	ros::Time time = ros::Time::now();
	ros::Rate loop_rate(100);

	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("event", 1);
	sensor_msgs::ImagePtr msg;
  

	// CeleX5 *pCeleX = new CeleX5;
	if (pCeleX == NULL)
		return 0;
	pCeleX->openSensor(CeleX5::CeleX5_MIPI);
	pCeleX->setFpnFile(FPN_PATH);

	CeleX5::CeleX5Mode sensorMode = CeleX5::Event_Address_Only_Mode;//Event_Address_Only_Mode  Event_Intensity_Mode
	pCeleX->setSensorFixedMode(sensorMode);
	pCeleX->setClockRate(10);

	// Camera_bin camera(pCeleX);

    char filename[100] = "/home/pro/data/binfile/%d.txt";
    int count = 0;
	while (ros::ok())
	{
		// pCeleX->startRecording("/home/pro/data/binfile/MipiData_20190511_211306550_E_100M.bin");
		
		if (!pCeleX->getEventPicMat(CeleX5::EventDenoisedBinaryPic).empty())
		{
			count++;
			cout << count << endl;


			std::vector<EventData> vecEvent;
			pCeleX->getEventDataVector(vecEvent);
		    
			// cout << "vecEvent.size()" << vecEvent.size() << endl;
			// cv::Mat mat = cv::Mat::zeros(cv::Size(1280, 800), CV_8UC1);
			// for (int i = 0; i < vecEvent.size(); i++)
			// {
			// 	mat.at<uchar>(800 - vecEvent[i].row - 1, 1280 - vecEvent[i].col - 1) = 255;
			// }
			// if (vecEvent.size() > 0)
			// {
			// 	cv::imshow("show", mat);
			// 	cv::waitKey(1);
			// }

			cv::Mat eventPicMat = pCeleX->getEventPicMat(CeleX5::EventDenoisedBinaryPic);
			// cv::imshow("Event-EventBinaryPic", eventPicMat);
			// cv::waitKey(1);

			msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", eventPicMat).toImageMsg();
			pub.publish(msg);

			// record raw data
			sprintf(filename,"/home/pro/data/raw_event/%d.txt",count);
			ofstream out(filename);
			for (int i = 0 ; i < vecEvent.size() ; i++)
			{
				out << i << "\t" << vecEvent[i].row << "\t" << vecEvent[i].col << endl; 
			}
			out.close();

			
		}


		loop_rate.sleep();
	}

	return 1;

}

// void handler(int sig)
// {
// 	// cout << "Recording ended." << endl;
// 	pCeleX->stopRecording();
// 	cout << "Recording ended1234" << endl;
// 	exit(0);
// }