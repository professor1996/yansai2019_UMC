#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Int16.h>
#include "control/serial.h"

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//#include <move_base_msgs/MoveBaseAction.h>
//#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>

#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
 
using namespace sensor_msgs;
using namespace message_filters;

using namespace cv;
using namespace std;
  
#define PI 3.1415926535898

//msg
int serial_ready_flag;
 
Mat imgframe;
void frame(const sensor_msgs::Image &msg_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img, "bgr8");
    imgframe = cv_ptr->image;
    imshow("frame", imgframe);
    waitKey(1);
}

Mat imginfrared, flip_imginfrared;
void infrared(const sensor_msgs::Image &msg_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img, "bgr8");
    imginfrared = cv_ptr->image;

    flip(imginfrared, flip_imginfrared, -1);
    imshow("infrared", flip_imginfrared);
    waitKey(1);
}

Mat imgevent, flip_imgevent;
void event(const sensor_msgs::Image &msg_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_img, "bgr8");
    imgevent = cv_ptr->image;

    flip(imgevent, flip_imgevent, 1);
    imshow("event", flip_imgevent);
    waitKey(1);
}

char IMGframe[30];
char IMGinfrared[30];   
char IMGevent[30];
int count = 0;
void imgfusion_callback(const ImageConstPtr& infrared, const ImageConstPtr& frame, const ImageConstPtr& event)  //回调中包含多个消息
{
    // count = (int)event->header.seq;
    cout << event->header.seq << endl;


    cv_bridge::CvImagePtr cv_ptr_frame;
    cv_ptr_frame = cv_bridge::toCvCopy(frame, "bgr8");
    imgframe = cv_ptr_frame->image;
    imshow("frame", imgframe);

    cv_bridge::CvImagePtr cv_ptr_infrared;
    cv_ptr_infrared = cv_bridge::toCvCopy(infrared, "bgr8");
    imginfrared = cv_ptr_infrared->image;
    flip(imginfrared, flip_imginfrared, -1);
    imshow("infrared", flip_imginfrared);

    cv_bridge::CvImagePtr cv_ptr_event;
    cv_ptr_event = cv_bridge::toCvCopy(event, "bgr8");
    imgevent = cv_ptr_event->image;
    flip(imgevent, flip_imgevent, 1);
    imshow("event", flip_imgevent);


    sprintf(IMGframe,"%d%s",event->header.seq,".jpg");
    std::string str1(IMGframe);
    imwrite("/home/pro/data/frame/" + str1, imgframe);

    sprintf(IMGinfrared,"%d%s",event->header.seq,".jpg");
    std::string str2(IMGinfrared);
    imwrite("/home/pro/data/infrared/" + str2, flip_imginfrared);

    sprintf(IMGevent,"%d%s",event->header.seq,".jpg");
    std::string str3(IMGevent);
    imwrite("/home/pro/data/event/" + str3, flip_imgevent);

    waitKey(1);
}

// void imgfusion_callback(const sensor_msgs::ImageConstPtr& infrared, const sensor_msgs::ImageConstPtr& frame)  //回调中包含多个消息
// {
//     // cout << infrared->header.stamp << endl;

//     cv_bridge::CvImagePtr cv_ptr_frame;
//     cv_ptr_frame = cv_bridge::toCvCopy(frame, "bgr8");
//     imgframe = cv_ptr_frame->image;
//     imshow("frame", imgframe);

//     cv_bridge::CvImagePtr cv_ptr_infrared;
//     cv_ptr_infrared = cv_bridge::toCvCopy(infrared, "bgr8");
//     imginfrared = cv_ptr_infrared->image;
//     flip(imginfrared, flip_imginfrared, -1);
//     imshow("infrared", flip_imginfrared);

//     // sprintf(imgframe,"%d%s",count,".jpg");
//     // std::string str(imgframe);
//     // imwrite("/home/pro/data/frame/" + str, imgframe);

//     // sprintf(imginfrared,"%d%s",count,".jpg");
//     // std::string str(imginfrared);
//     // imwrite("/home/pro/data/frame/" + str, flip_imginfrared);

//     waitKey(1);
// }


float x=0, y=0, yaw = 0;
void VSLAM(const geometry_msgs::Pose& msg)
{
	//printf("dwqd");
	yaw = msg.orientation.z;
	x   = msg.position.x  ;
	y   = msg.position.y  ;
 
	ROS_INFO("yaw:%f",yaw);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;

    ros::Publisher pub_imu_data=nh.advertise<sensor_msgs::Imu>("imu_data",1);
    sensor_msgs::Imu imu_msg;

    ros::Subscriber sub_VSLAM_info = nh.subscribe("VSLAM_info", 1000, &VSLAM);

    // ros::Subscriber sub_frame    = nh.subscribe("frame", 1, frame);
    // ros::Subscriber sub_infrared = nh.subscribe("infrared", 1, infrared);
    // ros::Subscriber sub_event = nh.subscribe("event", 1, event);

    message_filters::Subscriber<Image> frame_sub(nh, "frame", 1);             // topic1 输入
    message_filters::Subscriber<Image> infrared_sub(nh, "infrared", 1);   // topic2 输入
    message_filters::Subscriber<Image> event_sub(nh, "event", 1);   // topic3 输入
    // TimeSynchronizer<Image, Image> sync(frame_sub, infrared_sub, 10);       // 同步
    // sync.registerCallback(boost::bind(&imgfusion_callback, _1, _2)); 

    TimeSynchronizer<Image, Image, Image> sync(frame_sub, infrared_sub, event_sub, 10);       // 同步
    sync.registerCallback(boost::bind(&imgfusion_callback, _1, _2, _3));                   // 回调

    ros::Rate loop_rate(1);

    Serial serial("/dev/ttyUSB1");
    serial.configurePort();
    struct RobotMsgToMCU msg_tomcu;
    struct RobotMsgFromMCU msg_frommcu;
    
    float  pit = 0, rol = 0;

    while (ros::ok())
    {
    //    ROS_INFO( "a " );
    //     //msg_frommcu
    //    if (serial.ReadData(msg_frommcu))
    //    {
	//    ros::Time time=ros::Time::now();

	//    if (msg_frommcu.ax > 16000) msg_frommcu.ax = msg_frommcu.ax - 32639;
	//    if (msg_frommcu.ay > 16000) msg_frommcu.ay = msg_frommcu.ay - 32639;
	//    if (msg_frommcu.az > 16000) msg_frommcu.az = msg_frommcu.az - 32639;
	//    if (msg_frommcu.gx > 16000) msg_frommcu.gx = msg_frommcu.gx - 32639;
	//    if (msg_frommcu.gy > 16000) msg_frommcu.gy = msg_frommcu.gy - 32639;
	//    if (msg_frommcu.gz > 16000) msg_frommcu.gz = msg_frommcu.gz - 32639;
	//    if (msg_frommcu.pit > 16000) msg_frommcu.pit = msg_frommcu.pit - 32639;
	//    if (msg_frommcu.rol > 16000) msg_frommcu.rol = msg_frommcu.rol - 32639;

	//    pit = msg_frommcu.pit; pit = pit/10;
	//    rol = msg_frommcu.rol; rol = rol/10;

  	//    ROS_INFO( "ax:%d  ay:%d  az:%d ", msg_frommcu.ax, msg_frommcu.ay, msg_frommcu.az);
  	//    ROS_INFO( "gx:%d  gy:%d  gz:%d ", msg_frommcu.gx, msg_frommcu.gy, msg_frommcu.gz);
  	//    ROS_INFO( "pit:%f  rol:%f    \n", pit , rol);

    //        imu_msg.header.stamp=time;
    //        imu_msg.header.frame_id="imu_link";
    //        imu_msg.angular_velocity.x=msg_frommcu.gx * 1.0 / 1000;
    //        imu_msg.angular_velocity.y=msg_frommcu.gy * 1.0 / 1000;
    //        imu_msg.angular_velocity.z=msg_frommcu.gz * 1.0 / 1000;
    //        imu_msg.linear_acceleration.x=msg_frommcu.ax;
    //        imu_msg.linear_acceleration.y=msg_frommcu.ay;
    //        imu_msg.linear_acceleration.z=msg_frommcu.az;
	//    imu_msg.orientation.x        =pit;
	//    imu_msg.orientation.y        =rol;
    //        pub_imu_data.publish(imu_msg);



 	//    //msg_tomcu.setMsg(yaw*10000,2,3,4,5,6,7);
 	//    //serial.SendData(msg_tomcu);
    //    }
       ros::spinOnce();
    }
}


