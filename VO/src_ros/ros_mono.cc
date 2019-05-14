#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Pose.h>
#include<opencv2/core/core.hpp>
#include"../../../include/Tracking.h"
#include"../../../include/System.h"
#include"../../../include/FrameDrawer.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, 
                 ros::Publisher* ppub):mpSLAM(pSLAM), mppub(ppub){}
                 

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    ros::Publisher* mppub;

    geometry_msgs::Pose p;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nodeHandler;
    ros::Publisher chatter_pub = nodeHandler.advertise<geometry_msgs::Pose>("VSLAM_info", 100);
 

    ImageGrabber igb(&SLAM, &chatter_pub);
    ros::Subscriber sub = nodeHandler.subscribe("frame_image", 1, &ImageGrabber::GrabImage, &igb);

    ros::Rate loop_rate(1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

 
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    p.orientation.z = ORB_SLAM2::car_pose[0]; 
    p.position.x    = ORB_SLAM2::car_pose[1]; 
    p.position.y    = ORB_SLAM2::car_pose[2];
    p.position.z    = ORB_SLAM2::left_right_porta;
    cout << "ORB_SLAM2::left_right_porta: " << ORB_SLAM2::left_right_porta << endl;
    mppub->publish(p);

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}



