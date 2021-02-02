#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "include/run_yolo.h"
#include <string>

using namespace std;
static cv::Mat rbg_ptr;

static cv::String weightpath ="/home/patrick/camera_ws/src/camera/src/include/yolov4_lais.weights";
static cv::String cfgpath ="/home/patrick/camera_ws/src/camera/src/include/yolov4_lais.cfg";
static cv::String classnamepath = "/home/patrick/camera_ws/src/camera/src/include/obj.names";
static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

void imageCb(const sensor_msgs::ImageConstPtr & depth)
{
    //ROS_INFO("Image received");
    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        ROS_INFO("copied image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_dep = depth_ptr->image;

}

void imagecbb(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
        rbg_ptr = cv::imdecode(cv::Mat(msg->data),1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Yolonet.rundarknet(rbg_ptr);
    Yolonet.display(rbg_ptr);
}

int main(int argc, char** argv)
{
    cout<<"start"<<endl;

    ros::init(argc, argv, "trial");
    ros::NodeHandle nh;
//    ros::Subscriber sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::CompressedImage>("camera/color/image_raw/compressed", 1, imagecbb);

    cout<<"end"<<endl;

    ros::spin();
    return 0;
}


