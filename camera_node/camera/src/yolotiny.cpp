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
static cv::Mat frame;

static cv::String weightpath ="/home/patrick/camera_ws/src/camera/src/include/yolov3-tiny.weights";
static cv::String cfgpath ="/home/patrick/camera_ws/src/camera/src/include/yolov3-tiny.cfg";
static cv::String classnamepath = "/home/patrick/camera_ws/src/camera/src/include/coco.names";
static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));


void depthcallback(const sensor_msgs::ImageConstPtr & depth)
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

    //    cv::imshow("OPENCV_WINDOW", image_dep);
    //    cout << image_dep.size()<<endl;


    //    cv::waitKey(1);

}

void imagecallback(const sensor_msgs::CompressedImageConstPtr &image)
{
    try
    {
        frame = cv::imdecode(cv::Mat(image->data),1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Yolonet.rundarknet(frame);
    Yolonet.display(frame);

}


int main(int argc, char** argv)
{
    cout<<"start"<<endl;

    ros::init(argc, argv, "trial");
    ros::NodeHandle nh;
    //    ros::Subscriber sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthcallback);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::CompressedImage>("camera/color/image_raw/compressed", 1, imagecallback);

    cout<<"end"<<endl;

    ros::spin();
    return 0;
}



