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
#include <cmath>
#include <numeric>

#include <librealsense2/rs.hpp>

#include "include/run_yolo.h"
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "visualization_msgs/Marker.h"

using namespace std;
static cv::Mat frame;
static cv::Mat image_dep;

static cv::String weightpath ="/home/patrick/camera_ws/src/camera/src/include/yolov4_lais.weights";
static cv::String cfgpath ="/home/patrick/camera_ws/src/camera/src/include/yolov4_lais.cfg";
static cv::String classnamepath = "/home/patrick/camera_ws/src/camera/src/include/obj.names";
static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

void find_IOU(cv::Mat &frame, cv::Mat &loadeddepth);
void getcorner(cv::Mat &frame, cv::Mat loadeddepth);
void getstraightline(cv::Mat &frame, cv::Mat loadeddepth);
void featureextract(cv::Mat &frame, cv::Mat loadeddepth);
void depth_calculate(cv::Point whichpixel, cv::Mat loadeddepth);
void getedge(cv::Mat &frame, cv::Mat loadeddepth);

double last_request;

static visualization_msgs::Marker UAVtrajectory;
static visualization_msgs::Marker edge_points;
static visualization_msgs::Marker final_publish;

typedef struct UAVpose
{
    double x;
    double y;
    double z;
    double ow;
    double ox;
    double oy;
    double oz;
}UAVpose;

static UAVpose uavinfo;
void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{
    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image_dep = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Rect a(cv::Point(300,160),cv::Point(370,320));
    double leftest = a.x + a.width/2 - 0.1 * a.width;
    double righest = a.x + a.width/2 + 0.1 * a.width;
    double uppest = a.y + a.height/2 - 0.1 * a.height;
    double lowest = a.y + a.height/2 + 0.1 * a.height;
    vector<cv::Point> edgepoints;
    cv::findNonZero(image_dep,edgepoints);
//    cout<<1<<endl;
    for(auto &pixel : edgepoints)//get cross edgepoints
    {
        if(image_dep.at<ushort>(pixel) * 0.001 < 4.0 && (pixel.x > leftest && pixel.x < righest) )
        {
            cv::circle(frame, pixel, 2.0, CV_RGB(200,0,0));
        }
        if(image_dep.at<ushort>(pixel) * 0.001 < 4.0 && (pixel.y > uppest && pixel.y < lowest ))
        {
            cv::circle(frame, pixel, 2.0, CV_RGB(200,0,0));
        }
    }
    cv::rectangle(frame,cv::Point(a.x,a.y),cv::Point(a.x+a.width,a.y+a.height),CV_RGB(0,0,200),2);
    cv::imshow("test",frame);
    cv::waitKey(20);
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo.x = pose->pose.position.x;
    uavinfo.y = pose->pose.position.y;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;

    //    UAVtrajectory.points.clear();
//    UAVtrajectory.header.frame_id = "map";
//    UAVtrajectory.header.stamp = ros::Time::now();
//    UAVtrajectory.ns = "GT_points";
//    UAVtrajectory.id = 0;
//    UAVtrajectory.action = visualization_msgs::Marker::ADD;
//    UAVtrajectory.pose.orientation.w = 1.0;
//    UAVtrajectory.type = visualization_msgs::Marker::SPHERE_LIST;
//    UAVtrajectory.scale.x = UAVtrajectory.scale.y = UAVtrajectory.scale.z = 0.08;
//    UAVtrajectory.color.a = 1;
//    UAVtrajectory.color.r = 1;
//    UAVtrajectory.color.g = 1;
//    UAVtrajectory.color.r = 1;
//    geometry_msgs::Point UAV;
//    UAV.x = uavinfo.x;
//    UAV.y = uavinfo.y;
//    UAV.z = uavinfo.z;
//    UAVtrajectory.points.push_back(UAV);

}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

int main(int argc, char** argv)
{
    cout<<"Extracting features..."<<endl;

    ros::init(argc, argv, "feature");
    ros::NodeHandle nh;

    //below subscribe the image and the depth map of the camera
    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 10);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber sub_uabposition = nh.subscribe<geometry_msgs::PoseStamped>
                                      ("/mavros/local_position/pose", 10, position_callback);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);


    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",10);

    ros::Rate rate(20.0);

    while(ros::ok())
    {
        edge_points.header.frame_id = "map";
        edge_points.header.stamp = ros::Time::now();
        edge_points.ns = "GT_points";
        edge_points.id = 0;
        edge_points.action = visualization_msgs::Marker::ADD;
        edge_points.pose.orientation.w = 1.0;
        edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
        edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.01;
        edge_points.color.a = 1;
        geometry_msgs::Point UAV;
        UAV.x = uavinfo.x;
        UAV.y = uavinfo.y;
        UAV.z = uavinfo.z;
        std_msgs::ColorRGBA color_for_traj;
        color_for_traj.a=1;
        color_for_traj.b=1;
        color_for_traj.g=1;
        color_for_traj.r=1;
        edge_points.points.push_back(UAV);
        edge_points.colors.push_back(color_for_traj);
        if(edge_points.points.size()>=50000)
        {
            edge_points.points.clear();
            edge_points.colors.clear();
            cout<<"hi clear"<<endl;
        }
        rviz_visual.publish(edge_points);
//        rviz_visual.publish(UAVtrajectory);
//        edge_points.points.clear();
//        edge_points.colors.clear();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void find_IOU(cv::Mat &frame, cv::Mat &loadeddepth)
{
    cv::Mat tempframe = frame;
    vector<cv::Point> furthest;
    vector<cv::Point> furthest2;

    if(true)
    {
        //    for(int i = 0; i<loadeddepth.cols;i++)
        //    {
        //        for(int j = 0; j<loadeddepth.rows;j++)
        //        {
        //            cv::Point temp(i, j);
        //            if(loadeddepth.at<ushort>(temp) * 0.001 > 2.5 )
        //            {
        //                furthest.push_back(temp);
        //            }
        //            if(loadeddepth.at<ushort>(temp) * 0.001 < 2.5 && loadeddepth.at<ushort>(temp) * 0.001 > 1.0)
        //            {
        //                furthest2.push_back(temp);
        //            }
        //        }
        //    }
        //    for(auto &pixel : furthest)
        //    {
        //        tempframe.at<cv::Vec3b>(pixel)[0] = 255;
        //        tempframe.at<cv::Vec3b>(pixel)[1] = 255;
        //        tempframe.at<cv::Vec3b>(pixel)[2] = 255;
        //    }
        //    for(auto &pixel : furthest2)
        //    {
        //        tempframe.at<cv::Vec3b>(pixel)[0] = 0;
        //        tempframe.at<cv::Vec3b>(pixel)[1] = 255;
        //        tempframe.at<cv::Vec3b>(pixel)[2] = 255;
        //    }


        //    getcorner(tempframe, loadeddepth);
    }

    featureextract(tempframe, loadeddepth);
}

void getcorner(cv::Mat &frame, cv::Mat loadeddepth)
{
    cv::Mat grayframe;
    vector<cv::Point2f> whichpixelhascorner;

    cv::cvtColor(frame, grayframe, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(grayframe, whichpixelhascorner, 200, 0.01, 10, cv::noArray(), 3, false, 0.04);

    for (size_t i=0; i<whichpixelhascorner.size(); i++)
    {

        cv::Point whichpixel = whichpixelhascorner[i];
        if(loadeddepth.at<ushort>(whichpixel) * 0.001 < 8.0 && loadeddepth.at<ushort>(whichpixel) != 0)
        {
            cv::circle(frame, whichpixel, 4, cv::Scalar(255,255,0) );
            depth_calculate(whichpixel, loadeddepth);
        }

    }

//    cv::imshow("test1", frame);
//    cv::waitKey(60);

}

void depth_calculate(cv::Point whichpixel, cv::Mat loadeddepth)
{
    vector<double> edge_depth;
    double depth = loadeddepth.at<ushort>(whichpixel);
    double z = depth/1000,
           x = z * (whichpixel.x - cx)/fx,
           y = z * (whichpixel.y - cy)/fy;

    Eigen::Matrix<double, 4, 1> cam (x,y,z,1), body, world;
    Eigen::Matrix<double, 4, 4> cam_to_body;
    cam_to_body << 0,0,1,0,
                   -1,0,0,0,
                   0,-1,0,0,
                   0,0,0,1;

    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * cam_to_body * cam;

    edge_points.header.frame_id = "map";
    edge_points.header.stamp = ros::Time::now();
    edge_points.ns = "GT_points";
    edge_points.id = 0;
    edge_points.action = visualization_msgs::Marker::ADD;
    edge_points.pose.orientation.w = 1.0;
    edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
    edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.01;
    std_msgs::ColorRGBA color_for_edge;
    color_for_edge.a=1;
    color_for_edge.b=0;
    color_for_edge.g=1;
    color_for_edge.r=1;
    geometry_msgs::Point EDGE;
    EDGE.x = world(0);
    EDGE.y = world(1);
    EDGE.z = world(2);
    if(EDGE.z>0.4)
    {
        edge_points.points.push_back(EDGE);
        edge_points.colors.push_back(color_for_edge);
    }

}

void getstraightline(cv::Mat &frame, cv::Mat loadeddepth)
{
    cv::Mat grayframe;
    cv::cvtColor(frame, grayframe, cv::COLOR_RGB2GRAY);
    cv::blur(grayframe, grayframe, cv::Size(3,3));
    cv::Canny(grayframe, grayframe, 50, 150, 3);

    vector<cv::Vec4i> ext_points_of_line;
    cv::HoughLinesP(grayframe, ext_points_of_line, 1, CV_PI/180, 100, 50, 10);
    for (size_t i=0;i<ext_points_of_line.size();i++)
    {
        cv::Point point1pixel = cv::Point(ext_points_of_line[i][0], ext_points_of_line[i][1]);
        cv::Point point2pixel = cv::Point(ext_points_of_line[i][2], ext_points_of_line[i][3]);
        if(loadeddepth.at<ushort>(point1pixel) * 0.001 < 2.5 && loadeddepth.at<ushort>(point1pixel) != 0
            && loadeddepth.at<ushort>(point2pixel) * 0.001 < 2.5 && loadeddepth.at<ushort>(point2pixel) !=0)
        {
            line( frame, cv::Point(ext_points_of_line[i][0], ext_points_of_line[i][1]), cv::Point(ext_points_of_line[i][2], ext_points_of_line[i][3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::circle(frame, point1pixel, 5, cv::Scalar(255,0,0) );
            cv::circle(frame, point2pixel, 5, cv::Scalar(255,0,0) );
        }
    }

//    cv::imshow("test1", frame);
//    cv::waitKey(60);
}

void featureextract(cv::Mat &frame, cv::Mat loadeddepth)
{
    getedge(frame, loadeddepth);
}

void getedge(cv::Mat &frame, cv::Mat loadeddepth)
{
    cv::Mat gray;
    cv::Mat temp (frame.size(), CV_8UC3, cv::Scalar(255, 255, 255));;
    cv::cvtColor(frame, gray, CV_BGRA2GRAY);
    cv::GaussianBlur(gray, gray, cvSize(5, 5), 0);
    cv::Canny(gray, gray, 60, 120);
    vector<cv::Point> edgepoints;
    cv::findNonZero(gray, edgepoints);
    cv::Scalar color( rand()&255, rand()&255, rand()&255 );
    for(auto &pixel:edgepoints)
    {
        if(loadeddepth.at<ushort>(pixel)*0.001<8.0 && loadeddepth.at<ushort>(pixel)*0.001!=0.0)
        {
            cv::circle(frame, pixel, 0.5, color );
            depth_calculate(pixel, loadeddepth);
        }
        if(loadeddepth.at<ushort>(pixel)*0.001<8.0)
        {
            cv::circle(temp, pixel, 0.5, cv::Scalar (0,0,0));
        }

    }
    edgepoints.clear();

    cv::imshow("test1", gray);
    cv::waitKey(60);
    cv::imshow("test2", frame);
    cv::waitKey(60);
    cv::imshow("test3", temp);
    cv::waitKey(60);
}






