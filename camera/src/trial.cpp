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
#include <opencv2/video/tracking.hpp>
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
#include <opencv2/features2d.hpp>

#include "visualization_msgs/Marker.h"
#include <stdio.h>


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
void findcontour(cv::Mat &frame, cv::Mat loadeddepth); //with find contour


static visualization_msgs::Marker UAVtrajectory;
static visualization_msgs::Marker edge_points, p_point;
static visualization_msgs::Marker final_publish;

const int trackbarmax = 500;
int v1=0, v2=1;

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
    //    cout<<"hi"<<endl;
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
//    cv::imshow("test", frame);
//    cv::waitKey(20);
    find_IOU(frame, image_dep);
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
}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

static void on_trackbar1( int varing1, void* )
{
    v1 = varing1;
}
static void on_trackbar2( int varying2, void* )
{
    v2 = varying2;
}

int main(int argc, char** argv)
{
    cout<<"Extracting features..."<<endl;

    ros::init(argc, argv, "feature");
    ros::NodeHandle nh;
//    cv::Mat frame;
//    frame = cv::imread("/home/patrick/Pictures/test.png");
//    cv::imshow("2",frame);
//    cv::waitKey(0);


//    cout<<"hi"<<endl;
//    cv::Mat grayframe;
//    vector<cv::Point2f> whichpixelhascorner;

//    cv::cvtColor(frame, grayframe, cv::COLOR_RGB2GRAY);
//    cv::goodFeaturesToTrack(grayframe, whichpixelhascorner, 500, 0.01, 10, cv::noArray(), 3, false, 0.04);

//    for (size_t i=0; i<whichpixelhascorner.size(); i++)
//    {
//        cv::Point whichpixel = whichpixelhascorner[i];
//        cv::circle(frame, whichpixel, 4, cv::Scalar(255,255,0) );
//    }
//    cv::imshow("test",frame);
//    cv::waitKey(0);

//    cv::namedWindow("canny", cv::WINDOW_AUTOSIZE);
//    int localv1;
//    cv::createTrackbar("canny_with_thres_v1", "canny", &localv1, 250, on_trackbar1);
//    on_trackbar1(localv1,0);
//    int localv2;
//    cv::createTrackbar("canny_with_thres_v2", "canny", &localv2, 250, on_trackbar2);
//    on_trackbar2(localv2,0);


    //below subscribe the image and the depth map of the camera
//    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 10);
//    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 10);
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
//    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber sub_uabposition = nh.subscribe<geometry_msgs::PoseStamped>
                                      ("/mavros/local_position/pose", 10, position_callback);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);


    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",10);
    cv::Mat frame;
//    cv::VideoCapture input("/home/patrick/catkin_ws/src/offb/src/include/yolo/gazebo.avi");
//    input>>frame;
//    while(ros::ok())
//    {
//        cv::imshow("hey here", frame);
//        double fps = input.get(cv::CAP_PROP_FPS);
//        cout<<fps<<endl;
//        cout<<frame.size()<<endl;
//        cv::waitKey(20);
//        ros::spinOnce();
//    }
    string obj_file = "/home/patrick/camera_ws/src/camera/src/include/tracklog.txt";

    ifstream class_file(obj_file);
    if (!class_file)
    {
        cerr << "failed to open classes.txt\n";
    }

    string line;
    double values;
    vector<double> xyzs;
    while (getline(class_file, line))
    {
        values = atof(line.c_str());
        xyzs.push_back(values);
        cout<<"hi1:"<<endl;
    }

    int indicator = 60000;
    vector<geometry_msgs::Point> traj;
    geometry_msgs::Point traj_pt;
    cout<<indicator<<endl;
    while(indicator<=79000)
    {

        if(indicator%3 == 0)
        {
            traj_pt.x = xyzs[indicator];
        }
        if(indicator%3 == 1)
        {
            traj_pt.y = xyzs[indicator];
        }
        if(indicator%3 == 2)
        {
            traj_pt.z = xyzs[indicator];
        }
        if(indicator%3==0 )
            traj.push_back(traj_pt);
        indicator++;
    }

    string p = "/home/patrick/camera_ws/src/camera/src/include/p.txt";

    ifstream pclass_file(p);
    if (!pclass_file)
    {
        cerr << "failed to open classes.txt\n";
    }

    string pline;
    double pvalues;
    vector<double> pxyzs;
    while (getline(pclass_file, line))
    {
        //cout<<"hi"<<endl;
        values = atof(line.c_str());
        //cout<<values<<endl;
        pxyzs.push_back(values);
    }

    indicator = 232543;
    vector<geometry_msgs::Point> ptraj;
    geometry_msgs::Point ptraj_pt;
//    cout<<indicator<<endl;
    while(indicator<=257372)
    {
//        cout<<indicator<<endl;
        if(indicator%3 == 0)
        {
            ptraj_pt.x = pxyzs[indicator];
        }
        if(indicator%3 == 1)
        {
            ptraj_pt.y = pxyzs[indicator];
        }
        if(indicator%3 == 2)
        {
            ptraj_pt.z = pxyzs[indicator];
        }
        if(indicator%3==0 )
            ptraj.push_back(ptraj_pt);
        indicator++;
    }




    for(int i =0;i<traj.size();i++)
    {
        edge_points.header.frame_id = "/map";
        edge_points.header.stamp = ros::Time::now();
        edge_points.ns = "GT_points";
        edge_points.id = 0;
        edge_points.action = visualization_msgs::Marker::ADD;
        edge_points.pose.orientation.w = 1.0;
        edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
        edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.04;
        std_msgs::ColorRGBA color_for_edge;
        color_for_edge.a = 1;
        color_for_edge.g = 0;
        color_for_edge.r = 1;
        color_for_edge.b = 0;
        edge_points.points.push_back(traj[i]);
        edge_points.colors.push_back(color_for_edge);

    }
    for(int j=0;j<ptraj.size();j++)
    {
        edge_points.header.frame_id = "/map";
        edge_points.header.stamp = ros::Time::now();
        edge_points.ns = "GT_points";
        edge_points.id = 0;
        edge_points.action = visualization_msgs::Marker::ADD;
        edge_points.pose.orientation.w = 1.0;
        edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
        edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.04;
        std_msgs::ColorRGBA color_for_p;
        color_for_p.a = 1;
        color_for_p.g = 1;
        color_for_p.r = 1;
        color_for_p.b = 0;
        edge_points.points.push_back(ptraj[j]);
        edge_points.colors.push_back(color_for_p);
    }







    if(true)
    {
//    edge_points.header.frame_id = "/map";
//    edge_points.header.stamp = ros::Time::now();
//    edge_points.ns = "GT_points";
//    edge_points.id = 0;
//    edge_points.action = visualization_msgs::Marker::ADD;
//    edge_points.pose.orientation.w = 1.0;
//    edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
//    edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.05;
//    std_msgs::ColorRGBA color_for_edge;
//    edge_points.color.a=1;
//    edge_points.color.g=0;
//    edge_points.color.r=1;
//    edge_points.color.b=0;

//    cout<<traj.size()<<endl;
//    double distance = 0;
//    geometry_msgs::Point previous;
//    previous.x = 0;
//    previous.y = 0;
//    previous.z = 0;

//    for (int i=0;i<traj.size();i++)
//    {
//        double delta_s;
//        cout<<traj[i].x<<" "<<traj[i].y<<" "<<traj[i].z<<" "<<endl;
//        edge_points.points.push_back(traj[i]);

//        delta_s = sqrt( pow(traj[i].x-previous.x,2) + pow(traj[i].y-previous.y,2) + pow(traj[i].z-previous.z,2) );
//        distance = distance + delta_s;
//        previous = traj[i];

//    }

//    cout<<endl<<endl<< "Transverse length: "<<distance<<" m"<<endl;

//    visualization_msgs::Marker points, line_list;
//    int id = 0;
//    points.header.frame_id    = line_list.header.frame_id    = "/map";
//    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
//    points.ns                 = line_list.ns                 = "wp_point";
//    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
//    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
//    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
//    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
//    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

//    points.id    = id;
//    line_list.id = id;

//    points.type    = visualization_msgs::Marker::SPHERE_LIST;
//    line_list.type = visualization_msgs::Marker::LINE_STRIP;

//    points.scale.x = 0.3;
//    points.scale.y = 0.3;
//    points.scale.z = 0.3;
//    points.color.a = 1.0;
//    points.color.r = 1.0;
//    points.color.g = 0.0;
//    points.color.b = 0.0;

//    line_list.scale.x = 0.15;
//    line_list.scale.y = 0.15;
//    line_list.scale.z = 0.15;
//    line_list.color.a = 1.0;
//    line_list.color.r = 0.0;
//    line_list.color.g = 1.0;
//    line_list.color.b = 0.0;

//    line_list.points.clear();
//    geometry_msgs::Point wp0, wp1, wp2, wp3;

//    wp0.x = 0;
//    wp0.y = 0;
//    wp0.z = 1.2;

//    wp1.x = 0;
//    wp1.y = 14;
//    wp1.z = 1.2;

//    wp2.x = -10;
//    wp2.y = 7;
//    wp2.z = 1.2;

//    wp3.x = -5;
//    wp3.y = 0;
//    wp3.z =1.2;

//    points.points.push_back(wp0);
//    points.points.push_back(wp1);
//    points.points.push_back(wp2);
//    points.points.push_back(wp3);
//    points.points.push_back(wp0);


//    line_list.points.push_back(wp0);
//    line_list.points.push_back(wp1);
//    line_list.points.push_back(wp2);
//    line_list.points.push_back(wp3);
//    line_list.points.push_back(wp0);

    }






    int i=0;
    int k=0;

    ros::Rate rate(20.0);
    while(ros::ok())
    {


//        edge_points.header.frame_id = "/map";
//        edge_points.header.stamp = ros::Time::now();
//        edge_points.ns = "GT_points";
//        edge_points.id = 0;
//        edge_points.action = visualization_msgs::Marker::ADD;
//        edge_points.pose.orientation.w = 1.0;
//        edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
//        edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.05;
//        std_msgs::ColorRGBA color_for_edge;
//        color_for_edge.a = 1;
//        color_for_edge.g = 0;
//        color_for_edge.r = 1;
//        color_for_edge.b = 0;
//        std_msgs::ColorRGBA color_for_p;
//        color_for_p.a = 1;
//        color_for_p.g = 1;
//        color_for_p.r = 1;
//        color_for_p.b = 0;


//        k+=1;
//        if(k<ptraj.size() && i < traj.size())
//        {
//            edge_points.points.push_back(traj[i]);
//           // edge_points.points.push_back(ptraj[k]);
//            edge_points.colors.push_back(color_for_edge);
//            //edge_points.colors.push_back(color_for_p);
//        }
//                i+=1;
//                k+=1;




        /*







        line_list.scale.x = 0.15;
        line_list.scale.y = 0.15;
        line_list.scale.z = 0.15;
        line_list.color.a = 1.0;
        line_list.color.r = 0.0;
        line_list.color.g = 1.0;
        line_list.color.b = 0.0;

        line_list.points.clear();
        geometry_msgs::Point wp0, wp1, wp2, wp3;

        wp0.x = 0;
        wp0.y = 0;
        wp0.z = 1.2;

        wp1.x = 0;
        wp1.y = 14;
        wp1.z = 1.2;

        wp2.x = -10;
        wp2.y = 7;
        wp2.z = 1.2;

        wp3.x = -5;
        wp3.y = 0;
        wp3.z =1.2;

        points.points.push_back(wp0);
        points.points.push_back(wp1);
        points.points.push_back(wp2);
        points.points.push_back(wp3);
        points.points.push_back(wp0);


        line_list.points.push_back(wp0);
        line_list.points.push_back(wp1);
        line_list.points.push_back(wp2);
        line_list.points.push_back(wp3);
        line_list.points.push_back(wp0);*/









        rviz_visual.publish(edge_points);
        rviz_visual.publish(p_point);

        //rviz_visual.publish(points);
        //rviz_visual.publish(line_list);
        rate.sleep();
        ros::spinOnce();
    }

//    while(ros::ok())
//    {
//        rviz_visual.publish(edge_points);
//        //        rviz_visual.publish(UAVtrajectory);
//        ros::spinOnce();
//        rate.sleep();
//    }
    return 0;
}

void find_IOU(cv::Mat &frame, cv::Mat &loadeddepth)
{
    cv::Mat tempframe = frame;
    vector<cv::Point> furthest;
    vector<cv::Point> furthest2;
    featureextract(tempframe, loadeddepth);
}

void getcorner(cv::Mat &frame, cv::Mat loadeddepth) //with good feature
{
    cv::Mat grayframe;
    vector<cv::Point2f> whichpixelhascorner;

    cv::cvtColor(frame, grayframe, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(grayframe, whichpixelhascorner, 50, 0.01, 10, cv::noArray(), 3, false, 0.04);

    for (size_t i=0; i<whichpixelhascorner.size(); i++)
    {
        cv::Point whichpixel = whichpixelhascorner[i];
        if(loadeddepth.at<ushort>(whichpixel) != 0)
        {
            cv::circle(frame, whichpixel, 4, cv::Scalar(255,255,0) );
        }
    }

    //    cv::imshow("test1", frame);
    //    cv::waitKey(60);

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
        if(/*loadeddepth.at<ushort>(point1pixel) * 0.001 < 4.0 &&*/ loadeddepth.at<ushort>(point1pixel) != 0
            /*&& loadeddepth.at<ushort>(point2pixel) * 0.001 < 4.0*/ && loadeddepth.at<ushort>(point2pixel) !=0)
        {
            line( frame, cv::Point(ext_points_of_line[i][0], ext_points_of_line[i][1]), cv::Point(ext_points_of_line[i][2], ext_points_of_line[i][3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::circle(frame, point1pixel, 4, cv::Scalar(255,0,0) );
            cv::circle(frame, point2pixel, 4, cv::Scalar(255,0,0) );
        }
    }

    //    cv::imshow("test1", frame);
    //    cv::waitKey(60);
}

void featureextract(cv::Mat &frame, cv::Mat loadeddepth)
{
    cv::Mat temp;

    temp = frame;
//    cv::imshow("original", temp);
//    cv::waitKey(60);

    temp = frame;
    findcontour(temp, loadeddepth);
//    getcorner(temp, loadeddepth);



}

void findcontour(cv::Mat &frame, cv::Mat loadeddepth) //with find contour
{

    vector<cv::Point> nonzeroo;
    vector<double> interestedx, interestedy;
    cv::Mat white(loadeddepth.rows, loadeddepth.cols, CV_8UC3, cv::Scalar(0,0,0));
    cv::findNonZero(loadeddepth, nonzeroo);
    for(auto temp:nonzeroo)
    {
        double tempdepth = loadeddepth.at<ushort>(temp)*0.001;
        if(tempdepth<1.2)
        {
            white.at<cv::Vec3b>(temp)={255,255,255};
            interestedx.push_back(temp.x);
            interestedy.push_back(temp.y);
        }
    }
    int a = accumulate(interestedx.begin(), interestedx.end(), 0.0)/interestedx.size();
    int b = accumulate(interestedy.begin(), interestedy.end(), 0.0)/interestedy.size();

    cv::Point cg(a,b);


//    white = ~white;
    cv::Mat grey, blur, threshold_output;
    cv::cvtColor(white,grey, cv::COLOR_RGB2GRAY);
    cv::blur(grey, blur, cv::Size(3,3));
    cv::threshold(blur,threshold_output,50,255,cv::THRESH_BINARY);
    vector< vector<cv::Point>> contours;
    vector<cv::Vec4i> h;
    cv::findContours(threshold_output,contours,h,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    vector<vector<cv::Point>> hull(contours.size());

    for(int i=0; i<contours.size();i++)
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    cv::Mat draw = cv::Mat::zeros(threshold_output.size(), CV_8UC3);

    for(int i=0;i<contours.size();i++)
    {
        cv::Scalar color_contours = cv::Scalar(0, 255, 0); // green - color for contours
        cv::Scalar color = cv::Scalar(255, 0, 0); // blue - color for convex hull
        // draw ith contour
        drawContours(draw, contours, i, color_contours, 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
        // draw ith convex hull
        drawContours(draw, hull, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
    }



    cout<<cg<<endl;

    cv::circle(white,cg,10.0,cv::Scalar(0,255,0),-1);
    cv::imshow("frame", frame);
    cv::waitKey(20);
    cv::imshow("intersted", white);
    cv::waitKey(20);
    cv::imshow("test", draw);
    cv::waitKey(20);



}






