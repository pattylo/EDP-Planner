#include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include "include/run_yolo.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"



using namespace std;

enum flying_step
{
    IDLE,
    TO2Hover,
    Hover,
    Prep,
    Starto,
    Move2wp,
    Change,
    Avoid,
    HoverandSway,
    Land,
    gothrough,
    found,
    HoverSwaySearch1,
    HoverSwaySearch2,
    HoverSwaySearch3,
    locate,
    gotowp1,
    gotowp2,
    gotowp3,
    sethead,
    Prep2,
};

static flying_step  fly = IDLE;
static UAVpose uavinfo;
static mavros_msgs::State current_state;
static int indicator = 0;
static visualization_msgs::Marker UAVtrajectory;
static vector<waypts> waypoints;
static vector<waypts> referpts;
static vector<waypts> futurepts;
static cv::Mat frame;
static cv::Mat image_dep;
static visualization_msgs::Marker edge_points;
static cv::Rect whichframe_fp;
static double v;
static double predictdepth;
static double obs_w;
const double r_safe = 1.2;

bool checkavoidance(movement move, waypts start, waypts end);
cv::Rect getpredictedframe(cv::Point whichpixel, vector<double> cameracoordinate);
vector<double> calculatecameracoordinate(Eigen::Matrix<double, 4, 1> futurepoint);
cv::Point calculatewhichpixel(vector<double> cameracoordinate);
Eigen::Matrix<double,4,1> depth_calculate(cv::Point whichpixel, double average);
waypts FWP();
waypts getnewwaypoint(cv::Point finalreferencept, bool verticalornot);
void savereferencepoint(waypts refpts);
void savefuturepoint(waypts futurepts);
vector<cv::Point> filter(vector<cv::Point> selecededgepts, int vorh);
cv::Point finalselection(vector<cv::Point> tobefinalized);
cv::Point getCG();
void getobs_w();

Eigen::MatrixXd _polyCoeff;
Eigen::VectorXd _polyTime;
Eigen::Vector3d _startPos  = Eigen::Vector3d::Zero();
Eigen::Vector3d _startVel  = Eigen::Vector3d::Zero();
Eigen::Vector3d _startAcc  = Eigen::Vector3d::Zero();
Eigen::Vector3d _endVel    = Eigen::Vector3d::Zero();
Eigen::Vector3d _endAcc    = Eigen::Vector3d::Zero();
double _Vel = 0.5;
double _Acc = 0.5;
int _dev_order = 3;
int _min_order = 3;
int _poly_num1D = 2 * _dev_order;
vector<waypts> trajectory;
ros::Publisher _wp_traj_vis_pub;

void getwaypts(vector<waypts> wp);
void trajGeneration(Eigen::MatrixXd path);
Eigen::VectorXd timeAllocation( Eigen::MatrixXd Path);
void MJTG_execute( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);//for final movement
Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
void visWayPointTraj();

waypts c2w(Eigen::Matrix<double, 3, 1> camera_pt);
waypts b2w(Eigen::Matrix<double, 3, 1> body_pt);
vector<waypts> avoidwps(waypts c_gotten);
void TG(vector<waypts> selectedwps);

void IIR(double & y, double x);

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;    
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
    ofstream save("/home/patrick/catkin_ws/src/offb/src/flydata/searchuav.txt", ios::app);
    save<<uavinfo.x<<endl;
    save<<uavinfo.y<<endl;
    save<<uavinfo.z<<endl;
    save.close();
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
    double vx = velocity->twist.linear.x;
    double vy = velocity->twist.linear.y;
    double vz = velocity->twist.linear.z;
    v = sqrt(vx*vx+vy*vy+vz*vz);

    ofstream savevx("/home/patrick/catkin_ws/src/offb/src/flydata/searchuav_vx.txt", ios::app);
    ofstream savevy("/home/patrick/catkin_ws/src/offb/src/flydata/searchuav_vy.txt", ios::app);
    ofstream savevz("/home/patrick/catkin_ws/src/offb/src/flydata/searchuav_vz.txt", ios::app);

    savevx<<vx<<endl;
    savevy<<vy<<endl;
    savevz<<vz<<endl;
    savevx.close();
    savevy.close();
    savevz.close();

}

void rpy_to_Q(double yaw, double &w, double &x, double &y, double &z)
{
    w = cos(0) * cos (0) * cos (yaw/2) + sin (0) * sin (0) * sin (yaw/2) ;
    x = sin(0) * cos (0) * cos (yaw/2) - cos (0) * sin (0) * sin (yaw/2) ;
    y = cos(0) * sin (0) * cos (yaw/2) + sin (0) * cos (0) * sin (yaw/2) ;
    z = cos(0) * cos (0) * sin (yaw/2) - sin (0) * sin (0) * cos (yaw/2) ;
}


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
    //cout<<frame.size()<<endl;

}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
//    cout<<fx<<endl;
//    cout<<fy<<endl;
//    cout<<cx<<endl;
//    cout<<cy<<endl;
}

static double obj_x_c = 0, obj_y_c = 0, obj_z_c = 0;

void obj_info_cb(const geometry_msgs::PointStampedConstPtr& msg)
{
    obj_x_c = msg->point.x;
    obj_y_c = msg->point.y;
    obj_z_c = msg->point.z;
//    cout<<obj_x_c<<endl;
//    cout<<obj_y_c<<endl;
//    cout<<obj_z_c<<endl;
}

static bool foundornot;

void found_cb(const std_msgs::Bool::ConstPtr& msg)
{
    foundornot = msg->data;
    //cout<<"found:"<<foundornot<<endl;
}


int main(int argc, char **argv)
{
    cout<<"Movement..."<<endl;
    cv::VideoWriter video("/home/patrick/catkin_ws/src/offb/src/include/yolo/gazebo.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(640,360));
    ros::init(argc, argv, "offb_node"); //initialize ROS via passing argc argv. the "offb_node" will be the name of your node's name
    ros::NodeHandle nh;//the node handle to handle the process of the node. it'll as well intialize the node.

    //the followings are the instantiation of either subscriber, publisher, service & client
    //it tells the ROS master that what we're gonna execute in this node

    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_callback);
    ros::Subscriber sub_uabposition = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("/mavros/local_position/pose", 10, position_callback);
    ros::Subscriber sub_velocity = nh.subscribe<geometry_msgs::TwistStamped>
                                      ("/mavros/local_position/velocity_local", 10, velocity_callback);

    ros::Subscriber sub_obj = nh.subscribe<geometry_msgs::PointStamped>
                              ("/pose_camera", 10, obj_info_cb);
    ros::Subscriber sub_found= nh.subscribe<std_msgs::Bool>
                                ("/obj_found", 10, found_cb);

    ros::Publisher pub_traj_pts = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",10);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 10);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);

    geometry_msgs::PoseStamped pose;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
//    for(int i = 100; ros::ok() && i > 0; --i){
//        pub_traj_pts.publish(pose);
//        ros::spinOnce();
//        rate.sleep();
//    }

    //mode setting
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    double last_request = ros::Time::now().toSec();
    double last_request_= ros::Time::now().toSec();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int i=0,j=0,k=0,l=0,m=0,n=0, traj_i=20;
    double end_x=0, end_y=0, end_z=0;
    double yaw = M_PI/2;
    int step=0;

    waypts localstartpt, localendpt, finalpt = {-0,-11,1.2};
    waypts hoverpoint1 = {0,0,0}, hoverpointTO = {0,0,1.2}, hoverpoint3 = {-11,2,1.2};
    waypts wpt1 = {0,14,1.2}, wpt2 = {-10,7,1.2}, wpt3={-5,0,1.2}, wpt4 = {0,0,1.2};
//{4, 3,1.2}

    waypoints.push_back(hoverpoint1);
    waypts previous;

    movement move(waypoints);
    bool findwayptsuccess;
    waypts finalentry = {0,0,0};


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {

        if(!frame.empty())
            video.write(frame);
        //preparation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == IDLE)
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now().toSec();
            }
            else
            {
                if( !current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        fly=TO2Hover;
                        localstartpt = hoverpoint1;
                        localendpt = hoverpointTO;
                    }
                    last_request = ros::Time::now().toSec();
                }
            }
        }

        //move preperation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == TO2Hover && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), localstartpt, localendpt);
            if(move.switchflymode_==true)
            {
                last_request = ros::Time::now().toSec();
                finalpt = wpt1;
                fly = HoverandSway;
            }
        }

        if(fly == HoverandSway && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()) )
        {
            move.setorientation(uavinfo,pose,last_request,ros::Time::now(),hoverpointTO,finalpt);
            if(move.switchflymode_==true)
            {
                cout<<"change"<<endl;
                last_request = ros::Time::now().toSec();
                fly = Prep;
            }

        }


        if(fly == sethead)
        {

            move.setorientation(uavinfo,pose,last_request,ros::Time::now(),localstartpt,localendpt);
            if(move.switchflymode_==true)
            {
                cout<<"change"<<endl;
                last_request = ros::Time::now().toSec();
                fly = Prep2;
            }

        }

        if(fly == Prep && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()) )
        {
            if((ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
            {
                localstartpt = hoverpointTO;
                localendpt = finalpt;
                last_request = ros::Time::now().toSec();
                fly = Move2wp;
            }
        }

        if(fly == Prep2 && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()) )
        {
            if((ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
            {
                last_request = ros::Time::now().toSec();
                fly = Move2wp;
            }
        }

        //start move ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == Move2wp)
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), localstartpt, localendpt);
//            move.setorientation(localstartpt, localendpt, pose);
            double t0 = ros::Time::now().toSec();
            bool incoming = checkavoidance(move, localstartpt, localendpt);
            double t1 = ros::Time::now().toSec();
            cout<<endl<<endl<<endl<<endl;
            cout<<"check avoidance ms: "<<t1-t0<<endl<<endl<<endl<<endl;
            ofstream save("/home/patrick/catkin_ws/src/offb/src/flydata/ms_check_a.txt", ios::app);
            double t=t1-t0;
            save<<t<<endl;
            save.close();


//            waypts newlocalendpt = FWP(frame);
            if(incoming)
            {
                cout<<"incoming"<<endl<<endl;
                fly = Change;
            }
            else
            {
//                cout<<"safe"<<endl<<endl;
                if(move.switchflymode_==true)
                {
                    cout<<"point reahced"<<endl;
                    waypts temp = {uavinfo.x, uavinfo.y, uavinfo.z};
                    localstartpt = temp;
                    localendpt = finalpt;
                    double delta = sqrt(pow(temp.x-finalpt.x,2)+pow(temp.y-finalpt.y,2)+pow(temp.z-finalpt.z,2));
                    if(delta<0.5)
                    {
                        fly = gotowp2;//modify this line for setting FSM
                        last_request = ros::Time::now().toSec();
                    }
                }
            }
        }

        if(fly == gotowp2)
        {
            if(step == 0)
            {
                localstartpt = {uavinfo.x, uavinfo.y, uavinfo.z};
                localendpt = wpt2;
                finalpt = wpt2;
                fly = sethead;
            }
            else if(step == 1)
            {
                localstartpt = {uavinfo.x, uavinfo.y, uavinfo.z};
                localendpt = wpt3;
                finalpt = wpt3;

                fly = sethead;
            }
            else if(step == 2)
            {
                localstartpt = {uavinfo.x, uavinfo.y, uavinfo.z};
                localendpt = wpt4;
                finalpt = wpt4;
                fly = sethead;
            }
            else
            {
                fly = HoverSwaySearch1;
            }
            step ++;
        }

        if(fly == Change)
        {
            waypts newlocalstartpt = {uavinfo.x, uavinfo.y, uavinfo.z};
            double t0 = ros::Time::now().toSec();
            waypts newlocalendpt = FWP();
            double t1 = ros::Time::now().toSec();

            cout<<endl<<endl<<endl<<endl;
            cout<<"find waypoint ms: "<<t1-t0<<endl<<endl<<endl<<endl;
            ofstream save("/home/patrick/catkin_ws/src/offb/src/flydata/ms_get_wp.txt", ios::app);
            double t=t1-t0;
            save<<t<<endl;
            save.close();


            localstartpt = newlocalstartpt;
            localendpt = newlocalendpt;
            cout<<"changed!"<<endl;
//            cout<<newlocalstartpt.x<<endl;
//            cout<<newlocalstartpt.y<<endl;
//            cout<<newlocalstartpt.z<<endl;

//            cout<<newlocalendpt.x<<endl;
//            cout<<newlocalendpt.y<<endl;
//            cout<<newlocalendpt.z<<endl<<endl;
            move.resetindicator(true);
            fly = Avoid;
        }

        if(fly == Avoid)
        {
            getobs_w();
            vector<waypts> seriesofwps = avoidwps(localendpt);
            double t0 = ros::Time::now().toSec();
            TG(seriesofwps);
            double t1 = ros::Time::now().toSec();
            cout<<endl<<endl<<endl<<endl;
            cout<<"find tg ms: "<<t1-t0<<endl<<endl<<endl<<endl;
            ofstream save("/home/patrick/catkin_ws/src/offb/src/flydata/ms_get_MJ.txt", ios::app);
            double t=t1-t0;
            save<<t<<endl;
            save.close();
            fly = gothrough;
        }

        if(fly == gothrough)
        {
            cout<<"trajectory:"<<trajectory.size()<<endl;
            cout<<traj_i<<endl<<endl;

            pose.pose.position.x = trajectory[traj_i].x;
            pose.pose.position.y = trajectory[traj_i].y;
            pose.pose.position.z = trajectory[traj_i].z;
            traj_i++;
            if(traj_i == trajectory.size())
            {
                waypts temp = {uavinfo.x, uavinfo.y, uavinfo.z};
                localstartpt = temp;
                localendpt = finalpt;
                fly = Move2wp;
                traj_i = 20;
                trajectory.clear();
            }
        }

        if(fly == HoverSwaySearch1)
        {
            cout<<"now sway"<<endl;
            move.sway(uavinfo,pose);
            if(foundornot)
            {
                fly = locate;
                last_request = ros::Time::now().toSec();
            }
            if(ros::Time::now().toSec() - last_request > ros::Duration(30).toSec())
                fly = Hover;

        }

        if(fly == locate)
        {


            if(ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec())
            {
                fly = Hover;
            }
        }

        if(fly == Hover)
        {
            double z = obj_z_c; // the distance between center of the object is surface depth + object size
            double x = z * (obj_x_c - cx)/fx;  //pixel coordinate u,v -> camera coordinate x,y
            double y = z * (obj_y_c - cy)/fy;

            Eigen::Matrix<double,3,1> fromcameranode;
            fromcameranode(0) = x;
            fromcameranode(1) = y;
            fromcameranode(2) = z;
            waypts locationofpooh = c2w(fromcameranode);
            IIR(finalentry.x, locationofpooh.x);
            IIR(finalentry.y, locationofpooh.y);
            IIR(finalentry.z, locationofpooh.z);

            ofstream save("/home/patrick/catkin_ws/src/offb/src/flydata/searchpooh.txt",ios::app);
            save<<finalentry.x<<endl;
            save<<finalentry.y<<endl;
            save<<finalentry.z<<endl;
            save.close();

            ofstream savep("/home/patrick/catkin_ws/src/offb/src/flydata/cameraframepooh.txt",ios::app);
            savep<<obj_x_c<<endl;
            savep<<obj_y_c<<endl;
            savep<<obj_z_c<<endl;
            savep.close();
            cout<<obj_x_c<<endl;
            cout<<obj_y_c<<endl;
            cout<<obj_z_c<<endl;
            cout<<"Hover"<<endl;
            move.hover(pose, finalpt);
            if(ros::Time::now().toSec() - last_request > ros::Duration(20).toSec())
            {
                break;
            }
        }



        previous = {uavinfo.x, uavinfo.y, uavinfo.z};
        pub_traj_pts.publish(pose);
        rviz_visual.publish(edge_points);
//        cout<<v<<endl;
//        if(edge_points.points.size()>=50000)
//        {
//            edge_points.points.clear();
//            edge_points.colors.clear();
//            cout<<"hi clear"<<endl;
//        }

        ros::spinOnce();
        rate.sleep();
        last_request_=ros::Time::now().toSec();
    }

    return 0;
}

bool checkavoidance(movement move, waypts start, waypts end)
{
//    cout<<endl<<"check"<<endl;
    Eigen::Matrix<double,4,1> predictedpoint;

    predictedpoint = move.futurepoint(start, end);//after few seconds, I will be at predictedpoint
    vector<double> cameracoordinate = calculatecameracoordinate(predictedpoint);
    predictdepth = cameracoordinate[2];
    cv::Point futureptonframe = calculatewhichpixel(cameracoordinate);//and then transfer them on cv::frame
    cv::Rect whichframe = getpredictedframe(futureptonframe, cameracoordinate);//and the UAV's size frame

    cv::Mat ROI(image_dep, whichframe);
    cv::Mat ROIframe;
    ROI.copyTo(ROIframe);
    vector<cv::Point> nonzeros;
    cv::findNonZero(ROIframe, nonzeros);
//    cout<<"findnonzero in check"<<endl;
    for(auto &pixel : nonzeros)
    {
        if(ROIframe.at<ushort>(pixel)*0.001 < cameracoordinate[2])
            return true;
    }
    return false;
}

vector<double> calculatecameracoordinate(Eigen::Matrix<double, 4, 1> futurepoint)
{
    Eigen::Matrix<double, 4, 1> cam, body, world;
    Eigen::Matrix<double, 4, 1> offset(0.12,0,0,0);
    Eigen::Matrix<double, 4, 4> cam_to_body;
    cam_to_body <<
        0,0,1,0,
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

    world = futurepoint;
    cam = cam_to_body.inverse() * (body_to_world.inverse() * world - offset);
    //cout<<futurepoint<<endl;

    double x = cam(0),y = cam(1), z = cam(2);
    //cout<<body_to_world<<endl;
    cout<<x<<endl;
    cout<<y<<endl;
    cout<<z<<endl<<endl;
    vector<double> cameracoordinate;
    cameracoordinate.push_back(x);
    cameracoordinate.push_back(y);
    cameracoordinate.push_back(z);

    return cameracoordinate;
}

cv::Point calculatewhichpixel(vector<double> cameracoordinate)
{
    double x = cameracoordinate[0], y = cameracoordinate[1], z = cameracoordinate[2];
    int pixelx, pixely;
    pixelx = int(x * fx / z + cx);
    pixely = int(y * fy / z + cy);

    return cv::Point(pixelx, pixely);
}

cv::Rect getpredictedframe(cv::Point whichpixel, vector<double> cameracoordinate)
{
    double deltax_w = 0.6, deltay_w = 0.3;

    int deltax_onframe = int(round(deltax_w/cameracoordinate[2] * fx));
    int deltay_onframe = int(round(deltay_w/cameracoordinate[2] * fy));
//    cout<<deltax_onframe<<endl;
//    cout<<deltay_onframe<<endl<<endl;

//    cv::Rect rectforcheck(cv::Point(), frame.size());
    int ptx, pty, ptx_, pty_;

    if(whichpixel.x-deltax_onframe/2 > frame.cols)
    {
        ptx = frame.cols;
    }
    else if(whichpixel.x-deltax_onframe/2 < 0)
    {
        ptx = 4;
    }
    else
        ptx = whichpixel.x-deltax_onframe/2;

    if(whichpixel.y-deltay_onframe/2 > frame.rows)
    {
        pty = frame.rows;
    }
    else if(whichpixel.y-deltay_onframe/2 < 0)
    {
        pty = 4;
    }
    else
        pty = whichpixel.y-deltay_onframe/2;

    if(whichpixel.x+deltax_onframe/2 > frame.cols)
    {
        ptx_ = frame.cols-4;
    }
    else if(whichpixel.x+deltax_onframe/2 < 0)
    {
        ptx_ = 1;
    }
    else
        ptx_ = whichpixel.x+deltax_onframe/2;

    if(whichpixel.y+deltay_onframe/2 > frame.rows)
    {
        pty_ = frame.rows-4;
    }
    else if(whichpixel.y+deltay_onframe/2 < 0)
    {
        pty_ = 1;
    }
    else
        pty_ = whichpixel.y+deltay_onframe/2 ;

    cv::Point pt(ptx, pty), pt_(ptx_, pty_);

    cv::Rect temp(pt, pt_);
    whichframe_fp = temp;
    cv::Mat input_frame;
    frame.copyTo(input_frame);
    cv::rectangle(input_frame, cv::Point(whichframe_fp.x, whichframe_fp.y),cv::Point(whichframe_fp.x+whichframe_fp.width, whichframe_fp.y+whichframe_fp.height),cv::Scalar(0,255,0),1);
    cv::imshow("predicting", input_frame);
    cv::waitKey(20);
    cv::imshow("predictingg", frame);
    cv::waitKey(20);
    cv::imwrite("/home/patrick/Pictures/testt.png",frame);

    return whichframe_fp;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`

waypts FWP()
{
    cv::imwrite("/home/patrick/Pictures/test.png",frame);
    cv::Mat white(image_dep.rows, image_dep.cols, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat tempframe = frame;
//    getedge(tempframe, image_dep);
    cv::Mat gray;
    cv::Mat temp (frame.size(), CV_8UC3, cv::Scalar(255, 255, 255));;
    cv::cvtColor(frame, gray, CV_BGRA2GRAY);
    cv::GaussianBlur(gray, gray, cvSize(5, 5), 0);
    cv::Canny(gray, gray, 10, 20);
    cv::imwrite("/home/patrick/Pictures/canny.png", gray);

    vector<cv::Point> edgepoints;
    vector<cv::Point> selectededgepts_h, selectededgepts_v;
    cv::findNonZero(gray, edgepoints);
    cv::Scalar color(rand()&255, rand()&255, rand()&255);

    double leftest = whichframe_fp.x + whichframe_fp.width/2 - 0.1 * whichframe_fp.width;
    double righest = whichframe_fp.x + whichframe_fp.width/2 + 0.1 * whichframe_fp.width;
    double uppest = whichframe_fp.y + whichframe_fp.height/2 - 0.1 * whichframe_fp.height;
    double lowest = whichframe_fp.y + whichframe_fp.height/2 + 0.1 * whichframe_fp.height;

    cout<<1<<endl;
    for(auto &pixel : edgepoints)//get cross edgepoints
    {
        if(image_dep.at<ushort>(pixel) * 0.001 < 2.4 && (pixel.x > leftest && pixel.x < righest) )
        {
//            cv::circle(frame, pixel, 2.0, color);
            selectededgepts_v.push_back(pixel);
            white.at<cv::Vec3b>(pixel)={255,255,255};
        }
        if(image_dep.at<ushort>(pixel) * 0.001 < 2.4 &&(pixel.y > uppest && pixel.y < lowest ))
        {
//            cv::circle(frame, pixel, 2.0, color);
            selectededgepts_h.push_back(pixel);
            white.at<cv::Vec3b>(pixel)={255,255,255};
        }
    }
    cv::imwrite("/home/patrick/Pictures/cannywdwcross.png", white);
    cv::Mat temppp;
    white.copyTo(temppp);

    vector<cv::Point> nonzeroo;
    cv::findNonZero(image_dep, nonzeroo);
    for(auto temp:nonzeroo)
    {
        double tempdepth = image_dep.at<ushort>(temp)*0.001;
        if(tempdepth<2.4)
        {
            white.at<cv::Vec3b>(temp)={255,255,255};

        }
    }
    cv::imwrite("/home/patrick/Pictures/white.png",white);

    cout<<"size:"<<selectededgepts_h.size()<<endl;
    cout<<"size:"<<selectededgepts_v.size()<<endl<<endl;;
//    cv::imshow("test", frame);
//    cv::waitKey(30);
    cout<<2<<endl;
    vector<cv::Point> tobefinalized_v = filter(selectededgepts_v, 1);//filter out who has no potential for waypoint on frame
    cout<<3<<endl;
    vector<cv::Point> tobefinalized_h = filter(selectededgepts_h, 2);
    cout<<4<<endl;
    cout<<"size:"<<tobefinalized_h.size()<<endl;
    cout<<"size:"<<tobefinalized_v.size()<<endl<<endl;
    cv::Point finalpoint_onframe_v (2000,2000), finalpoint_onframe_h (2000,2000);

    //final selection of the points on frame by calculating the distance with the predicted frame

    if(tobefinalized_v.size() != 0)
    {
        finalpoint_onframe_v = finalselection(tobefinalized_v);
    }
    cout<<5<<endl;
    if(tobefinalized_h.size() != 0)
    {
        finalpoint_onframe_h = finalselection(tobefinalized_h);
    }
    cout<<6<<endl;

    edgepoints.clear();
    edgepoints.push_back(finalpoint_onframe_v);
    edgepoints.push_back(finalpoint_onframe_h);

    cv::Point finalpt_onframe = finalselection(edgepoints);
    cout<<7<<endl;

    bool verticalornot = false;

    if(finalpt_onframe == edgepoints[0])
    {
        //the final point is on the vertical of cross
        verticalornot = true;
    }
    cout<<8<<endl;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    waypts finall;
    cout<<finalpt_onframe<<endl;
    finall = getnewwaypoint(finalpt_onframe, verticalornot);
    cv::circle(frame,finalpt_onframe,8,CV_RGB(200,0,0),-1);
    cv::circle(white,finalpt_onframe,8,CV_RGB(200,0,0),-1);
    cv::circle(temppp,finalpt_onframe,8,CV_RGB(200,0,0),-1);

    cv::imwrite("/home/patrick/Pictures/framewwp.png",frame);
    cv::imwrite("/home/patrick/Pictures/whitewwp.png",white);
    cv::imwrite("/home/patrick/Pictures/whitetemppp.png",temppp);

    cout<<9<<endl;


    return finall;
}

waypts getnewwaypoint(cv::Point finalreferencept, bool verticalornot)
{
    double depth = image_dep.at<ushort>(finalreferencept);
    cout<<"hiendl"<<endl;
    double z = depth/1000,
           x = z * (finalreferencept.x - cx)/fx,
           y = z * (finalreferencept.y - cy)/fy;
    Eigen::Matrix<double, 4, 1> cam (x,y,z,1), body, body_, world, offset(0.12,0,0,0);
    Eigen::Matrix<double, 4, 4> cam_to_body;
    cam_to_body <<
        0,0,1,0,
        -1,0,0,0,
        0,-1,0,0,
        0,0,0,1;

    body = cam_to_body * cam + offset;
    body_ = cam_to_body * cam + offset;
    cv::Point CG = getCG();
    if(verticalornot)
    {
        if(finalreferencept.y < CG.y)
        {
            cout<<"turn up"<<endl;
            body(2) = body(2)+r_safe;
        }
        else
        {
            cout<<"turn dw"<<endl;
            body(2) = body(2)-r_safe;
        }
    }
    else
    {
        if(finalreferencept.x < CG.x)
        {
            cout<<"turn left"<<endl;
            body(1) = body(1)+r_safe;
        }
        else
        {
            cout<<"turn right"<<endl;
            body(1) = body(1)-r_safe;
        }
    }


    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * body;
    waypts returnvalue = {world(0), world(1), world(2)};
    cout<<"here"<<endl<< returnvalue.x<<endl;
    cout<< returnvalue.y<<endl;
    cout<< returnvalue.z<<endl;
    world = body_to_world * body_;
    cout<<"world here"<<endl;
    cout<< world(0)<<endl;
    cout<< world(1)<<endl;
    cout<< world(2)<<endl;;
    cout<<"body here"<<endl;
    cout<<body(0)<<endl;
    cout<<body(1)<<endl;
    cout<<body(2)<<endl<<endl;

    waypts returnvlue_c;
    returnvlue_c={body(0),body(1), body(2)};

    return returnvlue_c;
}

cv::Point getCG()
{
    vector<cv::Point> nonzeroo;

    vector<double> interestedx, interestedy;
    cv::findNonZero(image_dep, nonzeroo);
    cv::Mat white(image_dep.rows, image_dep.cols, CV_8UC3, cv::Scalar(0,0,0));

    for(auto temp:nonzeroo)
    {
        double tempdepth = image_dep.at<ushort>(temp)*0.001;
        if(tempdepth<2.4)
        {
            interestedx.push_back(temp.x);
            interestedy.push_back(temp.y);
            white.at<cv::Vec3b>(temp)={255,255,255};
        }
    }
    int a = accumulate(interestedx.begin(), interestedx.end(), 0.0)/interestedx.size();
    int b = accumulate(interestedy.begin(), interestedy.end(), 0.0)/interestedy.size();

    cv::Point cg(a,b);
    cv::circle(white,cg,10,CV_RGB(200,0,0),-1);
    cv::imwrite("/home/patrick/Pictures/whitecg.png",white);

    cout<<"cg: "<<cg<<endl;

    return cg;
}

void getobs_w()
{
    vector<cv::Point> nonzeroo;
     ;//= int(round(deltax_w/cameracoordinate[2] * fx));

    vector<double> interestedx, vector_z_c;
    cv::findNonZero(image_dep, nonzeroo);
    cv::Mat white(image_dep.rows, image_dep.cols, CV_8UC3, cv::Scalar(0,0,0));
    double ymax, ymin;
    ymin = whichframe_fp.y + 0.4 * whichframe_fp.height;
    ymax = whichframe_fp.y + 0.6 * whichframe_fp.height;
    for(auto temp:nonzeroo)
    {
        double tempdepth = image_dep.at<ushort>(temp)*0.001;
        if(tempdepth<2.4 && temp.y > ymin && temp.y < ymax)
        {
            interestedx.push_back(temp.x);
            vector_z_c.push_back(tempdepth);
        }
    }
    int max_i = max_element(interestedx.begin(),interestedx.end()) - interestedx.begin(),
        min_i = min_element(interestedx.begin(),interestedx.end()) - interestedx.begin();
    int deltax_onframe = interestedx[max_i] - interestedx[min_i];
    double z_c = accumulate(vector_z_c.begin(), vector_z_c.end(),0.0)/vector_z_c.size();;
    obs_w = deltax_onframe / fx * z_c;
    cout<<endl;
    cout<<deltax_onframe<<endl;
    cout<<fx<<endl;
    cout<<z_c<<endl;
    cout<<"obs_w: "<<obs_w<<endl;
}

vector<cv::Point> filter(vector<cv::Point> selecededgepts, int vorh) //filter out whose edgept surrounding has no accesible depth point
{
    int rs_onframe = 5;
    cv::Rect rectforcheck(cv::Point(), image_dep.size());
    vector<cv::Point> returnvector_v;
    vector<cv::Point> returnvector_h;

    if(vorh == 1)
    {
        for(auto pixel : selecededgepts)
        {
            double o = image_dep.at<ushort>(pixel) * 0.001;
            cv::Point u_, d_;
            u_ = cv::Point (pixel.x, pixel.y - rs_onframe);
            d_ = cv::Point (pixel.x, pixel.y + rs_onframe);
            double u, d;

            if(rectforcheck.contains(u_))
                u = image_dep.at<ushort>(u_) * 0.001;
            if(rectforcheck.contains(d_))
                d = image_dep.at<ushort>(d_) * 0.001;

//            if(o==0 || u==0 || d==0)
//                continue;

            if(abs(u-o)>0.1 || abs(d-o)>0.1)
                returnvector_v.push_back(pixel);
            else continue;
        }
        return returnvector_v;
    }

    if(vorh == 2)
    {
        for(auto pixel : selecededgepts)
        {
            double o = image_dep.at<ushort>(pixel) * 0.001;
            cv::Point r_, l_;
            r_ = cv::Point (pixel.x + rs_onframe, pixel.y);
            l_ = cv::Point (pixel.x - rs_onframe, pixel.y);
            double r, l;

//            if(o==0 || r==0 || l==0)
//                continue;

            if(rectforcheck.contains(r_))
                r = image_dep.at<ushort>(r_) * 0.001;
            if(rectforcheck.contains(l_))
                l = image_dep.at<ushort>(l_) * 0.001;
            if(abs(r-o)>0.1 || abs(l-o) >0.1)
                returnvector_h.push_back(pixel);
            else continue;
        }
        return returnvector_h;
    }
}

cv::Point finalselection(vector<cv::Point> tobefinalized)
{
    vector<double> temp;
    double tempvalue;
    double o_x = whichframe_fp.x + whichframe_fp.width/2;
    double o_y = whichframe_fp.y + whichframe_fp.height/2;
    for (auto pixel : tobefinalized)
    {
        tempvalue = pow(pixel.x - o_x,2) + pow(pixel.y - o_y,2);
        temp.push_back(tempvalue);
    }
    int minElementIndex = min_element(temp.begin(),temp.end()) - temp.begin();
    cv::Point returnvalue = tobefinalized[minElementIndex];
    return returnvalue;
}

Eigen::Matrix<double,4,1> depth_calculate(cv::Point whichpixel, double average)
{
    vector<double> edge_depth;
    double depth = average;
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
    return world;
}

void visualize_rviz(Eigen::Matrix<double, 4, 1> world)
{
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
    edge_points.points.push_back(EDGE);
    edge_points.colors.push_back(color_for_edge);
}

void getwaypts(vector<waypts> wp)
{
    vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.size(); k++)
    {
        Eigen::Vector3d pt( wp[k].x, wp[k].y, wp[k].z);
        wp_list.push_back(pt);
        //        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }

    Eigen::MatrixXd waypoints(wp_list.size() , 3);
    //    waypoints.row(0) = _startPos;

    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];
    trajGeneration(waypoints);
    //Trajectory generation: use minimum jerk/snap trajectory generation method
    //waypoints is the result of path planning (Manual in this project)
}

void trajGeneration(Eigen::MatrixXd path)
{
    Eigen::MatrixXd polyCoeff;
    ros::Time time_start = ros::Time::now();
    vector <waypts> temp;
    movement trajectoryGeneratorWaypoint(temp);

    Eigen::MatrixXd vel  = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc  = Eigen::MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    // cout << "_polyCoeff = " << endl;
    // cout << _polyCoeff << endl;

    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    //    visWayPointPath(path);    // visulize path
    //    visWayPointTraj( _polyCoeff, _polyTime);    // visulize trajectory

    //    Eigen::MatrixXd _polyCoeff_jerk;
    //    movement trajectoryGeneratorWaypoint_jerk(temp);
    //    _polyCoeff_jerk = trajectoryGeneratorWaypoint_jerk.PolyQPGeneration(_min_order, path, vel, acc, _polyTime);

    // cout << "_polyCoeff_jerk = " << endl;
    // cout << _polyCoeff_jerk << endl;
    ROS_WARN("Time consumed use minimum jerk trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);
    //    visWayPointPath_jerk(path);    // visulize path
    //    visWayPointTraj_jerk( _polyCoeff_jerk, _polyTime);    // visulize trajectory
}

Eigen::VectorXd timeAllocation( Eigen::MatrixXd Path)
{
    Eigen::VectorXd time(Path.rows() - 1);
    Eigen::VectorXd distance = Eigen::VectorXd::Zero(Path.rows()-1);
    for(int i = 0; i<Path.rows()-1;i++)
    {
        distance(i) = sqrt
            (
                pow(  (Path(i+1,0)-Path(i,0)) ,2)
                +pow(  (Path(i+1,1)-Path(i,1)) ,2)
                +pow(  (Path(i+1,2)-Path(i,2)) ,2)
                );
        double t1 = _Vel / _Acc;
        double t2 = distance(i)/_Vel - t1;
        time(i) = 2*t1 +t2 ;
    }
    return time;
}

void MJTG_execute( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)//for final movement
{
    Eigen::Vector3d pos, cur, pre;
    waypts traj_pt;
    int count = 0;
    double traj_len = 0.0;

    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {
        for (double t = 0.0; t < time(i); t += 0.05, count += 1)
        {
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = traj_pt.x = pos(0);
            cur(1) = traj_pt.y = pos(1);
            cur(2) = traj_pt.z = pos(2);
            trajectory.push_back(traj_pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }
}

Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t )
{
    Eigen:: Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( _poly_num1D );

        for(int j = 0; j < _poly_num1D; j ++)
            if(j==0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    // cout << "ret = " << endl;
    // cout << ret << endl;

    return ret;
}

vector<waypts> avoidwps(waypts c_gotten)
{
    waypts a,b,c,d,e,f;
    Eigen::Matrix<double, 3, 1> tempa, tempb, tempc, tempd, tempe, tempf;

    //a
    tempa(0) = 0;
    tempa(1) = 0;
    tempa(2) = 0.2;
    a = {uavinfo.x, uavinfo.y, uavinfo.z};

    //b
    tempa(0) = 0;
    tempa(1) = 0;
    tempa(2) = 0.2;
    b = c2w(tempb);

    //c
    tempc(0) = c_gotten.x;
    tempc(1) = c_gotten.y;
    tempc(2) = c_gotten.z;
    c = b2w(tempc);

    //d
    tempd(0) = c_gotten.x +  obs_w;
    tempd(1) = c_gotten.y;
    tempd(2) = c_gotten.z;
    d = b2w(tempd);

    //e
    tempe(0) = 0;
    tempe(1) = 0;
    tempe(2) = tempd(0) + 0.4 * obs_w;
    e = c2w(tempe);

    //f
    tempf(0) = 0;
    tempf(1) = 0;
    tempf(2) = tempd(0) +0.5 * obs_w;
    f = c2w(tempf);

    vector<waypts> returnvector;
    returnvector.push_back(a);
    returnvector.push_back(b);
    returnvector.push_back(c);
    returnvector.push_back(d);
    //returnvector.push_back(e);
    returnvector.push_back(f);
    cout<<"avoidance wps:"<<endl;
    cout<<a.x<<", "<<a.y<<", "<<a.z<<endl;
    cout<<b.x<<", "<<b.y<<", "<<b.z<<endl;
    cout<<c.x<<", "<<c.y<<", "<<c.z<<endl;
    cout<<d.x<<", "<<d.y<<", "<<d.z<<endl;
    cout<<e.x<<", "<<e.y<<", "<<e.z<<endl;
    cout<<f.x<<", "<<f.y<<", "<<f.z<<endl<<endl;


    return returnvector;
}

void TG(vector<waypts> selectedwps)
{
    getwaypts(selectedwps);
    MJTG_execute(_polyCoeff, _polyTime);
}

waypts c2w(Eigen::Matrix<double, 3, 1> camera_pt)
{
    double z = camera_pt(2),
           x = camera_pt(0),
           y = camera_pt(1);

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
    waypts world_pt = {world(0), world(1), world(2)};
    cout<<uavinfo.x<<endl;
    cout<<uavinfo.y<<endl;
    cout<<uavinfo.z<<endl;
    cout<<world_pt.x<<endl;
    cout<<world_pt.y<<endl;
    cout<<world_pt.z<<endl<<endl;;


    return world_pt;
}

waypts b2w(Eigen::Matrix<double, 3, 1> body_pt)
{
    Eigen::Matrix<double, 4, 1> body, world;
    body(0) = body_pt(0);
    body(1) = body_pt(1);
    body(2) = body_pt(2);
    body(3) = 1;
    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * body;
    waypts result = {world(0),world(1), world(2)};
    return result;

}

void IIR(double & y, double x)
{
    //    cout << "initial y:" << y << endl;
    //    cout << "initial x:" << x << endl;

    if (y == 0.0 && !isnan(x))
    {
        y = x;
        //        cout << " y == 0 after "  << y << endl;
    }
    else if (y != 0.0 ){
        y = 0.9 * y + 0.1 * x;
        //        cout << " y != 0 and " << y << endl;
    }
}
