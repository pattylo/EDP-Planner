#include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>

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
        Move,
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

Eigen::MatrixXd _polyCoeff;
Eigen::VectorXd _polyTime;
Eigen::Vector3d _startPos  = Eigen::Vector3d::Zero();
Eigen::Vector3d _startVel  = Eigen::Vector3d::Zero();
Eigen::Vector3d _startAcc  = Eigen::Vector3d::Zero();
Eigen::Vector3d _endVel    = Eigen::Vector3d::Zero();
Eigen::Vector3d _endAcc    = Eigen::Vector3d::Zero();
double _Vel = 2.0;
double _Acc = 2.0;
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
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
    double vx = velocity->twist.linear.x;
    double vy = velocity->twist.linear.y;
    double vz = velocity->twist.linear.z;
    v = sqrt(vx*vx+vy*vy+vz*vz);
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
}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];

}

int main(int argc, char **argv)
{
    cout<<"Movement..."<<endl;
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

    ros::Publisher pub_traj_pts = nh.advertise<geometry_msgs::PoseStamped>
                                  ("mavros/setpoint_position/local", 10);
    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",10);
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);


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
    int i=0,j=0,k=0,l=0,m=0,n=0;
    double end_x=0, end_y=0, end_z=0;
    double yaw = M_PI/2;

    waypts hoverpoint1={0,0,0}, hoverpointTO={0,0,1.4};
    waypts localstartpt, localendpt;

    waypts wp1={0,0,1.4},
           wp11={ 4,-3,5},
           wp2={ 6,-4,7},
           wp3={3,-2,4},
           wp4={8,3,9},
           wp5={10,6, 4},
           wp6={3,8 ,4};
    vector<waypts> waypoints;
    waypoints.push_back(wp1);
    //waypoints.push_back(wp11);
    //waypoints.push_back(wp2);
    waypoints.push_back(wp3);
    waypoints.push_back(wp4);
    waypoints.push_back(wp5);
    waypoints.push_back(wp6);


//    -0.2324 0.171211 1.20477
//        -0.2324 0.171211 1.20477
//        -1.60596 0.501374 1.22521
//        -2.0115 0.94327 1.20896
//        -1.49701 1.54919 1.15412
//        -1.5646 1.62284 1.15141

//    -1.63012, 1.58157, 1.14723
//                           -1.63012, 1.58157, 1.14723
//                     -2.60993, 2.73856, 0.656373
//                     -3.01043, 3.18532, 0.659438
//                     -3.10729, 3.22937, 1.15854
//                     -3.17403, 3.30383, 1.15905





    getwaypts(waypoints);
    MJTG_execute(_polyCoeff, _polyTime);

    edge_points.header.frame_id = "/map";
    edge_points.header.stamp = ros::Time::now();
    edge_points.ns = "GT_points";
    edge_points.id = 0;
    edge_points.action = visualization_msgs::Marker::ADD;
    edge_points.pose.orientation.w = 1.0;
    edge_points.type = visualization_msgs::Marker::SPHERE_LIST;
    edge_points.scale.x = edge_points.scale.y = edge_points.scale.z = 0.05;
    std_msgs::ColorRGBA color_for_edge;
    edge_points.color.a=1;
    edge_points.color.g=0;
    edge_points.color.r=1;
    edge_points.color.b=0;



    for (int i=0;i<trajectory.size();i++)
    {
        geometry_msgs::Point pt;
        pt.x = trajectory[i].x;
        pt.y = trajectory[i].y;
        pt.z = trajectory[i].z;
        edge_points.points.push_back(pt);
    }

    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_point";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;

    line_list.points.clear();

    for(int i = 0; i < waypoints.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = waypoints[i].x;
        p.y = waypoints[i].y;
        p.z = waypoints[i].z;

        points.points.push_back(p);
        line_list.points.push_back(p);
    }





    cout<<trajectory.size()<<endl;
    waypts previous;

    movement move(waypoints);
    bool findwayptsuccess;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {
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

        //move~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == TO2Hover && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), localstartpt, localendpt);
            if(move.switchflymode_==true)
            {
                last_request = ros::Time::now().toSec();
                fly = Move;
            }
        }
        if(fly == Move)
        {
//            cout<<"now fly"<<endl;
            double x,y,z;
            x=trajectory[i].x;
            y=trajectory[i].y;
            z=trajectory[i].z;
//            cout<<x<<endl;
//            cout<<y<<endl;
//            cout<<z<<endl;

            pose.pose.position.x=trajectory[i].x;
            pose.pose.position.y=trajectory[i].y;
            pose.pose.position.z=trajectory[i].z;
            if(i<trajectory.size())
                i++;
            else fly = HoverandSway;
        }
        if(fly == HoverandSway)
            move.hover(pose, trajectory[trajectory.size()-1]);


        previous = {uavinfo.x, uavinfo.y, uavinfo.z};
        pub_traj_pts.publish(pose);
        rviz_visual.publish(edge_points);
        rviz_visual.publish(points);
        rviz_visual.publish(line_list);
//        cout<<v<<endl;

        ros::spinOnce();
        rate.sleep();
        last_request_=ros::Time::now().toSec();
    }
    return 0;
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
    if(true)
    {
//        visualization_msgs::Marker _traj_vis;

//        _traj_vis.header.stamp       = ros::Time::now();
//        _traj_vis.header.frame_id    = "/map";

//        _traj_vis.ns = "traj_node/trajectory_waypoints";
//        _traj_vis.id = 0;
//        _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
//        _traj_vis.action = visualization_msgs::Marker::ADD;
//        _traj_vis.scale.x = _vis_traj_width;
//        _traj_vis.scale.y = _vis_traj_width;
//        _traj_vis.scale.z = _vis_traj_width;
//        _traj_vis.pose.orientation.x = 0.0;
//        _traj_vis.pose.orientation.y = 0.0;
//        _traj_vis.pose.orientation.z = 0.0;
//        _traj_vis.pose.orientation.w = 1.0;

//        _traj_vis.color.a = 1.0;
//        _traj_vis.color.r = 1.0;
//        _traj_vis.color.g = 0.0;
//        _traj_vis.color.b = 0.0;

//        double traj_len = 0.0;
//        int count = 0;
//        Vector3d cur, pre;
//        cur.setZero();
//        pre.setZero();

//        _traj_vis.points.clear();
//        Vector3d pos;
//        geometry_msgs::Point pt;
    }

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


inline void visWayPointTraj()
{
//    visualization_msgs::Marker _traj_vis;

//    _traj_vis.header.stamp       = ros::Time::now();
//    _traj_vis.header.frame_id    = "/map";

//    _traj_vis.ns = "traj_node/trajectory_waypoints";
//    _traj_vis.id = 0;
//    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
//    _traj_vis.action = visualization_msgs::Marker::ADD;
//    _traj_vis.scale.x = 2.5;
//    _traj_vis.scale.y = 2.5;
//    _traj_vis.scale.z = 2.5;
//    _traj_vis.pose.orientation.x = 0.0;
//    _traj_vis.pose.orientation.y = 0.0;
//    _traj_vis.pose.orientation.z = 0.0;
//    _traj_vis.pose.orientation.w = 1.0;

//    _traj_vis.color.a = 1.0;
//    _traj_vis.color.r = 1.0;
//    _traj_vis.color.g = 0.0;
//    _traj_vis.color.b = 0.0;
//    geometry_msgs::Point pt;
//    for(int i=0;i<trajectory.size();i++)
//    {
//        pt.x = trajectory[i].x;
//        pt.y = trajectory[i].y;
//        pt.z = trajectory[i].z;
//        _traj_vis.points.push_back(pt);
//    }
//    _wp_traj_vis_pub.publish(_traj_vis);

}

