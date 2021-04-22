/*structure of a mavros node
based upon the given offboard instance*/

//The following inclusions are the header files that is required
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <sstream>

using namespace std;

void movement(double &x, double &y, double &z, double destination_x, double destination_y, double destination_z,geometry_msgs::PoseStamped& pose_local, int i, int &fly_step, ros::Time &last_request_local, ros::Time instant_time);
double vectorx, vectory, vectorz;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double uavx,uavy,uavz;


void getposition(const geometry_msgs::PoseStamped::ConstPtr& pose){
    uavx = pose->pose.position.x;
    uavy = pose->pose.position.y;
    uavz = pose->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); //initialize ROS via passing argc argv. the "offb_node" will be the name of your node's name

    ros::NodeHandle nh;//the node handle to handle the process of the node. it'll as well intialize the node.


//the followings are the instantiation of either subscriber, publisher, service & client
//it tells the ROS master that what we're gonna execute in this node
	
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, getposition);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    bool force_start;
    nh.getParam("force_start", force_start);
    cout<<force_start;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //mode setting
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int fly=0,i=0,j=0,k=0,l=0,m=0,n=0;
    bool armed=false;
    bool offboard=false;


    double end_x=0, end_y=0, end_z=0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {
      if(fly==0)
      {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(4.0)))
        {
          if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          {
            ROS_INFO("Offboard enabled");
            offboard=true;
          }
          last_request = ros::Time::now();
          }
        else
        {
          if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(4.0)))
          {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
              ROS_INFO("Vehicle armed");
              armed=true;
            }
            last_request = ros::Time::now();
          }
        }
      }

      if(offboard && armed && fly==0)
      {
        fly=1;
      }

//let UAV hover to z=4
    if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 1)
    {      
       movement(end_x, end_y, end_z, 0, 0, 4, pose, j, fly,last_request, ros::Time::now());
       j++;
    }
    if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 2)
    {
       movement(end_x, end_y, end_z, 3, 4, 4, pose, k, fly,last_request, ros::Time::now());
       k++;
    }
    if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 3)
    {
       movement(end_x, end_y, end_z, -7, -3, 2, pose, l, fly,last_request, ros::Time::now());
       l++;
    }
    if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 4)
    {
       movement(end_x, end_y, end_z, 0, 0, 4, pose, m, fly,last_request, ros::Time::now());
       m++;
    }
//then landing
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (uavx-end_x < 0.1 && uavx-end_x >-0.1 && uavy-end_y < 0.1 && uavy-end_y >-0.1 && ros::Time::now() -last_request>ros::Duration (2.0) && fly==5)
    {
        cout<<"UAV now preparing to land...." <<endl;
        fly++;//landing parameter
        last_request=ros::Time::now();
    }

    if(ros::Time::now()-last_request >ros::Duration(2.0) && fly==6)
    {
        pose.pose.position.z=end_z-0.02;
        end_z=pose.pose.position.z;

        if(pose.pose.position.z < 0.05)
        {
            
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success )
            {
                cout << "UAV about to touch ground" << endl;
                cout << "Touched and end...."<< endl;
                return 0;//break the control UAV will land automatically
            }
         }
        if(n==0)
        {
            cout<<"UAV landing..."<<endl;
            n++;
        }
    }


        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}


//could be written into class later
void movement(double &x, double &y, double &z,double destination_x, double destination_y, double destination_z,geometry_msgs::PoseStamped& pose_local, int i, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time)
{
  if(i ==0 )
  {
    cout<<"UAV now moving to destination...., x="<<destination_x<<", y="<<destination_y<<", z="<<destination_z<<endl;
    vectorx=destination_x-x;
    vectory=destination_y-y;
    vectorz=destination_z-z;
  }

  if(x-destination_x<0.1 && x-destination_x>-0.1)
  {
    pose_local.pose.position.x = destination_x;
  }
  else
  {
    pose_local.pose.position.x = x+0.005*vectorx;
    x = pose_local.pose.position.x;
  }

  if(y-destination_y<0.1 && y-destination_y>-0.1)
  {
    pose_local.pose.position.y = destination_y;
  }
    else
  {
    pose_local.pose.position.y = y+0.005*vectory;
    y = pose_local.pose.position.y;
  }

  if(z-destination_z<0.1 && z-destination_z>-0.1)
  {
    pose_local.pose.position.z = destination_z;
  }
    else
  {
    pose_local.pose.position.z = z+0.005*vectorz;
    z = pose_local.pose.position.z;
  }

  if(x-destination_x<0.1 && x-destination_x>-0.1 && y-destination_y<0.1 && y-destination_y>-0.1 && z-destination_z<0.1 && z-destination_z>-0.1)
  {
    fly_mode++;
    last_request_local = instant_time;
  }

}




