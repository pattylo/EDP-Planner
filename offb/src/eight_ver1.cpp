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
#include <cmath>

using namespace std;

void movement(double &x, double &y, double &z, double destination_x, double destination_y, double destination_z,geometry_msgs::PoseStamped& pose_local, int i, int &fly_step, ros::Time &last_request_local, ros::Time instant_time);
void circle_no_heading(double &dg, double &rd, double &rd_end, double center_x, double center_y, geometry_msgs::PoseStamped& pose_local, int i, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time, double loop, bool clockwise);
void eight_no_heading(double &dg, double &rd,double &rd_end, double &center_x, double &center_y, geometry_msgs::PoseStamped& pose_local,  int &i_8, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time,  double times,  char pattern, double tilt, double radius_8, int &i, int &j);

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
    int i_8=0;
    double degree = 0; double radius = 0;double end_radius = 0;
    double enterx, entery;
    double end_x=0, end_y=0, end_z=0;
    bool clockwise = true;
    bool anticlockwise = false;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {
//preparation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      if(fly==0)
      {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(4.0)))
        {
          if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          {
            ROS_INFO("Offboard enabled");

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
              fly=1;
            }
            last_request = ros::Time::now();
          }
        }
      }

//preparation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//process~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 1)
      {
         movement(end_x, end_y, end_z, 0, 0, 4, pose, j, fly,last_request, ros::Time::now());
         j++;
      }
      if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 2)
      {
         movement(end_x, end_y, end_z, 2, 2,  4, pose, k, fly,last_request, ros::Time::now());
         k++;
      }
      if(ros::Time::now() - last_request > ros::Duration(2.0) && fly == 3)
      {
          eight_no_heading(degree, radius, end_radius, end_x, end_y, pose, i_8, fly, last_request, ros::Time::now(), 4, 'D', -M_PI/4, 2, l,m);
      }
      if(fly==4)
      {
          cout<<"end"<<endl;fly++;
      }

      /*
      if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 3)
      {
          circle_no_heading(degree, radius, end_radius, -7, -1, pose, l, fly,last_request, ros::Time::now(), 1, anticlockwise);
          l++;
      }
      if( fly == 4)
      {
          circle_no_heading(degree, radius, end_radius, 3, -1, pose, m, fly,last_request, ros::Time::now(), 1, clockwise);
          m++;
      }

      if( ros::Time::now() - last_request > ros::Duration(2.0) && fly == 3)
      {
        if(l==0)
        {
          cout<<"UAV now flying in circle...."<<endl;
          l++;
        }
        pose.pose.position.x = 5+2*cos(degree_x);
        degree_x = degree_x + 0.01;
        pose.pose.position.y = 2+2*sin(degree_y);
        degree_y = degree_y + 0.01;
      }*/

//process~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//landing~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
//landing~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}


//could be written into class later
void movement(double &x, double &y, double &z,double destination_x, double destination_y, double destination_z, geometry_msgs::PoseStamped& pose_local, int i, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time)
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
//the passing parameters are, respectively: instant x, y, z position, the destination coordinates, the publishing positions, an indicator for usage, the current fly step, the last request time and instant time

void circle_no_heading(double &dg, double &rd, double &rd_end, double center_x, double center_y, geometry_msgs::PoseStamped& pose_local, int i, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time, double loop, bool clockwise)
{
    double x_degree, y_degree, start_dg;
    if(i == 0)
    {
        rd = sqrt(pow((pose_local.pose.position.x - center_x), 2)+pow((pose_local.pose.position.y) - center_y, 2));

        x_degree = acos((pose_local.pose.position.x - center_x)/rd);
        y_degree = asin((pose_local.pose.position.y - center_y)/rd);

        if( y_degree >= 0)
        {
            start_dg = x_degree;
        }
        else if(y_degree < 0)
        {
            start_dg = -x_degree;
        }

        dg = start_dg;
        cout << "UAV now flying in circle, center("<<center_x<<", "<<center_y<<"), radius:"<<rd<<"....for "<<loop<<" times....start from dg = "<<start_dg<<endl;
        if(clockwise == false){rd_end = start_dg + loop*2*M_PI;}else{rd_end = start_dg + loop*2*(-M_PI);}
    }

    if(clockwise == false)
    {
        if(dg <=  rd_end)
        {
            pose_local.pose.position.x = center_x + rd * cos(dg);
            pose_local.pose.position.y = center_y + rd * sin(dg);
            dg = dg + 0.01;
        }else
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }else
    {
        if(dg >= rd_end)
        {
            pose_local.pose.position.x = center_x + rd * cos(dg);
            pose_local.pose.position.y = center_y + rd * sin(dg);
            dg = dg - 0.01;
        }else
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }


}
//the passing parameters are, respectively: the instant degree, the about-to-set center and final radius position, the publishing positions, an indication for usage, the current fly step, the last request time and instant time, the looping times and clockwise(anti)

void eight_no_heading(double &dg, double &rd,double &rd_end, double &center_x, double &center_y, geometry_msgs::PoseStamped& pose_local,  int &i_8, int &fly_mode, ros::Time &last_request_local, ros::Time instant_time,  double times,  char pattern, double tilt, double radius_8, int &i, int &j)
{
    if(pattern == 'A')
    {
        if(i_8 %2 == 0 && i_8 < 2 *times)
        {
            if(i==0)
            {
                center_x = pose_local.pose.position.x + radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y + radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, i, i_8, last_request_local, ros::Time::now(), 1, true);
            i=1;
        }

        if(i_8 %2 != 0 && i_8 <2 *times)
        {
            if(j==0)
            {
                center_x = pose_local.pose.position.x - radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y - radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, j, i_8, last_request_local, ros::Time::now(), 1, false);
            j=1;
        }
        if(i==1 && j==1 && i_8 %2 == 0 && i_8<2*times)
        {
            i=0;
            j=0;
        }
        if(i_8 >= 2*times)
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }

    if(pattern == 'B')
    {
        if(i_8 %2 == 0 && i_8 < 2 *times)
        {
            if(i==0)
            {
                center_x = pose_local.pose.position.x + radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y + radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, i, i_8, last_request_local, ros::Time::now(), 1, false);
            i=1;
        }

        if(i_8 %2 != 0 && i_8 <2 *times)
        {
            if(j==0)
            {
                center_x = pose_local.pose.position.x - radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y - radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, j, i_8, last_request_local, ros::Time::now(), 1, true);
            j=1;
        }
        if(i==1 && j==1 && i_8 %2 == 0 && i_8<2*times)
        {
            i=0;
            j=0;
        }
        if(i_8 >= 2*times)
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }

    if(pattern == 'C')
    {
        if(i_8 %2 == 0 && i_8 < 2 *times)
        {
            if(i==0)
            {
                center_x = pose_local.pose.position.x - radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y - radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, i, i_8, last_request_local, ros::Time::now(), 1, true);
            i=1;
        }

        if(i_8 %2 != 0 && i_8 <2 *times)
        {
            if(j==0)
            {
                center_x = pose_local.pose.position.x + radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y + radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, j, i_8, last_request_local, ros::Time::now(), 1, false);
            j=1;
        }
        if(i==1 && j==1 && i_8 %2 == 0 && i_8<2*times)
        {
            i=0;
            j=0;
        }
        if(i_8 >= 2*times)
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }

    if(pattern == 'D')
    {
        if(i_8 %2 == 0 && i_8 < 2 *times)
        {
            if(i==0)
            {
                center_x = pose_local.pose.position.x - radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y - radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, i, i_8, last_request_local, ros::Time::now(), 1, false);
            i=1;
        }

        if(i_8 %2 != 0 && i_8 <2 *times)
        {
            if(j==0)
            {
                center_x = pose_local.pose.position.x + radius_8 * cos(tilt);
                center_y = pose_local.pose.position.y + radius_8 * sin(tilt);
                cout<<i_8<<endl;
            }
            circle_no_heading(dg, rd, rd_end, center_x, center_y, pose_local, j, i_8, last_request_local, ros::Time::now(), 1, true);
            j=1;
        }
        if(i==1 && j==1 && i_8 %2 == 0 && i_8<2*times)
        {
            i=0;
            j=0;
        }
        if(i_8 >= 2*times)
        {
            fly_mode++;
            last_request_local = instant_time;
        }
    }
}
//the passing parameters are, respectively: the instant degree, the about-to-set center and final radius position, the publishing positions, an indication for the 8 trajectory, the current fly step, the last request time and instant time, the looping times, the 8-trajectory pattern and its tilting angle, the radius of the 8-trajectory, and 2 indications for usage
//the function is written based on the function circle_no_heading

