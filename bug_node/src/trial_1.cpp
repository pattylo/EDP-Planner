#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include <algorithm>



using namespace std;

void post_movement(gazebo_msgs::SetModelState &objstate,double &i, double &j);
void rpy_to_Q(double yaw, double &w, double &x, double &y, double &z);
void setorientation(double &i, double &j, double &i_previous, double &j_previous, gazebo_msgs::SetModelState &objstate );
void getposition(ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo);
void move(int &indicator, double &i, double &j, double endx, double endy, gazebo_msgs::SetModelState &objstate, ros::Time &last_request);

int decide_current_situation();
int set_situation(double local1, double local2, double local3, double local4, double local5);

void go_straight(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double positionposting);
void turn_orientation_left(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo);
void turn_orientation_right(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo);

void wall_following(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &i, double &j);


double Q_to_rpy(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo);
double Q_to_rpy_for_straight(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &sine, double &cosine);
bool back_to_mline(double endx, double endy, double startx, double starty,ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo);
void calculate_dg_instant(ros::ServiceClient &client_g);

void wall_following_alternative(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &i, double &j, geometry_msgs::Twist &give_vel);





double vectorx, vectory, dg=0, dg_instant=0, distancex, distancey, distanceall;

double savelaserpoint[720];
double savelaserpoint_right[144];
double savelaserpoint_fright[144];
double savelaserpoint_front[144];
double savelaserpoint_fleft[144];
double savelaserpoint_left[144];



#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
   int count = 720;
   for (int i = 0; i < count; i++)
   {
     if(i>=0 && i<144)
     {
       savelaserpoint_right[i] = scan ->ranges[i];
     }
     else if(i>=144 && i<288)
     {
       savelaserpoint_fright[i-144] = scan ->ranges[i];
     }
     else if(i>=288 && i<432)
     {
       savelaserpoint_front[i-288] = scan ->ranges[i];
     }
     else if(i>=432 && i<576)
     {
       savelaserpoint_fleft[i-432] = scan ->ranges[i];
     }
     else if (i>=576 && i<720)
     {
       savelaserpoint_left[i-576] = scan ->ranges[i];
     }
   }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bug_node");

    ros::NodeHandle nh;

    ros::ServiceClient client_s = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState objstate;

    ros::ServiceClient client_g = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState objstateinfo;

    ros::Subscriber sensing = nh.subscribe<sensor_msgs::LaserScan>("/mybot/laser/scan",1, scanCallback);
    sensor_msgs::LaserScan sensorinfo;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mybot/cmd_vel",1);


    std::pair<double*, double*>minmax = std::minmax_element(begin(savelaserpoint), std::end(savelaserpoint));

    ros::Time last_request = ros::Time::now();

    ros::Rate rate(20.0);

    for(int i = 100; ros::ok() && i > 0; --i)
    {
      ros::spinOnce();
      rate.sleep();
    }

    double i=0, j=0, i_previous = 0, j_previous = 0, qw, qx, qy, qz;double degree =0, endx, endy, startx, starty;int anotherindicator=0;
    int indicator =0;int mode=0;int hahaindicator = 0;
    while(ros::ok())
    {
      getposition(client_g, objstateinfo);
           startx = 0;
           starty = 0;
           endx = 8;
           endy = -8;
           if(anotherindicator == 0)
           {
             i=startx;
             j=starty;
             anotherindicator++;
           }

           double local3;
           pair<double*, double*>minmax_front = minmax_element(begin(savelaserpoint_front), end(savelaserpoint_front));
           local3 = *(minmax_front.first);
           if(local3>8){local3 = 8;}

           if( mode == 0)
           {
             if(local3<0.64)
             {
               mode = 1;
               if(hahaindicator==0)
               {
                 last_request = ros::Time::now();
                 hahaindicator=0;
               }
             }
           }
           else if(mode ==1)
           {
             if(back_to_mline(endx, endy, startx, starty, client_g, objstateinfo) && ros::Time::now()-last_request > ros::Duration(16.0))
             {
               mode = 0;
               indicator = 0;
               if(hahaindicator!=0)
               {
                 last_request = ros::Time::now();
                 hahaindicator=0;
               }
             }
           }

           if(mode == 0)
           {
             move(indicator, i, j,endx, endy, objstate, last_request);
           }
           if(mode == 1)
           {
             wall_following(objstate,client_g, objstateinfo,i, j);
           }



           cout<<"x now at: "<<i<<endl<<"y now at: "<<j<<endl<<"attitude: " << dg_instant<<endl;



           client_s.call(objstate);


      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}

bool back_to_mline(double endx, double endy, double startx, double starty,ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo)
{
  getposition(client_g, objstateinfo);
  double distance_to_line;
  distance_to_line = (abs((endx-startx) * (starty-objstateinfo.response.pose.position.y) - (startx - objstateinfo.response.pose.position.x) * (endy-starty) ) / sqrt( pow((endx-startx) ,2)+pow( (endy-starty) ,2)));
  cout<<distance_to_line<<endl;
  if(distance_to_line<0.1)
  {
    return true;
  }
  else {
    return false;
  }
}

void wall_following(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &i, double &j)
{
  int current_situation = decide_current_situation();
  if(current_situation == 10)
  {
    turn_orientation_right(objstate, client_g, objstateinfo);
    go_straight(objstate, client_g, objstateinfo, 0.0085);
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
  else if(current_situation == 20)
  {
    turn_orientation_left(objstate, client_g, objstateinfo);
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
  else if(current_situation == 30)
  {
    go_straight(objstate, client_g, objstateinfo, 0.015);
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
}

void wall_following_alternative(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &i, double &j, geometry_msgs::Twist &give_vel)
{
  int current_situation = decide_current_situation();
  if(current_situation == 10)
  {
    give_vel.linear.x=0.02;
    give_vel.angular.z=0.03;
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
  else if(current_situation == 20)
  {
    give_vel.angular.z=-0.03;
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
  else if(current_situation == 30)
  {
    give_vel.linear.x=0.05;
    getposition(client_g, objstateinfo);
    i=objstateinfo.response.pose.position.x;
    j=objstateinfo.response.pose.position.y;
    calculate_dg_instant(client_g);
  }
}

void post_movement(gazebo_msgs::SetModelState &objstate,double &i, double &j)
{
  objstate.request.model_state.model_name = "mybot";
  objstate.request.model_state.pose.position.x = i;
  objstate.request.model_state.pose.position.y = j;
  objstate.request.model_state.pose.position.z = 0.0;
  objstate.request.model_state.reference_frame = "world";
}

void go_straight(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double positionposting)
{
  double ratiox, ratioy, sine, cosine;
  double yaw = Q_to_rpy_for_straight(objstate, client_g, objstateinfo, sine, cosine);


  ratiox = cosine;
  ratioy = sine;

  objstate.request.model_state.model_name = "mybot";
  objstate.request.model_state.pose.position.x = objstateinfo.response.pose.position.x + positionposting * ratiox;
  objstate.request.model_state.pose.position.y = objstateinfo.response.pose.position.y + positionposting* ratioy;
  objstate.request.model_state.pose.position.z = 0.0;
  objstate.request.model_state.reference_frame = "world";
}

void turn_orientation_left(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo)
{
  double yaw = Q_to_rpy(objstate, client_g, objstateinfo);
  objstate.request.model_state.model_name = "mybot";
  rpy_to_Q(yaw+0.01,objstate.request.model_state.pose.orientation.w, objstate.request.model_state.pose.orientation.x, objstate.request.model_state.pose.orientation.y, objstate.request.model_state.pose.orientation.z );
  objstate.request.model_state.reference_frame = "world";
}

void turn_orientation_right(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo)
{
  double yaw = Q_to_rpy(objstate, client_g, objstateinfo);
  objstate.request.model_state.model_name = "mybot";
  rpy_to_Q(yaw-0.008,objstate.request.model_state.pose.orientation.w, objstate.request.model_state.pose.orientation.x, objstate.request.model_state.pose.orientation.y, objstate.request.model_state.pose.orientation.z );
  objstate.request.model_state.reference_frame = "world";
}

void rpy_to_Q(double yaw, double &w, double &x, double &y, double &z)
{

   w = cos(0) * cos (0) * cos (yaw/2) + sin (0) * sin (0) * sin (yaw/2) ;
   x = sin(0) * cos (0) * cos (yaw/2) - cos (0) * sin (0) * sin (yaw/2) ;
   y = cos(0) * sin (0) * cos (yaw/2) + sin (0) * cos (0) * sin (yaw/2) ;
   z = cos(0) * cos (0) * sin (yaw/2) - sin (0) * sin (0) * cos (yaw/2) ;
}

double Q_to_rpy(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo)
{
  objstateinfo.request.model_name = "mybot" ;
  objstateinfo.request.relative_entity_name = "world";
  client_g.call(objstateinfo);


  double siny_cosp = 2 * (objstateinfo.response.pose.orientation.w * objstateinfo.response.pose.orientation.z + objstateinfo.response.pose.orientation.x * objstateinfo.response.pose.orientation.y);
  double cosy_cosp = 1 - 2 * (objstateinfo.response.pose.orientation.y * objstateinfo.response.pose.orientation.y + objstateinfo.response.pose.orientation.z * objstateinfo.response.pose.orientation.z);
  double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

double Q_to_rpy_for_straight(gazebo_msgs::SetModelState &objstate, ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo, double &sine, double &cosine)
{
  objstateinfo.request.model_name = "mybot" ;
  objstateinfo.request.relative_entity_name = "world";
  client_g.call(objstateinfo);


  double siny_cosp = 2 * (objstateinfo.response.pose.orientation.w * objstateinfo.response.pose.orientation.z + objstateinfo.response.pose.orientation.x * objstateinfo.response.pose.orientation.y);
  double cosy_cosp = 1 - 2 * (objstateinfo.response.pose.orientation.y * objstateinfo.response.pose.orientation.y + objstateinfo.response.pose.orientation.z * objstateinfo.response.pose.orientation.z);
  double yaw = atan2(siny_cosp, cosy_cosp);

  sine = siny_cosp;
  cosine = cosy_cosp;
  return yaw;
}

void setorientation(double &i, double &j, double &i_previous, double &j_previous, gazebo_msgs::SetModelState &objstate )
{
  double sine, cosine;
  vectorx = i_previous - i;
  vectory = j_previous - j;
  sine = vectory/ sqrt(pow(vectorx,2)+pow(vectory,2));
  cosine = vectorx/ sqrt(pow(vectorx,2)+pow(vectory,2));
  dg = atan2(sine, cosine);

}

void move(int &indicator, double &i, double &j, double endx, double endy, gazebo_msgs::SetModelState &objstate, ros::Time &last_request)
{
  if(indicator==0)
  {
    setorientation(i, j, endx, endy, objstate);
    distanceall = sqrt(pow(endx-i,2)+pow(endy-j,2));
    distancex = endx - i;
    distancey = endy - j;
    indicator++;
  }
  if(indicator != 0 && (i-endx > 0.02 || i-endx<-0.02 || j-endy >0.2 || j-endy <-0.02) )
  {

    if(dg_instant-dg>0.02 || dg_instant-dg< -0.02)
    {
      objstate.request.model_state.model_name = "mybot";
      rpy_to_Q(dg_instant, objstate.request.model_state.pose.orientation.w, objstate.request.model_state.pose.orientation.x, objstate.request.model_state.pose.orientation.y, objstate.request.model_state.pose.orientation.z);
      objstate.request.model_state.reference_frame = "world";
      if(dg>dg_instant)
      {
        dg_instant = dg_instant + 0.01;
      }
      else if (dg<dg_instant)
      {
        dg_instant = dg_instant - 0.01;
      }
      last_request = ros::Time::now();
    }
    else if(dg_instant-dg<0.02 || dg_instant-dg> -0.02)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        post_movement(objstate, i, j);
        i = i+ 0.02*distancex/distanceall;
        j = j+ 0.02*distancey/distanceall;
      }
    }
  }
  if(i-endx < 0.02 && i-endx>-0.02 && j-endy <0.2 && j-endy >=-0.02)
  {
    cout<<"now arrive at distination"<<endl;
    post_movement(objstate, endx, endy);
  }
}

void getposition(ros::ServiceClient &client_g, gazebo_msgs::GetModelState &objstateinfo)
{
  objstateinfo.request.model_name = "mybot" ;
  objstateinfo.request.relative_entity_name = "world";
  client_g.call(objstateinfo);
}

int decide_current_situation()
{
  double local1, local2, local3, local4, local5;

  pair<double*, double*>minmax_right = minmax_element(begin(savelaserpoint_right), end(savelaserpoint_right));
  local1 = *(minmax_right.first);
  if(local1>8){local1 = 8;}


  pair<double*, double*>minmax_fright = minmax_element(begin(savelaserpoint_fright), end(savelaserpoint_fright));
  local2 = *(minmax_fright.first);
  if(local2>8){local2 = 8;}

  pair<double*, double*>minmax_front = minmax_element(begin(savelaserpoint_front), end(savelaserpoint_front));
  local3 = *(minmax_front.first);
  if(local3>8){local3 = 8;}

  pair<double*, double*>minmax_fleft = minmax_element(begin(savelaserpoint_fleft), end(savelaserpoint_fleft));
  local4 = *(minmax_fleft.first);
  if(local4>8){local4 = 8;}

  pair<double*, double*>minmax_left = minmax_element(begin(savelaserpoint_left), end(savelaserpoint_left));
  local5 = *(minmax_left.first);
  if(local5>8){local5 = 8;}

  int getmode = set_situation(local1,local2,local3,local4,local5);


  if(getmode == 1)
  {
    return 10;
  }
  else if(getmode == 2)
  {
    return 20;
  }
  else if(getmode == 3)
  {
    return 30;
  }
  else
  {
    return 40;
  }

}

int set_situation(double local1, double local2, double local3, double local4, double local5)
{
  double d=2, dd=2;
  if(local2 > d && local3 > dd && local4 > d)//local2 fright, local3 front, local4 fleft
  {
    return 1;//no obstacle in the front
  }
  else if(local2 > d && local3 < dd && local4 > d)
  {
    return 2;//obstacle in the front
  }
  else if(local2 < d && local3 > dd && local4 > d)
  {
    return 3;//obstacle fright
  }
  else if(local2 > d && local3 > dd && local4 < d)
  {
    return 1;//obstacle fleft
  }
  else if(local2 < d && local3 < dd && local4 > d)
  {
    return 2;//obstacle front fright
  }
  else if(local2 > d && local3 < dd && local4 < d)
  {
    return 2;//obstacle front fleft
  }
  else if(local2 < d && local3 < dd && local4 < d)
  {
    return 2;//obstacle all
  }
  else if(local2 < d && local3 > dd && local4 < d)
  {
    return 1;//tunnel situation
  }
  else
  {
    cout<<"no idea"<<endl;return 4;
  }
}

void calculate_dg_instant(ros::ServiceClient &client_g)
{
  double sine, cosine;
  gazebo_msgs::SetModelState objstate;
  gazebo_msgs::GetModelState objstateinfo;
  client_g.call(objstateinfo);
  dg_instant = Q_to_rpy_for_straight(objstate, client_g, objstateinfo, sine, cosine);
}



