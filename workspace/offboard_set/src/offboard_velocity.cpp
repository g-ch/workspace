#include "ros/ros.h"  
#include "geometry_msgs/TwistStamped.h" 
#include "mavros/CommandBool.h"
#include "mavros/SetMode.h"
#include <sstream>  
  
#define LOOP_RATE 4

int init_velocity(); 
int set_velocity(float x, float y, float z, float yr, float time);

geometry_msgs::TwistStamped msg;

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "offboard_velocity"); 

  ros::NodeHandle nh;  
  
  ros::Publisher offboard_pub_v = nh.advertise<geometry_msgs::TwistStamped>("offboard/velocity", 1000);  
  

  ros::Rate loop_rate(LOOP_RATE);
  int num = 0;
  int limit = 0;
  int counter = 0;
  while(ros::ok()){
    switch(num)
    {
      case 0:limit =init_velocity(); break;
      case 1:limit=set_velocity(0.0,0.0,1.0,0.0,2.0); break;
      case 2:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 3:limit=set_velocity(0.4,0.0,0.0,0.0,3.0); break;
      case 4:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 5:limit=set_velocity(0.0,-0.4,0.0,0.0,3.0); break;
      case 6:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 7:limit=set_velocity(-0.4,0.0,0.0,0.0,6.0); break;
      case 8:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 9:limit=set_velocity(0.0,0.4,0.0,0.0,3.0); break;
      case 10:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 11:limit=set_velocity(0.4,0.0,0.0,0.0,3.0); break;
      case 12:limit=set_velocity(0.0,0.0,0.0,0.0,1.0); break;

      case 13:limit=set_velocity(0.0,0.0,-1.0,0.0,2.0); break;
      default:limit=set_velocity(0.0,0.0,0.0,0.0,1.0);  break;

    }
    msg.header.stamp = ros::Time::now();
    counter+=1;
    if(counter >= limit)
    {
      counter = 0;
      num+=1;
    }
    ROS_INFO("%f %f %f",  msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z); 
    offboard_pub_v.publish(msg); 
    ros::spinOnce();
    loop_rate.sleep();
  } 
  
  
  return 0;  
}  

int init_velocity()
{
  //int counter = 0;
  //while(ros::ok()&&counter<=LOOP_RATE*3){
   //
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
    //ROS_INFO("%f",  msg.twist.linear.x); 
    //counter+=1;
    //offboard_pub_v.publish(msg); 
    //ros::spinOnce();
    //loop_rate.sleep();
  //}
    int limit = 10;
  return limit;   
}

int set_velocity(float x, float y, float z, float yr=0.0, float time=1.0)
{
    int limit = (int)(time/(1.0/LOOP_RATE));
  //int counter = 0;

  //while(ros::ok()&&counter<=limit){
    //msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = x*100;
    msg.twist.linear.y = y*100;
    msg.twist.linear.z = z;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = yr;

    //counter+=1;
    //offboard_pub_v.publish(msg);
    //ROS_INFO("%f",  msg.twist.linear.x); 
    //ros::spinOnce();
    //loop_rate.sleep();
  //}
  return limit;
}