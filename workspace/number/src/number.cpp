#include "ros/ros.h"  
#include "std_msgs/Float32.h"  
#include <math.h>
#include <sstream>  

#define Pi 3.1416
/**  
 * This tutorial demonstrates simple sending of messages over the ROS system.  
 */  
int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "talker");  
  
  ros::NodeHandle n;  
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("chatter", 1000);  
  
  ros::Rate loop_rate(40);  
  
  int counter = 0; 

  while (ros::ok())  
  {  
    std_msgs::Float32 msg;
    
    msg.data = sin(counter/180.0*Pi);

    ROS_INFO("%f", msg.data);  
  
    chatter_pub.publish(msg);  
  
    ros::spinOnce();  
  
    loop_rate.sleep();  
    counter+=6;
    if(counter == 360) counter = 0; 
  }  
  
  
  return 0;  
}  
