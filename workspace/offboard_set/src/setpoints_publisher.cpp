#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include "/home/chg/catkin_ws/devel/include/mavros/SetPointLocal.h"
#include <sstream>  

void chatterCallback_LocalPosition(const geometry_msgs::PoseStamped &msg);
void set_new_point(float x, float y, float z, float yaw, float t);
bool near_bool(float x, float y);

mavros::SetPointLocal setpoint;
bool ready_for_next = false;
int close_counter= 0;

int main(int argc, char **argv)  
{
	ros::init(argc, argv, "setpoints_publisher");  
  
    ros::NodeHandle nh;  
  
    ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 500,chatterCallback_LocalPosition);
      
    set_new_point(2.0, 1.0, 3.0, 0.0, 2.0);
    set_new_point(5.0, 1.0, 3.0, 0.0, 1.0);


    return 0;

}

void set_new_point(float x, float y, float z, float yaw, float t)
{
	
	ready_for_next = false;
	setpoint.x = x;
	setpoint.y = y;
	setpoint.z = z;
	setpoint.yaw = yaw;
    
    ros::NodeHandle n;
	ros::Publisher setpoints_pub = n.advertise<mavros::SetPointLocal>("offboard/setpoints_local", 500);
    
    int rest_counter = 0;
    int max = (int)(t*10);
	ros::Rate loop_rate(10);
	while(ros::ok()){
    	if(ready_for_next) rest_counter+=1;
        if(rest_counter >= max) break;

        setpoints_pub.publish(setpoint);
        ROS_INFO("%f %d", setpoint.x,rest_counter);
    	ros::spinOnce();  
    	loop_rate.sleep();
    }
}

void chatterCallback_LocalPosition(const geometry_msgs::PoseStamped &msg)
{
	if(near_bool(setpoint.x, msg.pose.position.x)&&near_bool(setpoint.y, msg.pose.position.y)&&
		near_bool(setpoint.z, msg.pose.position.z))
		close_counter += 1;
	else close_counter = 0;

	if(close_counter >= 3)
	    ready_for_next = true;
}

bool near_bool(float x, float y)
{
	if(x-y<0.5 && x-y>-0.5)
		return true;
	else return false;
}