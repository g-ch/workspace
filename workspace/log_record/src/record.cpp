#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include <mavros/State.h>
#include <mavros/SetPointLocal.h>
#include <mavros/Vector3.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <math.h>  
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <time.h> 
#include <stdio.h> 

using Eigen::MatrixXd;

MatrixXd St_matrix(3,3);//(p,v,a)
int mode = 0;
bool flag_p = false;
bool flag_v = false;
bool flag_a = false;
float time_stamp = 0.0;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_imu_data(const sensor_msgs::Imu &msg);
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg);
void chatterCallback_Mode(const mavros::State &msg);

int main(int argc, char **argv)  
{
	ros::init(argc, argv, "record");

	ros::NodeHandle nh; 
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 500,chatterCallback_local_position);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 500,chatterCallback_imu_data);
    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/local_velocity", 500,chatterCallback_local_velocity);
    ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 100,chatterCallback_Mode);
    //imitate data
    //ros::Subscriber localposition_sub = nh.subscribe("/offboard/position_imitate", 500,chatterCallback_local_position);
    //ros::Subscriber imu_sub = nh.subscribe("/offboard/acceleration_imitate", 500,chatterCallback_imu_data);
    //ros::Subscriber velocity_sub = nh.subscribe("/offboard/velocity_imitate", 500,chatterCallback_local_velocity);
    
    St_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;
    time_t tt = time(NULL);
    tm* t= localtime(&tt);
    char name[20];
    sprintf(name,"%d-%02d-%02d-%02d:%02d:%02d.txt",
      t->tm_year + 1900,
      t->tm_mon + 1,
      t->tm_mday,
      t->tm_hour,
      t->tm_min,
      t->tm_sec);

    char path[50]="/home/chg/catkin_ws/log/";
    strcat(path,name);
    std::cout<<"file saved in "<<path;
    FILE *pTxtFile = NULL;

    pTxtFile = fopen(path, "w+");
    if (pTxtFile == NULL)
    {
        printf("Open file failed! The program exist!\n");
        return 0;
    }
    
    std::cout<<"writing...\n";
    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(flag_p&&flag_v&&flag_a){
           fprintf(pTxtFile,"%f %d %f %f %f %f %f %f %f %f %f\n", time_stamp, mode,
        	 St_matrix(0,0),St_matrix(1,0),St_matrix(2,0),St_matrix(0,1),St_matrix(1,1),St_matrix(2,1),St_matrix(0,2),St_matrix(1,2),St_matrix(2,2));
           time_stamp += 0.1;
        }

    	ros::spinOnce();  
    	loop_rate.sleep();
    }
    
    fclose(pTxtFile);
	return 0;
}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
	St_matrix(0,0) = msg.pose.position.x;
	St_matrix(1,0) = msg.pose.position.y;
	St_matrix(2,0) = msg.pose.position.z;
	flag_p = true;
}

void chatterCallback_imu_data(const sensor_msgs::Imu &msg)
{
	St_matrix(0,2) = msg.linear_acceleration.x;
	St_matrix(1,2) = msg.linear_acceleration.y;
	St_matrix(2,2) = msg.linear_acceleration.z;
	flag_a = true;
}

void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg)
{
	St_matrix(0,1) = msg.x;
	St_matrix(1,1) = msg.y;
	St_matrix(2,1) = msg.z;
	flag_v = true;
}

void chatterCallback_Mode(const mavros::State &msg)//模式
{
    if(msg.mode=="MANUAL") mode=1;
    else if(msg.mode=="OFFBOARD") mode=7;
    else if(msg.mode=="POSCTR") mode=3;
    else mode = 5;
}