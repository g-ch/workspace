#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include <mavros/SetPointLocal.h>
#include <mavros/Vector3.h>
#include <mavros/State.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_extras/FieldSize.h>
#include <mavros_extras/FieldSizeConfirm.h>
#include <Eigen/Dense>
#include <sstream>
#include <math.h>  
#include <iostream>

#define LOOP_RATE 20
#define Pi 3.14159265

using Eigen::MatrixXd;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_imu_data(const sensor_msgs::Imu &msg);
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg);
void chatterCallback_Mode(const mavros::State &msg);
void chatterCallback_field_size(const mavros_extras::FieldSize &msg);

void set_new_point(float x, float y, float z, float yaw, float t);
bool near_bool(float x, float y);
void trajectory_Paras_generation_i(int num, float p0, float v0, float a0, float pf, float vf, float af, float T);
void trajectory_generation(float T,float pxf, float pyf, float pzf,float vxf, float vyf, 
	         float vzf, float axf, float ayf, float azf);
void field_2_setpoint(float l, float w, float h,int times, float yaw);

float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t);

mavros::SetPointLocal setpoint;
mavros::Vector3 acceleration_vector;
mavros::Vector3 velocity_vector;
mavros_extras::FieldSizeConfirm field_size_confirm_msg;

//paras for strategy without trajectory generation
bool ready_for_next = false;
int close_counter= 0;

bool p_received = false;
bool v_received = false;
bool a_received = false;
bool offboard_ready = false;
bool field_size_received = false;

MatrixXd S0_matrix(3,3);//(p,v,a)
MatrixXd St_matrix(3,3);//(p,v,a)
MatrixXd Paras_matrix(3,3);//(alfa,beta,gamma)
MatrixXd St_optimal_matrix(3,3);//(p,v,a)
MatrixXd j_optimal_matrix(1,3);//(jx,jy,jz)
MatrixXd init_p_matrix(1,3);
MatrixXd field_size_matrix(1,4);//(length, width, height, times)

double yaw = 0.0; //hudu
double init_yaw = 0.0;

int main(int argc, char **argv)  
{
	ros::init(argc, argv, "setpoints_publisher");  
    S0_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;
    St_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;

    field_size_confirm_msg.length = 0.0;
    field_size_confirm_msg.width = 0.0;
    field_size_confirm_msg.height = 0.0;
    field_size_confirm_msg.times = 0;
    field_size_confirm_msg.confirm = 0;

    ros::NodeHandle nh;  
   
    ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 500,chatterCallback_local_position);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 500,chatterCallback_imu_data);
    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/local_velocity", 1000,chatterCallback_local_velocity);
    ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 100,chatterCallback_Mode);
    ros::Subscriber field_sub = nh.subscribe("/mavros/field_size_receiver/field_size_receiver", 100,chatterCallback_field_size);
    ros::Publisher field_size_confirm_pub = nh.advertise<mavros_extras::FieldSizeConfirm>("field_size_confirm", 100);
    //imitate data
    //ros::Subscriber localposition_sub = nh.subscribe("/offboard/position_imitate", 500,chatterCallback_local_position);
    //ros::Subscriber imu_sub = nh.subscribe("/offboard/acceleration_imitate", 500,chatterCallback_imu_data);
    //ros::Subscriber velocity_sub = nh.subscribe("/offboard/velocity_imitate", 500,chatterCallback_local_velocity);
    //ros::Subscriber mode_sub = nh.subscribe("/offboard/mode_imitate", 100,chatterCallback_Mode);
    
    ros::Rate wait_rate(4);

    //wait for field points
    while(ros::ok() && !field_size_received)
    {  
        field_size_confirm_pub.publish(field_size_confirm_msg);
        ros::spinOnce();  
        wait_rate.sleep();
    }
    ROS_INFO("Field Size Received!");
    //wait for offboard mode
    while(ros::ok())
    {
    	if(p_received && v_received && a_received)
    	{
    		if(offboard_ready) break;
    		set_new_point(St_matrix(0,0),St_matrix(1,0),St_matrix(2,0),yaw, 0.0);             
    	}

        field_size_confirm_msg.length = field_size_matrix(0,0);
        field_size_confirm_msg.width = field_size_matrix(0,1);
        field_size_confirm_msg.height = field_size_matrix(0,2);
        field_size_confirm_msg.times = (int)field_size_matrix(0,3);
        field_size_confirm_msg.confirm = 1;
        field_size_confirm_pub.publish(field_size_confirm_msg);

    	ros::spinOnce();  
    	wait_rate.sleep();
    }
    ROS_INFO("Offboard Ready!");

    init_p_matrix(0,0) = St_matrix(0,0);
    init_p_matrix(0,1) = St_matrix(1,0);
    init_p_matrix(0,2) = St_matrix(2,0);
    init_yaw = yaw;
    
    ROS_INFO("Initial Point Set, Auto Flying!");
    field_2_setpoint(field_size_matrix(0,0), field_size_matrix(0,1), field_size_matrix(0,2), (int)field_size_matrix(0,3), init_yaw);

    //trajectory_generation(5.0, init_p_matrix(0,0)+10.0, init_p_matrix(0,1)+0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //ROS_INFO("NEXT POINT");
    //trajectory_generation(4.0, init_p_matrix(0,0)+10.0, init_p_matrix(0,1)+8.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //ROS_INFO("NEXT POINT");
    //trajectory_generation(5.0, init_p_matrix(0,0)+0.0, init_p_matrix(0,1)+8.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //ROS_INFO("NEXT POINT");
    //trajectory_generation(4.0, init_p_matrix(0,0)+0.0, init_p_matrix(0,1)+0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    return 0;

}

void field_2_setpoint(float l, float w, float h, int times=1, float yaw=0.0)
{
	int counter;
    set_new_point(init_p_matrix(0,0), init_p_matrix(0,1), h, yaw, 0.4);
	for(counter = 0;counter<times;counter++)
	{
		if(counter%2==0){
			set_new_point(init_p_matrix(0,0)+l*cos(yaw)+counter*w*sin(yaw), init_p_matrix(0,1)+counter*w*cos(yaw)-l*sin(yaw), h, yaw, 0.1);
            set_new_point(init_p_matrix(0,0)+l*cos(yaw)+(counter*w+w)*sin(yaw), init_p_matrix(0,1)+(counter*w+w)*cos(yaw)-l*sin(yaw), h, yaw, 0.1);
		}
		else{
			set_new_point(init_p_matrix(0,0)+counter*w*sin(yaw), init_p_matrix(0,1)+counter*w*cos(yaw), h, yaw, 0.1);
			set_new_point(init_p_matrix(0,0)+(counter*w+w)*sin(yaw), init_p_matrix(0,1)+(counter*w+w)*cos(yaw), h, yaw, 0.1);
		}
	}
}

void set_new_point(float x, float y, float z, float yaw, float t) //t is the time to hover
{
	setpoint.x = x;
	setpoint.y = y;
	setpoint.z = z;
	setpoint.yaw = 2*Pi-yaw;
    
    ros::NodeHandle n;
	ros::Publisher setpoints_pub = n.advertise<mavros::SetPointLocal>("offboard/setpoints_local", 500);
    
    int rest_counter = 0;
    int max = (int)(t*10);
	ros::Rate loop_rate(10);

	ready_for_next = false;

	while(ros::ok()){
    	setpoints_pub.publish(setpoint);
        ROS_INFO("%f %f %f %f %d", setpoint.x,setpoint.y,setpoint.z,setpoint.yaw,rest_counter);

    	if(ready_for_next) rest_counter+=1;

        if(rest_counter > max) break;

    	ros::spinOnce();  
    	loop_rate.sleep();
    }
}

void trajectory_generation(float T,float pxf, float pyf, float pzf,float vxf=0.0, float vyf=0.0, 
	         float vzf=0.0, float axf=0.0, float ayf=0.0, float azf=0.0)
{
	S0_matrix = St_matrix; 

	ros::NodeHandle n;
	ros::Publisher setpoints_pub = n.advertise<mavros::SetPointLocal>("offboard/setpoints_local", 500);
    ros::Publisher acceleration_pub = n.advertise<mavros::Vector3>("offboard/velocity_test", 500);
    ros::Publisher velocity_pub = n.advertise<mavros::Vector3>("offboard/acceleration_test", 500);

    ros::Rate loop_rate(LOOP_RATE);
    float t=0.0;

    trajectory_Paras_generation_i(0, S0_matrix(0,0),S0_matrix(0,1),S0_matrix(0,2),pxf,vxf,axf,T);
	trajectory_Paras_generation_i(1, S0_matrix(1,0),S0_matrix(1,1),S0_matrix(1,2),pyf,vyf,ayf,T);
	trajectory_Paras_generation_i(2, S0_matrix(2,0),S0_matrix(2,1),S0_matrix(2,2),pzf,vzf,azf,T);

    while(ros::ok()&&t<=T){

	    j_optimal_matrix(0,0) = j_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t); //x
	    j_optimal_matrix(0,1) = j_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t); //y
	    j_optimal_matrix(0,2) = j_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t); //z
        //x
        St_optimal_matrix(0,0) = p_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t); 
        St_optimal_matrix(0,1) = v_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t);
        St_optimal_matrix(0,2) = a_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t);

        //y
        St_optimal_matrix(1,0) = p_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t); 
        St_optimal_matrix(1,1) = v_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t);
        St_optimal_matrix(1,2) = a_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t);

        //z
        St_optimal_matrix(2,0) = p_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t); 
        St_optimal_matrix(2,1) = v_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t);
        St_optimal_matrix(2,2) = a_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t);

        setpoint.x = St_optimal_matrix(0,0);
	    setpoint.y = St_optimal_matrix(1,0);
	    setpoint.z = St_optimal_matrix(2,0);
	    setpoint.yaw = 0.0;

    	setpoints_pub.publish(setpoint);
        ROS_INFO("%f %f %f", setpoint.x,setpoint.y,setpoint.z);
        
        velocity_vector.x = St_optimal_matrix(0,2);
        velocity_vector.y = St_optimal_matrix(1,2);
        velocity_vector.z = St_optimal_matrix(2,2);
        velocity_pub.publish(velocity_vector);

        acceleration_vector.x = St_optimal_matrix(0,2);
        acceleration_vector.y = St_optimal_matrix(1,2);
        acceleration_vector.z = St_optimal_matrix(2,2);
        acceleration_pub.publish(acceleration_vector);
        

        t += 1.0/LOOP_RATE;

    	ros::spinOnce();  
    	loop_rate.sleep();
    }

}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
	//judge if close to set ready
	if(near_bool(setpoint.x, msg.pose.position.x)&&near_bool(setpoint.y, msg.pose.position.y)&&
		near_bool(setpoint.z, msg.pose.position.z))
		close_counter += 1;
	else {
		close_counter = 0;
		//ready_for_next = false;
	}

	if(close_counter >= 1){
	    ready_for_next = true;
        close_counter = 0;
	}
	//set values
	St_matrix(0,0) = msg.pose.position.x;
	St_matrix(1,0) = msg.pose.position.y;
	St_matrix(2,0) = msg.pose.position.z;

    double q2=msg.pose.orientation.x;
    double q1=msg.pose.orientation.y;
    double q0=msg.pose.orientation.z;
    double q3=msg.pose.orientation.w;
    //message.local_position.orientation.pitch = (asin(2*q0*q2-2*q1*q3 ))*57.3;
    //message.local_position.orientation.roll  = (atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2))*57.3;
    yaw = (-atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1))+Pi;//North:0, south:Pi, East:Pi/2, West: Pi*3/2
    
    if(!p_received) ROS_INFO("P ready!");
    p_received = true;

}

void chatterCallback_imu_data(const sensor_msgs::Imu &msg)
{
	//St_matrix(0,2) = msg.linear_acceleration.x;
	//St_matrix(1,2) = msg.linear_acceleration.y;
	//St_matrix(2,2) = msg.linear_acceleration.z-9.8;
	St_matrix(0,2) = 0.0;
	St_matrix(1,2) = 0.0;
	St_matrix(2,2) = 0.0;
    if(!a_received)ROS_INFO("a ready!");
	a_received = true;
}

void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg)
{
	St_matrix(0,1) = msg.x;
	St_matrix(1,1) = msg.y;
	St_matrix(2,1) = msg.z;
    if(!v_received)ROS_INFO("v ready!");
    v_received = true;
}

void chatterCallback_Mode(const mavros::State &msg)//模式
{
    if(msg.mode=="OFFBOARD") offboard_ready=true;
    //ROS_INFO("offboard ready!");
}

void chatterCallback_field_size(const mavros_extras::FieldSize &msg)
{
    field_size_matrix(0,0) = msg.length;
    field_size_matrix(0,1) = msg.width;
    field_size_matrix(0,2) = msg.height;
    field_size_matrix(0,3) = msg.times;
    field_size_received = true;

}
void trajectory_Paras_generation_i(int num, float p0, float v0, float a0, float pf, float vf, float af, float T)//num 0,1,2 reoresents  x, y, z
{
	MatrixXd delt_s(3,1);
	delt_s(0,0) = af-a0;
	delt_s(1,0) = vf-v0-a0*T;
	delt_s(2,0) = pf-p0-v0*T-0.5*a0*T*T;

	MatrixXd temp(3,3);
	temp << 60/pow(T,3),-360/pow(T,4),720/pow(T,5),-24/pow(T,2),168/pow(T,3),-360/pow(T,4),3/T,-24/pow(T,2),60/pow(T,3);
	//std::cout << temp;
	MatrixXd const_paras(3,1);//(alfa,beta,gamma)
	const_paras = temp * delt_s;
	//std::cout << const_Paras_matrix;
	Paras_matrix(num,0) = const_paras(0,0);
	Paras_matrix(num,1) = const_paras(1,0);
	Paras_matrix(num,2) = const_paras(2,0);
}

bool near_bool(float x, float y)
{
	if(x-y<0.6 && x-y>-0.6)
		return true;
	else return false;
}

float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return 0.5*alfa*t*t+beta*t+gamma;
}

float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return alfa*pow(t,5)/120+beta*pow(t,4)/24+gamma*pow(t,3)/6+S0_matrix(num,2)*t*t/2+S0_matrix(num,1)*t+S0_matrix(num,0);
}

float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return alfa*pow(t,4)/24+beta*pow(t,3)/6+gamma*t*t/2+S0_matrix(num,2)*t+S0_matrix(num,1);
}

float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return alfa*pow(t,3)/6+beta*t*t/2+gamma*t+S0_matrix(num,2);
}