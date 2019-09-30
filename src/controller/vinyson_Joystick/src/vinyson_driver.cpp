#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <cmath>
using namespace std;

geometry_msgs::Twist cmd_vel;
bool ispub =false;
bool stoprobot=false;
double speed_v= 0;
double speed_w= 0;
//定义按键
double v,w;
int num[10]; //分别是 1,2,3,4, 左1，右1，左2，右2，select，start

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg){
    //判断键位
    int numsize = msg->buttons.size();
    for(int numcount=0; numcount<numsize; ++numcount){
	num[numcount]=msg->buttons[numcount];
    }
    v=msg->axes[1];
    w=msg->axes[0];//左正右负
    
}
bool cul_velocity(){
    //计算速度
    double speed_v_max = 1.5;
    double speed_w_max = 1.5;
    if (v != 0){
	if (abs(speed_v) < speed_v_max) speed_v = speed_v+ v*0.01;
	else if(speed_v>0) speed_v = speed_v_max;
	else if(speed_v<0) speed_v = -speed_v_max;
    }
    else{
	if(abs(speed_v)<0.03) speed_v = 0;
	else{
	    if(speed_v>0) speed_v = speed_v -0.02;
	    if(speed_v<0) speed_v = speed_v +0.02;
	}
    }
//**************************************
    if (w != 0){
	if (abs(speed_w) < speed_w_max) speed_w = speed_w+ w*0.01;
	else if(speed_w>0) speed_w = speed_w_max;
	else if(speed_w<0) speed_w = -speed_w_max;
    }
    else{
	if(abs(speed_w)<0.03) speed_w = 0;
	else{
	    if(speed_w>0) speed_w = speed_w -0.02;
	    if(speed_w<0) speed_w = speed_w +0.02;
	}
    }

    cmd_vel.linear.x=speed_v;
    cmd_vel.linear.y=0;
    cmd_vel.linear.z=0;
    cmd_vel.angular.x=0;
    cmd_vel.angular.y=0;
    cmd_vel.angular.z=speed_w;

    if(num[4]==1 || num[5]==1 || num[6]==1 || num[7]==1){
	ispub = true;
	stoprobot = false;
    }
    else{
	ispub = false;
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vinyson");
    ROS_INFO("vinyson_Joy started");
    ros::NodeHandle n;

    ros::Subscriber joy=n.subscribe("joy",100,joyCallback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",20);
    ros::Rate loop_rate(100);
    while(ros::ok()){
	ros::spinOnce();
	cul_velocity();
	if(ispub) vel_pub.publish(cmd_vel);
	else{
	    cmd_vel.linear.x=0;
	    cmd_vel.angular.z=0;
	    speed_v = 0;
	    speed_w = 0;
	    if(!stoprobot){
		vel_pub.publish(cmd_vel);
		stoprobot = true;
	    } 

	}
	loop_rate.sleep();
    }
   return 0; 
}
