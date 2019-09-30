#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <iostream>
#include <cmath>
using namespace std;
//定义按键
double v,w;
int num[10]; //分别是 1,2,3,4, 左1，右1，左2，右2，select，start
bool ispub_goal  = false;
bool ispub       = false;
bool have_return = false;
geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped return_pose;
ros::Time current_time;
ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg){
    //判断键位
    int numsize = msg->buttons.size();
    for(int numcount=0; numcount<numsize; ++numcount){
	num[numcount]=msg->buttons[numcount];
    }
    if(num[9]==1) ispub_goal = true;
    else ispub_goal = false;
    
}

bool pub_goal(){
    ispub = true;
    if(ispub_goal){
	if(num[4]==1){
	    current_time = ros::Time::now();

	    goal.header.stamp =current_time; 
	    goal.header.frame_id = "map";
	    goal.pose.position.x = 1.26063621044;
	    goal.pose.position.y = -3.67480754852;
	    goal.pose.position.z = 0.0;
	    
	    goal.pose.orientation.x = 0.0;
	    goal.pose.orientation.y = 0.0;
	    goal.pose.orientation.z = -0.731290996284;
	    goal.pose.orientation.w = 0.682065597105;
	}
	if(num[5]==1){
	    current_time = ros::Time::now();

	    goal.header.stamp =current_time; 
	    goal.header.frame_id = "map";
	    goal.pose.position.x = -2.25127696991;
	    goal.pose.position.y = 4.85765981674;
	    goal.pose.position.z = 0.0;
	    
	    goal.pose.orientation.x = 0.0;
	    goal.pose.orientation.y = 0.0;
	    goal.pose.orientation.z = -0.724264784602;
	    goal.pose.orientation.w = 0.689521951635;
	}
	have_return = false;
	pub.publish(goal);
    }
}


bool return_to(){
    current_time = ros::Time::now();

    return_pose.header.stamp =current_time; 
    return_pose.header.frame_id = "map";
    return_pose.pose.position.x = -2.25127696991;
    return_pose.pose.position.y = 4.85765981674;
    return_pose.pose.position.z = 0.0;

    return_pose.pose.orientation.x = 0.0;
    return_pose.pose.orientation.y = 0.0;
    return_pose.pose.orientation.z = -0.724264784602;
    return_pose.pose.orientation.w = 0.689521951635;

    pub.publish(return_pose);
    have_return = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "auto_goal");
    ros::NodeHandle n;
    ros::Time::init();

    ros::Subscriber joy=n.subscribe("joy",100,joyCallback);
    pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",20);
    ros::Rate loop_rate(100);

    while(ros::ok()){
	ros::spinOnce();
	pub_goal();

	loop_rate.sleep();
    }
   return 0; 


}
