#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <fstream>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

using namespace std;

class Recorder_Manual{
	public:
	   Recorder_Manual();
	   ~Recorder_Manual();
	   bool createFile(string);
	   bool closeFile();
	   bool recordinit(string);
	   bool recordbegin(double d1);
	   bool recordbegin(double d1, double d2);
	   bool recordbegin(double d1, double d2, double d3);
	   bool recordbegin(double d1, double d2, double d3, double d4);
	
	private:
	   ofstream outFile;
	   ostringstream oss;

};


Recorder_Manual::Recorder_Manual(){}
bool Recorder_Manual::createFile(string filename){
	oss<<"/home/ganxin/ROS/articulated_vehicle/data/mpc/"<<filename<<".txt";

    outFile.open(oss.str().c_str());
    outFile.close();
	return true;
}

bool Recorder_Manual::recordinit(string filename){

	outFile.open(oss.str().c_str(),std::ios::out | std::ios::app);
	cout<<"开始记录文件："<<filename<<endl;
    //ROS_INFO("Data has been saved at:%s",oss.str().c_str());
	return true;

}

bool Recorder_Manual::recordbegin(double d1){
   static double i = 0;
    outFile<<i<<"\t"<<d1<<endl;
	i++;
	return true;
}
bool Recorder_Manual::recordbegin(double d1, double d2){
    outFile<<d1<<"\t"<<d2<<endl;
	return true;
}
bool Recorder_Manual::recordbegin(double d1,double d2,double d3){
    outFile<<d1<<"\t"<<d2<<"\t"<<d3<<endl;
	return true;
}
bool Recorder_Manual::recordbegin(double d1,double d2,double d3,double d4){
    outFile<<d1<<"\t"<<d2<<"\t"<<d3<<"\t"<<d4<<endl;
	return true;
}

bool Recorder_Manual::closeFile(){
   outFile.close();
   cout<<"文件关闭"<<endl;
}

Recorder_Manual::~Recorder_Manual(){
   outFile.close();
   cout<<"文件关闭"<<endl;
}

double v, steeling;
double articulate_angle, d_articulate_angle;
bool begin_record = false;
nav_msgs::Odometry _odom = nav_msgs::Odometry();

void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg){
   _odom = *odomMsg;
}

void velCommondCB(const geometry_msgs::Twist::ConstPtr &msg){
   v  = msg->linear.x;     //前进速度
   steeling = msg->angular.z;   //转向角的角速度dot{gama}
   if(v!=0||steeling!=0){
	  begin_record = true;
   }
   else begin_record =false;


}
void getArticulateAngleCB(const sensor_msgs::JointState::ConstPtr &msg){
   int modelConst = msg->name.size();
   for(int i =0; i<modelConst; ++i){
	  if(msg->name[i]=="behindbase_to_frontbase"){
		 articulate_angle = msg->position[i];
		 d_articulate_angle = msg->velocity[i];
		 break;
	  }
   }


}

bool isinit = false;
int main(int argc, char **argv){
   ros::init(argc, argv, "Manual_Node");
   Recorder_Manual input_data;
   Recorder_Manual output_data;
   ros::NodeHandle n;
   ros::Subscriber sub      = n.subscribe("cmd_vel_for_control",1000,velCommondCB);
   ros::Subscriber subangle = n.subscribe("/xbot/joint_states",1000, getArticulateAngleCB);
   ros::Subscriber subodom  = n.subscribe("/odom",1000, odomCB);
   input_data.createFile("input_data");
   output_data.createFile("output_data");
   input_data.recordinit("input_data");
   output_data.recordinit("output_data");

   ros::Rate r(50);
   while(ros::ok()){
	  ros::spinOnce();
	  if(begin_record){
		 input_data.recordbegin(v,steeling);
		 output_data.recordbegin(_odom.twist.twist.linear.x,articulate_angle,d_articulate_angle);
	  }
	  r.sleep();

   }
   return 0;
}
