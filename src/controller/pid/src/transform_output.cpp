
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "pid/wheel_vel.h"
#include <iostream>
using namespace std;
namespace sp
{
     
// 创建一个消息类型 setpoint
  std_msgs::Float64 left_wheel_v;
  std_msgs::Float64 left_wheel_v_behind;
  std_msgs::Float64 right_wheel_v;
  std_msgs::Float64 right_wheel_v_behind;
  std_msgs::Float64 articulate_v;
  pid::wheel_vel wheel_v;
 //设置铰接车长度宽度 
    const double WHEEL_RADIUS = 0.75;
    const double VEHICLE_WIDTH = 1.5;
    const double lf = 1.500;
    const double lr = 1.467;

	double articulate_angle = 0;
	double d_articulate_angle = 0;
    double RobotV  = 0;
    double YawRate = 0;
    double setpoint_left = 0;
    double setpoint_right = 0;
    double setpoint_left_behind = 0;
    double setpoint_right_behind = 0;
}

using namespace sp;

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

void getcarcontrolCB(const geometry_msgs::Twist vel_msg)
{
	
    RobotV  = vel_msg.linear.x;     //前进速度
    YawRate = vel_msg.angular.z;   //转向角的角速度dot{gama}

}
void pubVelocity(){
    // 计算左右轮期望速度
   if(abs(articulate_angle)>0.1){
	  double LRr =0;
	  double LRl =0;
	  double Rf = (lf*cos(articulate_angle)+lr)/sin(articulate_angle);
	  LRr = Rf - VEHICLE_WIDTH/2; //计算右轮实际速度用
	  LRl = Rf + VEHICLE_WIDTH/2; //计算左轮实际速度用
	  setpoint_left = RobotV*(LRl/Rf);
	  setpoint_right = RobotV*(LRr/Rf);
	  //setpoint_left_behind =  setpoint_left ;
	  //setpoint_right_behind = setpoint_right;

	  double Rr = (lr*cos(articulate_angle)+lf)/sin(articulate_angle);
	  LRr = Rr - VEHICLE_WIDTH/2; //计算右轮实际速度用
	  LRl = Rr + VEHICLE_WIDTH/2; //计算左轮实际速度用
	  setpoint_left_behind = RobotV*(LRl/Rr);
	  setpoint_right_behind = RobotV*(LRr/Rr);

   }
   else {
	setpoint_left = RobotV;
	setpoint_right = RobotV;
	setpoint_left_behind =  setpoint_left ;
	setpoint_right_behind = setpoint_right;
   }
	//
	//
	left_wheel_v.data         = setpoint_left/WHEEL_RADIUS;
	left_wheel_v_behind.data  = setpoint_left_behind/WHEEL_RADIUS;
	right_wheel_v.data        = setpoint_right/WHEEL_RADIUS;
	right_wheel_v_behind.data = setpoint_right_behind/WHEEL_RADIUS;
	articulate_v.data         = - YawRate;
	
	wheel_v.left_vel.data  = left_wheel_v.data;
	wheel_v.right_vel.data = right_wheel_v.data;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_output");
  ROS_INFO("Starting setpoint publisher");
  ros::NodeHandle n;

  while (ros::ok() && ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning, waiting for time to become non-zero");
    sleep(1);
  }


//创建话题订阅 
  ros::Subscriber subcar = n.subscribe("cmd_vel_for_control", 1000, getcarcontrolCB);
  ros::Subscriber subangle = n.subscribe("/xbot/joint_states",1000, getArticulateAngleCB);
//创建一个话题发布
  ros::Publisher setpoint_pub_l   = n.advertise<std_msgs::Float64>("left_wheel_v_front", 1000);
  ros::Publisher setpoint_pub_lb   = n.advertise<std_msgs::Float64>("left_wheel_v_behind", 1000);
  ros::Publisher setpoint_pub_r   = n.advertise<std_msgs::Float64>("right_wheel_v_front", 1000);
  ros::Publisher setpoint_pub_rb   = n.advertise<std_msgs::Float64>("right_wheel_v_behind", 1000);
  ros::Publisher articulate_pub_v = n.advertise<std_msgs::Float64>("articulate_v", 1000);
  ros::Publisher setpoint_pub     = n.advertise<pid::wheel_vel>("wheel_v", 1000);

  ros::Rate loop_rate(20);  
  while (ros::ok())
  {
    ros::spinOnce();
	pubVelocity();
    setpoint_pub_l.publish(left_wheel_v);  
    setpoint_pub_lb.publish(left_wheel_v_behind);  
    setpoint_pub_r.publish(right_wheel_v);
    setpoint_pub_rb.publish(right_wheel_v_behind);
    articulate_pub_v.publish(articulate_v);
    setpoint_pub.publish(wheel_v);

    loop_rate.sleep();
  }
  return 0;
}
