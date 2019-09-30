
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "pid/wheel_vel.h"
#include <iostream>
using namespace std;
namespace sp
{
     
// 创建一个消息类型 setpoint
  std_msgs::Float64 left1_wheel_v;
  std_msgs::Float64 left2_wheel_v;
  std_msgs::Float64 left3_wheel_v;
  std_msgs::Float64 right1_wheel_v;
  std_msgs::Float64 right2_wheel_v;
  std_msgs::Float64 right3_wheel_v;
  pid::wheel_vel wheel_v;
 //设置履带车辆的宽度和长度 
    const double WHEEL_RADIUS = 0.078;
    const double ROBOT_RADIUS = 0.2095;
    const double ROBOT_LENGTH = 1.096;
}

using namespace sp;

void getcarcontrol(const geometry_msgs::Twist vel_msg)
{
	
    double RobotV = vel_msg.linear.x;
    double YawRate = vel_msg.angular.z;
    double r = RobotV / YawRate;
    double setpoint_left;
    double setpoint_right;
    //
    // 计算左右轮期望速度
	if(RobotV == 0)
	{
		setpoint_left  = -YawRate * ROBOT_RADIUS;
		setpoint_right = YawRate * ROBOT_RADIUS;
	} 
    else if(YawRate == 0)
	{
		setpoint_left  = RobotV;
		setpoint_right = RobotV;
	}
	else
	{
		setpoint_left  = YawRate * (r - ROBOT_RADIUS);
		setpoint_right = YawRate * (r + ROBOT_RADIUS);
		
	}
	//车轮正值逆时针旋转，负值顺时针旋转
	left1_wheel_v.data  = - setpoint_left/WHEEL_RADIUS;
	left2_wheel_v.data  = - setpoint_left/WHEEL_RADIUS;
	left3_wheel_v.data  = - setpoint_left/WHEEL_RADIUS;
	right1_wheel_v.data = - setpoint_right/WHEEL_RADIUS;
	right2_wheel_v.data = - setpoint_right/WHEEL_RADIUS;
	right3_wheel_v.data = - setpoint_right/WHEEL_RADIUS;
	
	wheel_v.left_vel.data  = - setpoint_left/WHEEL_RADIUS;
	wheel_v.right_vel.data = - setpoint_right/WHEEL_RADIUS;

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


//创建话题订阅 cmd_vel
  ros::Subscriber subcar = n.subscribe("cmd_vel_for_control", 1000, getcarcontrol);
//创建一个话题发布
  ros::Publisher setpoint_pub_l1 = n.advertise<std_msgs::Float64>("left1_wheel_v", 1000);
  ros::Publisher setpoint_pub_l2 = n.advertise<std_msgs::Float64>("left2_wheel_v", 1000);
  ros::Publisher setpoint_pub_l3 = n.advertise<std_msgs::Float64>("left3_wheel_v", 1000);
  ros::Publisher setpoint_pub_r1 = n.advertise<std_msgs::Float64>("right1_wheel_v", 1000);
  ros::Publisher setpoint_pub_r2 = n.advertise<std_msgs::Float64>("right2_wheel_v", 1000);
  ros::Publisher setpoint_pub_r3 = n.advertise<std_msgs::Float64>("right3_wheel_v", 1000);
  ros::Publisher setpoint_pub = n.advertise<pid::wheel_vel>("wheel_v", 1000);

  ros::Rate loop_rate(20);  
  while (ros::ok())
  {

    setpoint_pub_l1.publish(left1_wheel_v);  
    setpoint_pub_l2.publish(left2_wheel_v);  
    setpoint_pub_l3.publish(left3_wheel_v);  
    setpoint_pub_r1.publish(right1_wheel_v);
    setpoint_pub_r2.publish(right2_wheel_v);
    setpoint_pub_r3.publish(right3_wheel_v);
    setpoint_pub.publish(wheel_v);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
