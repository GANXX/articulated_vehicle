
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
using namespace std;
namespace sp
{
     
// 创建一个消息类型 setpoint
  std_msgs::Float64 left_wheel_v;
  std_msgs::Float64 right_wheel_v;
 //设置履带车辆的宽度和长度 
    const double ROBOT_RADIUS = 0.46;
    const double ROBOT_LENGTH = 1;
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
	left_wheel_v.data  = 20*setpoint_left;
	right_wheel_v.data = 20*setpoint_right;

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
  ros::Publisher setpoint_pub_v = n.advertise<std_msgs::Float64>("left_wheel_v", 1000);
  ros::Publisher setpoint_pub_w = n.advertise<std_msgs::Float64>("right_wheel_v", 1000);
  ros::Rate loop_rate(100);  
  while (ros::ok())
  {
    ros::spinOnce();

    setpoint_pub_v.publish(left_wheel_v);  
    setpoint_pub_w.publish(right_wheel_v);

   // ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
