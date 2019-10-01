
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "pid/wheel_vel.h"
#include <iostream>
using namespace std;
namespace sp
{
     
// 创建一个消息类型 setpoint
  std_msgs::Float64 left_wheel_v;
  std_msgs::Float64 right_wheel_v;
  std_msgs::Float64 articulate_v;
  pid::wheel_vel wheel_v;
 //设置铰接车长度宽度 
    const double WHEEL_RADIUS = 0.75;
    const double ROBOT_RADIUS = 0.2095;
    const double ROBOT_LENGTH = 1.096;
}

using namespace sp;

void getcarcontrol(const geometry_msgs::Twist vel_msg)
{
	
    double RobotV  = vel_msg.linear.x;     //前进速度
    double YawRate = vel_msg.angular.z;   //转向角的角速度dot{gama}
    double r       = RobotV / YawRate;
    double setpoint_left;
    double setpoint_right;
    //
    // 计算左右轮期望速度
	setpoint_left = RobotV;
	setpoint_right = RobotV;
	//
	//
	//车轮正值逆时针旋转，负值顺时针旋转
	left_wheel_v.data      = - setpoint_left/WHEEL_RADIUS;
	right_wheel_v.data     = - setpoint_right/WHEEL_RADIUS;
	articulate_v.data      = YawRate;
	
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


//创建话题订阅 cmd_vel
  ros::Subscriber subcar = n.subscribe("cmd_vel_for_control", 1000, getcarcontrol);
//创建一个话题发布
  ros::Publisher setpoint_pub_l   = n.advertise<std_msgs::Float64>("left_wheel_v", 1000);
  ros::Publisher setpoint_pub_r   = n.advertise<std_msgs::Float64>("right_wheel_v", 1000);
  ros::Publisher articulate_pub_v = n.advertise<std_msgs::Float64>("articulate_v", 1000);
  ros::Publisher setpoint_pub     = n.advertise<pid::wheel_vel>("wheel_v", 1000);

  ros::Rate loop_rate(20);  
  while (ros::ok())
  {

    setpoint_pub_l.publish(left1_wheel_v);  
    setpoint_pub_r.publish(right1_wheel_v);
    articulate_pub_v.publish(articulate_v);
    setpoint_pub.publish(wheel_v);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
