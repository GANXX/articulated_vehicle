#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
using namespace std;

tf2_ros::Buffer& tf_(tf);
bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const
{
  tf2_ros::Buffer& tf_;
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = std::string("base_link");
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later  

  // get the global pose of the robot
  try
  {
    tf_.transform(robot_pose, global_pose, std::string("map"));
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

int main (int argc, char** argv){
     ros::init(argc, argv, "getrobotpose");
     ros::NodeHandle n;
     ros::Time::init();
     geometry_msgs::PoseStamped global_pose;

     ros::Rate rate(100);
     while(n.ok()){
	  getRobotPose(global_pose);
	  double yaw = tf2::getYaw(global_pose.pose.orientation);
	  cout<<"   x   "<<global_pose.pose.position.x<<"  y  "<<global_pose.pose.position.y<<"  yaw  "<<yaw<<endl;
	  ros::spinOnce();
	  rate.sleep();
     }

}
