#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>//获取文件目录
#include "std_srvs/Empty.h"

using namespace std;
    class Showpath{
	public:
	    Showpath(ros::NodeHandle _n);
	    ~Showpath();
	    bool getpose();
	    double getfix(double, double);
	    double getround(double, double);
	    bool drawPath();
	    bool starRrecordPath();
	    bool clearPath();
	    string getCwd();
	    bool clear_response(std_srvs::Empty::Request &req,
			         std_srvs::Empty::Response &res);
	    bool record_response(std_srvs::Empty::Request &req,
			         std_srvs::Empty::Response &res);
	private:
	    ros::Time current_time_;
	    ros::NodeHandle _nh;
	    tf::TransformListener listener;
	    tf::StampedTransform transform_;

	    ros::Publisher pub_path;
	    nav_msgs::Path baselink_path;
	    geometry_msgs::Pose Model_pose;
	    geometry_msgs::Twist Model_twist;
	    double Model_positionx_old= 0;
	    double Model_positiony_old= 0;

	    ofstream outFile;

	    int file_num=1;
    
    };    
