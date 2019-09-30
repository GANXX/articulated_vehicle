#include "pid/odom_cul.h"
#include "pid/wheel_vel.h"
#include <cmath>
using namespace std;
 //设置履带车辆的宽度和长度 
    const double WHEEL_RADIUS = 0.078;
    const double ROBOT_RADIUS = 0.2095;
    const double ROBOT_LENGTH = 1.096;

    //协方差
    //boost::array<double, 36> odom_pose_covariance = {
	//{1e-9, 0, 0, 0, 0, 0, 
	//0, 1e-3, 1e-9, 0, 0, 0, 
	//0, 0, 1e6, 0, 0, 0,
	//0, 0, 0, 1e6, 0, 0, 
	//0, 0, 0, 0, 1e6, 0, 
	//0, 0, 0, 0, 0, 1e-9}};
    //boost::array<double, 36> odom_twist_covariance = {
	//{1e-9, 0, 0, 0, 0, 0, 
	//0, 1e-3, 1e-9, 0, 0, 0, 
	//0, 0, 1e6, 0, 0, 0, 
	//0, 0, 0, 1e6, 0, 0, 
	//0, 0, 0, 0, 1e6, 0, 
	//0, 0, 0, 0, 0, 1e-9}};


    Odomcul::Odomcul(ros::NodeHandle _n){
	_nh = _n;
	ros::Time::init();
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();

	// dynamic reconfiguration
	dynamic_reconfigure::Server<pid::PidConfig> config_server;
	dynamic_reconfigure::Server<pid::PidConfig>::CallbackType f;
	f = boost::bind(&Odomcul::reconfigureCallback, this, _1, _2);
	config_server.setCallback(f);
	cout<<"dynamic"<<endl;

        pub_odom = _nh.advertise<nav_msgs::Odometry>("odom", 20);		
	Model_pose.position.x    = 0;
	Model_pose.position.y    = 0;
	Model_pose.position.z    = 0;
	Model_pose.orientation.x = 0;
	Model_pose.orientation.y = 0;
	Model_pose.orientation.z = 0;
	Model_pose.orientation.w = 1;
	Model_twist.linear.x     = 0;
	Model_twist.linear.y     = 0;
	Model_twist.linear.z     = 0;

         ros::Subscriber gazebo_model_states =_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",20, boost::bind(&Odomcul::cmdCallback, this, _1));
         ROS_INFO("simulate state is true");
         //ros::Subscriber wheel_vel =nh.subscribe("wheel_v",20, &Odomcul::velCallback, &cul);
         //ROS_INFO("simulate state is false");

	while (ros::ok())
	{
	  ros::spinOnce();

	  // Add a small sleep to avoid 100% CPU usage
	  ros::Duration(0.02).sleep();
	}
    }

void Odomcul::reconfigureCallback(pid::PidConfig& config, uint32_t lever){
    slip_n = config.slip_n;
    cout<<"新设置的n值："<<slip_n<<endl;


} 


void Odomcul::cmdCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){

	int modelCount = msg->name.size();

	for(int modelind=0; modelind<modelCount; ++modelind){
	    if(msg->name[modelind]=="xbot-u"){
		Model_pose  = msg->pose[modelind];
		//四元数转欧拉角
		//tf::Quaternion quat;
		//tf::quaternionMsgToTF(Model_pose.pose.orientation,quat);
		//double roll, pitch, yaw;
		//tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

		Model_twist = msg->twist[modelind];
		double dx =Model_twist.linear.x;
		double dy=Model_twist.linear.y;
		Model_twist.linear.x = sqrt(dx*dx+dy*dy);
		Model_twist.linear.y = 0;
		
		publishOdom();
	    }
	}
    }

void Odomcul::velCallback(const pid::wheel_vel::ConstPtr &msg){
    vel_left = - msg->left_vel.data;
    vel_right= - msg->right_vel.data;
    encodercul();
}

 bool Odomcul::encodercul(){
 //轮速转换成线速度
	vel_left = vel_left*WHEEL_RADIUS;
	vel_right= vel_right*WHEEL_RADIUS;
	//积分里程计信息
	vx_ = (vel_right+vel_left)/2;
	vth_= (vel_right-vel_left)/(ROBOT_RADIUS*2);

	Model_twist.linear.x=vx_*cos(th_) - vy_*sin(th_);
	Model_twist.linear.y=vx_*sin(th_) + vy_*cos(th_);
	Model_twist.angular.z=vth_;

	current_time_ = ros::Time::now();
	cout<<"current_time"<<current_time_<<endl;
	cout<<"last_time"<<last_time_<<endl;
	double dt = (current_time_ -last_time_).toSec();
	double delta_x =(vx_*cos(th_) - vy_*sin(th_))*dt;
	double delta_y =(vx_*sin(th_) + vy_*cos(th_))*dt;
	double delta_th =vth_*dt;

	x_ = x_ + delta_x;
	y_ = y_ + delta_y;
	th_= th_ +delta_th;
	if (th_>6.283185307){
		th_=th_-6.283185307;
	}
	if (th_<-6.283185307){
		th_=th_+6.283185307;
	}
	last_time_= current_time_;
	Model_pose.orientation = tf::createQuaternionMsgFromYaw(th_);
	//odom_quat = tf::createQuaternionMsgFromYaw(th_);
	Model_pose.position.x = x_;
	Model_pose.position.y = y_;
	Model_pose.position.z = 0.0;
	
	//发布消息
	publishOdom();
	return true;
 }

 bool Odomcul::publishOdom(){
	current_time_=ros::Time::now();
	//发布TF
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time_;
	odom_trans.header.frame_id ="odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = Model_pose.position.x;
	odom_trans.transform.translation.y = Model_pose.position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation      = Model_pose.orientation;

	odom_broadcaster_.sendTransform(odom_trans);

	// 发布里程计消息
	nav_msgs::Odometry msgl;
	msgl.header.stamp = current_time_;
	msgl.header.frame_id = "odom";

	msgl.pose.pose.position = Model_pose.position;
	msgl.pose.pose.orientation = Model_pose.orientation;
	//msgl.pose.covariance = odom_pose_covariance;

	msgl.child_frame_id = "base_footprint";
	msgl.twist.twist.linear = Model_twist.linear;
	msgl.twist.twist.angular= Model_twist.angular;
	//msgl.twist.covariance = odom_twist_covariance;
	
	pub_odom.publish(msgl);
	return true;
	
 }
