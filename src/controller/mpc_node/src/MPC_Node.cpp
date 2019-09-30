/*# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <math.h>
#include <time.h>
#include <algorithm>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "MPC.h"
#include <Eigen/Core>
#include <Eigen/QR>

#include <dynamic_reconfigure/server.h>
#include <mpc_node/mpc_dynamicConfig.h>
#define MAX_DOUBLE 1.7976931348623158e+308 /* max value */
using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        int get_thread_numbers();
		geometry_msgs::Pose getpose(string ,string);
		void isreached();
		void pathCycle(nav_msgs::Path *pathMsg);
		bool clear_mappath(nav_msgs::Path *pathMsg, int min_i);
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_path, _sub_goal;
        ros::Publisher _pub_odompath, _pub_twist, _pub_mpctraj;
        ros::Timer _timer1;
		ros::Time current_time, after_time, data_using_time;
		tf::TransformListener listener;
	    Recorder etc_data;
		Recorder calculate_time;
		Recorder epsi_data;
		Recorder car_path;
		Recorder reference_path;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _map_path, _mpc_traj, movebase_path; 
        geometry_msgs::Twist _twist_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_epsi, _ref_vel, _w_cte, _w_epsi, _w_vel, 
               _w_delta, _w_accel, _w_delta_d, _w_accel_d, _max_steering, _max_throttle, _bound_value;

        double _Lf, _dt, _steering, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius ;
		double _waypointsDist = 0;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;
		bool isrecord_reference_path = false;
		bool ispathpointenough = true;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void controlLoopCB();
        //void controlLoopCB(const ros::TimerEvent&);
        void dynamicCB(mpc_node::mpc_dynamicConfig &config, uint32_t level);
		void evaluateCte(Eigen::VectorXd coeffs);
		void evaluateEpsi(Eigen::VectorXd coeffs);

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node ros节点的线程数
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, false);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 2.0); // unit: m/s
    pn.param("path_length", _pathLength, 8.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    pn.param("vehicle_Lf", _Lf, 0.25); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_epsi", _ref_epsi, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 1.5);
    pn.param("mpc_w_cte", _w_cte, 100.0);
    pn.param("mpc_w_epsi", _w_epsi, 100.0);
    pn.param("mpc_w_vel", _w_vel, 100.0);
    pn.param("mpc_w_delta", _w_delta, 100.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_delta_d", _w_delta_d, 0.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 0.0);
    pn.param("mpc_max_steering", _max_steering, 0.523); // Maximal steering radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "map" );
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_link" );

    //Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: "  << _pub_twist_flag << endl;
    cout << "debug_info: "  << _debug_info << endl;
    cout << "delay_mode: "  << _delay_mode << endl;
    cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "frequency: "   << _dt << endl;
    cout << "mpc_steps: "   << _mpc_steps << endl;
    cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "mpc_w_cte: "   << _w_cte << endl;
    cout << "mpc_w_epsi: "  << _w_epsi << endl;
    cout << "mpc_max_steering: "  << _max_steering << endl;

    //Publishers and Subscribers
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    _sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);
    _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this);
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
    
  // dynamic reconfiguration
  dynamic_reconfigure::Server<mpc_node::mpc_dynamicConfig> config_server;
  dynamic_reconfigure::Server<mpc_node::mpc_dynamicConfig>::CallbackType f;
  f = boost::bind(&MPCNode::dynamicCB, this, _1, _2);
  config_server.setCallback(f);

    //Timer
	//时间中断器
    //_timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz

    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _steering = 0.0;
    _speed = 0.0;

	//这里应该是初始化操作,不知道这样做的意义何在
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_EPSI"] = _ref_epsi;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_epsi;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_DELTA"]  = _w_delta;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DDELTA"] = _w_delta_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["MAXSTR"]   = _max_steering;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params); //把MPC_Node中参数信息通过_mpc_params传送到MPC.cpp中
	


	ros::Time::init();

	ros::Rate loop_rate(_controller_freq);
	while (ros::ok()){
	   //current_time = ros::Time::now();
	   controlLoopCB();
	   //after_time = ros::Time::now();
	   ros::spinOnce();
	}

}


geometry_msgs::Pose MPCNode::getpose(string frame_id="/map", string child_frame_id="/base_footprint"){
    tf::StampedTransform transform_;
    geometry_msgs::Pose Model_pose;
	Model_pose = geometry_msgs::Pose(); //初始化
    try{
	listener.lookupTransform(frame_id,
				 child_frame_id,
				 ros::Time(0),
				 transform_);
	//listener.lookupTransform(frame_id,
				 //child_frame_id,
				 //ros::Time(0),
				 //transform_);
    }
    catch(tf::TransformException &ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
    }
    Model_pose.position.x =transform_.getOrigin().x();
    Model_pose.position.y =transform_.getOrigin().y();
    Model_pose.position.z =transform_.getOrigin().z();

    Model_pose.orientation.x = transform_.getRotation().x();
    Model_pose.orientation.y = transform_.getRotation().y();
    Model_pose.orientation.z = transform_.getRotation().z();
    Model_pose.orientation.w = transform_.getRotation().w();
    
    return Model_pose;
}

// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}


// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

//
void MPCNode::dynamicCB(mpc_node::mpc_dynamicConfig &config, uint32_t level){
    _max_speed       = config.max_speed;      // unit: m/s
    _pathLength      = config.path_length;    // unit: m
    _goalRadius      = config.goal_radius;    // unit: m
    _controller_freq = config.controller_freq;
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver
    _mpc_steps    = config.mpc_steps;
    _ref_cte      = config.mpc_ref_cte;
    _ref_epsi     = config.mpc_ref_epsi;
    _ref_vel      = config.mpc_ref_vel;
    _w_cte        = config.mpc_w_cte;
    _w_epsi       = config.mpc_w_epsi;
    _w_vel        = config.mpc_w_vel;
    _w_delta      = config.mpc_w_delta;
    _w_accel      = config.mpc_w_accel;
    _w_delta_d    = config.mpc_w_delta_d;
    _w_accel_d    = config.mpc_w_accel_d;
    _max_steering = config.mpc_max_steering;
    _max_throttle = config.mpc_max_throttle;
   
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_EPSI"] = _ref_epsi;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_epsi;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_DELTA"]  = _w_delta;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DDELTA"] = _w_delta_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["MAXSTR"]   = _max_steering;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params); //把MPC_Node中参数信息通过_mpc_params传送到MPC.cpp中
}
// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg;
}

bool MPCNode::clear_mappath(nav_msgs::Path *pathMsg, int min_i){
   if(ispathpointenough){
   int i =min_i*_downSampling;
   pathMsg->poses.erase(pathMsg->poses.begin(),pathMsg->poses.begin()+i);
   return true;
	}
   else return false;
}

void MPCNode::pathCycle(nav_msgs::Path *pathMsg){
    if(_goal_received && !_goal_reached)
    {    
        nav_msgs::Path map_path = nav_msgs::Path();
        try
        {
            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy); //获取相邻点之间的路径的长度
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }
            double total_length = 0.0;
            int sampling = _downSampling;  
			int predict_sample=10;
			int i=0;
			if(pathMsg->poses.size()<=predict_sample) i = 0;
			else i=predict_sample;
            // Cut and downsampling the path
			while(map_path.poses.size()<10)
            {
                if(total_length > _pathLength) //_pathLength＜0,那就直接退出
                    break;

                if(sampling == _downSampling){   
                    geometry_msgs::PoseStamped tempPose =pathMsg->poses[i];
					//_odom_frame是目标坐标; pathMsg->pose是输入值,_map_frame是固定坐标; tempPose是输出值
					//将path从map坐标转化成odom坐标这样在后面的controlLoopCB中就可以都采用odom坐标进行计算
					//(我觉得这样是多此一举的 )
					//_tf_listener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
                    map_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist;  //total_length 累加有什么用? 没看到用的地方
                sampling = sampling + 1;  
				i++;
				if(i>=pathMsg->poses.size()) break;
            }

            if(map_path.poses.size() > 6 )
            {
				_map_path = map_path; // Path waypoints in map frame
				// publish odom path
				map_path.header.frame_id = _odom_frame;
				map_path.header.stamp = ros::Time::now();
				_pub_odompath.publish(map_path);

            }
			else{
			    int sample_piont =map_path.poses.size();
			    ROS_ERROR("Sample points less than 6, are %d",sample_piont);
				ispathpointenough = false;
			}
            //DEBUG            
            //cout << endl << "N: " << map_path.poses.size() << endl <<  "Car path[0]: " << map_path.poses[0] << ", path[N]: " << _map_path.poses[_map_path.poses.size()-1] << endl;


        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

}
// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
   movebase_path = *pathMsg;
   _path_computed = true;
   ispathpointenough = true;
   if(!isrecord_reference_path){
	  for(int i=0;i<movebase_path.poses.size();i++){
		 reference_path.recordbegin(movebase_path.poses[i].pose.position.x,movebase_path.poses[i].pose.position.y);
	  }
	  isrecord_reference_path = true;
   }
}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
    ROS_INFO("Goal Received !");

	etc_data.createFile("etc_data");
	calculate_time.createFile("calculate_time");
	epsi_data.createFile("epsi_data");
	car_path.createFile("car_path");
	reference_path.createFile("reference_path");
	//初始化数据记录对象
	etc_data.recordinit("etc_data");
	epsi_data.recordinit("epsi_data");
	calculate_time.recordinit("calculate_time");
	car_path.recordinit("car_path");
	reference_path.recordinit("reference_path");
}


void MPCNode::isreached(){
    if(_goal_received)
    {
	   geometry_msgs::Pose carPose = getpose("/map", "/base_footprint");
        double car2goal_x = _goal_pos.x - carPose.position.x;
        double car2goal_y = _goal_pos.y - carPose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
		car_path.recordbegin(carPose.position.x,carPose.position.y);
        if(dist2goal < _goalRadius)
        {
            _goal_reached = true;
            _goal_received = false;
            _path_computed = false;
            ROS_INFO("Goal Reached !");
			isrecord_reference_path = false;
			etc_data.closeFile();
			epsi_data.closeFile();
		    calculate_time.closeFile();
			car_path.closeFile();
			reference_path.closeFile();
        }
    }

}
// Callback: Check if the car is inside the goal area or not 


// Timer: Control Loop (closed loop nonlinear MPC)
//void MPCNode::controlLoopCB(const ros::TimerEvent&)
void MPCNode::controlLoopCB()
{      
    if(_goal_received && !_goal_reached && _path_computed ) //received goal & goal not reached    
    {    
	   //clock_t current_time =clock();
        current_time = ros::Time::now();
	    pathCycle(&movebase_path);
        nav_msgs::Odometry odom = _odom;         //_odom是相对于odom坐标系车的位置信息
		geometry_msgs::Pose carPose = getpose("/map", "/base_footprint");
        nav_msgs::Path map_path = _map_path;   

        // Update system states: X=[x, y, psi, v]
        const double px = carPose.position.x; //pose: map frame
        const double py = carPose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(carPose , pose);
        const double psi = tf::getYaw(pose.getRotation()); //车在map坐标系下面的的航向角
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame
        // Update system inputs: U=[steering, throttle]
        const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;
        const double Lf = _Lf;

        // Waypoints related parameters
        const int N = map_path.poses.size(); // Number of waypoints
        const double cospsi = cos(psi);
        const double sinpsi = sin(psi);
		double minimize_i = MAX_DOUBLE;
		int end_num;

        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
		   //因为map_path和px,py都是map坐标系下面的值
		   //psi也是使用map坐标系下计算得到的相对与map坐标系的航向角
            const double dx = map_path.poses[i].pose.position.x - px;
            const double dy = map_path.poses[i].pose.position.y - py;
			double e_dis=sqrt(dx*dx+dy*dy);
			if (minimize_i>e_dis){
			   end_num = i;
			   minimize_i = e_dis;
			}
            x_veh[i] = dx * cospsi + dy * sinpsi; //在base_link坐标系下面参考路径点和车辆位置点之间的偏差值x
            y_veh[i] = dy * cospsi - dx * sinpsi; //在base_link坐标系下面参考路径点和车辆位置点之间的偏差值y
        }
		//clear_mappath(&movebase_path,end_num);
		if(!clear_mappath(&movebase_path,end_num)){
		   ROS_WARN("Can not clear reference path that car have been cross");
		}
        
        // Fit waypoints
		// 计算三次样条曲线
        auto coeffs = polyfit(x_veh, y_veh, 3); 

        const double cte  = polyeval(coeffs, 0.0);
        const double epsi = atan(coeffs[1]);
        VectorXd state(6);
        if(_delay_mode)  //delay_mode默认是true 但是不知道指代的是什么
        {
            // Kinematic model is used to predict vehicle state at the actual
            // moment of control (current time + delay dt)
			// 运动模型用于预测实际控制时刻的车辆状态（当前时间+延迟dt）
            const double px_act = v * dt;
            const double py_act = 0;
            const double psi_act = steering*dt;
            const double v_act = v + throttle * dt;          //速度
            const double cte_act = cte + v * sin(epsi) * dt; //横线误差
            const double epsi_act = -epsi + psi_act;         //航向角误差
            state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, epsi;
        }
        
        // Solve MPC Problem
		// 调用MPC求解器进行求解
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
              
        // MPC result (all described in car frame)        
        _steering = mpc_results[0]; // 角速度
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "psi: " << psi << endl;
            cout << "V: " << v << endl;
            //cout << "map_path: \n" << map_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_steering: \n" << _steering << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();
        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose); 
        }     
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);

		//
		evaluateCte(coeffs);
		evaluateEpsi(coeffs);

		isreached();
	   //clock_t after_time =clock();
	   after_time = ros::Time::now();
	   //double calcu_time = (double)(after_time - current_time)/CLOCKS_PER_SEC;
	   double calcu_time=(after_time-current_time).toSec();
	   calculate_time.recordbegin(calcu_time);
	   cout<<calcu_time<<endl;
    }
	// 已经到达目标点
    else
    {
        _steering = 0.0;
        _throttle = 0.0;
        _speed = 0.0;
        if(_goal_reached && _goal_received)
            cout << "Goal Reached !" << endl;
    }


	//发布控制速度
    if(_pub_twist_flag)
    {
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _steering;
        _pub_twist.publish(_twist_msg);
    }
    
}
void MPCNode::evaluateCte(Eigen::VectorXd coeffs){
    double etc0 = polyeval(coeffs,0);
	etc_data.recordbegin(etc0);

}
void MPCNode::evaluateEpsi(Eigen::VectorXd coeffs){
   double epsi_0 = atan(coeffs[coeffs.size()-2]);
   epsi_data.recordbegin(epsi_0);
}

Recorder::Recorder(){}
bool Recorder::createFile(string filename){
	oss<<"/home/ganxin/ROS/ipc/data/mpc/"<<filename<<".txt";

    outFile.open(oss.str().c_str());
    outFile.close();
	return true;
}

bool Recorder::recordinit(string filename){

	outFile.open(oss.str().c_str(),std::ios::out | std::ios::app);
	cout<<"开始记录文件："<<filename<<endl;
    ROS_INFO("Data has been saved at:%s",oss.str().c_str());
	return true;

}

bool Recorder::recordbegin(double d1){
   static double i = 0;
    outFile<<i<<"\t"<<d1<<endl;
	i++;
	return true;
}
bool Recorder::recordbegin(double d1, double d2){
    outFile<<d1<<"\t"<<d2<<endl;
	return true;
}
bool Recorder::recordbegin(double d1,double d2,double d3){
    outFile<<d1<<"\t"<<d2<<"\t"<<d3<<endl;
	return true;
}
bool Recorder::recordbegin(double d1,double d2,double d3,double d4){
    outFile<<d1<<"\t"<<d2<<"\t"<<d3<<"\t"<<d4<<endl;
	return true;
}

//获取当前目录
string Recorder::getCwd(){
    string path;
    path = getcwd(NULL,0);
    return path;
}

bool Recorder::closeFile(){
   outFile.close();
   cout<<"文件关闭"<<endl;
}

Recorder::~Recorder(){
   outFile.close();
   cout<<"文件关闭"<<endl;
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
	//ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
	//spinner.start();
    //ros::waitForShutdown();
    return 0;
}

