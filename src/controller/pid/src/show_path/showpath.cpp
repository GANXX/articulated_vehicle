#include "pid/showpath.h"

using namespace std;

Showpath::Showpath(ros::NodeHandle _n){
    _nh = _n;
    ros::Time::init();
    pub_path = _nh.advertise<nav_msgs::Path>("baselink_path", 20);		
    //ros::ServiceServer clear_path =_nh.advertiseService("clear_robot_path",&Showpath::clear_response, this);

    starRrecordPath();
}

bool Showpath::clear_response(std_srvs::Empty::Request &req,
			     std_srvs::Empty::Response &res){
    clearPath();
return true;
}
bool Showpath::record_response(std_srvs::Empty::Request &req,
			     std_srvs::Empty::Response &res){
    starRrecordPath();
return true;
}

/*向零取整函数,
 * n值为保留的小数点位,如n=0.01就是保留后两位小数点,默认为1
 */
double Showpath::getfix(double x, double n=1){
    int m= 1/n;
    if(x>=0){
	x = floor(x*m)/m;
    }
    else x =ceil(x*m)/m;
    return(x);
}

double Showpath::getround(double x, double n=1){
    int m= 1/n;
	x = round(x*m)/m;
    return(x);
}

bool Showpath::getpose(){
    try{
	listener.lookupTransform("/map",
				 "/base_link",
				 ros::Time(0),
				 transform_);
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
    
    drawPath();
    return true;
}

bool Showpath::drawPath(){
    if(Model_pose.position.x!=Model_positionx_old 
	    && Model_pose.position.y!=Model_positiony_old){
	geometry_msgs::PoseStamped this_pose_stamped;

	this_pose_stamped.pose            = Model_pose;
	this_pose_stamped.header.stamp    = ros::Time::now();
	this_pose_stamped.header.frame_id = "map";

	baselink_path.header.stamp        = ros::Time::now();
	baselink_path.header.frame_id     = "map";
	baselink_path.poses.push_back(this_pose_stamped);

	pub_path.publish(baselink_path);

	//将数据记录到文件
	outFile<<Model_pose.position.x<<" "<<Model_pose.position.y<<endl;

    }

    Model_positionx_old = Model_pose.position.x;
    Model_positiony_old = Model_pose.position.y;
    

    return true;
}


bool Showpath::starRrecordPath(){
    //将数据记录到文件
    string path = getCwd();
    ostringstream oss;
    oss<<path<<"robot_path_x_y_"<<file_num<<".txt";

    outFile.open(oss.str().c_str());
    outFile<<"x坐标"<<" "<<"y坐标"<<endl;
    outFile.close();

    outFile.open(oss.str().c_str(),
		 std::ios::out |
		 std::ios::app);

    file_num++;
    cout<<"开始记录路径"<<endl;
    ROS_INFO("Data has been saved at:%s",oss.str().c_str());
}

bool Showpath::clearPath(){
    baselink_path.poses.clear();
    outFile.close();
    cout<<"机器人路径已经清零"<<endl;
    //

}

//获取当前目录
string Showpath::getCwd(){
    string path;
    path = getcwd(NULL,0);
    return path;
}

Showpath::~Showpath(){
	outFile.close();
	cout<<"文件关闭"<<endl;
}




int main(int argc, char** argv){
    ros::init(argc, argv, "showpath");
    ros::NodeHandle nh;
    Showpath Spath(nh);
    ros::Rate rate(50); 
    ros::ServiceServer clear_path =nh.advertiseService("/showpath/clear_robot_path",&Showpath::clear_response, &Spath);
    ros::ServiceServer record_path =nh.advertiseService("/showpath/record_robot_path",&Showpath::record_response, &Spath);

    while(nh.ok()){
	ros::spinOnce();
	Spath.getpose();
	rate.sleep();
    }
    return 0;
}
