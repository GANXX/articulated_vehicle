#include "pid/odom_cul.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
using namespace std;


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle nh;
    Odomcul cul(nh);
    //while(1){
	//cul.reconfigureCallback();
    //}
    

   /*************** 
    //ros::Subscriber left_wheel_sub =nh.subscribe("left_wheel_v",20, cmdCallback_left);
    //ros::Subscriber right_wheel_sub =nh.subscribe("right_wheel_v",20, cmdCallback_right);
    //ros::MultiThreadedSpinner s(2); //多线程
    *****************/

    return 0;
}
