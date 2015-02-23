#include <iostream>
#include <string>
#include <math.h> 
#include <ros/ros.h>
#include <drone_control_msgs/send_control_data.h>

# define PI           3.14159265358979323846
	
int main(int argc, char** argv){
    
ros::init(argc, argv, "send_control_position_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

double TargetAltitude = 1.0, TargetYaw = 0.0;
std::vector<double> TargetPoint (2,0);

ros::Publisher control_data_pub_=nh_.advertise<drone_control_msgs::send_control_data>("control_data_message", 1);
drone_control_msgs::send_control_data control_data;

	while (ros::ok()){
	nh_.getParam("/drone_target_points/new_position",TargetPoint);
	nh_.getParam("/drone_target_points/new_yaw",TargetYaw);
	nh_.getParam("/drone_target_points/new_altitude",TargetAltitude);

	// Obtaining target point info
	control_data.position.x = TargetPoint[0];
	control_data.position.y = TargetPoint[1];
	control_data.position.z = TargetAltitude;
	control_data.yaw = TargetYaw;

	control_data_pub_.publish(control_data);
	
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
	}

 return 0;
}
