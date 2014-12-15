#include <iostream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gesture_msgs/RigidBodyData.h>
#include "std_msgs/String.h"


int main(int argc, char** argv){
    
ros::init(argc, argv, "optitrack_simulator");
ros::NodeHandle nh_;
ros::Rate rate(10.0);

ros::Publisher SimOptitrack_message = nh_.advertise<gesture_msgs::RigidBodyData>("sim_optitrack", 1);
gesture_msgs::RigidBodyData data;

int body_id;
const std::string body_name = "quadrotor";


nh_.param("body_id",body_id,1);
nh_.param("body_name",body_name,"quadrotor");

  
while (nh_.ok()){

	gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name=body_name;
	    
	ros::ServiceClient gmscl = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gmscl.call(getmodelstate);
	
	//
	data.id = body_id;
	data.name = body_name;	

	// Position
	data.pose.position.x = getmodelstate.response.pose.position.x;
	data.pose.position.y = getmodelstate.response.pose.position.y;
	data.pose.position.z = getmodelstate.response.pose.position.z;

	// Twist 
	
	data.twist.linear.x = getmodelstate.response.twist.linear.x;
	data.twist.linear.y = getmodelstate.response.twist.linear.y;
	data.twist.linear.z = getmodelstate.response.twist.linear.z;
	data.twist.angular.x = getmodelstate.response.twist.angular.x;
	data.twist.angular.y = getmodelstate.response.twist.angular.y;
	data.twist.angular.z = getmodelstate.response.twist.angular.z; 

	//std::cout << "Received position " << data.pose.position.x  << std::endl;
	
	SimOptitrack_message.publish(data);

	ros::spinOnce();
	rate.sleep();
}

return 0;
};
