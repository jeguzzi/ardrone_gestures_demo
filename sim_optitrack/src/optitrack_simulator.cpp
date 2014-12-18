#include <iostream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <sim_optitrack_msgs/SimRigidBodyData.h>



int main(int argc, char** argv){
    
ros::init(argc, argv, "optitrack_simulator");
ros::NodeHandle nh_;
ros::Rate rate(120.0);

ros::Publisher SimOptitrack_message = nh_.advertise<sim_optitrack_msgs::SimRigidBodyData>("sim_optitrack", 1);
sim_optitrack_msgs::SimRigidBodyData data;

int body_id;
int mean_error = 0.0;
std::string body_name;


nh_.param("body_id",body_id,1);
nh_.param<std::string>("body_name",body_name,"quadrotor");

  
while (nh_.ok()){

	gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name=body_name;

	//Call ros_gazebo service    
	ros::ServiceClient gmscl = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gmscl.call(getmodelstate);
	
	// Id 
	data.id = body_id;
	data.name = body_name;	
	data.mean_error = mean_error;

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
