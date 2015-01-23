#include <iostream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>


int main(int argc, char** argv){
    
ros::init(argc, argv, "SimOptitrack_node");
ros::NodeHandle nh_;
ros::Rate rate(120.0);

ros::Publisher SimOptitrack_message_ = nh_.advertise<optitrack_msgs::RigidBodies>("/optitrack/rigid_bodies", 1);
std::vector<std::string> bodies_Vec;
  
while (nh_.ok()){

	nh_.getParam("sim_optitrack_node/bodies_list", bodies_Vec);
	gazebo_msgs::GetModelState getmodelstate;

	optitrack_msgs::RigidBodies rigid_bodies_msg;
	//rigid_bodies_msg.header = header;
	
	for(unsigned i=0; i < bodies_Vec.size(); i++) {

		getmodelstate.request.model_name=bodies_Vec[i];
		//Call ros_gazebo service    
		ros::ServiceClient gmscl = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
		gmscl.call(getmodelstate);

		optitrack_msgs::RigidBodyData data;
		// Id 
		data.id = i;
		data.name = bodies_Vec[i];	
		data.mean_error =0.0;

		// Position
		data.pose.position.x = getmodelstate.response.pose.position.x;
		data.pose.position.y = getmodelstate.response.pose.position.y;
		data.pose.position.z = getmodelstate.response.pose.position.z;

		data.pose.orientation.x = getmodelstate.response.pose.orientation.x;
		data.pose.orientation.y = getmodelstate.response.pose.orientation.y;
		data.pose.orientation.z = getmodelstate.response.pose.orientation.z;
		data.pose.orientation.w = getmodelstate.response.pose.orientation.w;

		rigid_bodies_msg.rigid_bodies.push_back(data);
		}
	//std::cout << "Orientation " << getmodelstate.response.pose.orientation.w  << std::endl;
	SimOptitrack_message_.publish(rigid_bodies_msg);

	ros::spinOnce();
	rate.sleep();
}

return 0;
};
