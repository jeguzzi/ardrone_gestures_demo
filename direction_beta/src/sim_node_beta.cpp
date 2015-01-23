#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <sim_optitrack_msgs/SimRigidBodyData.h>

// Some global variables
float position;
geometry_msgs::Twist velocityMsg;

void hasReceivedModelState(const optitrack_msgs::RigidBodyData::ConstPtr& msg){

  	position=msg->pose.position.x;


  return;
}

double sendDirection(double Nposition, double Lposition){

	if (position<Nposition-0.2)
			velocityMsg.linear.x = 0.2;
	else if (position>Nposition+0.2)
			velocityMsg.linear.x = -0.2;
	else{
		velocityMsg.linear.x = 0.0;
		std::cout << "Drone has arrived " << std::endl;
		Lposition = Nposition;
	}
  return Lposition;
}


		
int main(int argc, char** argv){
    
ros::init(argc, argv, "direction_sim_node_2");
ros::NodeHandle nh_;
ros::Rate rate(120.0);

velocityMsg.linear.z = 0.0;
velocityMsg.linear.y = 0.0;
double Nposition= 0.0;
double Lposition = 0.0;

ros::Subscriber trail_classification_sub_=nh_.subscribe("/sim_optitrack", 1, hasReceivedModelState);
ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (ros::ok()){
	nh_.getParam("New_Position",Nposition);
	if (Lposition != Nposition){
		Lposition = sendDirection(Nposition,Lposition);
		vel_pub_.publish(velocityMsg);
	}

	std::cout << "Current Position " << position << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
