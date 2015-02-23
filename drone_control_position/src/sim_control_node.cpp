#include <iostream>
#include <string>
#include <math.h> 
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <drone_control_msgs/drone_control_info.h>
#include <drone_control_msgs/send_control_data.h>
# define PI           3.14159265358979323846

// Some global variables

geometry_msgs::Twist velocityMsg;
drone_control_msgs::drone_control_info publish_data;
double errorAntPitch, errorAntRoll;
std::vector<double> PointsDroneFrame(4,0), Drone_info(4,0), Target_point_info(4,0),virtual_fence (5); // [maxX, minX, maxY, minY, maxAltitude]
std::vector<float> drone_quaternion (4,0); 


double quaternion2angles(std::vector<float> &quaternion){
	double roll, pitch, yaw, Dyaw;
	tf::Quaternion quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw );
	Dyaw=(double)(yaw*180)/PI;

return Dyaw;
}

// Define callbacks 
void hasReceivedModelState(const optitrack_msgs::RigidBodies::ConstPtr& msg){
	
  	Drone_info[0] = msg->rigid_bodies[0].pose.position.x; 
	Drone_info[1] = msg->rigid_bodies[0].pose.position.y;
	Drone_info[2] = msg->rigid_bodies[0].pose.position.z;
	drone_quaternion[0] = msg->rigid_bodies[0].pose.orientation.x;
	drone_quaternion[1] = msg->rigid_bodies[0].pose.orientation.y;
	drone_quaternion[2] = msg->rigid_bodies[0].pose.orientation.z;
	drone_quaternion[3] = msg->rigid_bodies[0].pose.orientation.w;
	Drone_info[3] = quaternion2angles(drone_quaternion);

	publish_data.yaw = Drone_info[3];
	publish_data.position.x = Drone_info[0];
	publish_data.position.y = Drone_info[1];
	publish_data.position.z = Drone_info[2];


  return;
} 


/*
void hasReceivedModelState(const optitrack_msgs::RigidBodyData::ConstPtr& msg){
	
	// Obtaining drone info 
  	Drone_info[0] = msg->pose.position.x; 
	Drone_info[1] = msg->pose.position.y;
	Drone_info[2] = msg->pose.position.z;
	drone_quaternion[0] = msg->pose.orientation.x;
	drone_quaternion[1] = msg->pose.orientation.y;
	drone_quaternion[2] = msg->pose.orientation.z;
	drone_quaternion[3] = msg->pose.orientation.w;
	Drone_info[3] = quaternion2angles(drone_quaternion);

	// Publishing info 
	publish_data.yaw = Drone_info[3];
	publish_data.position.x = Drone_info[0];
	publish_data.position.y = Drone_info[1];
	publish_data.position.z = Drone_info[2];

  return;
} */

void hasReceivedControlInfo(const drone_control_msgs::send_control_data::ConstPtr& msg){

	// Obtaining target point info
	Target_point_info[0] = msg->position.x; 
	Target_point_info[1] = msg->position.y;
	Target_point_info[2] = msg->position.z;
	Target_point_info[3] = msg->yaw;

	// Publishing info 
	publish_data.targetYaw = Target_point_info[3];
	publish_data.target.x = Target_point_info[0];
	publish_data.target.y = Target_point_info[1];
	publish_data.target.z = Target_point_info[2];

  return;
}


// This function calculates the Yaw obtained after compute the vector between the actual position and the target point
double NewYawAngle(double taX, double taY){

	double direction_angle;
	direction_angle=((atan2(taY,taX)*180)/PI);
	return direction_angle;
} 

void ControlYaw(double targetA, double velocity_limit, double Kp){
		double currentA, betha, tetha;
	//K=80;
	currentA = Drone_info[3];

	if (targetA >= 0){ // When targetA is positive
		if (currentA>=0)
			velocityMsg.angular.z=(targetA-currentA)*Kp; // 
		else{
			tetha = abs(currentA)+targetA;
			betha = 360 - tetha;
			if(betha>tetha)
				velocityMsg.angular.z = tetha*Kp; // vel+
			else
				velocityMsg.angular.z = -betha*Kp; // vel-
		}
	}
	else{ // When targetA is negative
		if (currentA<0)
			velocityMsg.angular.z = -(abs(targetA)-abs(currentA))*Kp; //
		else{
			tetha = currentA+abs(targetA); //
			betha = 360 - tetha;
			if(betha>tetha)
			velocityMsg.angular.z = -tetha*Kp; // vel -
			else
			velocityMsg.angular.z = betha*Kp; // vel +
		}
	}

	// Limit velocity
	velocityMsg.angular.z=std::min(velocity_limit,velocityMsg.angular.z);
	velocityMsg.angular.z=std::max(-velocity_limit,velocityMsg.angular.z);
	publish_data.yawVel=velocityMsg.angular.z;	
} 

void WorldToDroneframe(double targetX, double targetY, double angle){

	angle=(PI*angle)/180;
	
	PointsDroneFrame[0]=cos(angle)*Drone_info[0]+sin(angle)*Drone_info[1];
	PointsDroneFrame[1]=-sin(angle)*Drone_info[0]+cos(angle)*Drone_info[1];
	PointsDroneFrame[2]=cos(angle)*targetX+sin(angle)*targetY;
	PointsDroneFrame[3]=-sin(angle)*targetX+cos(angle)*targetY;

	publish_data.framePosition.x=PointsDroneFrame[0];
	publish_data.framePosition.y=PointsDroneFrame[1];
	publish_data.framePosition.z=Drone_info[2];
	publish_data.frameTarget.x=PointsDroneFrame[2];
	publish_data.frameTarget.y=PointsDroneFrame[3];
	publish_data.frameTarget.z=Target_point_info[2];
}

void ControlPitch(double actualX, double targetX, double velocity_limit, double Kp, double Kd, double fs){
	
	double vel, error, ts= 1/fs;
	error = (targetX-actualX);
	vel = error*Kp + ((error-errorAntPitch)/ts)*Kd;	
	// Limit velocity
	vel=std::min(velocity_limit,vel);
	vel=std::max(-velocity_limit,vel);
	velocityMsg.linear.x = vel;
	errorAntPitch = error;
	publish_data.vel.x=velocityMsg.linear.x;		
} 

void ControlRoll(double actualY, double targetY, double velocity_limit, double Kp, double Kd, double fs){

	double vel, error, ts= 1/fs;
	error = (targetY-actualY);
	vel = error*Kp + ((error-errorAntRoll)/ts)*Kd;
	// Limit velocity
	vel=std::min(velocity_limit,vel);
	vel=std::max(-velocity_limit,vel);
	velocityMsg.linear.y = vel;
	errorAntRoll = error;
	publish_data.vel.y=velocityMsg.linear.y;	
} 

void ControlAltitude(double actualAlt, double targetAlt, double velocity_limit, double Kp){

	//double K = 4;
	velocityMsg.linear.z = (targetAlt-actualAlt)*Kp;
	// Limit velocity
	velocityMsg.linear.z=std::min(velocity_limit,velocityMsg.linear.z);
	velocityMsg.linear.z=std::max(-velocity_limit,velocityMsg.linear.z);
	publish_data.vel.z=velocityMsg.linear.z;
} 
	
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "sim_control_drone_position_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

double fs = 20;
std::vector<double> dPose (4,0), Kp (4,0), Kd (2,0), velocity_limit (4,0);

ros::Subscriber optitrack_sub_=nh_.subscribe("drone_info_topic", 1, hasReceivedModelState);
ros::Subscriber contol_sub = nh_.subscribe("control_info", 1, hasReceivedControlInfo);

ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Publisher drone_info_pub_=nh_.advertise<drone_control_msgs::drone_control_info>("/drone_control_info", 1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);

std_msgs::Empty EmergencyMsg;


	while (ros::ok()){	
	nh_.getParam("/drone_control_node/Kp",Kp);
	nh_.getParam("/drone_control_node/Kd",Kd);
	nh_.getParam("/drone_control_node/delta_pose", dPose);
	nh_.getParam("/drone_control_node/virtual_fence",virtual_fence);
	nh_.getParam("/drone_control_node/velocity_limit",velocity_limit);

	//std::cout << "En el while" << std::endl;

	if (Drone_info[0]>virtual_fence[0] ||  Drone_info[0]<virtual_fence[1] || Drone_info[1]>virtual_fence[2] || Drone_info[1]<virtual_fence[3] || Drone_info[2]>virtual_fence[4]){
		//std::cout << "En 1" << std::endl;
		std::cout << "Emergency! Drone out of fence" << std::endl;
		land_pub_.publish(EmergencyMsg);
		publish_data.mode = "Emergency";
		}
	else{
		//std::cout << "En 3" << std::endl;
		WorldToDroneframe(Target_point_info[0], Target_point_info[1], Drone_info[3]);
		ControlPitch(PointsDroneFrame[0],PointsDroneFrame[2]+dPose[0], velocity_limit[0], Kp[0], Kd[0], fs);
		ControlRoll(PointsDroneFrame[1],PointsDroneFrame[3]+dPose[1], velocity_limit[1], Kp[1], Kd[1], fs);
		ControlAltitude(Drone_info[2], Target_point_info[2]+dPose[2], velocity_limit[2], Kp[2]);
		ControlYaw(Target_point_info[3]+dPose[3], velocity_limit[3], Kp[3]);
		vel_pub_.publish(velocityMsg);
		publish_data.mode = "Drone controlling";
		}

	drone_info_pub_.publish(publish_data);

	//std::cout << "Current yaw " << Drone_info[3] << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
