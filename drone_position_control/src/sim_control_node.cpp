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
#include <gesture_msgs/drone_control_info.h>
#include <ardrone_autonomy/Navdata.h>
# define PI           3.14159265358979323846

// Some global variables
float quaternion[4] = {0,0,0,0};
geometry_msgs::Twist velocityMsg;
gesture_msgs::drone_control_info publish_data;
double DroneAltitude, DroneYaw, DroneX, DroneY, KPitch, KRoll;
double PointsDroneFrame[4] = {0.0,0.0,0.0,0.0};
double virtual_fence[4] = {5.0,-3.8,4.2,-3.9}; // [maxX, minX, maxY, minY]


double quaternion2angles(float quaternion[]){
	double roll, pitch, yaw, Dyaw;
	tf::Quaternion quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw );
	Dyaw=(double)(yaw*180)/PI;

return Dyaw;
}

// Define callbacks
void hasReceivedModelState(const optitrack_msgs::RigidBodies::ConstPtr& msg){
	
  	DroneX=msg->rigid_bodies[0].pose.position.x; 
	DroneY=msg->rigid_bodies[0].pose.position.y;/**/
	quaternion[0] = msg->rigid_bodies[0].pose.orientation.x;
	quaternion[1] = msg->rigid_bodies[0].pose.orientation.y;
	quaternion[2] = msg->rigid_bodies[0].pose.orientation.z;
	quaternion[3] = msg->rigid_bodies[0].pose.orientation.w;
	DroneYaw=quaternion2angles(quaternion);/**/
	publish_data.DroneYaw=DroneYaw;
	publish_data.DroneX=DroneX;
	publish_data.DroneY=DroneY;

  return;
}

void hasReceivedNavdataInfo(const ardrone_autonomy::NavdataConstPtr msg){
    	DroneAltitude = (msg->altd)/1000.0;
	publish_data.DroneAltitude = DroneAltitude;
}

// This function calculates the Yaw obtained after compute the vector between the actual position and the target point
double NewYawAngle(double taX, double taY){

	double direction_angle;
	direction_angle=((atan2(taY,taX)*180)/PI);
	return direction_angle;
} 

void ControlYaw(double targetA){
	double currentA, betha, tetha, K;
	K=80;
	currentA = DroneYaw;

	if (targetA >= 0){ // When targetA is positive
		if (currentA>=0)
			velocityMsg.angular.z=(targetA-currentA)/K; // 
		else{
			tetha = abs(currentA)+targetA;
			betha = 360 - tetha;
			if(betha>tetha)
				velocityMsg.angular.z = tetha/K; // vel+
			else
				velocityMsg.angular.z = -betha/K; // vel-
		}
	}
	else{ // When targetA is negative
		if (currentA<0)
			velocityMsg.angular.z = -(abs(targetA)-abs(currentA))/K; //
		else{
			tetha = currentA+abs(targetA); //
			betha = 360 - tetha;
			if(betha>tetha)
			velocityMsg.angular.z = -tetha/K; // vel -
			else
			velocityMsg.angular.z = betha/K; // vel +
		}
	}

	// Limit velocity
	velocityMsg.angular.z=std::min(1.2,velocityMsg.angular.z);
	velocityMsg.angular.z=std::max(-1.2,velocityMsg.angular.z);
	publish_data.YawVel=velocityMsg.angular.z;	
} 

void WorldToDroneframe(double targetX, double targetY, double angle){

	angle=(PI*angle)/180;
	
	PointsDroneFrame[0]=cos(angle)*DroneX+sin(angle)*DroneY;
	PointsDroneFrame[1]=-sin(angle)*DroneX+cos(angle)*DroneY;
	PointsDroneFrame[2]=cos(angle)*targetX+sin(angle)*targetY;
	PointsDroneFrame[3]=-sin(angle)*targetX+cos(angle)*targetY;

	publish_data.FrameDroneX=PointsDroneFrame[0];
	publish_data.FrameDroneY=PointsDroneFrame[1];
	publish_data.FrameTargetX=PointsDroneFrame[2];
	publish_data.FrameTargetY=PointsDroneFrame[3];
}

void ControlPitch(double actualX, double targetX, double Kp){
	
	//double currentX = DroneX;
	velocityMsg.linear.x = (targetX-actualX)/Kp;
	// Limit velocity
	velocityMsg.linear.x=std::min(1.0,velocityMsg.linear.x);
	velocityMsg.linear.x=std::max(-1.0,velocityMsg.linear.x);

	publish_data.PitchVel=velocityMsg.linear.x;	
} 

void ControlRoll(double actualY, double targetY, double Kp){

	//double currentY = DroneY;
	velocityMsg.linear.y = (targetY-actualY)/Kp;
	// Limit velocity
	velocityMsg.linear.y=std::min(1.0,velocityMsg.linear.y);
	velocityMsg.linear.y=std::max(-1.0,velocityMsg.linear.y);
	publish_data.RollVel=velocityMsg.linear.y;	
} 

void ControlAltitude(double actualAlt, double targetAlt){

	double K = 4;
	//double currentY = DroneY;
	velocityMsg.linear.z = (targetAlt-actualAlt)/K;
	// Limit velocity
	velocityMsg.linear.y=std::min(0.5,velocityMsg.linear.z);
	velocityMsg.linear.y=std::max(-0.5,velocityMsg.linear.z);
	publish_data.RollVel=velocityMsg.linear.z;	
} 
	
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "sim_control_direction_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

velocityMsg.linear.z = 0.0;
velocityMsg.linear.y = 0.0;

double TargetAltitude = 1.0, TargetYaw = 0.0;
std::vector<double> TargetPoint (2,0);
double Kp = (abs(virtual_fence[0])+abs(virtual_fence[1]))/4.0 ;


ros::Subscriber optitrack_sub_=nh_.subscribe("/optitrack/rigid_bodies", 1, hasReceivedModelState);
ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Publisher direction_pub_=nh_.advertise<gesture_msgs::drone_control_info>("/direction_outputs", 1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
ros::Subscriber alt_sub = nh_.subscribe("/ardrone/navdata", 1, hasReceivedNavdataInfo);
std_msgs::Empty EmergencyMsg;


	while (ros::ok()){
	nh_.getParam("new_position",TargetPoint);
	nh_.getParam("new_yaw_angle",TargetYaw);
	nh_.getParam("new_altitude",TargetAltitude);

	publish_data.TargetYaw=TargetYaw;
	publish_data.TargetX=TargetPoint[0];
	publish_data.TargetY=TargetPoint[1];
	publish_data.TargetAltitude=TargetAltitude;
	//std::cout << "En el while" << std::endl;

	if (DroneX>virtual_fence[0] ||  DroneX<virtual_fence[1] || DroneY>virtual_fence[2] || DroneY<virtual_fence[3]){
		//std::cout << "En 1" << std::endl;
		std::cout << "Emergency! Drone out of fence" << std::endl;
		land_pub_.publish(EmergencyMsg);
		publish_data.mode = "Emergency";
		}
	else{
		//std::cout << "En 3" << std::endl;
		ControlYaw(TargetYaw);
		WorldToDroneframe(TargetPoint[0], TargetPoint[1], DroneYaw);
		ControlPitch(/*TargetPoint[0]*/PointsDroneFrame[0],PointsDroneFrame[2],Kp);
		ControlRoll(/*TargetPoint[1]*/PointsDroneFrame[1],PointsDroneFrame[3],Kp);
		ControlAltitude(DroneAltitude, TargetAltitude);
		vel_pub_.publish(velocityMsg);
		publish_data.mode = "Drone controlling";
		}

	direction_pub_.publish(publish_data);

	//std::cout << "Current yaw " << DroneYaw << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
