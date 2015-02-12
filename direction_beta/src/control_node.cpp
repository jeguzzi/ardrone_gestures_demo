#include <iostream>
#include <string>
#include <math.h> 
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <direction_beta/direction_msgs.h>
#include <ardrone_autonomy/Navdata.h>
#include "ardrone_autonomy/LedAnim.h"
# define PI           3.14159265358979323846

// Some global variables
float quaternion[4] = {0,0,0,0};
geometry_msgs::Twist velocityMsg;
direction_beta::direction_msgs publish_data;
double DroneAltitude, DroneYaw, DroneX, DroneY, KPitch, KRoll,errorAntPitch,errorAntRoll, VxDrone, VyDrone, DroneBattery;
double PointsDroneFrame[4] = {0.0,0.0,0.0,0.0};
std::vector<double> virtual_fence (5); // [maxX, minX, maxY, minY, maxAltitude]


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
	
	//std::cout << "en el threat" << std::endl;
  	DroneX = msg->rigid_bodies[0].pose.position.x; 
	DroneY = msg->rigid_bodies[0].pose.position.y;
	DroneAltitude = msg->rigid_bodies[0].pose.position.z;
	quaternion[0] = msg->rigid_bodies[0].pose.orientation.x;
	quaternion[1] = msg->rigid_bodies[0].pose.orientation.y;
	quaternion[2] = msg->rigid_bodies[0].pose.orientation.z;
	quaternion[3] = msg->rigid_bodies[0].pose.orientation.w;
	DroneYaw = quaternion2angles(quaternion);/**/
	publish_data.DroneYaw = DroneYaw;
	publish_data.DroneX = DroneX;
	publish_data.DroneY = DroneY;
	publish_data.DroneAltitude = DroneAltitude;

  return;
}

void hasReceivedNavdataInfo(const ardrone_autonomy::NavdataConstPtr msg){
    	//DroneAltitude = (msg->altd)/1000.0;
	VxDrone = msg->vx;
	VyDrone = msg->vy;
	DroneBattery = msg->batteryPercent;
	//publish_data.DroneAltitude = DroneAltitude;
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

void ControlPitch(double actualX, double targetX, double velocity_limit, double Kp, double Kd, double fs){
	
	double vel, error, ts= 1/fs;
	error = (targetX-actualX);
	vel = error*Kp + ((error-errorAntPitch)/ts)*Kd;	
	// Limit velocity
	vel=std::min(velocity_limit,vel);
	vel=std::max(-velocity_limit,vel);
	velocityMsg.linear.x = vel;
	errorAntPitch = error;
	publish_data.PitchVel=velocityMsg.linear.x;	
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
	publish_data.RollVel=velocityMsg.linear.y;	
} 

void ControlAltitude(double actualAlt, double targetAlt){

	double K = 4;
	velocityMsg.linear.z = (targetAlt-actualAlt)/K;
	// Limit velocity
	velocityMsg.linear.z=std::min(1.0,velocityMsg.linear.z);
	velocityMsg.linear.z=std::max(-1.0,velocityMsg.linear.z);	
}
	
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "control_direction_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

velocityMsg.linear.z = 0.0;
velocityMsg.linear.y = 0.0;
velocityMsg.linear.x = 0.0;

double TargetAltitude = 1.0, TargetYaw = 0.0, Kp = 0.2, Kd = 0.0, velocity_limit=0.4, fs=20;
std::vector<double> TargetPoint (2,0);
int  BatteryFlag = 0;

ros::Subscriber optitrack_sub_=nh_.subscribe("/optitrack/rigid_bodies", 1, hasReceivedModelState);
ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Publisher direction_pub_=nh_.advertise<direction_beta::direction_msgs>("/direction_outputs", 1);
ros::Publisher reset_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
ros::Publisher land_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
ros::Subscriber alt_sub = nh_.subscribe("/ardrone/navdata", 1, hasReceivedNavdataInfo);


ros::ServiceClient drone_led =  nh_.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
std_msgs::Empty EmergencyMsg;
ros::Duration(5).sleep();

/* Difinition of the led animation parameters */
ardrone_autonomy::LedAnim srv;
srv.request.freq = fs/5;
srv.request.duration = 0;

	while (ros::ok()){
	nh_.getParam("/control_drone_node/new_position",TargetPoint);
	nh_.getParam("/control_drone_node/new_yaw",TargetYaw);
	nh_.getParam("/control_drone_node/new_altitude",TargetAltitude);	
	nh_.getParam("/control_drone_node/Kp",Kp);
	nh_.getParam("/control_drone_node/Kd",Kd);
	nh_.getParam("/control_drone_node/virtual_fence",virtual_fence);
	nh_.getParam("/control_drone_node/velocity_limit",velocity_limit);


	publish_data.TargetYaw=TargetYaw;
	publish_data.TargetX=TargetPoint[0];
	publish_data.TargetY=TargetPoint[1];
	publish_data.TargetAltitude=TargetAltitude;
	//std::cout << "En el while" << std::endl;


	if (DroneX){
		if (DroneX>virtual_fence[0] ||  DroneX<virtual_fence[1] || DroneY>virtual_fence[2] || DroneY<virtual_fence[3] || DroneAltitude>virtual_fence[4]){

			std::cout << "Emergency! Drone out of fence" << std::endl;
			
			if (abs(VxDrone)<200 || abs(VyDrone)<200)
				land_pub_.publish(EmergencyMsg);
			else{
				reset_pub_.publish(EmergencyMsg);
				srv.request.type = 7; // RED
				drone_led.call(srv);
			}
		
			publish_data.mode = "Emergency";
			ros::Duration(0.5).sleep();
			break;
			}
		else{

			ControlYaw(TargetYaw);
			WorldToDroneframe(TargetPoint[0], TargetPoint[1], DroneYaw);
			ControlPitch(PointsDroneFrame[0],PointsDroneFrame[2],velocity_limit,Kp,Kd,fs);
			ControlRoll(PointsDroneFrame[1],PointsDroneFrame[3],velocity_limit,Kp,Kd,fs);
			ControlAltitude(DroneAltitude, TargetAltitude);
			vel_pub_.publish(velocityMsg);
			publish_data.mode = "Drone controlling";
			}
	
		direction_pub_.publish(publish_data);
		}
	else
		std::cout << "Waiting for optitrack data" << std::endl;

	/* Led animation to alert about low battery */
	if(DroneBattery <= 20 && DroneBattery > 10 && BatteryFlag == 0){
		srv.request.type = 0;  //BLINK_GREEN_RED
		drone_led.call(srv);
		BatteryFlag = 1;
	}
	else if(DroneBattery <= 10 && BatteryFlag == 1){
		srv.request.type = 2; // BLINK_RED
		drone_led.call(srv);
		BatteryFlag = 0;
	}
		
	
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
