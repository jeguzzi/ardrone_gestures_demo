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
# define PI           3.14159265358979323846

// Some global variables
float positionX, positionY;
float quaternion[4] = {0,0,0,0};
geometry_msgs::Twist velocityMsg;
direction_beta::direction_msgs publish_data;
double DroneYaw;


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
  	positionX=msg->rigid_bodies[1].pose.position.x; 
	positionY=msg->rigid_bodies[1].pose.position.y;/**/
	quaternion[0] = msg->rigid_bodies[1].pose.orientation.x;
	quaternion[1] = msg->rigid_bodies[1].pose.orientation.y;
	quaternion[2] = msg->rigid_bodies[1].pose.orientation.z;
	quaternion[3] = msg->rigid_bodies[1].pose.orientation.w;
	DroneYaw=quaternion2angles(quaternion);/**/
	publish_data.DroneYaw=DroneYaw;
	publish_data.CurrentX=positionX;
	publish_data.CurrentY=positionY;

  return;
}

double NewYawAngle(double taX, double taY){

	double direction_angle;
	direction_angle=((atan2(taY,taX)*180)/PI);
	return direction_angle;
} 

void ControlYawDirection(double targetA){
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
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "control_direction_node");
ros::NodeHandle nh_;
ros::Rate rate(20.0);

velocityMsg.linear.z = 0.0;
velocityMsg.linear.y = 0.0;

double NewYaw = 0.0;

std::vector<double> new_position (2,0);
std::vector<double> last_position (2,0);


ros::Subscriber optitrack_sub_=nh_.subscribe("/optitrack/rigid_bodies", 1, hasReceivedModelState);
ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Publisher direction_pub_=nh_.advertise<direction_beta::direction_msgs>("/direction_outputs", 1);
ros::Publisher reset_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
std_msgs::Empty EmergencyMsg;
ros::Duration(5).sleep();

	while (ros::ok()){
	nh_.getParam("new_position",new_position);
	//std::cout << "En el while" << std::endl;
	if (positionX){
		if (positionX>-1.6 ||  positionX<-7.0 || positionY>-1.0 || positionY<-5.7){
			//std::cout << "En 1" << std::endl;
			std::cout << "Emergency! Drone out of fence" << std::endl;
			reset_pub_.publish(EmergencyMsg);
			publish_data.mode = 0;
			ros::Duration(0.5).sleep();
			break;
			}
		else if (new_position[0] != last_position[0] || new_position[1] != last_position[1]){
			//std::cout << "En 2" << std::endl;
			NewYaw=NewYawAngle(new_position[0],new_position[1]);
			publish_data.NewYaw=NewYaw;
			publish_data.NewX=new_position[0];
			publish_data.NewY=new_position[1];
			publish_data.LastX=last_position[0];
			publish_data.LastY=last_position[1];

			//std::cout << "NewX " << new_position[0] << " NewY " << new_position[1] << " NewYaw " << NewYaw << std::endl;
			last_position[0]=new_position[0];
			last_position[1]=new_position[1];
			publish_data.mode = 1;
			}
		else{
			//std::cout << "En 3" << std::endl;
			ControlYawDirection(NewYaw);
			vel_pub_.publish(velocityMsg);
			publish_data.mode = 2;
			//std::cout << "X " << positionX << " Y " << positionY << " NewYaw " << NewYaw << std::endl;
			}
	
		direction_pub_.publish(publish_data);
		}
	else
		std::cout << "Waiting for optitrack data" << std::endl;

	//std::cout << "Current yaw " << DroneYaw << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
