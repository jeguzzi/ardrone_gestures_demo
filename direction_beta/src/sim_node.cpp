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
float positionX, positionY, r, directionX, directionY;
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
	
  	positionX=msg->rigid_bodies[0].pose.position.x; 
	positionY=msg->rigid_bodies[0].pose.position.y;/**/
	quaternion[0] = msg->rigid_bodies[0].pose.orientation.x;
	quaternion[1] = msg->rigid_bodies[0].pose.orientation.y;
	quaternion[2] = msg->rigid_bodies[0].pose.orientation.z;
	quaternion[3] = msg->rigid_bodies[0].pose.orientation.w;
	DroneYaw=quaternion2angles(quaternion);/**/
	publish_data.DroneYaw=DroneYaw;
	publish_data.CurrentX=positionX;
	publish_data.CurrentY=positionY;

  return;
}

void sendPitchDirection(std::vector<double> new_position){
	
	velocityMsg.angular.z = 0.0;
	velocityMsg.linear.y = 0.0;
	velocityMsg.linear.z = 0.0;

	if (positionX<=new_position[0]-1 || positionX>=new_position[0]+1 || positionY>=new_position[1]+1 || positionY<=new_position[1]-1 )
		velocityMsg.linear.x = 0.2;
	else
		velocityMsg.linear.x = 0.05;
	publish_data.PitchVel=velocityMsg.angular.z;
}


void sendYawDirection(double NewYaw){

	velocityMsg.linear.x = 0.0;
	velocityMsg.linear.y = 0.0;
	velocityMsg.linear.z = 0.0;

	if (NewYaw-25>DroneYaw || NewYaw+25<=DroneYaw)	
		velocityMsg.angular.z=0.2;
	else
		velocityMsg.angular.z=0.05;
	publish_data.YawVel=velocityMsg.angular.z;
}


double NewDirection(double Lx,double Ly, double Nx, double Ny){

	double NewYaw, X, Y; 

	X = Nx-Lx;
	Y = Ny-Ly;

	if(X>0.0 && Y>0.0){
		NewYaw = (atan(Y/X)*180)/PI;
		std::cout << "First " << NewYaw << std::endl;}
	else if (X<0.0 && Y>0.0){
		std::cout << "Second " << ((atan(abs(X)/Y)*180)/PI) << std::endl;
		NewYaw = ((atan(abs(X)/Y)*180)/PI)+90;}
	else if (X<0.0 && Y<0.0){
		NewYaw = (atan(Y/X)*180)/PI-180;
		std::cout << "Third " <<  NewYaw << std::endl;}
	else if (X>0.0 && Y<0.0){
		NewYaw = (atan(X/Y)*180)/PI;
		std::cout << "Fourth " << NewYaw << std::endl;}
	else if (X == 0.0 && Y>0.0)
		NewYaw = 90.0;
	else if (X == 0.0 && Y<0.0)
		NewYaw = -90.0;
	else if (X < 0.0 && Y == 0.0)
		NewYaw = 180.0;	
	else 
		NewYaw = 0.0;

	std::cout << "X " << X << " Y " << Y << " NewYaw " << NewYaw << std::endl;
	publish_data.X = X;
	publish_data.Y = Y;
	publish_data.NewYaw = NewYaw;	
	return NewYaw;
} 
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "direction_sim_node");
ros::NodeHandle nh_;
ros::Rate rate(120.0);

velocityMsg.linear.z = 0.0;
velocityMsg.linear.y = 0.0;

double NewYaw = 0.0;

std::vector<double> new_position (2,0);
std::vector<double> last_position (2,0);


ros::Subscriber optitrack_sub_=nh_.subscribe("/optitrack/rigid_bodies", 1, hasReceivedModelState);
ros::Publisher vel_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Publisher direction_pub_=nh_.advertise<direction_beta::direction_msgs>("/direction_outputs", 1);
ros::Publisher takeOff_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
std_msgs::Empty EmergencyMsg;

	while (ros::ok()){
	nh_.getParam("new_position",new_position);

	if (positionX>5.0 ||  positionX<-3.8 || positionY>4.2 || positionY<-3.9){
		takeOff_pub_.publish(EmergencyMsg);
		std::cout << "Atrapado " << std::endl;
		std::cout << "NewX " << positionX << " NewY " << positionY << std::endl;
		publish_data.mode = 0;
		}
		
	else if (new_position[0] != last_position[0] || new_position[1] != last_position[1]){
		std::cout << "NewX " << new_position[0] << " NewY " << new_position[1] << std::endl;
		NewYaw=NewDirection(last_position[0],last_position[1],new_position[0],new_position[1]);
		// Esperar

		publish_data.NewX=new_position[0];
		publish_data.NewY=new_position[1];
		publish_data.LastX=last_position[0];
		publish_data.LastY=last_position[1];

		last_position[0]=new_position[0];
		last_position[1]=new_position[1];
		publish_data.mode = 1;
		//std::cout << "Estoy en 1 " << std::endl;
	}

	else if (NewYaw-5>DroneYaw || NewYaw+5<DroneYaw){
		sendYawDirection(NewYaw);
		vel_pub_.publish(velocityMsg);
		publish_data.mode = 2;
		//std::cout << "Estoy en 2 " << std::endl;
		}
		
	else if (positionX<=new_position[0]-0.2 || positionX>=new_position[0]+0.2 || positionX<=new_position[0]-0.2 || positionY<=new_position[1]-0.2){
		sendPitchDirection(new_position);
		vel_pub_.publish(velocityMsg);
		//std::cout << "Estoy en 3 " << std::endl;
		publish_data.mode = 3;
		}
	else{
		velocityMsg.linear.x = 0.0;
		velocityMsg.linear.y = 0.0;
		velocityMsg.linear.z = 0.0;
		velocityMsg.angular.z = 0.0;
		publish_data.mode = 4;}
		//std::cout << "Estoy en 4 " << std::endl;

	direction_pub_.publish(publish_data);
		

	//std::cout << "Current yaw " << DroneYaw << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
