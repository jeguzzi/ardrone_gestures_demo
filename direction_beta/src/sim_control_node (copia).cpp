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

double NewYawAngle(double NewX, double NewY){
	double V1[2] = {1,0};
 	double V2[2] = {NewX, NewY};
	double NewYaw;

	NewYaw = acos((V1[0]*V2[0]+V1[1]*V2[1])/(sqrt(pow(V1[0],2.0)+pow(V1[1],2.0))*sqrt(pow(V2[0],2.0)+pow(V2[1],2.0))));
	if(NewY<0)
		NewYaw = ((NewYaw*180)/PI)*-1;
	else
		NewYaw = ((NewYaw*180)/PI);
	
	return NewYaw;
} 

void ControlYawDirection(double NewYaw){

	if (NewYaw-DroneYaw != 0.0){
		velocityMsg.angular.z=(NewYaw-DroneYaw)/180.0;	
	}
	else 
		velocityMsg.angular.z=0.0;
} 
		
int main(int argc, char** argv){
    
ros::init(argc, argv, "sim_control_direction_node");
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
ros::Publisher reset_pub_=nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
std_msgs::Empty EmergencyMsg;

	while (ros::ok()){
	nh_.getParam("new_position",new_position);
	//std::cout << "En el while" << std::endl;

	if (positionX>5.0 ||  positionX<-3.8 || positionY>4.2 || positionY<-3.9){
		//std::cout << "En 1" << std::endl;
		reset_pub_.publish(EmergencyMsg);
		publish_data.mode = 0;
		std::cout << "NewX " << positionX << " NewY " << positionY << std::endl;
		}
	else if (new_position[0] != last_position[0] || new_position[1] != last_position[1]){
		//std::cout << "En 2" << std::endl;

		NewYaw=NewYawAngle(new_position[0],new_position[1]);
		publish_data.NewYaw=NewYaw;
		publish_data.NewX=new_position[0];
		publish_data.NewY=new_position[1];
		publish_data.LastX=last_position[0];
		publish_data.LastY=last_position[1];

		std::cout << "NewX " << new_position[0] << " NewY " << new_position[1] << " NewYaw " << NewYaw << std::endl;
		last_position[0]=new_position[0];
		last_position[1]=new_position[1];
		publish_data.mode = 1;
		}
	else{
		//std::cout << "En 3" << std::endl;
		ControlYawDirection(NewYaw);
		vel_pub_.publish(velocityMsg);
		publish_data.mode = 2;
		}

	direction_pub_.publish(publish_data);

	//std::cout << "Current yaw " << DroneYaw << std::endl;
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
