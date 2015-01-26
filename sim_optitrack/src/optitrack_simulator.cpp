#include <iostream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <optitrack_msgs/RigidBodyData.h>
#include <optitrack_msgs/RigidBodies.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
    
  ros::init(argc, argv, "sim_optitrack_node");
  ros::NodeHandle nh;
  ros::Rate rate(120.0);
  ros::Publisher rigid_bodies_publisher = nh.advertise<optitrack_msgs::RigidBodies>("/optitrack/rigid_bodies", 1);
  std::vector<std::string> rigid_bodies_name;
  std::map<std::string,ros::Publisher> pose_publishers;

  nh.getParam("/optitrack/rigid_bodies_name", rigid_bodies_name);
  
  std::vector<std::string>::const_iterator it=rigid_bodies_name.begin();
  for(;it!=rigid_bodies_name.end();it++)
    {

      //std::cout<<"R B "<<*it<<std::endl;
        ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/optitrack/"+*it, 1);
	pose_publishers[*it]=pub;
    }

  uint32_t seq=0;
  std_msgs::Header header;
  header.frame_id = "World";

  while (nh.ok()){

    header.stamp = ros::Time::now();
    header.seq = seq;

    gazebo_msgs::GetModelState getmodelstate;
    optitrack_msgs::RigidBodies rigid_bodies_msg;
    //rigid_bodies_msg.header = header;
	
    rigid_bodies_msg.header=header;

    for(unsigned i=0; i < rigid_bodies_name.size(); i++) {

      std::string name=rigid_bodies_name[i];

      getmodelstate.request.model_name=name;
      ros::ServiceClient gmscl = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gmscl.call(getmodelstate);

      geometry_msgs::Pose pose;
      pose.position.x = getmodelstate.response.pose.position.x;
      pose.position.y = getmodelstate.response.pose.position.y;
      pose.position.z = getmodelstate.response.pose.position.z;
      pose.orientation.x = getmodelstate.response.pose.orientation.x;
      pose.orientation.y = getmodelstate.response.pose.orientation.y;
      pose.orientation.z = getmodelstate.response.pose.orientation.z;
      pose.orientation.w = getmodelstate.response.pose.orientation.w;

      
      //std::cout<<name<<" : "<<getmodelstate.response<<std::endl;
      //std::cout<<getmodelstate.response.pose<<std::endl<<pose<<std::endl;

      optitrack_msgs::RigidBodyData data;
      // Id 
      data.id = i;
      data.name =name;	
      data.mean_error =0.0;
      data.pose=pose;

      rigid_bodies_msg.rigid_bodies.push_back(data);

      geometry_msgs::PoseStamped msg;
      msg.header=header;
      msg.pose=pose;
      pose_publishers[name].publish(msg);
    }
    rigid_bodies_publisher.publish(rigid_bodies_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
