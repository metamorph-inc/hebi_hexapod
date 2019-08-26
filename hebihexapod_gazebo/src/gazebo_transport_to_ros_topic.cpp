/*
*  Name: gazebo_transport_to_ros_topic.cpp
*  Author: Joseph Coombe
*  Date: 11/22/2017
*  Edited: 11/27/2017
*  Description:
*   Subscribe to a Gazebo transport topic and publish to a ROS topic
*/

// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <iostream>

ros::Publisher pub;

void torquesCb(const ConstWrenchStampedPtr &_msg)
{
  std::cout << "Received msg: " << std::endl;
  std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _msg->wrench().force().x();
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _msg->wrench().force().z();
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  pub.publish(msgWrenchedStamped);
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting gazebo_transport_to_ros_topic node");

  // Load Gazebo
  gazebo::client::setup(argc, argv);

  // Load ROS
  ros::init(argc, argv, "gazebo_transport_to_ros_topic");

  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle n;

  // Get ROS params
  std::string gazebo_transport_topic_to_sub;
  std::string ros_topic_to_pub;
  double ros_rate;
  ros::param::get("~gazebo_transport_topic_to_sub", gazebo_transport_topic_to_sub); // e.g: "/gazebo/default/hebihexapod_01_a_rough/HEBI_X5_4_hip_leg1_to_HEBI_X5_9_knee_leg1_joint/leg1_hip_force_torque/wrench"
  ros::param::get("~ros_topic_to_pub", ros_topic_to_pub);                           // e.g: "hebihexapod/joint_wrench"
  ros::param::get("~ros_rate", ros_rate);                                           // e.g. 100.0

  pub = n.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub, 100);

  // Listen to Gazebo force_torque sensor topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe(gazebo_transport_topic_to_sub, torquesCb);
  ros::Rate loop_rate(ros_rate); // 100 hz
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  gazebo::shutdown();
  return 0;
}