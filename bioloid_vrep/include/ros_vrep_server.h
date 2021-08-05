#ifndef BIOLOID_VREP_ROS_VREP_SERVER_H
#define BIOLOID_VREP_ROS_VREP_SERVER_H

// ROS Library
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float32.h>


extern void joint_callback_ser(const sensor_msgs::JointState& data);

extern void pos_callback_ser(const tf2_msgs::TFMessage& message);

extern void time_callback_ser(const std_msgs::Float32& simulation_time);

#endif //BIOLOID_VREP_ROS_VREP_SERVER_H
