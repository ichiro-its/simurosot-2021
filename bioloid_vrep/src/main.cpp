// C++ Standard Library
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>
#include <time.h>

// ROS Library
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>

// Custom C++ Library
#include "../include/vrep_control.h"
#include "../include/ros_vrep_server.h"


int main(int argc, char **argv){
    VrepControl::getInstance().connect();
    VrepControl::getInstance().init_handle();

    ros::init(argc, argv, "vrep_communication");
    ros::NodeHandle nh = ros::NodeHandle();

    ros::Subscriber sub = nh.subscribe("/joint_states", 1, joint_callback_ser);
    ros::Subscriber sub1 = nh.subscribe("/tf", 1,  pos_callback_ser);
    ros::Subscriber sub2 = nh.subscribe("/simulationTime", 1, time_callback_ser);

    ros::spin();
    VrepControl::getInstance().disconnect();
}