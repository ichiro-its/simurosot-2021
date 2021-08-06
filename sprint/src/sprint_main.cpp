// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <aruku/walking.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Please specify the config path!" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "joint_state_node");
  ros::NodeHandle ros_node;
  ros::Publisher joint_state_publisher = ros_node.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Rate ros_rate(8);

  std::string path = argv[1];

  auto walking = std::make_shared<aruku::Walking>();
  walking->load_data(path);
  walking->initialize();
  walking->start();

  std::map<std::string, int> joints_index = walking->get_joints_index();
  std::vector<std::string> joints_name = {
    "joint_0_1",
    "joint_0_2",
    "joint_0_3",
    "joint_0_4",
    "joint_0_5",
    "joint_0_6",
    "joint_0_7",
    "joint_0_8",
    "joint_0_9",
    "joint_0_10",
    "joint_0_11",
    "joint_0_12",
    "joint_0_13",
    "joint_0_14",
    "joint_0_15",
    "joint_0_16",
    "joint_0_17",
    "joint_0_18",
    "joint_0_19",
    "joint_0_20",
    "joint_0_21"
  };

  std_msgs::Header header;
  sensor_msgs::JointState joints_state_msg;
  joints_state_msg.header = header;
  joints_state_msg.header.stamp = ros::Time::now();
  joints_state_msg.name = joints_name;

  while (ros::ok()) {
    walking->process();
    std::vector<float> joints = walking->get_joints();

    std::vector<double> joints_state;
    for (auto joint_name : joints_name) {
      if (joints_index.find(joint_name) != joints_index.end()) {
        joints_state.push_back(joints.at(joints_index.at(joint_name)));
      } else {
        joints_state.push_back(0.0);
      }
    }

    joints_state_msg.position = joints_state;
    joint_state_publisher.publish(joints_state_msg);

    ros::spinOnce();
    ros_rate.sleep();
  }

  walking->stop();
  return 0;
}
