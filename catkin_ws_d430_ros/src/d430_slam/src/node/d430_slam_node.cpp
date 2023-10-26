// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/d430_slam.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "d430_node");
  ros::NodeHandle nh;
  realsense::Slam slam_node(nh);
  ros::spin();
  return 0;
}