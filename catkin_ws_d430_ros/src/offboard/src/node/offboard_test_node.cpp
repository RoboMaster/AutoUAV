// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/offboard_rover.h"
#include "../../include/offboard_postest.h"
#include "../../include/offboard_veltest.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_test_node");
  ros::NodeHandle nh;
  offboard::OffboardRover test(nh);
  // offboard::OffboardPosTest test(nh);
  // offboard::OffboardVelTest test(nh);
  return 0;
}
