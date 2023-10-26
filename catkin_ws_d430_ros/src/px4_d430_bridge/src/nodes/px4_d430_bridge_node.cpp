#include "../../include/PX4_d430_bridge.h"

using namespace bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "PX4_d430_bridge_node");
  ros::NodeHandle nh;
  PX4_D430_Bridge Bridge(nh);

  ros::spin();

  Bridge.worker_.join();

  return 0;
}
