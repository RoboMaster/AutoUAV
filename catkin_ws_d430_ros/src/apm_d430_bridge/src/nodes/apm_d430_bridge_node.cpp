#include "../../include/apm_d430_bridge.h"

using namespace bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "APM_d430_bridge_node");
  ros::NodeHandle nh;
  APM_D430_Bridge Bridge(nh);

  ros::spin();

  Bridge.worker_.join();

  return 0;
}
