#ifndef PX4_D430_BRIDGE
#define PX4_D430_BRIDGE

#include <mavros_msgs/CompanionProcessStatus.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
namespace bridge {

  enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class PX4_D430_Bridge {
 public:
  PX4_D430_Bridge(const ros::NodeHandle& nh);
  ~PX4_D430_Bridge();

  void publishSystemStatus();

  std::thread worker_;


 private:
  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber pose_sub_;
  // Publishers
  ros::Publisher mavros_odom_pub_, mavros_pose_pub_;
  ros::Publisher mavros_system_status_pub_;
  nav_msgs::Odometry output;
  MAV_STATE system_status_{MAV_STATE::MAV_STATE_UNINIT};
  MAV_STATE last_system_status_{MAV_STATE::MAV_STATE_UNINIT};

  std::unique_ptr<std::mutex> status_mutex_;

  void poseCallback(const geometry_msgs::PoseStamped& msg);

  bool flag_first_pose_received{false};

  ros::Time last_callback_time;

};
}
#endif  // PX4_REALSENSE_BRIDGE
