// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
// #include <mavros_msgs/AttitudeTarget.h>
#include <chrono>
#include <eigen3/Eigen/Dense>

namespace offboard
{
    class OffboardPosTest
    {
    public:
        OffboardPosTest(const ros::NodeHandle& nh);
        ~OffboardPosTest();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber state_sub, pose_sub;
        ros::Publisher local_pos_pub;
        ros::Publisher vel_pub;
        ros::ServiceClient arming_client, takeoff_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state_;

        mavros_msgs::PositionTarget cmd_;
        geometry_msgs::PoseStamped pose;

        Eigen::Vector4f cur_pose;
        int waypointsIdx;
        std::vector<Eigen::Vector4f> waypoints;
        // mavros_msgs::AttitudeTarget cmd_att_;
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void cmd_cb();
    };
} 