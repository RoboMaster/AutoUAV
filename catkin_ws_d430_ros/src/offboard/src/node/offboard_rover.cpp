// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/offboard_rover.h"

offboard::OffboardRover::OffboardRover(const ros::NodeHandle& nh):nh_(nh)
{
    uint64_t sys_time=std::chrono::duration_cast<std::chrono::microseconds>
                        (std::chrono::system_clock::now().time_since_epoch()).count();
    int32_t time_second=sys_time / 1000000;
    int32_t time_nsecs=sys_time % 1000000 * 1000;
    waypoints.emplace_back(0.0, 0.0, 1.5, 0.0);
    cmd_.header.stamp.sec = time_second;
    cmd_.header.stamp.nsec = time_nsecs;
    cmd_.coordinate_frame = 1;
    cmd_.type_mask = 0;
    cmd_.header.frame_id="map";
    cmd_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX|
                    mavros_msgs::PositionTarget::IGNORE_VY|
                    mavros_msgs::PositionTarget::IGNORE_VZ|
                    mavros_msgs::PositionTarget::IGNORE_AFX|
                    mavros_msgs::PositionTarget::IGNORE_AFY|
                    mavros_msgs::PositionTarget::IGNORE_AFZ|
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    cmd_.position.x = 0.0;
    cmd_.position.y = 0.0;
    cmd_.position.z = 1.5;
    cmd_.yaw_rate = 0.7;
    state_sub = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffboardRover::state_cb, this);
    // pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>
    //         ("/d430_camera/pose", 10, &OffboardVelTest::pose_cb, this);
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &OffboardRover::pose_cb, this);
    cmd_pub = nh_.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10); 
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    ros::Rate rate(2.0);
    ros::Rate rate2(0.5);
    ros::Rate rate3(0.2);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "GUIDED";

    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    }
    while(ros::ok() && current_state_.mode != "GUIDED"){
        ros::spinOnce();
        if(set_mode_client.call(offb_set_mode))break;
        rate2.sleep();
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while(ros::ok() && !current_state_.armed){
        ros::spinOnce();
        arming_client.call(arm_cmd);
        if(arm_cmd.response.success)break;
        rate2.sleep();
    }
    rate.sleep();
    ROS_INFO("connected to FCU!");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 1.5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    takeoff_client.call(srv_takeoff);
    rate3.sleep();
    ROS_INFO("takeoffed");
    ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    ros::Timer cmd_timer = nh_.createTimer(ros::Duration(0.1), std::bind(&OffboardRover::cmd_cb, this), false, true);

    ros::spin();
}

offboard::OffboardRover::~OffboardRover()
{
    nh_.shutdown();
}

void offboard::OffboardRover::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternionf q(msg->pose.orientation.w,msg->pose.orientation.x,
                            msg->pose.orientation.y,msg->pose.orientation.z);
    cur_pose[0] = msg->pose.position.x;
    cur_pose[1] = msg->pose.position.y;
    cur_pose[2] = msg->pose.position.z;
    cur_pose[3] = q.matrix().eulerAngles(0, 1, 2)[2];
}

void offboard::OffboardRover::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
}

void offboard::OffboardRover::cmd_cb()
{
    cmd_pub.publish(cmd_);
    ROS_INFO("cur pose: %f, %f, %f, %f",cur_pose[0],
                cur_pose[1],cur_pose[2],cur_pose[3]);
    // ROS_INFO("sendingcmd: %f, %f, %f, %f", cmd_.position.x,  cmd_.position.y, cmd_.position.z, cmd_.yaw);
}



