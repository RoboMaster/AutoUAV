// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/offboard_postest.h"

offboard::OffboardPosTest::OffboardPosTest(const ros::NodeHandle& nh):nh_(nh)
{
    uint64_t sys_time=std::chrono::duration_cast<std::chrono::microseconds>
                        (std::chrono::system_clock::now().time_since_epoch()).count();
    int32_t time_second=sys_time / 1000000;
    int32_t time_nsecs=sys_time % 1000000 * 1000;
    waypoints.emplace_back(0.0, -1.0, 2., 0);
    waypoints.emplace_back(0.0, 1.0, 2., 0);
    waypoints.emplace_back(3.0, 1.0, 2., 0);
    waypoints.emplace_back(3.0, -1.0, 2., 0);
    waypointsIdx = 0;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 1.5;
    cmd_.header.stamp.sec = time_second;
    cmd_.header.stamp.nsec = time_nsecs;
    cmd_.coordinate_frame = 1;
    cmd_.type_mask = 0;
    cmd_.header.frame_id="map";
    cmd_.velocity.x = 0.0;
    cmd_.velocity.y = 0.0;
    cmd_.velocity.z = -1;
    state_sub = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffboardPosTest::state_cb, this);
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/d430_camera/pose", 10, &OffboardPosTest::pose_cb, this);
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    vel_pub = nh_.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10); 
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    ros::Rate rate(2.0);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "GUIDED";

    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    }
    ros::Rate rate2(0.5);
    ros::Rate rate3(0.33);
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
    // while(ros::ok()){
    //     ros::spinOnce();
        takeoff_client.call(srv_takeoff);
        // if(srv_takeoff.response.success)break;
        rate3.sleep();
    // }
    ROS_INFO("takeoffed");
    ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    ros::Timer cmd_timer = nh_.createTimer(ros::Duration(0.1), std::bind(&OffboardPosTest::cmd_cb, this), false, true);

    ros::spin();
}

offboard::OffboardPosTest::~OffboardPosTest()
{
    nh_.shutdown();
}

void offboard::OffboardPosTest::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternionf q(msg->pose.orientation.w,msg->pose.orientation.x,
                            msg->pose.orientation.y,msg->pose.orientation.z);
    cur_pose[0] = msg->pose.position.x;
    cur_pose[1] = msg->pose.position.y;
    cur_pose[2] = msg->pose.position.z;
    cur_pose[3] = q.matrix().eulerAngles(0, 1, 2)[2];
    ROS_INFO("cur pose: %f, %f, %f, %f",cur_pose[0],
                    cur_pose[1],cur_pose[2],cur_pose[3]);
    if(std::sqrt((cur_pose[0] - pose.pose.position.x)*(cur_pose[0] - pose.pose.position.x)+
	(cur_pose[1] - pose.pose.position.y)*(cur_pose[1] - pose.pose.position.y)+
	(cur_pose[2] - pose.pose.position.z)*(cur_pose[2] - pose.pose.position.z)) < 0.2)
	{
	    ROS_INFO("reached waypoint.");
        waypointsIdx += 1;
        waypointsIdx %= waypoints.size();
        pose.pose.position.x = waypoints[waypointsIdx][0];
        pose.pose.position.y = waypoints[waypointsIdx][1];
        pose.pose.position.z = waypoints[waypointsIdx][2];
	}
    ROS_INFO("sendingposecmd: %f, %f, %f",pose.pose.position.x,
                    pose.pose.position.y,pose.pose.position.z);
}

void offboard::OffboardPosTest::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
//     ROS_INFO("update state.");
    current_state_ = *msg;
    ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
}

void offboard::OffboardPosTest::cmd_cb()
{
    // vel_pub.publish(cmd_); 
    // ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    // ROS_INFO("sendingvelcmd: %f, %f, %f", cmd_.velocity.x,  cmd_.velocity.y, cmd_.velocity.z);
    local_pos_pub.publish(pose);
    // ROS_INFO("sendingposecmd: %f, %f, %f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
    
}



