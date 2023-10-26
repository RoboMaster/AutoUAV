// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/offboard_veltest.h"

offboard::OffboardVelTest::OffboardVelTest(const ros::NodeHandle& nh):nh_(nh)
{
    last_z = 0.0;
    velXPID_ptr = std::make_unique<RM::PIDController>(0.5, 0.0015, 0.0005, 2.5);
    velYPID_ptr = std::make_unique<RM::PIDController>(0.6, 0.0018, 0.00045, 2.5);
    velZPID_ptr = std::make_unique<RM::PIDController>(0.5, 0.0003, 0.0001, 1.50);
    velYawPID_ptr = std::make_unique<RM::PIDController>(0.5, 0.01, 0.0, 1.575);
    uint64_t sys_time=std::chrono::duration_cast<std::chrono::microseconds>
                        (std::chrono::system_clock::now().time_since_epoch()).count();
    int32_t time_second=sys_time / 1000000;
    int32_t time_nsecs=sys_time % 1000000 * 1000;
    // waypoints.emplace_back(0.0, 0.0, 2.0, -0.707);
    // waypoints.emplace_back(0.0, 0.0, 2.0, 0.707);
    waypoints.emplace_back(0.0, -1.0, 2., 0);
    waypoints.emplace_back(0.0, 1.0, 2., 0);
    waypoints.emplace_back(3.0, 1.0, 2., 0);
    waypoints.emplace_back(3.0, -1.0, 2., 0);
    // waypoints.emplace_back(0.0, 0.0, 1.5, 0);
    waypointsIdx = 0;
    cmd_.header.stamp.sec = time_second;
    cmd_.header.stamp.nsec = time_nsecs;
    cmd_.coordinate_frame = 1; // wiki 说是ned坐标系，实测还是enu.
    // cmd_.type_mask = 0;
    cmd_.type_mask = mavros_msgs::PositionTarget::IGNORE_PX|
                        mavros_msgs::PositionTarget::IGNORE_PY|
                        mavros_msgs::PositionTarget::IGNORE_PZ|
                        mavros_msgs::PositionTarget::IGNORE_YAW;
    // cmd_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX|
    //                      mavros_msgs::PositionTarget::IGNORE_VY|
    //                      mavros_msgs::PositionTarget::IGNORE_VZ|
    //                      mavros_msgs::PositionTarget::IGNORE_YAW|
    //                      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    cmd_.header.frame_id="map";
    cmd_.velocity.x = 0.0;
    cmd_.velocity.y = 0.0;
    cmd_.velocity.z = 0.0;
    cmd_.yaw = 0.0;
    cmd_.position.x = 0.0;
    cmd_.position.y = 0.0;
    cmd_.position.z = 1.3;
    state_sub = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffboardVelTest::state_cb, this);
    // pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>
    //         ("/d430_camera/pose", 10, &OffboardVelTest::pose_cb, this);
    pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &OffboardVelTest::pose_cb, this);
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
    ros::Rate rate3(0.2);
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
    ros::Timer cmd_timer = nh_.createTimer(ros::Duration(0.02), std::bind(&OffboardVelTest::cmd_cb, this), false, true);

    ros::spin();
}

offboard::OffboardVelTest::~OffboardVelTest()
{
    nh_.shutdown();
}

void offboard::OffboardVelTest::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternionf q(msg->pose.orientation.w,msg->pose.orientation.x,
                            msg->pose.orientation.y,msg->pose.orientation.z);
    cur_pose[0] = msg->pose.position.x;
    cur_pose[1] = msg->pose.position.y;
    cur_pose[2] = msg->pose.position.z;
    cur_pose[3] = std::atan2(2.0*(q.w()*q.z()+q.y()*q.x()), 1.0-2.0*(q.z()*q.z()+q.y()*q.y()));
    if(cur_pose[3]<0)cur_pose[3]+=6.283;
    // if(std::abs(cur_pose[2]-last_z)>0.2)cur_pose[2] = last_z + 0.01*(cur_pose[2]-last_z);
    last_z = cur_pose[2];
    ROS_INFO("cur pose: %f, %f, %f, %f",cur_pose[0],
                    cur_pose[1],cur_pose[2],cur_pose[3]);
    if(std::sqrt((cur_pose[0] - waypoints[waypointsIdx][0])*(cur_pose[0] - waypoints[waypointsIdx][0])+
	(cur_pose[1] - waypoints[waypointsIdx][1])*(cur_pose[1] - waypoints[waypointsIdx][1])+
	(cur_pose[2] - waypoints[waypointsIdx][2])*(cur_pose[2] - waypoints[waypointsIdx][2])) < 0.2)
	{
        float e = 0.0;
        if(waypoints[waypointsIdx][3]-cur_pose[3]<-3.1415926)e=std::abs(waypoints[waypointsIdx][3]-cur_pose[3]+6.283);
        else if(waypoints[waypointsIdx][3]-cur_pose[3]>3.1415926)e=std::abs(waypoints[waypointsIdx][3]-cur_pose[3]-6.283);
	    else e = std::abs(waypoints[waypointsIdx][3]-cur_pose[3]);
        if(e<0.2)
	    {
	    ROS_INFO("reached waypoint: %f, %f, %f, %f", waypoints[waypointsIdx][0],
            waypoints[waypointsIdx][1],waypoints[waypointsIdx][2],waypoints[waypointsIdx][3]);
            waypointsIdx += 1;
            waypointsIdx %= waypoints.size();
	    }
	}
    float x = velXPID_ptr->compute(cur_pose[0], waypoints[waypointsIdx][0]);
    cmd_.velocity.x = x;
    float y = velYPID_ptr->compute(cur_pose[1], waypoints[waypointsIdx][1]);
    cmd_.velocity.y = y;
    float z = velZPID_ptr->compute(cur_pose[2], waypoints[waypointsIdx][2]);
    cmd_.velocity.z = z;
    if(waypoints[waypointsIdx][3] - cur_pose[3] > 3.14159) cur_pose[3] += 6.283;
    if(waypoints[waypointsIdx][3] - cur_pose[3] < -3.14159) cur_pose[3] -= 6.283;
    float yaw = velYawPID_ptr->compute(cur_pose[3], waypoints[waypointsIdx][3]);
    cmd_.yaw_rate = yaw;
    // ROS_INFO("sendingposecmd: %f, %f, %f",pose.pose.position.x,
    //                 pose.pose.position.y,pose.pose.position.z);
}

void offboard::OffboardVelTest::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
//     ROS_INFO("update state.");
    current_state_ = *msg;
    ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
}

void offboard::OffboardVelTest::cmd_cb()
{
    vel_pub.publish(cmd_); 
    // ROS_INFO("connetced: %d, armed: %d, mode: %s.", current_state_.connected, current_state_.armed, current_state_.mode.c_str());
    // ROS_INFO("sendingvelcmd: %f, %f, %f, %f", cmd_.velocity.x,  cmd_.velocity.y, cmd_.velocity.z, cmd_.yaw_rate);
    // local_pos_pub.publish(pose);
    // ROS_INFO("sendingposecmd: %f, %f, %f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
    
}



