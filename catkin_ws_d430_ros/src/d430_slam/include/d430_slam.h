// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#ifndef D430_SLAM_H
#define D430_SLAM_H
#include "d430_camera.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv4/opencv2/core/core.hpp>
#include "System.h"
#include <eigen3/Eigen/Dense>

namespace realsense{
    class Slam{
    public:
        Slam(const ros::NodeHandle& nh);
        ~Slam();
    private:
        ros::NodeHandle nh_;
        std::unique_ptr<D430> d430_camera_ptr_;
        std::unique_ptr<ORB_SLAM3::System> orbslam_ptr_;
        cv::Mat left_img_, right_img_;
        uint64_t time_stamp_;
        ros::Timer timer;
        ros::Publisher pose_pub_;
        geometry_msgs::PoseStamped pose_;
        uint32_t slam_count_;
        void image_grap();
    };
}

#endif
