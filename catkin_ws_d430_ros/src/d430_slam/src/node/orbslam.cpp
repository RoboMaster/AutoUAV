// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/d430_slam.h"


realsense::Slam::Slam(const ros::NodeHandle& nh):nh_(nh)
{
    slam_count_ = 0;
    d430_camera_ptr_ = std::make_unique<realsense::D430>();
    orbslam_ptr_ = std::make_unique<ORB_SLAM3::System>("/home/nvidia/Softwares/ORB_SLAM3/Vocabulary/ORBvoc.txt", 
                        "/home/nvidia/catkin_ws_d430_ros/src/d430_slam/config/settings.yaml",
                        ORB_SLAM3::System::STEREO, false);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/d430_camera/pose", 1);
    timer = nh_.createTimer(ros::Duration(0.077), std::bind(&Slam::image_grap, this));    
}

realsense::Slam::~Slam()
{
    timer.stop();
    timer.~Timer();
    d430_camera_ptr_->~D430();
    
}

void realsense::Slam::image_grap()
{
    auto res = d430_camera_ptr_->get_stereo_images(left_img_, right_img_, time_stamp_);
    if(res)
    {
        Sophus::SE3f pose =  orbslam_ptr_->TrackStereo(left_img_, right_img_, time_stamp_/1000000000);
        Eigen::Matrix3f Rw2c = pose.matrix3x4().block(0, 0, 3, 3).transpose();//w is the orbslam w;
        Eigen::Vector3f Pw2c = -1*Rw2c*pose.matrix3x4().col(3);
        Eigen::Quaternionf Qw2c(Rw2c);
        Eigen::Quaternionf Qflu2w(0.5, -0.50, 0.50, -0.50);
        Eigen::Vector3f p = Qflu2w*Pw2c;
        Eigen::Quaternionf q = Qflu2w*Qw2c*Qflu2w.inverse();
	    pose_.header.frame_id = "map";
        pose_.header.seq = slam_count_;
        pose_.header.stamp.sec = time_stamp_/1000000000;
        pose_.header.stamp.nsec = time_stamp_;
        pose_.pose.orientation.w = q.w();
        pose_.pose.orientation.x = q.x();
        pose_.pose.orientation.y = q.y();
        pose_.pose.orientation.z = q.z();
        pose_.pose.position.x = p[0];
        pose_.pose.position.y = p[1];
        pose_.pose.position.z = p[2];
        pose_pub_.publish(pose_);
        slam_count_ += 1;
    }
}

