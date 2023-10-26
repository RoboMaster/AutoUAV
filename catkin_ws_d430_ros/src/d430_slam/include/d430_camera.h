// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#ifndef ORBBEC_CAMERA_H
#define ORBBEC_CAMERA_H

// #include <ros/ros.h>
#include <memory>
#include <thread>
#include <mutex>
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/calib3d.hpp"
#include <iostream>
#include <chrono>
#include <librealsense2/rs.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>

namespace realsense{
    class D430{
    public:
        D430();
        ~D430();
        bool get_stereo_images(cv::Mat& left_img, cv::Mat& right_img, uint64_t& timestamp_nsec_);
    private:
        // ros::NodeHandle nh_;
        // std::unique_ptr<image_transport::ImageTransport> it;
        // image_transport::Publisher rgb_publisher, depth_publisher;
        // ob::Context ctx;
        rs2::pipeline pipe;
        rs2::config cfg;
        bool quit_;
        std::mutex mutex_;
        cv::Mat left_image_, right_image_;
        uint64_t timestamp_nsec_;
        cv::Mat cameraMatrix, distortionMatrix;
        // void update_image();
    };
}

#endif

