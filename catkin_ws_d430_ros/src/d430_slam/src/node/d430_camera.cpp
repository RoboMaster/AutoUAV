// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/d430_camera.h"

realsense::D430::D430()
{
    cameraMatrix = (cv::Mat1d(3, 3) << 512.813, 0, 330.969, 0, 512.813, 235.736, 0, 0, 1); 
    distortionMatrix = (cv::Mat1d(1, 4) << -0.00748422, -0.0175567, 0.000408898, 0.000128157);

    // rs2::context ctx;
    // rs2::device dev = ctx.query_devices()[0];
    // auto sensors = dev.query_sensors();
    // std::cout<<"sensor name: "<<sensors[0].get_info(RS2_CAMERA_INFO_NAME)<<"\n";
    // cfg.device_orientation(rs2.180_DEGREES_CLOCKWISE);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    pipe.start(cfg);
    // pipe.wait_for_frames();
    // rs2::frameset frameSet;
    // bool flag = pipe.try_wait_for_frames(&frameSet, 50);
    // std::cout<<"get frame: "<<int(flag)<<std::endl;
    quit_ = false;
    std::thread waitFrameThread(
        [&]()
        {
            rs2::frameset frameSet;
            while(!quit_) {
                //以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据，
                //并设置帧的等待超时时间为50ms
                if(!pipe.try_wait_for_frames(&frameSet, 100)) {
                    continue;
                }
                std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
                if(lock.try_lock()) {
                    left_image_ = cv::Mat(cv::Size(640, 480), CV_8UC1, (void*)frameSet.get_infrared_frame(1).get_data());
                    right_image_ = cv::Mat(cv::Size(640, 480), CV_8UC1, (void*)frameSet.get_infrared_frame(2).get_data());
                    timestamp_nsec_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    // depth_image_ = cv::Mat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
                    // cv::undistort(cv::Mat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data()), 
                    //     rgb_image_, cameraMatrix, distortionMatrix);
                    // rgb_image_ = cv::Mat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
                    // cv::convertScaleAbs(cv::Mat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data()), depth_image, 1.0f / pow(2, depthFrame->pixelAvailableBitSize() - 10), 0.0);
                    // cv::imshow("left", left_image_);
                    // std::cout<<"depth time stamp: "<<depth_timestamp_ms_<<", rgb time stamp: "<<rgb_timestamp_ms_<<std::endl;
                    // cv::waitKey(1);
                }      
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            pipe.stop();
            std::cout<<"frame receive thread stop.";
        }
    );
    waitFrameThread.detach();
    // ros::Timer image_wrapper = nh_.createTimer(ros::Duration(0.033), std::bind(&orbbec::Orbbec::update_image, this));

    // ros::spin();
}

realsense::D430::~D430()
{
    // ROS_INFO("shutdown.");
    quit_ = true;
    // ctx.~Context();
}

bool realsense::D430::get_stereo_images(cv::Mat& left_img, cv::Mat& right_img, uint64_t& timestamp_nsec)
{
    std::unique_lock<std::mutex> lock(mutex_);
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
    // if(now-timestamp_nsec_ > 100000000)
    // {
    //     std::cout<<"loss images."<<" rgbd time escape: "<<now-timestamp_nsec_<<std::endl;
    //     return false;
    // }
    // cv::convertScaleAbs(depth_image_, depth_image_, 1.0f / 64.0f, 0.0);
    // cv::imshow("depth", depth_image_);
    // cv::imshow("rgb", rgb_image_);
    // cv::waitKey(1);
    // std::cout<<"rgbd time escape: "<<now-rgbd_timestamp_ms_<<std::endl;
    left_img = left_image_.clone();
    right_img = right_image_.clone();
    // cv::convertScaleAbs(depth_image_, depth_image_, 1.0f / 64.0f, 0.0);
    // cv::imshow("depth", depth_image_);
    // cv::imshow("rgb", rgb_img);
    // cv::waitKey(1);
    // std::cout<<"rgbd time escape: "<<now-rgbd_timestamp_ms_<<std::endl;
    timestamp_nsec = timestamp_nsec_;
    return true;
}

