/**
 * This file is part of flame_ros.
 * Copyright (C) 2017 W. Nicholas Greene (wng@csail.mit.edu)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * @file tracked_image_stream.cc
 * @author W. Nicholas Greene
 * @date 2016-12-14 11:25:22 (Wed)
 */

#include "ros_sensor_streams/tracked_image_stream.h"

#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sophus/se3.hpp>

#include <cv_bridge/cv_bridge.h>

#include "ros_sensor_streams/conversions.h"

namespace ros_sensor_streams {


TrackedImageStream::TrackedImageStream(const std::string& world_frame_id,
                                       ros::NodeHandle& nh,
                                       int queue_size) :
    nh_(nh),
    inited_(false),
    use_external_cal_(false),
    resize_factor_(1),
    undistort_(false),
    world_frame_id_(world_frame_id),
    live_frame_id_(),
    width_(0),
    height_(0),
    K_(),
    D_(5),
    //tf_buffer_(),/ca

    //image_transport_(nullptr),
    //cam_sub_(),
    //transform_sub_(),
    queue_(queue_size),
    image_sub_(nh_, "/camera/rgb/image_rect", 50),
    //tf_filter_(transform_sub_,tf_buffer_, "world", 50, nh_),
    cam_info_sub_(nh_, "/camera/rgb/camera_info", 50),
    transform_sub_(nh_, "/transform_rgb_frame", 50),

    sync_image_transform_(TrackedImageStream::SyncPolicyImageTransform(100), transform_sub_, image_sub_, cam_info_sub_){
  // Subscribe to topics.
  //image_transport::ImageTransport it_(nh_);
  //image_transport_.reset(new image_transport::ImageTransport(nh_));

  //cam_sub_ = image_transport_->subscribeCamera("image", 50,
  //                                             &TrackedImageStream::callback,
  //                                             this);

    //openni_rgb_frame
  //tf_filter_.registerCallback([](const geometry_msgs::TransformStamped::ConstPtr& tf){std::cout<<"got transform"<<std::endl;});
  //transform_sub_.registerCallback([](const geometry_msgs::TransformStamped::ConstPtr& tf){std::cout<<"got transform sub"<<std::endl;});
  //image_sub_.registerCallback([](const sensor_msgs::Image::ConstPtr& rgb_msg){std::cout<<"got image"<<std::endl;});
  //cam_info_sub_.registerCallback([](const sensor_msgs::CameraInfo::ConstPtr& info){std::cout<<"got cam_info"<<std::endl;});

  sync_image_transform_.registerCallback(boost::bind(&TrackedImageStream::imageTransformCallback, this, _1, _2, _3));

  return;
}

/*void TrackedImageStream::callback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                                  const sensor_msgs::CameraInfo::ConstPtr& info) {
  ROS_DEBUG("Received image data!");

  // Grab rgb data.
  cv::Mat3b rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

  assert(rgb.isContinuous());

  if (resize_factor_ != 1) {
    cv::Mat3b resized_rgb(static_cast<float>(rgb.rows)/resize_factor_,
                          static_cast<float>(rgb.cols)/resize_factor_);
    cv::resize(rgb, resized_rgb, resized_rgb.size());
    rgb = resized_rgb;
  }

  if (!inited_) {
    live_frame_id_ = rgb_msg->header.frame_id;

    // Set calibration.
    width_ = rgb.cols;
    height_ = rgb.rows;

    if (!use_external_cal_) {
      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          K_(ii, jj) = info->P[ii*4 + jj];
        }
      }

      if (K_(0, 0) <= 0) {
        ROS_ERROR("Camera intrinsics matrix is probably invalid!\n");
        ROS_ERROR_STREAM("K = " << std::endl << K_);
        return;
      }

      for (int ii = 0; ii < 5; ++ii) {
        D_(ii) = info->D[ii];
      }
    }

    inited_ = true;

    ROS_DEBUG("Set camera calibration!");
  }

  if (undistort_) {
    cv::Mat1f Kcv, Dcv;
    cv::eigen2cv(K_, Kcv);
    cv::eigen2cv(D_, Dcv);
    cv::Mat3b rgb_undistorted;
    cv::undistort(rgb, rgb_undistorted, Kcv, Dcv);
    rgb = rgb_undistorted;
  }

  Img img;
  img.id = rgb_msg->header.seq;
  img.time = rgb_msg->header.stamp.toSec();
  //frame.quat = pose.unit_quaternion();
  //frame.trans = pose.translation();
  img.img = rgb;

  if(!image_queue_.push(img)){
      image_queue_.pop();
      image_queue_.push(img);
  }

  return;
}

void TrackedImageStream::tfCallback(const geometry_msgs::TransformStamped::ConstPtr& tf) {
    ROS_DEBUG("Received transform data!");
    std::cout<<"recieving transform..."<<std::endl;

    Sophus::SE3f pose;
    tfToSophusSE3<float>(tf->transform, &pose);

    Pose POSE;
    //img.id = rgb_msg->header.seq;
    //POSE.id = tf->header.seq;
    POSE.time = tf->header.stamp.toSec();
    POSE.quat = pose.unit_quaternion();
    POSE.trans = pose.translation();
    //img.img = rgb;

    pose_queue_.push(POSE);

    return;
}*/

void TrackedImageStream::imageTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& tf, const sensor_msgs::Image::ConstPtr& rgb_msg,
                                                const sensor_msgs::CameraInfo::ConstPtr& info) {
    ROS_DEBUG("Received data!");
    // Grab rgb data.
    cv::Mat1b gray_img = cv_bridge::toCvCopy(rgb_msg, "mono8")->image;

    assert(gray_img.isContinuous());

    if (resize_factor_ != 1) {
        cv::Mat1b resized_rgb(static_cast<float>(gray_img.rows) / resize_factor_,
                              static_cast<float>(gray_img.cols) / resize_factor_);
        cv::resize(gray_img, resized_rgb, resized_rgb.size());
        gray_img = resized_rgb;
    }

    if (!inited_) {
        live_frame_id_ = rgb_msg->header.frame_id;

        // Set calibration.
        width_ = gray_img.cols;
        height_ = gray_img.rows;

        if (!use_external_cal_) {
            for (int ii = 0; ii < 3; ++ii) {
                for (int jj = 0; jj < 3; ++jj) {
                    K_(ii, jj) = info->P[ii * 4 + jj];
                }
            }

            if (K_(0, 0) <= 0) {
                ROS_ERROR("Camera intrinsics matrix is probably invalid!\n");
                ROS_ERROR_STREAM("K = " << std::endl << K_);
                return;
            }

            for (int ii = 0; ii < 5; ++ii) {
                D_(ii) = info->D[ii];
            }
        }

        inited_ = true;

        ROS_DEBUG("Set camera calibration!");
    }

    Sophus::SE3f pose;
    tfToSophusSE3<float>(tf->transform, &pose);

    Frame FRAME;
    FRAME.id = rgb_msg->header.seq;
    FRAME.time = tf->header.stamp.toSec();
    FRAME.quat = pose.unit_quaternion();
    FRAME.trans = pose.translation();
    FRAME.img = gray_img;

    queue_.push(FRAME);

    return;
}
}  // namespace ros_sensor_streams
