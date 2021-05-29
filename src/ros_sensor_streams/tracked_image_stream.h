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
 * @file tracked_image_stream.h
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:31:08 (Mon)
 */

#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/time_synchronizer.h>


#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <ros_sensor_streams/thread_safe_queue.h>
#include <sophus/se3.hpp>

namespace ros_sensor_streams {

/**
 * \brief Class that represents an input stream of tracked images.
 *
 * Designed for use in a nodelet - thus there is no ros::spin() call.
 */
class TrackedImageStream final  {
 public:
  /**
   * @brief Struct to hold image and pose data.
   */
  struct Frame {
    uint32_t id; // Image ID.
    double time; // Timestamp.
    Eigen::Quaternionf quat; // Orientation as quaternion.
    Eigen::Vector3f trans; // Translsation.
    cv::Mat1b img; // gray image.
  };

  /**
   * @brief Constructor for ROS-calibrated image stream.
   *
   * @param[in] world_frame_id Frame ID of the camera world frame.
   * @param[in] nh External nodehandle.
   * @param[in] queue_size Message queue size.
   */
  TrackedImageStream(const std::string& world_frame_id, ros::NodeHandle& nh,
                     int queue_size = 8);

  /**
   * @brief Constructor for un-ROS-calibrated image stream.
   *
   * Use this constructor when CameraInfo messages are not filled in.
   *
   * @param[in] world_frame_id Frame ID of the camera world.
   * @param[in] nh External nodehandle.
   * @param[in] K Camera intrinsices.
   * @param[in] D Camera distortion parameterse (k1, k2, p1, p2, k3).
   * @param[in] undistort True if images should be undistorted before adding to queue.
   * @param[in] resize_factor Downsample image in each dimension by this factor.
   * @param[in] queue_size Message queue size.
   */
  ~TrackedImageStream() = default;

  TrackedImageStream(const TrackedImageStream& rhs) = delete;
  TrackedImageStream& operator=(const TrackedImageStream& rhs) = delete;

  TrackedImageStream(const TrackedImageStream&& rhs) = delete;
  TrackedImageStream& operator=(const TrackedImageStream&& rhs) = delete;

  ThreadSafeQueue<Frame>& queue() {
    return queue_;
  }

  /**
   * \brief Returns true if stream is initialized.
   */
  bool inited() {
    return inited_;
  }

  /**
   * \brief Get the image width.
   */
  int width() {
    return width_;
  }

  /**
   * \brief Get the image height.
   */
  int height() {
    return height_;
  }

  /**
   * \brief Return the camera intrinsic matrix.
   */
  const Eigen::Matrix3f& K() {
    return K_;
  }

  /**
   * @brief Return the distortion params.
   *
   * k1, k2, p1, p2, k3

  const Eigen::VectorXf& D() {
    return D_;
  }*/

  /**
   * \brief Return id of the world frame.
   */
  const std::string& world_frame_id() {
    return world_frame_id_;
  }

  /**
   * \brief Return id of the live frame (i.e. the pose of the camera).
   */
  const std::string& live_frame_id() {
    return live_frame_id_;
  }

 private:

  void imageTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& tf, const sensor_msgs::Image::ConstPtr& rgb_msg);

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, sensor_msgs::Image>
        SyncPolicyImageTransform;
typedef message_filters::Synchronizer<SyncPolicyImageTransform> SynchronizerImageTransform;

    void imageOdomCallback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Image::ConstPtr& rgb_msg);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image>
            SyncPolicyImageOdom;
    typedef message_filters::Synchronizer<SyncPolicyImageOdom> SynchronizerImageOdom;

    void processPoseImage(const sensor_msgs::Image::ConstPtr& rgb_msg);

    ros::NodeHandle& nh_;

  bool inited_;

  // Use an external calibration instead of what's in the camera_info message.
  bool use_external_cal_;
  int resize_factor_; // Factor to resize image. resize_factor_ = 2 will
                      // downsample by 2 in each dimension.
  bool undistort_; // Whether to undistort images.

  std::string world_frame_id_;
  std::string live_frame_id_;
  int width_;
  int height_;
  Eigen::Matrix3f K_; // Camera intrinsics.

  Sophus::SE3f pose_;

  sensor_msgs::CameraInfo::ConstPtr info_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> transform_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

  SynchronizerImageTransform sync_image_transform_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Publisher received_pub_;

  ThreadSafeQueue<Frame> queue_;

};

}  // namespace ros_sensor_streams
