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
    queue_(queue_size),
    image_sub_(nh_, "/image", 15),
    cam_info_sub_(nh_, "/info", 15),
    transform_sub_(nh_, "/transform", 15),
    odom_sub_(nh_, "/odom", 15),
    received_pub_(nh.advertise<std_msgs::Header>("received", 500)),
    sync_image_transform_(TrackedImageStream::SyncPolicyImageTransform(15), transform_sub_, image_sub_),
    sync_image_odom_(TrackedImageStream::SyncPolicyImageOdom(15), odom_sub_, image_sub_)
{
    //transform_sub_.registerCallback([](const geometry_msgs::TransformStamped::ConstPtr& tf){std::cout<<"got transform sub "<<tf->header.stamp<<std::endl;});
    //image_sub_.registerCallback([](const sensor_msgs::Image::ConstPtr& rgb_msg){std::cout<<"got image "<<rgb_msg->header.stamp<<std::endl;});
    cam_info_sub_.registerCallback([this](const sensor_msgs::CameraInfo::ConstPtr& info){info_=info;});
    sync_image_transform_.registerCallback(boost::bind(&TrackedImageStream::imageTransformCallback, this, _1, _2));
    sync_image_odom_.registerCallback(boost::bind(&TrackedImageStream::imageOdomCallback, this, _1, _2));

  return;
}

void TrackedImageStream::imageOdomCallback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::Image::ConstPtr& rgb_msg){
    Sophus::Quaternion<float> q(static_cast<float>(odom->pose.pose.orientation.w),
                               static_cast<float>(odom->pose.pose.orientation.x),
                               static_cast<float>(odom->pose.pose.orientation.y),
                               static_cast<float>(-odom->pose.pose.orientation.z));
    Sophus::Matrix<float, 3, 1> trans(static_cast<float>(odom->pose.pose.position.x),
                                     static_cast<float>(odom->pose.pose.position.y),
                                     static_cast<float>(odom->pose.pose.position.z));
    Eigen::Matrix4f Pose_receive = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f cam2world, cam02body;
    cam02body <<
              0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    Pose_receive.block<3, 3>(0, 0) = q.toRotationMatrix();
    Pose_receive(0, 3) = trans(0);
    Pose_receive(1, 3) = trans(1);
    Pose_receive(2, 3) = trans(2);
    cam2world = cam02body * Pose_receive;
    trans = Eigen::Vector3f(cam2world(0, 3),
                          cam2world(1, 3),
                          cam2world(2, 3));
    q = cam2world.block<3, 3>(0, 0);
    //std::cout<<"X: "<<odom->pose.pose.position.x<<" Y: "<<odom->pose.pose.position.y<<std::endl<<"Z: "<<odom->pose.pose.position.z<<std::endl;
    /* static tf2_ros::Buffer tf_buffer_(ros::Duration(30));
     static tf2_ros::TransformListener tf_listener_(tf_buffer_);

     geometry_msgs::TransformStamped tf;

     tf = tf_buffer_.lookupTransform("world_enu", "front_center_custom_optical",
                                     ros::Time(0),
                                     ros::Duration(1.0/10));
    tfToSophusSE3(tf.transform, &pose_);
         Eigen::Quaternionf q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
    */
    /*Eigen::Matrix3f R_rfu_to_rdf;
    R_rfu_to_rdf << 1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
            0.0, 1.0, 0.0;

    Eigen::Quaternionf q_rfu_to_rdf(R_rfu_to_rdf);*/
    //Eigen::Quaternionf q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
    //q = q_flu_to_rdf * q * q_flu_to_rdf.inverse();
    //trans = q_flu_to_rdf * trans;
    //R_rdf_to_flu << 0.0f, 0.0f, 1.0f,
    //        -1.0f, 0.0f, 0.0f,
    //        0.0f, -1.0f, 0.0f;
    // Local RDF frame in global FLU frame.
    //Eigen::Quaternionf q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
    //q = q_flu_to_rdf * q;
    //trans = q_flu_to_rdf * trans;

    pose_ = Sophus::SE3Group<float>(q, trans);
    processPoseImage(rgb_msg);
}
void TrackedImageStream::imageTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& tf, const sensor_msgs::Image::ConstPtr& rgb_msg){
    std_msgs::Header header_msg;
    header_msg.stamp = rgb_msg->header.stamp;
    received_pub_.publish(header_msg);
    tfToSophusSE3<float>(tf->transform, &pose_);
    processPoseImage(rgb_msg);
}

void TrackedImageStream::processPoseImage(const sensor_msgs::Image::ConstPtr& rgb_msg) {
    std::cout<<"Received data!"<<std::endl;
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
                    K_(ii, jj) = info_->P[ii * 4 + jj];
                }
            }

            if (K_(0, 0) <= 0) {
                ROS_ERROR("Camera intrinsics matrix is probably invalid!\n");
                ROS_ERROR_STREAM("K = " << std::endl << K_);
                return;
            }

            /*for (int ii = 0; ii < 5; ++ii) {
                D_(ii) = info_->D[ii];
            }*/
        }

        inited_ = true;

        ROS_DEBUG("Set camera calibration!");
    }

    Frame FRAME;
    static int id = 0;
    FRAME.id = id++;
    FRAME.time = rgb_msg->header.stamp.toSec();
    FRAME.quat = pose_.unit_quaternion();
    FRAME.trans = pose_.translation();
    FRAME.img = gray_img;
    queue_.push(FRAME);

    return;
}
}  // namespace ros_sensor_streams
