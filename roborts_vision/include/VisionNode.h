/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

/**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */
#pragma once

#include "dep.h" // depends of Macro

#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include "pnp_solver.h"
#include "gimbal/gimbal_control.h"
// ros
#include <ros/ros.h>
#include <ros/package.h>
// opencv
#include "opencv2/opencv.hpp"
#include "uvc/RMVideoCapture.hpp"
#include "mercure/mercure_driver.h"
#include "Filter.hpp"

#include "armor/RM2017/constraint_set.h"
#include "armor/CC/constraint_set.h"
#include "armor/HYY/constraint_set.h"
#include "armor/LCP/constraint_set.h"
#include "rune/Rune_fiall.h"

class VisionNode
{
  // handle
  ros::NodeHandle ros_nh_; // public
  ros::NodeHandle nh_;     // private

  // gimbal
  roborts_msgs::Gimbal  gimbal_info_;
  roborts_msgs::Gimbal  gimbal_ctrl_;
  ros::Publisher  ros_pub_gimbal;
  ros::Subscriber ros_sub_gimbal;
  void GimbalInfoCallBack(const roborts_msgs::Gimbal::ConstPtr & info);
  void GimbalInfoCallBackWithoutPitch(const roborts_msgs::Gimbal::ConstPtr & info);
  
  ros::Publisher  ros_pub_gimbal_target;   // 用于云台间通信
  ros::Subscriber ros_sub_gimbal_target;   // 用于云台间通信
  geometry_msgs::Point  target_shooted_;   // 射击

  // chassis
  roborts_msgs::Chassis  chassis_;
  ros::Subscriber ros_sub_chassis;
  void ChassisInfoCallBack(const roborts_msgs::Chassis::ConstPtr & info);

  //! ros tf
  geometry_msgs::TransformStamped gimbal_tf_;
  tf::TransformBroadcaster tf_br_;
  tf::TransformListener tf_;
  

  // gimbal prarm
  std::string gimbal_link_;  // used by tf.frame_id
  double gimbal_x_,   // tf
         gimbal_y_,   // tf
         gimbal_z_;   // tf

  // camera prarm
  std::string camera_type_;
  int camera_id_;
  int resolution_width_;
  int resolution_height_;
  std::string video_dir_;  // Video Recording
  std::string video_path_; // Video RePlay

  // image buffer
  std::vector<cv::Mat> image_buffer_;
  std::vector<BufferState> buffer_state_;
  std::mutex lock_;
  int latest_index_;
  int current_index_;
  cv::Mat image_raw_;

  // thread
	std::thread camera_thread_;          // 相机线程
	std::thread armor_detection_thread_; // 自瞄线程

  void ImageProducer();
  template<typename CameraType>void CameraLoop(CameraType& camera_driver); // CameraLoop Module
  void ArmorExecuteLoop();    // 装甲识别的主循环
  
  // flag & update
  Detect_Factory::ArmorInfo last_armor_;
  Detect_Factory::ArmorInfo current_armor_;
  History history_;

  // ros tf
  geometry_msgs::PoseStamped last_armor_in_chassis_;
  geometry_msgs::PoseStamped current_armor_in_chassis_;

  geometry_msgs::PoseStamped temp_stamp_;
  geometry_msgs::PoseStamped current_armor_in_gimbal_;

  // executor
  GimbalContrl gimbal_contrl;

public:
  explicit VisionNode(): latest_index_(-1),  // buffer
                         current_index_(-1)  // buffer
  {
    LoadParam();
    InitRos();
    
    // init buffer
    image_buffer_.resize(BUFFER_SIZE);
    buffer_state_.resize(BUFFER_SIZE);

    for (int i = 0; i < buffer_state_.size(); ++i) {
      image_buffer_[i].create(resolution_height_, resolution_width_, CV_8UC3);
      buffer_state_[i] = BufferState::IDLE;
    }
    // init thread
#ifndef VIDEO_PLAY
    camera_thread_ = std::thread(&VisionNode::ImageProducer, this);
#endif
    armor_detection_thread_ = std::thread(&VisionNode::ArmorExecuteLoop, this);
  }

  void InitRos() {
		ros_nh_  = ros::NodeHandle();

		ros_sub_chassis  = ros_nh_.subscribe("chassis_info", 1, &VisionNode::ChassisInfoCallBack, this);

    ros_sub_gimbal = ros_nh_.subscribe("big_gimbal_info", 1, &VisionNode::GimbalInfoCallBack, this);
    ros_pub_gimbal = ros_nh_.advertise<roborts_msgs::Gimbal>("big_gimbal_ctrl", 100);
        
		ros_pub_gimbal_target = ros_nh_.advertise<geometry_msgs::Point>("big_gimbal_target", 100);
	}

  void LoadParam() {
    nh_ = ros::NodeHandle("~");

    // camera param
    nh_.getParam("camera_type", camera_type_);
    nh_.getParam("camera_id", camera_id_);
    nh_.getParam("resolution_width", resolution_width_);
    nh_.getParam("resolution_height", resolution_height_);
    nh_.getParam("video_path", video_path_);

    // gimbal param
    nh_.getParam("gimbal_type", gimbal_link_);
    nh_.getParam("gimbal_x", gimbal_x_);
    nh_.getParam("gimbal_y", gimbal_y_);
    nh_.getParam("gimbal_z", gimbal_z_);

    // tf_init
    gimbal_tf_.header.frame_id = "base_link";
    gimbal_tf_.child_frame_id = gimbal_link_;
    gimbal_tf_.transform.translation.x = gimbal_x_;
    gimbal_tf_.transform.translation.y = gimbal_y_;
    gimbal_tf_.transform.translation.z = gimbal_z_;
  }
protected:
  void ArmorFilter(std::vector<Detect_Factory::ArmorInfo> armors);

  void GetArmorPoseStamped(const Detect_Factory::ArmorInfo & armor, geometry_msgs::PoseStamped & stamp);

  void TransformToChassis(Detect_Factory::ArmorInfo & armor, geometry_msgs::PoseStamped & target_stamp);

  void TransformToGimbal();
  
  int SelectNearestArmor(std::vector<Detect_Factory::ArmorInfo> & armors, geometry_msgs::PoseStamped & stamp);

  double GetPoseDistance(const geometry_msgs::PoseStamped & stamp1, 
                         const geometry_msgs::PoseStamped & stamp2);

  void GetPointFormPoseStamp(cv::Point3f& target_3d, 
                             const geometry_msgs::PoseStamped & stamp);

  void DrawNeighborhood(const cv::Mat &img, Detect_Factory::ArmorInfo &armor, const cv::Scalar &color, int thickness);

  void UpateShootingState(const cv::Point3f & target_3d);
  Detect_Factory::ArmorInfo& ChooseArmorById(Detect_Factory::ArmorInfo& armor1, 
                                             Detect_Factory::ArmorInfo& armor2);
  
  void GimbalTargetCallBack(const geometry_msgs::Point::ConstPtr & info);
                            
};