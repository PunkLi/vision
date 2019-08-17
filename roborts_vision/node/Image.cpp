
/**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "VisionNode.h"

#define SIMPLE_TEST

void VisionNode::ImageProducer()
{
  if(camera_type_ == "mercure")
  {
    camera::MercureDriver camera_driver(camera_id_);
    CameraLoop(camera_driver);
  }
  else if(camera_type_ == "uvc")
  {
    camera::RMVideoCapture camera_driver("/dev/video0", 3); 
    camera_driver.setVideoFormat(resolution_width_, resolution_height_, 1);
    camera_driver.startStream();
    camera_driver.setExposureTime(false, 100);
    CameraLoop(camera_driver);
  }
  else if(camera_type_ == "realsense")
  {
    ;
  }
}

  template<typename CameraType>
  void VisionNode::CameraLoop(CameraType& camera_driver)
  {
#ifdef VIDEO_REC
    char filename[128];
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(filename, "%s/armor_%04d%02d%02d_%02d%02d%02d.avi", "../",
              timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday,
              timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    cv::VideoWriter writer(filename, 
                           CV_FOURCC('M', 'J', 'P', 'G'), 
                           60.0, 
                           cv::Size(resolution_width_, resolution_height_));
#endif
    int idx = 0;
    while(1) {
      auto speed_test_start_begin_time = std::chrono::steady_clock::now();
      if (buffer_state_[idx] != BufferState::READ) {
        camera_driver >> image_buffer_[idx];    // API
        buffer_state_[idx] = BufferState::WRITE;
        lock_.lock();
        latest_index_ = idx;
        //ROS_INFO("latest_index_: %d", latest_index_);
        lock_.unlock();
        //cv::imshow("image_buffer_[idx]", image_buffer_[idx]);
        //cv::waitKey(1);
#ifdef VIDEO_REC
        writer.write(image_buffer_[idx]);
#endif
      }
      idx = (idx + 1) % BUFFER_SIZE;
      auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
      //ROS_INFO("camera capture time cost = %.2f ms", cost);
    }
  }

void VisionNode::ArmorExecuteLoop() 
{
  ROS_WARN("line: %d: Armor detection Thread started!", __LINE__);
  // Init
  Detect_Factory::ConstraintSet constraintset(camera_type_);

#ifdef VIDEO_PLAY
  cv::VideoCapture camera_driver;
  camera_driver.open(video_path_);
#endif

  while(1)
  {
#ifdef VIDEO_PLAY
    camera_driver >> image_raw_;
    auto speed_test_start_begin_time = std::chrono::steady_clock::now();
#else
    auto speed_test_start_begin_time = std::chrono::steady_clock::now();
    lock_.lock();
    if (buffer_state_[latest_index_] == BufferState::WRITE) {
        buffer_state_[latest_index_] = BufferState::READ;
    } else {
      //ROS_INFO("No image is available.");
      //usleep(5000);
      lock_.unlock();
      continue;
    }
    current_index_ = latest_index_;
    //ROS_WARN("current_index_: %d\n", current_index_);
    lock_.unlock();
    image_raw_ = image_buffer_[current_index_];
#endif
    /////////////////////////////////////////////////////////////
    uint8_t flag;
    constraintset.src_img_ = image_raw_;
    std::vector<Detect_Factory::ArmorInfo> armors;
    constraintset.Detect(flag, armors);
     
    gimbal_contrl.UpdateFricSpeed(gimbal_info_.fric_speed); // 实时更新射速

  #ifdef SIMPLE_TEST
    gimbal_ctrl_.mode = 0;
    gimbal_ctrl_.yaw = 0;
    gimbal_ctrl_.pitch = 0;
    if (armors.size())
    {
      gimbal_ctrl_.mode = 1;
      cv::Point3f target_3d = armors[0].pt;
      gimbal_contrl.SolveContrlAgnle(target_3d, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
      gimbal_ctrl_.yaw = gimbal_ctrl_.yaw * 0.7;
      gimbal_ctrl_.pitch = gimbal_ctrl_.pitch * 0.3;
      ROS_WARN("Task_id\t %d \t dis: %f \t yaw: %f \t pitch: %f", gimbal_ctrl_.mode, target_3d.z, gimbal_ctrl_.yaw, gimbal_ctrl_.pitch);
      
      if (gimbal_link_ == "big_gimbal_link") // 如果是大云台，就发送用于协同的target
      {
        target_shooted_.x = target_3d.x;
        target_shooted_.y = target_3d.y;
        target_shooted_.z = target_3d.z;
        ros_pub_gimbal_target.publish(target_shooted_);
      }
    }
    ros_pub_gimbal.publish(gimbal_ctrl_);
    
  #else
    ArmorFilter(armors);
  #endif    
    
    //cv::imshow("Armor_"+gimbal_link_, image_raw_);
    //cv::waitKey(1);
    /////////////////////////////////////////////////////////////
    buffer_state_[current_index_] = BufferState::IDLE;

    auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
    ROS_INFO("Armor total time cost = %.2f ms", cost);

#ifdef VIDEO_PLAY
    //cv::waitKey(0);
#endif
  } // while
}
