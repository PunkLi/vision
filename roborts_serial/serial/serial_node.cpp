/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "serial_node.h"

SerialNode::SerialNode(): serial("/dev/robomaster", 115200)
{
  ros_pub_small_gimbal = ros_nh.advertise<roborts_msgs::Gimbal>("small_gimbal_info", 100);
  ros_sub_small_gimbal = ros_nh.subscribe("small_gimbal_ctrl", 1, &SerialNode::SmallGimbalCtrlCallBack, this);
#ifdef HERO_ROBOT
  ros_pub_big_gimbal   = ros_nh.advertise<roborts_msgs::Gimbal>("big_gimbal_info", 100);
  ros_sub_big_gimbal   = ros_nh.subscribe("big_gimbal_ctrl", 1, &SerialNode::BigGimbalCtrlCallBack, this);
#endif
  ros_pub_chassis      = ros_nh.advertise<roborts_msgs::Chassis>("chassis_info", 100);

  //data_write.sof  = 0x0C;
  //data_write.end  = 0xD0;
  
  read_thread_ = std::thread(&SerialNode::ReadExecutor, this);
  write_thread_ = std::thread(&SerialNode::WriteExecutor, this);
}

void SerialNode::ReadExecutor()
{
  int len = sizeof(data_read_t);
  uint8_t buff_read[len];
  uint8_t buff_read_fix[len];
  ROS_WARN("Serial ReadExecutor init OK.");

  while(1) { // while(ros::ok())
    serial.Read(buff_read, len);  // 设备未上电时，Read()暂停
    for(int i = 0; i < len; ++i) {
      if(buff_read[i] == 0x0A) {
        memcpy(buff_read_fix, buff_read + i, len - i);
        memcpy(buff_read_fix + len - i, buff_read, i);
        memcpy(&data_read, buff_read_fix, len);        
      }
    }
    if(data_read.sof == 0x0A && data_read.end == 0xB0)
    {
      small_gimbal_.mode         = data_read.small_gimbal.mode.cmd_id;
      small_gimbal_.yaw          = data_read.small_gimbal.yaw.gyro_angle   / 100.;         
      small_gimbal_.pitch        = data_read.small_gimbal.pitch.gyro_angle / 100.;
      small_gimbal_.fric_speed   = data_read.small_gimbal.fric_speed       / 100.;
      ros_pub_small_gimbal.publish(small_gimbal_);

#ifdef HERO_ROBOT
      big_gimbal_.mode         = data_read.big_gimbal.mode.cmd_id;
      big_gimbal_.yaw          = data_read.big_gimbal.yaw.gyro_angle   / 100.;         
      big_gimbal_.pitch        = data_read.big_gimbal.pitch.gyro_angle / 100.;
      big_gimbal_.fric_speed   = data_read.big_gimbal.fric_speed       / 100.;        
      ros_pub_big_gimbal.publish(big_gimbal_);
#endif
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void SerialNode::WriteExecutor()
{
  ROS_WARN("Serial WriteExecutor init OK.");

  int len = sizeof(data_write_t);
  uint8_t buff_write[len];

  while(1) // while(ros::ok())
  {
    memcpy(buff_write, &data_write, len);
    serial.Write(buff_write, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void SerialNode::SmallGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
{
  data_write.small_gimbal.mode.task_id     = info->mode;
  data_write.small_gimbal.yaw.ecd_angle    = static_cast<int16_t>(info->yaw * 100.);
  data_write.small_gimbal.pitch.ecd_angle  = static_cast<int16_t>(info->pitch * 100.);
  data_write.small_gimbal.fric_speed       = static_cast<int16_t>(info->fric_speed * 100.);

  //ROS_INFO("small_gimbal_mode: %d",  data_write.small_gimbal.mode.task_id);
  //ROS_INFO("small_gimbal_yaw: %d",   data_write.small_gimbal.yaw.ecd_angle);
  //ROS_INFO("small_gimbal_pitch: %d", data_write.small_gimbal.pitch.ecd_angle);
  //ROS_INFO("small_fric_speed: %d\n", data_write.small_gimbal.fric_speed);
}

#ifdef HERO_ROBOT
void SerialNode::BigGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
{
  data_write.big_gimbal.mode.task_id     = info->mode;
  data_write.big_gimbal.yaw.ecd_angle    = static_cast<int16_t>(info->yaw * 100.);
  data_write.big_gimbal.pitch.ecd_angle  = static_cast<int16_t>(info->pitch * 100.);
  data_write.big_gimbal.fric_speed       = static_cast<int16_t>(info->fric_speed * 100.);

  //ROS_INFO("big_gimbal_mode: %d",  data_write.big_gimbal.mode.task_id);
  //ROS_INFO("big_gimbal_yaw: %d",   data_write.big_gimbal.yaw.ecd_angle);
  //ROS_INFO("big_gimbal_pitch: %d", data_write.big_gimbal.pitch.ecd_angle);
  //ROS_INFO("big_fric_speed: %d\n", data_write.big_gimbal.fric_speed);
}
#endif