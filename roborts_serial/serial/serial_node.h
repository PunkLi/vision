/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */
#pragma once 

#include "../ros_dep.h"    // ros
#include "protocol.h"      // protocol
#include "serial_device.h" // sdk
#include <thread>          // thread

class SerialNode
{
  // sdk
  roborts_sdk::SerialDevice serial;
  data_read_t data_read;
  data_write_t data_write;

  // ros
  ros::NodeHandle ros_nh;

  roborts_msgs::Gimbal  small_gimbal_;
  ros::Publisher  ros_pub_small_gimbal;
  ros::Subscriber ros_sub_small_gimbal;
  void SmallGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info);

#ifdef HERO_ROBOT
  roborts_msgs::Gimbal  big_gimbal_;
  ros::Publisher  ros_pub_big_gimbal;
  ros::Subscriber ros_sub_big_gimbal;
  void BigGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info);
#endif
  roborts_msgs::Chassis  chassis_;
  ros::Publisher  ros_pub_chassis;

  //thread
  std::thread read_thread_;
  std::thread write_thread_;
public:
  SerialNode();
  void ReadExecutor();
  void WriteExecutor();
};
