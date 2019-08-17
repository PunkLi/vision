/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */


#include "serial_node.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_Write");

  ROS_WARN("sizeof(gimbal_t) \t %d", sizeof(gimbal_t));
  ROS_WARN("sizeof(chassis_t) \t %d", sizeof(chassis_t));
  ROS_WARN("sizeof(data_read_t)\t %d", sizeof(data_read_t));
  ROS_WARN("sizeof(data_write_t)\t %d", sizeof(data_write_t));

  SerialNode serialnode;
  //roborts_sdk::SerialDevice serial("/dev/robomaster",115200);

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}