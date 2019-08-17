/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

// todo: 通过鼠标or键盘操作，改变tf

#include "../ros_dep.h"
#include <thread>

class FakeLoop
{
  // ros
  ros::NodeHandle ros_nh;

  roborts_msgs::Gimbal  small_gimbal_;
  ros::Publisher  ros_pub_small_gimbal;
  ros::Subscriber ros_sub_small_gimbal;

  roborts_msgs::Gimbal  big_gimbal_;
  ros::Publisher  ros_pub_big_gimbal;
  ros::Subscriber ros_sub_big_gimbal;

  roborts_msgs::Chassis  chassis_;
  ros::Subscriber ros_sub_chassis;
  
  std::thread thread_;

public:
  FakeLoop()
  {
    ros_pub_small_gimbal = ros_nh.advertise<roborts_msgs::Gimbal>("small_gimbal_ctrl", 100);
    ros_sub_small_gimbal = ros_nh.subscribe("small_gimbal_info", 1, &FakeLoop::SmallGimbalInfoCallBack, this);

    ros_pub_big_gimbal   = ros_nh.advertise<roborts_msgs::Gimbal>("big_gimbal_ctrl", 100);
    ros_sub_big_gimbal   = ros_nh.subscribe("big_gimbal_info", 1, &FakeLoop::BigGimbalInfoCallBack, this);

    ros_sub_chassis      = ros_nh.subscribe("chassis_info", 1, &FakeLoop::ChassisInfoCallBack, this);
    
    thread_ = std::thread(&FakeLoop::ExecuteLoop, this);
  }

  void SmallGimbalInfoCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
  {
    small_gimbal_.mode       = info->mode;
    small_gimbal_.yaw        = info->yaw;
    small_gimbal_.pitch      = info->pitch;
    small_gimbal_.fric_speed = info->fric_speed;

    /*ROS_INFO("small_gimbal_.mode: %d",  small_gimbal_.mode);
    ROS_INFO("small_gimbal_.yaw: %f",   small_gimbal_.yaw);
    ROS_INFO("small_gimbal_.pitch: %f", small_gimbal_.pitch);
    ROS_INFO("small_gimbal_.fric_speed: %f\n", small_gimbal_.fric_speed);
    */
  }

  void BigGimbalInfoCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
  {
    big_gimbal_.mode       = info->mode;
    big_gimbal_.yaw        = info->yaw;
    big_gimbal_.pitch      = info->pitch;
    big_gimbal_.fric_speed = info->fric_speed;

    /* ROS_INFO("big_gimbal_.mode: %d",  big_gimbal_.mode);
    ROS_INFO("big_gimbal_.yaw: %f",   big_gimbal_.yaw);
    ROS_INFO("big_gimbal_.pitch: %f", big_gimbal_.pitch);
    ROS_INFO("big_gimbal_.fric_speed: %f\n", big_gimbal_.fric_speed);
    */
  }

  void ChassisInfoCallBack(const roborts_msgs::Chassis::ConstPtr & info)
  {
    chassis_.mode     = info->mode;
    chassis_.speed_x  = info->speed_x;
    chassis_.speed_y  = info->speed_y;
    /*
    ROS_INFO("chassis_.mode: %d", chassis_.mode);
    ROS_INFO("chassis_.speed_x: %f", chassis_.speed_x);
    ROS_INFO("chassis_.speed_y: %f\n", chassis_.speed_y);
    */
  }

  void ExecuteLoop()
  {
    while(true) // while(ros::ok())
    {
      roborts_msgs::Gimbal  small_gimbal;
      small_gimbal.mode = 0;
      small_gimbal.yaw  = 3.14;
      small_gimbal.pitch = 3.14;
      small_gimbal.fric_speed = 16;
      ros_pub_small_gimbal.publish(small_gimbal);

      roborts_msgs::Gimbal  big_gimbal;
      big_gimbal.mode = 2;
      big_gimbal.yaw  = 1.23;
      big_gimbal.pitch = 1.23;
      big_gimbal.fric_speed = 20;
      ros_pub_big_gimbal.publish(big_gimbal);
      
      usleep(500000); //50ms
    }
  }


};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_Write");
  FakeLoop loop;
  ros::AsyncSpinner async_spinner(3);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}