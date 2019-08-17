/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

// todo: 通过鼠标or键盘操作，改变tf

#include "../ros_dep.h"
#include "../serial/protocol.h"      // protocol
#include <thread>

class FakeSerial
{
  // ros
  ros::NodeHandle ros_nh;

  roborts_msgs::Gimbal  small_gimbal_;
  ros::Publisher  ros_pub_small_gimbal;
  ros::Subscriber ros_sub_small_gimbal;
  void SmallGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
  {
    uint8_t gimbal_mode        = info->mode;
    int16_t gimbal_yaw         = static_cast<int16_t>(info->yaw * 100);
    int16_t gimbal_pitch       = static_cast<int16_t>(info->pitch * 100);
    int16_t gimbal_fric_speed  = static_cast<int16_t>(info->fric_speed * 100);

    //ROS_INFO("small_gimbal_mode: %d",  gimbal_mode);
    //ROS_INFO("small_gimbal_yaw: %d",   gimbal_yaw);
    //ROS_INFO("small_gimbal_pitch: %d", gimbal_pitch);
    //ROS_INFO("small_fric_speed: %d\n", gimbal_fric_speed);
  }

  roborts_msgs::Gimbal  big_gimbal_;
  ros::Publisher  ros_pub_big_gimbal;
  ros::Subscriber ros_sub_big_gimbal;
  void BigGimbalCtrlCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
  {
    uint8_t gimbal_mode        = info->mode;
    int16_t gimbal_yaw         = static_cast<int16_t>(info->yaw * 100);
    int16_t gimbal_pitch       = static_cast<int16_t>(info->pitch * 100);
    int16_t gimbal_fric_speed  = static_cast<int16_t>(info->fric_speed * 100);

    //ROS_INFO("big_gimbal_mode: %d",  gimbal_mode);
    //ROS_INFO("big_gimbal_yaw: %d",   gimbal_yaw);
    //ROS_INFO("big_gimbal_pitch: %d", gimbal_pitch);
    //ROS_INFO("big_fric_speed: %d\n", gimbal_fric_speed);
  }

  roborts_msgs::Chassis  chassis_;
  ros::Publisher  ros_pub_chassis;

  //thread
  std::thread read_thread_;
public:
  FakeSerial()
  {
    ros_pub_small_gimbal = ros_nh.advertise<roborts_msgs::Gimbal>("small_gimbal_info", 100);
    ros_sub_small_gimbal = ros_nh.subscribe("small_gimbal_ctrl", 1, &FakeSerial::SmallGimbalCtrlCallBack, this);
    
    ros_pub_big_gimbal   = ros_nh.advertise<roborts_msgs::Gimbal>("big_gimbal_info", 100);
    ros_sub_big_gimbal   = ros_nh.subscribe("big_gimbal_ctrl", 1, &FakeSerial::BigGimbalCtrlCallBack, this);
    
    ros_pub_chassis      = ros_nh.advertise<roborts_msgs::Chassis>("chassis_info", 100);
  
    read_thread_ = std::thread(&FakeSerial::ReadExecutor, this);
  }

  void ReadExecutor()
  {
    uint8_t raw[sizeof(data_read_t)]; // 串口读进来的原始数据

    raw[0] = 0x0A;  // sof

    raw[1] = 0x00;  // mode
    raw[2] = 0x00;  // yaw
    raw[3] = 0x01;  // yaw
    raw[4] = 0x00;  // pitch
    raw[5] = 0x01;  // pitch
    
    raw[6] = 0xe8;  // fric
    raw[7] = 0x03;  // fric

    raw[8]  = 0x00;  // mode
    raw[9]  = 0x00;  // yaw
    raw[10] = 0x00;  // yaw
    raw[11] = 0x00;  // pitch
    raw[12] = 0x00;  // pitch
    raw[13] = 0x00;  // fric
    raw[14] = 0x00;  // fric

    raw[15] = 0x00;  // mode
    raw[16] = 0x00;  // chassis_x
    raw[17] = 0x00;  // chassis_x
    raw[18] = 0x00;  // chassis_y
    raw[19] = 0x00;  // chassis_y

    raw[20] = 0xB0;  // end

    data_read_t buffer;

    memcpy(&buffer, raw, sizeof(data_read_t));   
    
    while(true)
    {
      small_gimbal_.mode       = 1;//buffer.small_gimbal.mode.cmd_id;
      small_gimbal_.yaw        = buffer.small_gimbal.yaw.ecd_angle / 100.;
      small_gimbal_.pitch      = buffer.small_gimbal.pitch.ecd_angle / 100.;
      small_gimbal_.fric_speed = 20;//buffer.small_gimbal.fric_speed / 100.;

      //ROS_INFO("buffer.small_gimbal.mode \t %d", buffer.small_gimbal.mode);
      //ROS_INFO("buffer.small_gimbal.yaw.ecd_angle \t %d", buffer.small_gimbal.yaw.ecd_angle);
      //ROS_INFO("buffer.small_gimbal.pitch.ecd_angle \t %d", buffer.small_gimbal.pitch.ecd_angle);
      //ROS_INFO("buffer.small_gimbal.fric_speed \t %d", buffer.small_gimbal.fric_speed);

      //ROS_INFO("small_gimbal_.mode \t %d", small_gimbal_.mode);
      //ROS_INFO("small_gimbal_.yaw \t %f", small_gimbal_.yaw);
      //ROS_INFO("small_gimbal_.pitch \t %f", small_gimbal_.pitch);
      //ROS_INFO("small_gimbal_.fric_speed \t %f\n", small_gimbal_.fric_speed);

      ros_pub_small_gimbal.publish(small_gimbal_);

      big_gimbal_.mode = 0;
      big_gimbal_.yaw = 6.18;
      big_gimbal_.pitch = 6.18;
      big_gimbal_.fric_speed = 16;
      ros_pub_big_gimbal.publish(big_gimbal_);

      chassis_.mode = 0;
      chassis_.speed_x = 3;
      chassis_.speed_y = 4;
      ros_pub_chassis.publish(chassis_);

      usleep(1000000); // wait 50ms
    }
  }
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_Write");

  ROS_WARN("sizeof(gimbal_t) \t %d", sizeof(gimbal_t));
  ROS_WARN("sizeof(chassis_t) \t %d", sizeof(chassis_t));
  ROS_WARN("sizeof(data_read_t)\t %d", sizeof(data_read_t));
  ROS_WARN("sizeof(data_write_t)\t %d", sizeof(data_write_t));
  
  FakeSerial fake_serial;

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}