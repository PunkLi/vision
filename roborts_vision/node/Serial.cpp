  
  /**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

  #include "VisionNode.h"
  
  void VisionNode::ChassisInfoCallBack(const roborts_msgs::Chassis::ConstPtr & info)
  {
    chassis_.mode     = info->mode;
    chassis_.speed_x  = info->speed_x;
    chassis_.speed_y  = info->speed_y;

    //ROS_INFO("chassis_.mode: %d", chassis_.mode);
    //ROS_INFO("chassis_.speed_x: %f", chassis_.speed_x);
    //ROS_INFO("chassis_.speed_y: %f\n", chassis_.speed_y);
  }
  
  void VisionNode::GimbalInfoCallBack(const roborts_msgs::Gimbal::ConstPtr & info)
  {
    gimbal_info_.mode       = info->mode;
    gimbal_info_.yaw        = info->yaw;
    gimbal_info_.pitch      = info->pitch;
    gimbal_info_.fric_speed = info->fric_speed;
    
    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                        info->pitch / 180.0 * M_PI,
                                                                        info->yaw / 180.0 * M_PI);
    gimbal_tf_.header.stamp = current_time;
    gimbal_tf_.transform.rotation = q;
    tf_br_.sendTransform(gimbal_tf_);
    
    //ROS_INFO("received gimbal_info_.mode: %d",  gimbal_info_.mode);
    //ROS_INFO("gimbal_info_.yaw: %f",   gimbal_info_.yaw);
    //ROS_INFO("gimbal_info_.pitch: %f", gimbal_info_.pitch);
    //ROS_INFO("gimbal_info_.fric_speed: %f\n", gimbal_info_.fric_speed);
  }

  void VisionNode::GimbalTargetCallBack(const geometry_msgs::Point::ConstPtr & info)
  {
    target_shooted_.x = info->x;
    target_shooted_.y = info->y;
    target_shooted_.z = info->z;
  }
  