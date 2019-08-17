
#pragma once

//#define Detect_Factory RM2017
//#define Detect_Factory CC
//#define Detect_Factory HYY
#define Detect_Factory LCP

#include <roborts_msgs/Chassis.h>
#include <roborts_msgs/Gimbal.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "stdio.h" // __LINE__

//#define VIDEO_PLAY    // 视频播放

#ifndef VIDEO_PLAY
  //#define VIDEO_REC // 视频录制
#endif

#define BUFFER_SIZE 3

enum BufferState { 
  IDLE = 0,       // 空闲 
  WRITE,          // 写入状态
  READ            // 读取状态
};

enum TaskState {
  AWAIT = 0,       // 待命 
  SHOOT_STOP,      // 瞄准跟随
  SHOOT_ONCE,      // 哒～
  SHOOT_TWICE,     // 哒哒～
  SHHOT_THREE,     // 哒哒哒～
  SHOOT_CONTINUOUS // 连射
};
