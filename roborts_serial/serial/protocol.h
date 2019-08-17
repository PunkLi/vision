
/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#pragma once

#include <stdint.h>

#define HERO_ROBOT

/* 
 * sizeof: 7 byte
 */
typedef struct {
  union {
    uint8_t cmd_id;             // 命令模式, 1: 自瞄  2: 打符 ...
    uint8_t task_id;            // 任务模式, AWAIT、SHOOT_STOP、SHOOT_ONCE、SHOOT_CONTINUOUS
  }mode;
  union {
    int16_t ecd_angle;          // 当底盘正常时，读编码器角度 x 100
    int16_t gyro_angle;         // 当底盘陀螺时，读陀螺仪角度 x 100
  }yaw;
  union {
    int16_t ecd_angle;          // 当底盘正常时，读编码器角度 x 100
    int16_t gyro_angle;         // 当底盘陀螺时，读陀螺仪角度 x 100
  }pitch;
  int16_t   fric_speed;         // 摩擦轮射速 x 100
}__attribute__((packed)) gimbal_t;

/*
 * sizeof: 5 byte
 */
typedef struct {
  uint8_t mode;                 // gyro_mode, 0: 正常 1: 陀螺
  int16_t speed_x;              // 底盘前行的速度，单位：todo
  int16_t speed_y;              // 底盘平移的速度，单位：todo
}__attribute__((packed)) chassis_t;

/*
 * sizeof: 2 + 7 + 7 + 5 = 21 byte (hero)
 *             2 + 7 + 5 = 14 byte
 */
typedef struct {
  uint8_t   sof;                // 帧头 = 0A
  gimbal_t  small_gimbal;       // 小云台
#ifdef HERO_ROBOT
  gimbal_t  big_gimbal;         // 大云台
#endif
  chassis_t chassis;            // 底盘前行的速度，单位：todo
  uint8_t   end;                // 帧尾 = B0
}
__attribute__((packed)) data_read_t;

/*
 * sizeof: 2 + 7 + 7 = 16 byte (hero)
 *             2 + 7 = 9  byte
 */
typedef struct {
  uint8_t   sof = 0x0C;         // 帧头 = 0C
  gimbal_t  small_gimbal;       // 小云台
#ifdef HERO_ROBOT
  gimbal_t  big_gimbal;         // 大云台
#endif
  uint8_t   end = 0xD0;         // 帧尾 = D0
}
__attribute__((packed)) data_write_t;