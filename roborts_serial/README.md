# roborts_serial

运行`robort_bringup`下的`create_udev_rules.sh`即可生成USB接口映射规则。

- DJI-STM32-A板对应`"/dev/serial_sdk"` 
- 串口转USB模块对应`"/dev/robomaster"`
- DJI-ManiFold2-C对应`"/dev/ttyS0"`

## 一、设计准则

视觉电控双方同时调试, 应当尽量在逻辑上解耦, 否则很难定位问题所在。

## 二、具体协议

### 机器人串口通信接收 -- 14字节（英雄21字节)
```c++
uint8_t sof;        // 帧头 = 0A

uint8_t cmd_id;     // 17mm云台, 命令模式, 0: 自瞄  1: 打符  2: 跟随另一个云台 
int16_t yaw;        // 17mm云台, 角度 x 100
int16_t pitch;      // 17mm云台, 角度 x 100
int16_t fric_speed; // 17mm云台, 测速机构测得的射速 x 100

uint8_t cmd_id;     // 17mm云台, 命令模式, 0: 自瞄  1: 打符  2: 跟随另一个云台
int16_t yaw;        // 42mm云台, 角度 x 100
int16_t pitch;      // 42mm云台, 角度 x 100
int16_t fric_speed; // 42mm云台, 测速机构测得的射速 x 100

uint8_t mode;       // 底盘模式, 0: 正常 1: 陀螺
int16_t speed_x;    // 底盘前行的速度, 单位:todo
int16_t speed_y;    // 底盘平移的速度, 单位:todo

uint8_t             // 帧尾 = B0
```

### 机器人串口通信发送 -- 9字节（英雄16字节)
```c++
uint8_t // 帧头 = 0C

uint8_t task_id;    // 17mm云台模式, TaskState
int16_t yaw;        // 17mm云台, 角度 x 100
int16_t pitch;      // 17mm云台, 角度 x 100
int16_t fric_speed; // 17mm云台, 测速机构测得的射速 x 100

uint8_t task_id;    // 42mm云台模式, TaskState
int16_t yaw;        // 42mm云台, 角度 x 100
int16_t pitch;      // 42mm云台, 角度 x 100
int16_t fric_speed; // 42mm云台, 测速机构测得的射速 x 100

uint8_t // 帧尾 = D0
```
**Note:**

|            | 自瞄 cmd_mode: 0    | 打符 cmd_mode: 1     | 
|:-----------|:--------------------|:--------------------|
| task_id: 0 | AWAIT               |  当前打符进度0/5     |
| task_id: 1 | SHOOT_STOP          |  当前打符进度1/5     |
| task_id: 2 | SHOOT_ONCE          |  当前打符进度2/5     |
| task_id: 3 | SHOOT_TWICE         |  当前打符进度3/5     |
| task_id: 4 | SHOOT_THREE         |  当前打符进度4/5     |
| task_id: 5 | SHOOT_CONTINUOUS    |  当前打符进度5/5     |


## 三、代码参考
```c++
enum TaskState {
  AWAIT = 0,       // 待命 
  SHOOT_STOP,      // 瞄准跟随
  SHOOT_ONCE,      // 哒～
  SHOOT_TWICE,     // 哒哒～
  SHHOT_THREE,     // 哒哒哒～
  SHOOT_CONTINUOUS // 连射
};

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
  int16_t chassis_speed_x;      // 底盘前行的速度，单位：todo
  int16_t chassis_speed_y;      // 底盘平移的速度，单位：todo
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
  uint8_t   sof;                // 帧头 = 0C
  gimbal_t  small_gimbal;       // 小云台
#ifdef HERO_ROBOT
  gimbal_t  big_gimbal;         // 大云台
#endif
  uint8_t   end;                // 帧尾 = D0
}
__attribute__((packed)) data_write_t;

```