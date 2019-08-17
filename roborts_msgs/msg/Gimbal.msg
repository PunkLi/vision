uint8   mode
float32 yaw
float32 pitch
float32 fric_speed

# typedef struct {
#   union {
#     uint8_t cmd_id;             // 命令模式, 1: 自瞄  2: 打符 ...
#     uint8_t task_id;            // 任务模式, AWAIT、SHOOT_STOP、SHOOT_ONCE、SHOOT_CONTINUOUS
#   }mode;       
#   union {
#     int16_t ecd_angle;          // 当底盘正常时，读编码器角度 x 100
#     int16_t gyro_angle;         // 当底盘陀螺时，读陀螺仪角度 x 100
#   }yaw;
#   union {
#     int16_t ecd_angle;          // 当底盘正常时，读编码器角度 x 100
#     int16_t gyro_angle;         // 当底盘陀螺时，读陀螺仪角度 x 100
#   }pitch;
#   int16_t   fric_speed;         // 摩擦轮射速 x 100
# }__attribute__((packed)) gimbal_t;