uint8   mode
float32 speed_x
float32 speed_y

# typedef struct {
#   uint8_t mode;                 // gyro_mode, 0: 正常 1: 陀螺
#   int16_t speed_x;              // 底盘前行的速度，单位：todo
#   int16_t speed_y;              // 底盘平移的速度，单位：todo
# }__attribute__((packed)) chassis_t;