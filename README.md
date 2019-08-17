# vision2019

## 环境依赖
- ubuntu + ros(opencv)

## 规范
- 代码规范参考 google
- 日志统一用ROS_INFO \ ROS_WARN \ ROS_ERROR
- 参数统一用ROS package + opencv + .xml / ros param
- 如果配环境的水平有所提高，可以考虑 glog + google protobuf

## roborts_bringup
所有的脚本、启动文件都在这里，但是.xml参数配置文件不在这里，而是和代码放在一起。

launch/包含所有机器人的启动文件，scripts/是所有的脚本文件，udev/创建接口规则，upstart/是自启动文件。

## roborts_msgs
所有的ROS消息统一在这里，目前的代码是双ROS节点（serial + vision ），所以msg只用于这两个ROS节点之间互传。延迟大约0.3ms。

## roborts_serial
这是一个统一所有机器人的串口的实现+协议，不要修改它，只给它发数据，以及读它发来的数据即可。

## roborts_vision
- camera / 相机统一接口
  - mercure / 大恒水星
  - realsense / 深度相机
  - stereo / 双目相机
  - uvc / 免驱相机
- detect_factory / 目标检测统一接口
  - armor / 装甲板
  - rune / 大符
- executor / 硬件层控制统一接口
  - chassis / 底盘麦轮解算
  - gimbal / 云台角度解算
- node / 核心节点