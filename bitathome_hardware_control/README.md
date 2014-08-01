# bitathome_hardware_control

---

## Package 简介
本包用于家庭组机器人的硬件控制，现在包括控制机器人运动、获取里程数据的功能

- 维护状态：正在维护
- 维护者：家庭组成员（人太多了！0.0)

## 节点
- hc_cmd_interface.py: 用于控制机器人的基本运动
    - 发布主题
        - code_disk_data（bitathome_hardware_control/CodeDisk）:这个主题用于实时发布码盘数据
    - 提供服务
        - /hc_cmd_interface/motor_speed（bitathome_hardware_control/ MotorSpeed）：此服务用于电机转速对机器运动的控制
        - /hc_cmd_interface/vector_speed（bitathome_hardware_control/VectorSpeed）：此服务用于向量和转速对机器运动的控制
- hc_odom_control.py: 用于计算里程数据
    - 订阅主题
        - code_disk_data（bitathome_hardware_control/CodeDisk）:用于获取码盘数据

## 启动文件
- vision.launch : 这个启动文件用于启动openni,用来获取kinect的RGBD数据。
