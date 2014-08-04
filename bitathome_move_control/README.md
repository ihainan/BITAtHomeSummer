# bitathome_move_control

---

## Package 简介
本包用于家庭组机器人的运动了控制，现在包括两种运动控制方式,和避障算法

 - 维护状态：正在维护
 - 维护者：家庭组运动控制成员

## 节点
- hc_joy_control.py: 用于手柄对机器人运动的控制
    - 使用服务
          - /hc_cmd_interface/vector_speed（bitathome_hardware_control/VectorSpeed）：此服务以向量和转速控制机器运动
- hc_keyboard_control.py: 用于键盘对机器人运动的控制
    - 使用服务
          - /hc_cmd_interface/motor_speed（bitathome_hardware_control/ MotorSpeed）：此服务以三个转速控制机器运动
- hc_avoid_obstacle.py: 用于机器人的避障
    - 订阅主题
          - /scan（bitathome_move_control/LaserScan）： 这个主题用于监听激光数据
    - 使用服务
          - /hc_cmd_interface/vector_speed（bitathome_hardware_control/VectorSpeed）： 此服务以向量和转速控制机器运动

##启动文件
- keyboard.launch : 这个启动文件用于键盘控制机器人运动。
- joy.launch : 这个启动文件用于手柄控制机器人运动。
- avoid_obstachle.launch : 这个文件用于启动避障功能。
