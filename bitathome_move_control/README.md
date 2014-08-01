# bitathome_move_control

---

## Package 简介
本包用于家庭组机器人的运动了控制，现在包括两种运动控制方式

 - 维护状态：正在维护
 - 维护者：家庭组运动控制成员

## 节点
- hc_joy_control.py: 用于手柄对机器人运动的控制
    - 使用服务
          - /hc_cmd_interface/vector_speed：此服务以向量和转速控制机器运动（数据类型：VectorSpeed）
- hc_keyboard_control.py:用于键盘对机器人运动的控制
    - 使用服务
          - /hc_cmd_interface/motor_speed：此服务以三个转速控制机器运动（数据类型：MotorSpeed）
