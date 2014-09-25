# bitathome_audio_control
---

## package简介
本包使用ros-indigo-sound-play包进行封装，实现了TTS功能
- 维护状态：维护中
- 维护者：Alan Snape

## 节点
- AudioPlay：用于实现TTS功能
    - 提供服务：
        - /AudioPlay/TTS：
        - 参数：string TTS传入的转换内容
        - 返回值：int8：0为成功完成，-1为失败

## 启动文件
- soundplay.launch：用于启动soundplay节点和AudioPlay节点

## 依赖包安装
- install.sh（命令行执行）

