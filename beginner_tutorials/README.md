# beginner_tutorials

---

## Package 简介
本包用于向诸位开发者展示项目的基本开发流程，包中包含 Python 代码编写规范，Msg 示例，Srv 示例，Topic 示例，Service 示例，Launch 示例，README.md 示例，等等。

- 维护状态：正在维护
- 维护者: ihainan (ihainan72@bitathome.org)

## 节点
- listener.py：用于演示 Topic 的订阅。具体参见 [Writing a Simple Publisher and Subscriber (Python)
](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
    - 订阅主题
        - chatter（std_msgs/String）：订阅一个奇怪的主题，传输一个字符串。

- talker.py：用于演示 Topic 的发布。具体参见 [Writing a Simple Publisher and Subscriber (Python)
](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
    - 发布主题
        - chatter（std_msgs/String）：这个主题被用于发布一个字符串。

- add\_two\_ints\_server.py：用于演示 Service 的服务端。具体参见 [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)。
    - 开放服务
        - add\_two\_ints（beginner_tutorial/AddTwoInts）：两个数字相加，返回和。            

- add\_two\_ints\_client.py：用于演示 Service 的客户端。具体参见 [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)。

## 启动文件
- turtlemimic.launch：这个启动文件用于启动两只乌龟，并使其运动同步。
