#!/usr/bin/env python
# coding=utf-8
# Filename : hc_cmd_interface.py
# Author : AbigCarrot
# E-mail : liudayuan@bitathome.org
# Created Date : 2014/07/23 11:11
# Description : 电机驱动和码盘数据的发布
# History
#   2014/07/23 11:11 : 创建文件 [刘达远]


import rospy
from hc_serialport_communication import Communtcation
from bitathome_hardware_control.srv import *
from bitathome_hardware_control.msg import *
import math


node_name = "hc_cmd_interface"  # 节点名称
serial_name = "/dev/ttyUSB0"  # 串口名称
max_wheel_speed = 600  # 最大电机转速
request_code_disk_buf = [0x55, 0xaa, 0x38, 0x02, 0x08, 0x60]  # 码盘数据返回申请


def run():

    #发布话题
    pub = rospy.Publisher("code_disk_data", CodeDisk, queue_size=10)  # 码盘数据话题,发布三个码盘值
    request_code_disk_buf.extend(split_hl(50))  # 50ms
    request_code_disk_buf.append(solve_sum(request_code_disk_buf))
    link.write(request_code_disk_buf)
    # 55 AA 38 02 08 60 TH TL Sum

    rospy.init_node(node_name)  # 初始化节点
    r = rospy.Rate(50)  # 50Hz

    # 开启服务
    rospy.Service("/hc_cmd_interface/motor_speed", MotorSpeed, handler_motor)  # 电机
    print "Open /hc_cmd_interface/motor_speed successful ^_^"
    rospy.Service("/hc_cmd_interface/vector_speed", VectorSpeed, handler_vector)  # 向量
    print "Open /hc_cmd_interface/vector_speed successful ^_^"

    link.read()  # 读取返回数据

    while not rospy.is_shutdown():
        distinguish(pub)  # 解析数据
        r.sleep()


# 解析机器返回数据
def distinguish(pub):
    #print len(link.Buffer)
    if len(link.Buffer) < 7:
        return False

    b_buf = bytearray(link.Buffer)
    start1 = b_buf[0]
    start2 = b_buf[1]
    my_node = b_buf[2]

    if start1 is not 0x55 or start2 is not 0xaa:  # 检验开头是否合法,不合法的话进行去头继续检验,直到有开始标志
        #link.read_switch = False
        link.Buffer.pop(0)
        #link.read_switch = True
        print "开头不对"
        return False

    if my_node is 0x38:  # 检验本机节点是否可用,0x38节点包括: 码盘数据读取
    #if True:
        #link.read_switch = False
        length = b_buf[3]  # 此帧数据长度
        module = b_buf[4]  # 此帧模块编号
        method = b_buf[5]  # 此帧方法编号
        status = b_buf[6]  # 此帧状态位
        #link.read_switch = True

        while len(b_buf) < length + 8:  # 等待读取数据达到此帧长度
            #print "长度不够"
            return False

        '''
        # 检测此帧数据内容
        for it in b_buf[0: length + 8]:
            print "%x " % it,
        print ""
        '''

        if module is 0x08:  # 检测模块编号是否有用,0x08模块包括: 码盘数据读取

            if method is 0x60:  # 检测方法编号是否有用,0x60方法包括: 码盘数据读取

                if status is 0x00:  # 检测转台位是否可用,0x00状态为: 码盘数据读取

                    if length is not 0x12:  # 检测实际长度是否为协议长度,码盘数据一帧长度为0x12
                        #link.read_switch = False
                        n = length + 8
                        while n > 0:
                            n -= 1
                            link.Buffer.pop(0)
                        #link.read_switch = True
                        #print "长度不对"
                        return False

                    else:  # 解析码盘数据成3个int32的数据并发布出去
                        for it in b_buf[0: length + 8]:
                            print "%x " % it,
                        print ""

                        buf = []
                        n = length + 8
                        while n > 0:
                            n -= 1
                            buf.append(b_buf[0])
                            b_buf.pop(0)
                            link.Buffer.pop(0)

                        s_sum = buf[25]
                        buf.pop(25)
                        if solve_sum(buf) is not s_sum:
                            return False

                        data = CodeDisk()
                        data.data1 = combination_hhll([buf[7], buf[8], buf[9], buf[10]])
                        data.data2 = combination_hhll([buf[13], buf[14], buf[15], buf[16]])
                        data.data3 = combination_hhll([buf[19], buf[20], buf[21], buf[22]])

                        pub.publish(data)
                        print "%d %d %d" % (data.data1, data.data2, data.data3)
                        return True

                else:
                    n = length + 8
                    while n > 0:
                        n -= 1
                        link.Buffer.pop(0)
                    #print "状态位无效"
                    return False

            else:
                n = length + 8
                while n > 0:
                    n -= 1
                    link.Buffer.pop(0)
                #print "方法编码无效"
                return False

        else:
            n = length + 8
            while n > 0:
                n -= 1
                link.Buffer.pop(0)
            #print "模块编码无效"
            return False

    else:
        #link.read_switch = False
        length = link.Buffer[3]
        #link.read_switch = True

        while len(b_buf) < length + 8:
            pass

        n = length + 8
        while n > 0:
            n -= 1
            link.Buffer.pop(0)
        #print "本机节点无效"
        return False


# 以三个转速控制机器运动
def handler_motor(data):
    """
    :param data: 三个轮子的速度
    :return: 成功命令返回True, 速度过大或失败返回False
    """
    sv1 = data.v1
    sv2 = data.v2
    sv3 = data.v3
    buf = [0x55, 0xaa, 0x38, 0x0a, 0x08, 0x70]
    print "%d %d %d" % (sv1, sv2, sv3)
    #55 AA 38 0A 08 70 1H 1L 2H 2L 3H 3L 4H 4L 5H 5L

    if math.fabs(sv1) > max_wheel_speed or math.fabs(sv2) > max_wheel_speed or math.fabs(sv3) > max_wheel_speed:
        print "You fly too low!!! ⊙_⊙"
        return False

    buf.extend(split_hl(sv1))
    buf.extend(split_hl(sv2))
    buf.extend(split_hl(sv3))
    buf.extend([0, 0, 0, 0])
    buf.append(solve_sum(buf))
    if link.write(buf):
        return True
    return False


# 以向量和转速控制运动
def handler_vector(data):
    """
    :param data: x,y轴方向向量大小和转动角度theta,其x轴为机器正前方,坐标系符合右手螺旋定则
    :return: 成功命令返回True, 速度过大或失败返回False
    """
    sx = data.x
    sy = data.y
    theta = data.theta
    sv1 = int((sx * math.sqrt(3) - sy + theta) / 3)
    sv2 = int(- (sx * math.sqrt(3) + sy - theta) / 3)
    sv3 = int((2 * sy + theta) / 3)
    buf = [0x55, 0xaa, 0x38, 0x0a, 0x08, 0x70]
    print "%d %d %d" % (sv1, sv2, sv3)
    #55 AA 38 0A 08 70 1H 1L 2H 2L 3H 3L 4H 4L 5H 5L Sum

    if math.fabs(sv1) > max_wheel_speed or math.fabs(sv2) > max_wheel_speed or math.fabs(sv3) > max_wheel_speed:
        print "You fly too low!!! ⊙_⊙"
        return False

    buf.extend(split_hl(sv1))
    buf.extend(split_hl(sv2))
    buf.extend(split_hl(sv3))
    buf.extend([0, 0, 0, 0])
    buf.append(solve_sum(buf))
    if link.write(buf):
        return True
    return False


# 将一个16位的int型数拆成两个8位的
def split_hl(item):
    item &= 0xffff
    result = [(item & 0xff00) >> 8, item & 0xff]
    return result


# 将四个8位的int型数合并成一个32位的
def combination_hhll(buf):
    item = 0
    item |= buf[0]
    item <<= 8
    item |= buf[1]
    item <<= 8
    item |= buf[2]
    item <<= 8
    return item | buf[3]


# 求校验和
def solve_sum(buf):
    sum_m = 0
    for it in buf:
        sum_m += it
    sum_m &= 0xff
    return sum_m


if __name__ == '__main__':

    link = Communtcation(serial_name, 2000000, 8)  # 串口链接

    if link.open():
        run()
    else:
		rospy.loginfo("Open serialport %s fail" % (serial_name))
		exit(0)
    link.close()
