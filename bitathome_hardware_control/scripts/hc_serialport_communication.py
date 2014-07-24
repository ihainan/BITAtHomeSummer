#!/usr/bin/env python
# coding=utf-8
# Filename : hc_serialport_communication.py
# Author : AbigCarrot
# E-mail : liudayuan@bitathome.org
# Created Date : 2014/07/23 11:28
# Description : 串口通信类
# History
#   2014/07/23 11:28 : 创建文件 [刘达远]

import serial
import thread


class Communtcation():

    def __init__(self, port, band, byte):
        """
        :param port: 串口名称
        :param band: 比特率
        :param byte: 位长
        :return:
        """
        self.port = port
        self.band = band
        self.byte = byte
        self.link = None  # 串口链接
        self.read_switch = True  # 读写开关
        self.Buffer = []  # 存储机器返回数据缓冲区

    """
    :function open:打开串口链接
    :return:是否成功打开,打开不成功返回False
    """

    def open(self):
        if self.link is not None and self.link.isOpen():
            self.link.close()

        try:
            self.link = serial.Serial(self.port, self.band, self.byte)
        except serial.SerialException:

            print "Has error T_T"
            return False
        print "Has successfully linked ^_^"
        return True

    """
    :function close:关闭串口链接
    :return: 是否成功关闭
    """

    def close(self):
        if self.link is not None and self.link.isOpen():
            self.link.close()
            print "Link is closed ^_^"

        return True

    """
    :function write:通过串口传输命令
    """

    def write(self, data):
        if self.link is None or not self.link.isOpen():
            if not self.open():
                print "Serial Port is not Opened"
                return False

        buf = bytearray(data)

        for it in buf:
            print "%x " % it,
        print ""

        self.link.write(buf)
        print "Has successfully wrote ^_^"

        return True

    """
    :function __read:读取数据,压入缓冲区
    """

    def __read(self):
        buf = []
        while True:
            buf.append(self.link.read(1))
            if self.read_switch:
                self.Buffer.extend(buf)
                buf = []

    """
    :function read:开启子线程读取机器返回数据
    """

    def read(self):
        if self.link is None or not self.link.isOpen():
            if self.open():
                print "Serial Port is cannot Opened"
                return False

        thread.start_new_thread(self.__read, ())

        return True
