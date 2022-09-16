#!/usr/bin/python
# -*-coding: utf-8 -*-
# author: Zenan-SSR
# data: 2022年9月16日
# description: 用于潇洒师兄项目pico-IMU，采集来自pico-IMU的UDP数据并绘图.

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import sys
import socket
import csv
import datetime
import serial  # 导入模块


class SerialPort:
    def __init__(self, port, boud):
        self.port = serial.Serial(port, boud)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

# 修改串口端口&波特率
serialPort = 'COM1'
baudRate = 115200
# 打开串口
mSerial = SerialPort(serialPort, baudRate)

# 全局变量用于UDP数据保存
str_time = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
f_UDP = open('PICO_IMU-SR_' + str_time + '.csv', 'w', encoding='utf-8', newline='')
header1 = [str_time, 'AFS_16G', 'GFS_2000DPS', 'AODR_1000Hz', 'GODR_1000Hz']
header2 = ['Frame', 'SysTime(ms)', '1000xax', '1000xay', '1000xaz', '10xgx',
          '10xgy', '10xgz', 'Gtemperature']
writer_UDP = csv.writer(f_UDP)
writer_UDP.writerow(header1)
writer_UDP.writerow(header2)

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
client.setblocking(1)
client.bind(('10.168.1.127', 3355))

win = pg.GraphicsLayoutWidget(show=True, title='PICO_IMU-SR IMU raw data')
# win.setWindowTitle('pyqtgraph example: Scrolling Plots')

# raw-acc
p1 = win.addPlot(row=0, col=0)
data1 = 0*np.ones(500)
curve1 = p1.plot(data1, pen=(1, 6))
p2 = win.addPlot(row=1, col=0)
data2 = 0*np.ones(500)
curve2 = p2.plot(data2, pen=(2, 6))
p3 = win.addPlot(row=2, col=0)
data3 = 0*np.ones(500)
curve3 = p3.plot(data3, pen=(3, 6))

# raw-gyro
p4 = win.addPlot(row=0, col=1)
data4 = 0*np.ones(500)
curve4 = p4.plot(data4, pen=(4, 6))
p5 = win.addPlot(row=1, col=1)
data5 = 0*np.ones(500)
curve5 = p5.plot(data5, pen=(5, 6))
p6 = win.addPlot(row=2, col=1)
data6 = 0*np.ones(500)
curve6 = p6.plot(data6, pen=(6, 6))

# sensor-fusion
p7 = win.addPlot(row=0, col=2)
data7 = 0*np.ones(500)
curve7 = p7.plot(data7, pen=(4, 6))
p8 = win.addPlot(row=1, col=2)
data8 = 0*np.ones(500)
curve8 = p8.plot(data8, pen=(5, 6))
p9 = win.addPlot(row=2, col=2)
data9 = 0*np.ones(500)
curve9 = p9.plot(data9, pen=(6, 6))

acc_X = 0
acc_Y = 0
acc_Z = 0
gyro_X = 0
gyro_Y = 0
gyro_Z = 0
pitch = 0
roll = 0
yaw = 0

# 画图进程
def update1():
    global data1, curve1, acc_X, \
        data2, curve2, acc_Y, \
        data3, curve3, acc_Z, \
        data4, curve4, gyro_X, \
        data5, curve5, gyro_Y, \
        data6, curve6, gyro_Z, \
        data7, curve7, pitch, \
        data8, curve8, roll, \
        data9, curve9, yaw

    data1[:-1] = data1[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data1[-1] = acc_X
    curve1.setData(data1)

    data2[:-1] = data2[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data2[-1] = acc_Y
    curve2.setData(data2)

    data3[:-1] = data3[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data3[-1] = acc_Z
    curve3.setData(data3)

    data4[:-1] = data4[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data4[-1] = gyro_X
    curve4.setData(data4)

    data5[:-1] = data5[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data5[-1] = gyro_Y
    curve5.setData(data5)

    data6[:-1] = data6[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data6[-1] = gyro_Z
    curve6.setData(data6)

    data7[:-1] = data7[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data7[-1] = pitch
    curve7.setData(data7)

    data8[:-1] = data8[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data8[-1] = roll
    curve8.setData(data8)

    data9[:-1] = data9[1:]  # shift data in the array one sample left
    # (see also: np.roll)
    data9[-1] = yaw
    curve9.setData(data9)

    mSerial.port.write(f"{pitch}:{roll}:{yaw}\r\n".encode("utf-8"))


# 获取UDP传感器数据
def UDP_data_collection():
    try:
        global writer_UDP, acc_X, acc_Y, acc_Z, gyro_X, gyro_Y, gyro_Z, pitch, roll, yaw
        # Data received
        data, addr = client.recvfrom(1024)
        # print("received message: %s from %s" % (data, addr))
        split_line = data.decode().split('|')
        print(split_line)
        writer_UDP.writerow(split_line)
        # return the decoded bytes as a string
        acc_X = int(split_line[2]) / 1000.0
        acc_Y = int(split_line[3]) / 1000.0
        acc_Z = int(split_line[4]) / 1000.0

        gyro_X = int(split_line[5]) / 100.0
        gyro_Y = int(split_line[6]) / 100.0
        gyro_Z = int(split_line[7]) / 100.0

        pitch = int(split_line[9]) / 100.0
        roll = int(split_line[10]) / 100.0
        yaw = int(split_line[11]) / 100.0

        return data.decode()
    # If no data is received just return None
    except socket.error:
        return None


# UDP数据采集进程
def update2():
    # Check for UDP data
    line = UDP_data_collection()
    # If there is data split it and print it to console
    if line:
        split_line = line.split('|')
        # print(split_line)


timer1 = pg.QtCore.QTimer()
timer1.timeout.connect(update1)
timer1.start(20)

timer2 = pg.QtCore.QTimer()
timer2.timeout.connect(update2)
timer2.start(10)


if __name__ == '__main__':
    pg.exec()
