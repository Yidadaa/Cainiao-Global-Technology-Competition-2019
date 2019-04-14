#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError

import os
import numpy as np
import ntpath
import time

imu_publisher = rospy.Publisher('/cainiao_camera/imu', Imu, queue_size=1)

def publish_imu_data(acceleration, angular):
    imu = Imu()
    [ax, ay, az, at] = acceleration
    [gx, gy, gz, gt] = angular
    # ts = rospy.Time.from_sec((float(at) / 1000))
    ts = rospy.Time.now()
    
    print(at)

    header = Header(stamp=ts)
    header.frame_id = 'world'
    
    imu.angular_velocity.x = float(gx)
    imu.angular_velocity.y = float(gy)
    imu.angular_velocity.z = float(gz)
    imu.linear_acceleration.x = float(ax)
    imu.linear_acceleration.y = float(ay)
    imu.linear_acceleration.z = float(az)
    imu.header = header
    imu_publisher.publish(imu)

def publish_imu(imu_path):
    base_string = ntpath.basename(imu_path).replace('.data', '')
    base_ts = int(base_string.split('_')[0].split('-')[3])
    a_data = []
    g_data = []
    with open(imu_path, 'r') as f:
        line = f.readline()
        # 加速度数据
        count = int(line.split(',')[1])
        while count > 0:
            a_data.append(f.readline().split(','))
            count -= 1
        line = f.readline()
        # 处理陀螺仪数据
        count = int(line.split(',')[1])
        while count > 0:
            g_data.append(f.readline().split(','))
            count -= 1
    base_ts = min(int(a_data[0][3]), int(g_data[0][3]))
    count = min(len(a_data), len(g_data))
    for i in range(count):
        publish_imu_data(a_data[i], g_data[i])
        time.sleep(1.0 / 200)

if __name__ == '__main__':
    root = './files/'
    files = os.listdir(root)
    imu_path = ''
    for f in files:
        if f[-1] == 'a':
            imu_path = root + f
    publish_imu(imu_path)