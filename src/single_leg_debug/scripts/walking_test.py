#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
单腿和连杆的步态基础调试节点 (Walking / Motion Generation)
生成简单的周期性正弦函数，发送给三个关节（或通过逆向运动学结算）。
这里使用最基础的基于相对偏移量的步态模拟。
"""

import rospy
from std_msgs.msg import Int32MultiArray
import math
import time

def walking_generator():
    rospy.init_node('walking_test_node', anonymous=True)
    pub = rospy.Publisher('set_leg_positions', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(50) # 50 Hz

    t = 0.0
    freq = 1.0  # Gait frequency in Hz
    amp_coxa  = 200  # Amplitude in raw DXL units
    amp_femur = 400
    amp_tibia = 300

    rospy.loginfo("Starting basic cyclic walking pattern on single leg...")

    while not rospy.is_shutdown():
        # Generate simple sinusoidal patterns for the three joints
        # To simulate a stepping motion
        
        coxa_cmd = int(amp_coxa * math.sin(2 * math.pi * freq * t))
        femur_cmd = int(amp_femur * math.cos(2 * math.pi * freq * t + math.pi/4))   # 相位差
        tibia_cmd = int(amp_tibia * math.sin(2 * math.pi * freq * t + math.pi/2)) # 相位差

        msg = Int32MultiArray()
        msg.data = [coxa_cmd, femur_cmd, tibia_cmd]
        
        pub.publish(msg)

        t += 1.0 / 50.0 # Time step
        rate.sleep()

if __name__ == '__main__':
    try:
        walking_generator()
    except rospy.ROSInterruptException:
        pass