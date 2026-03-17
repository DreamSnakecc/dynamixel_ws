#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
单个机械腿标定节点 (Single Leg Calibration Node)
包含关节零点标定、单腿联动调试。
"""

import rospy
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import *
import time

# Control table address
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

PROTOCOL_VERSION            = 2.0
TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
HOME_POSITION               = 2048

class LegCalibrationNode:
    def __init__(self):
        rospy.init_node('leg_calibration_node', anonymous=True)

        # 读取参数
        self.port_name = rospy.get_param('~leg_info/port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~leg_info/baudrate', 1000000)
        self.joint_ids = rospy.get_param('~leg_info/joint_ids', [1, 2, 3])
        self.offsets = rospy.get_param('~leg_info/offsets', [0, 0, 0])
        self.directions = rospy.get_param('~leg_info/directions', [1, 1, 1])

        # 初始化 Dynamixel
        self.portHandler = PortHandler(self.port_name)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # 1. 打开端口
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port: %s" % self.port_name)
            exit()
        
        # 2. 设置波特率
        if not self.portHandler.setBaudRate(self.baudrate):
            rospy.logerr("Failed to change the baudrate: %d" % self.baudrate)
            exit()

        # 3. 使能 Torque
        for dxl_id in self.joint_ids:
            self._write_1byte(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            rospy.loginfo("Dynamixel ID: %d Torque Enabled." % dxl_id)

        # 初始化位置到标定零点
        self.go_to_home()

        # 订阅回调接口（数组接收三个位姿）
        rospy.Subscriber('set_leg_positions', Int32MultiArray, self.position_callback)
        self.pos_pub = rospy.Publisher('present_leg_positions', Int32MultiArray, queue_size=10)

    def _write_1byte(self, dxl_id, addr, data):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, addr, data)
        # 异常处理略 (为简洁省略具体打印)

    def _write_4byte(self, dxl_id, addr, data):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, addr, data)
    
    def _read_4byte(self, dxl_id, addr):
        data, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, addr)
        return data

    def go_to_home(self):
        rospy.loginfo("Going to calibrated home position...")
        for i, dxl_id in enumerate(self.joint_ids):
            target = HOME_POSITION + self.offsets[i] * self.directions[i]
            self._write_4byte(dxl_id, ADDR_GOAL_POSITION, target)
        rospy.sleep(1.0)
        rospy.loginfo("Home position reached.")

    def position_callback(self, msg):
        """
        接收相对零点的偏移指令 (数组长度3)
        msg.data = [coxa_cmd, femur_cmd, tibia_cmd]
        """
        if len(msg.data) == len(self.joint_ids):
            for i, dxl_id in enumerate(self.joint_ids):
                # Target = Home + 结构零点偏置 + (正负向旋转 * 运动指令)
                target = HOME_POSITION + self.offsets[i] * self.directions[i] + (self.directions[i] * msg.data[i])
                
                # 限幅保护
                target = max(0, min(4095, target))
                self._write_4byte(dxl_id, ADDR_GOAL_POSITION, target)
        else:
            rospy.logwarn("Received array size mismatch! Expected %d, got %d", len(self.joint_ids), len(msg.data))

    def spin(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            current_positions = Int32MultiArray()
            # 读取当前所有关节位置
            for i, dxl_id in enumerate(self.joint_ids):
                raw_pos = self._read_4byte(dxl_id, ADDR_PRESENT_POSITION)
                current_positions.data.append(raw_pos)
            
            self.pos_pub.publish(current_positions)
            rate.sleep()

        # 断电
        for dxl_id in self.joint_ids:
            self._write_1byte(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()

if __name__ == '__main__':
    try:
        node = LegCalibrationNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass