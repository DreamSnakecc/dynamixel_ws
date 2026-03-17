#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

# Control table address
# This differs depending on the Dynamixel model. (These are typical for X-series)
ADDR_OPERATING_MODE         = 11          # Operating Mode (1: Velocity, 3: Position, 4: Extended Position, 16: PWM, etc)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
BAUDRATE                    = 1000000

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting
DXL_ID                      = 1                 # Dynamixel ID: 1
DEVICENAME                  = '/dev/ttyUSB0'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

def configure_dynamixel():
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not portHandler.openPort():
        rospy.logerr("Failed to open the port")
        return None, None
    rospy.loginfo("Succeeded to open the port")

    # Set port baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Failed to change the baudrate")
        return portHandler, None
    rospy.loginfo("Succeeded to change the baudrate")

    # Read and Log Current Operating Mode
    mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE)
    if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
        rospy.loginfo("Current Operating Mode (1:Vel, 3:Pos, 4:ExtPos, 16:PWM): %d", mode)
    else:
        rospy.logwarn("Failed to read Operating Mode.")

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.logerr("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel has been successfully connected")

    return portHandler, packetHandler

def goal_position_callback(msg, args):
    portHandler, packetHandler = args
    goal_pos = msg.data
    rospy.loginfo("Setting Goal Position: %d", goal_pos)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_pos)
    
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.logerr("%s" % packetHandler.getRxPacketError(dxl_error))

def main():
    rospy.init_node('single_servo_node', anonymous=True)
    global DXL_ID, DEVICENAME, BAUDRATE

    DXL_ID = rospy.get_param('~dynamixel_info/dxl_id', 1)
    DEVICENAME = rospy.get_param('~dynamixel_info/port', '/dev/ttyUSB0')
    BAUDRATE = rospy.get_param('~dynamixel_info/baudrate', 1000000)

    portHandler, packetHandler = configure_dynamixel()

    if packetHandler is None:
        rospy.logerr("Failed to configure Dynamixel, exiting...")
        return

    rospy.Subscriber('set_position', Int32, goal_position_callback, (portHandler, packetHandler))
    pos_pub = rospy.Publisher('present_position', Int32, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            pos_pub.publish(dxl_present_position)
        
        rate.sleep()

    # Disable Dynamixel Torque
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
