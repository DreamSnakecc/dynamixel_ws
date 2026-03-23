#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
四足机器人底层核心运动学与初始化节点 (Quadruped Core)
负责：读取电机及连杆配置、平滑上电归零(插值到名义预备姿态)、进行运动学逆解（IK）、并将角度指令下发给舵机底层的C++节点。
"""

from __future__ import division

import math
import numpy as np

import rospy
from dynamixel_control.msg import SetPosition
from std_msgs.msg import Bool, Float64MultiArray
from dynamixel_control.srv import GetPosition

class LegKinematics(object):
    def __init__(self, name, motor_ids, hip_yaw_deg):
        self.name = name
        self.motor_ids = motor_ids
        self.hip_yaw = math.radians(hip_yaw_deg)

class QuadrupedCoreNode(object):
    def __init__(self):
        rospy.init_node("quadruped_core", anonymous=False)

        self.pub = rospy.Publisher("set_position", SetPosition, queue_size=200)
        self.ready_pub = rospy.Publisher("core_ready", Bool, queue_size=1, latch=True)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/gait_controller/" + name, default))

        # Base nominal limits & setup
        self.nominal_x = get_cfg("nominal_x", 200.0)
        self.nominal_y = get_cfg("nominal_y", 0.0)
        self.nominal_z = get_cfg("nominal_z", -180.0)

        self.l_coxa = get_cfg("link_coxa", 44.75)
        self.l_femur = get_cfg("link_femur", 74.0)
        self.l_tibia = get_cfg("link_tibia", 150.0)

        # 提取被动关节和基础配置参数
        self.l_a3 = get_cfg("link_a3", 41.5)
        self.l_d6 = get_cfg("link_d6", -13.5)
        self.l_d7 = get_cfg("link_d7", -106.7)
        self.base_r = get_cfg("base_radius", 203.06)

        self.gear_ratio_joint23 = get_cfg("gear_ratio_joint23", 4.421)

        self.motor_home = get_cfg(
            "motor_home_ticks",
            {
                "1": 2048, "2": 0, "3": 0,
                "4": 2048, "5": 0, "6": 0,
                "7": 2048, "8": 0, "11": 0,
                "12": 2048, "13": 0, "14": 0,
            },
        )

        self.motor_dir = get_cfg(
            "motor_dir",
            {
                "1": 1, "2": 1, "3": 1,
                "4": -1, "5": -1, "6": -1,
                "7": 1, "8": 1, "11": 1,
                "12": -1, "13": -1, "14": -1,
            },
        )

        self.joint_limit_deg = get_cfg(
            "joint_limit_deg",
            {
                "j1": [-90.0, 90.0],
                "j2": [-100.0, 100.0],
                "j3": [-10.0, 190.0],
            },
        )

        self.joint_zero_deg = get_cfg(
            "joint_zero_deg",
            {
                "j1": 0.0,
                "j2": 0.0,
                "j3": 90.0,
            },
        )

        self.single_turn_ids = set([int(v) for v in get_cfg("single_turn_ids", [11, 12, 13, 14])])
        
        default_leg_cfg = {
            "lf": {"motor_ids": [11, 1, 2], "hip_yaw_deg": 45.0},
            "rf": {"motor_ids": [12, 3, 4], "hip_yaw_deg": -45.0},
            "lr": {"motor_ids": [14, 7, 8], "hip_yaw_deg": 135.0},
            "rr": {"motor_ids": [13, 5, 6], "hip_yaw_deg": -135.0},
        }
        leg_cfg = get_cfg("legs", default_leg_cfg)
        
        # 强制按照特定顺序排列（与上层步态话题接收对齐）
        self.legs = []
        for name in ["lf", "rf", "lr", "rr"]:
            motor_ids = [int(v) for v in leg_cfg[name]["motor_ids"]]
            hip_yaw_deg = float(leg_cfg[name]["hip_yaw_deg"])
            self.legs.append(LegKinematics(name, motor_ids, hip_yaw_deg))

        self.ready_for_commands = False
        self.ready_pub.publish(Bool(data=False))
        rospy.loginfo("quadruped_core started. Preparing for smooth homing...")

    def _base_to_leg_transform(self, dx_base, dy_base, tz_base, leg): 
        yaw = leg.hip_yaw
        r_yaw = self.base_r + self.l_coxa
        
        T_b_l = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, r_yaw * math.cos(yaw)],
            [math.sin(yaw),  math.cos(yaw), 0, r_yaw * math.sin(yaw)],
            [0,              0,             1, 0],  
            [0,              0,             0, 1]
        ])  # Base -> Leg 的齐次变换矩阵    
        T_l_b = np.linalg.inv(T_b_l)  # 计算逆变换矩阵：Leg -> Base 的逆为 Base -> Leg
        P_nom_leg = np.array([self.nominal_x, self.nominal_y, self.nominal_z, 1.0])
        P_nom_base = np.dot(T_b_l, P_nom_leg)
        
        P_target_base = P_nom_base.copy()
        P_target_base[0] += dx_base
        P_target_base[1] += dy_base
        P_target_base[2] = tz_base  
        
        P_target_leg = np.dot(T_l_b, P_target_base)
        return P_target_leg[0], P_target_leg[1], P_target_leg[2]

    def _ik_transform_matrix_solve(self, x, y, z):
        universal_joint_h = abs(self.l_d6 + self.l_d7)
        p_z = z + universal_joint_h
        
        q1 = math.atan2(y, x)
        R_total = math.hypot(x, y)
        R_prime = max(R_total - self.l_femur, 1.0)
        
        L1 = self.l_tibia
        L2 = self.l_a3
        D_sq = R_prime**2 + p_z**2
        cos_theta3 = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_theta3 = self._clamp(cos_theta3, -1.0, 1.0)
        
        theta3 = math.atan2(-math.sqrt(max(0.0, 1.0 - cos_theta3**2)), cos_theta3)
        q3 = theta3 + math.radians(90.0)
        q2 = math.atan2(p_z, R_prime) - math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3))

        q1_deg = self._clamp(math.degrees(q1), self.joint_limit_deg["j1"][0], self.joint_limit_deg["j1"][1])
        q2_deg = self._clamp(math.degrees(q2), self.joint_limit_deg["j2"][0], self.joint_limit_deg["j2"][1])
        q3_deg = self._clamp(math.degrees(q3), self.joint_limit_deg["j3"][0], self.joint_limit_deg["j3"][1])

        return (q1_deg, q2_deg, q3_deg)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _output_deg_to_ticks(self, motor_id, output_deg, single_turn):
        unit = 4096.0 / 360.0
        key = str(int(motor_id))
        home = float(self.motor_home.get(key, 0.0))
        direction = float(self.motor_dir.get(key, 1.0))

        if single_turn:
            ticks = home + direction * output_deg * unit
            ticks = self._clamp(ticks, 0.0, 4095.0)
        else:
            ticks = home + direction * (output_deg * self.gear_ratio_joint23) * unit
            ticks = self._clamp(ticks, -1048575.0, 1048575.0)
        return int(round(ticks))

    def compute_and_output_leg_ik(self, dx_base, dy_base, dz_delta_base, leg):
        """进行IK解算并可将数值转化为发送端所需的控制帧(或立即下发)"""
        tz_base = self.nominal_z + dz_delta_base
        tx, ty, tz_leg = self._base_to_leg_transform(dx_base, dy_base, tz_base, leg)
        q1, q2, q3 = self._ik_transform_matrix_solve(tx, ty, tz_leg)

        q1_cmd = q1 + float(self.joint_zero_deg["j1"])
        q2_cmd = q2 + float(self.joint_zero_deg["j2"])
        q3_cmd = q3 + float(self.joint_zero_deg["j3"])
        
        cmd = [q1_cmd, q2_cmd, q3_cmd]
        tick_results = {}
        for i in range(3):
            mid = leg.motor_ids[i]
            single_turn = mid in self.single_turn_ids
            ticks = self._output_deg_to_ticks(mid, cmd[i], single_turn)
            tick_results[mid] = ticks
            
            # 如果核心已就位准备接收高层命令，则立即发布给底层
            if self.ready_for_commands:
                msg = SetPosition()
                msg.id = int(mid)
                msg.position = int(ticks)
                self.pub.publish(msg)
                
        return tick_results

    def _smooth_homing(self):
        rospy.loginfo("Waiting for get_position services to become available...")
        rospy.wait_for_service('/left_board/get_position')
        rospy.wait_for_service('/right_board/get_position')
        
        get_pos_left = rospy.ServiceProxy('/left_board/get_position', GetPosition)
        get_pos_right = rospy.ServiceProxy('/right_board/get_position', GetPosition)
        
        rospy.loginfo("Detecting current physical positions...")
        start_positions = {}
        left_ids = [11, 1, 2, 14, 7, 8]
        right_ids = [12, 3, 4, 13, 5, 6]
        
        for leg in self.legs:
            for mid in leg.motor_ids:
                try:
                    if mid in left_ids:
                        res = get_pos_left(mid)
                    else:
                        res = get_pos_right(mid)
                        
                    raw_pos = res.position
                    if raw_pos > 2147483647:
                        raw_pos -= 4294967296
                    start_positions[mid] = raw_pos
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed for ID %d: %s" % (mid, e))
                    start_positions[mid] = int(self.motor_home.get(str(mid), 2048))
                    
        # 预计算：标定回零位 (以读取到的 motor_home 为目标零位)
        target_positions = {}
        for leg in self.legs:
            for mid in leg.motor_ids:
                target = int(self.motor_home.get(str(mid), 2048))
                target_positions[mid] = target
                rospy.loginfo("Joint %d: Current Position %d -> Target Home %d", mid, start_positions[mid], target)

        rospy.loginfo("Starting smooth interpolation to home position...")
        duration = 4.0  
        hz = 50.0
        steps = int(duration * hz)
        rate = rospy.Rate(hz)
        
        for step in range(1, steps + 1):
            for leg in self.legs:
                for mid in leg.motor_ids:
                    target_pos = float(target_positions[mid])
                    start_pos = float(start_positions[mid])
                    
                    current_target = start_pos + (target_pos - start_pos) * (step / float(steps))
                    current_target = int(current_target)
                    
                    single_turn = mid in self.single_turn_ids
                    if single_turn:
                        current_target = max(0, min(4095, current_target))
                    else:
                        current_target = max(-1048575, min(1048575, current_target))
                        
                    msg = SetPosition()
                    msg.id = int(mid)
                    msg.position = current_target
                    self.pub.publish(msg)
            rate.sleep()
            
        rospy.loginfo("Smooth stance initialization completed. Core is ready to receive foot trajectories.")
        self.ready_for_commands = True
        self.ready_pub.publish(Bool(data=True))

    def gait_command_callback(self, msg):
        """
        接收来自高层步态生成器的时间序列相对偏移值
        msg.data 数组含 12 个 float (dx, dy, dz), 每个腿占3个，按 [lf, rf, lr, rr] 顺序排列
        """
        if not self.ready_for_commands:
            return
            
        if len(msg.data) != 12:
            rospy.logwarn("Expected 12 elements in cmd_foot_pose, got %d", len(msg.data))
            return
            
        for i, leg in enumerate(self.legs):
            base_idx = i * 3
            dx = msg.data[base_idx + 0]
            dy = msg.data[base_idx + 1]
            dz = msg.data[base_idx + 2]
            
            # 使用算好的偏移做实时 IK 下发
            self.compute_and_output_leg_ik(dx, dy, dz, leg)

    def spin(self):
        # 1. 启动完成平滑归零与站立初始化
        self._smooth_homing()
        
        # 2. 挂载订阅话题，准备接收高层四足运动指令
        rospy.Subscriber("cmd_foot_pose", Float64MultiArray, self.gait_command_callback)
        
        # 3. 阻塞维护
        rospy.spin()

if __name__ == "__main__":
    try:
        node = QuadrupedCoreNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass