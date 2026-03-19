#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import math
import numpy as np

import rospy
from dynamixel_control.msg import SetPosition


class LegState(object):
    def __init__(self, name, motor_ids, phase_offset, hip_yaw_deg):
        self.name = name
        self.motor_ids = motor_ids  # [joint1, joint2, joint3]
        self.phase_offset = phase_offset
        self.hip_yaw = math.radians(hip_yaw_deg)


class QuadrupedGaitController(object):
    def __init__(self):
        rospy.init_node("gait_controller", anonymous=False)

        self.pub = rospy.Publisher("set_position", SetPosition, queue_size=200)

        self.rate_hz = rospy.get_param("~rate_hz", 100.0)
        self.gait = rospy.get_param("~gait", "crawl")  # crawl or trot
        self.freq_hz = rospy.get_param("~gait_frequency", 0.4)

        # Step command in base frame. +x is robot forward.
        self.step_x = rospy.get_param("~step_x", 40.0)
        self.step_y = rospy.get_param("~step_y", 0.0)
        self.swing_height = rospy.get_param("~swing_height", 25.0)

        self.nominal_x = rospy.get_param("~nominal_x", 130.0)
        self.nominal_y = rospy.get_param("~nominal_y", 0.0)
        self.nominal_z = rospy.get_param("~nominal_z", -170.0)

        self.l_coxa = rospy.get_param("~link_coxa", 44.75)
        self.l_femur = rospy.get_param("~link_femur", 74.0)
        self.l_tibia = rospy.get_param("~link_tibia", 150.0)

        # 提取被动关节和基础配置参数
        self.l_a3 = rospy.get_param("~link_a3", 41.5)
        self.l_d6 = rospy.get_param("~link_d6", -13.5)
        self.l_d7 = rospy.get_param("~link_d7", -106.7)
        self.base_r = rospy.get_param("~base_radius", 203.06)

        self.gear_ratio_joint23 = rospy.get_param("~gear_ratio_joint23", 4.421)

        self.motor_home = rospy.get_param(
            "~motor_home_ticks",
            {
                "1": 2048, "2": 0, "3": 0,
                "4": 2048, "5": 0, "6": 0,
                "7": 2048, "8": 0, "11": 0,
                "12": 2048, "13": 0, "14": 0,
            },
        )

        self.motor_dir = rospy.get_param(
            "~motor_dir",
            {
                "1": 1, "2": 1, "3": 1,
                "4": -1, "5": -1, "6": -1,
                "7": 1, "8": 1, "11": 1,
                "12": -1, "13": -1, "14": -1,
            },
        )

        self.joint_limit_deg = rospy.get_param(
            "~joint_limit_deg",
            {
                "j1": [-90.0, 90.0],
                "j2": [-100.0, 100.0],
                "j3": [-10.0, 190.0],
            },
        )

        self.joint_zero_deg = rospy.get_param(
            "~joint_zero_deg",
            {
                "j1": 0.0,
                "j2": 0.0,
                "j3": 90.0,
            },
        )

        self.single_turn_ids = set([int(v) for v in rospy.get_param("~single_turn_ids", [11, 12, 13, 14])])

        self.legs = self._build_legs()

        rospy.loginfo("gait_controller started: gait=%s, freq=%.3f Hz", self.gait, self.freq_hz)

    def _build_legs(self):
        # 4 条腿的 base->leg yaw，按图中的 45 度对称布局。
        # motor_ids 默认采用 [关节1(XW540), 关节2(XW430), 关节3(XW430)]
        default_leg_cfg = {
            "lf": {"motor_ids": [11, 1, 2], "hip_yaw_deg": 45.0},
            "rf": {"motor_ids": [12, 3, 4], "hip_yaw_deg": -45.0},
            "lr": {"motor_ids": [14, 7, 8], "hip_yaw_deg": 135.0},
            "rr": {"motor_ids": [13, 5, 6], "hip_yaw_deg": -135.0},
        }
        leg_cfg = rospy.get_param("~legs", default_leg_cfg)

        if self.gait == "trot":
            phase = {"lf": 0.0, "rr": 0.0, "rf": 0.5, "lr": 0.5}
        else:
            # crawl: 同时仅 1 条腿摆动
            phase = {"lf": 0.00, "rf": 0.25, "rr": 0.50, "lr": 0.75}  

        legs = []
        for name in ["lf", "rf", "rr", "lr"]:
            motor_ids = [int(v) for v in leg_cfg[name]["motor_ids"]]
            hip_yaw_deg = float(leg_cfg[name]["hip_yaw_deg"])
            legs.append(LegState(name, motor_ids, phase[name], hip_yaw_deg))
        return legs

    def _swing_ratio(self):
        if self.gait == "trot":
            return rospy.get_param("~trot_swing_ratio", 0.5)
        return rospy.get_param("~crawl_swing_ratio", 0.25)

    def _foot_delta_base(self, phase):
        swing_ratio = self._swing_ratio()

        if phase < swing_ratio:
            u = phase / max(swing_ratio, 1e-6)
            x = -self.step_x * 0.5 + self.step_x * u
            y = -self.step_y * 0.5 + self.step_y * u
            z = self.nominal_z + self.swing_height * math.sin(math.pi * u)
        else:
            u = (phase - swing_ratio) / max(1.0 - swing_ratio, 1e-6)
            x = self.step_x * 0.5 - self.step_x * u
            y = self.step_y * 0.5 - self.step_y * u
            z = self.nominal_z

        return (x, y, z)

    def _base_to_leg_transform(self, dx_base, dy_base, tz_base, leg):
        """
        严格通过齐次变换矩阵(HTM)计算坐标系转换。
        """
        yaw = leg.hip_yaw
        
        # 根据图示：基座中心到旋转偏航关节(Yaw)的固定偏距为 203.06 + 44.75
        r_yaw = self.base_r + self.l_coxa
        
        # T_base_leg0: 从 base 坐标系变换到 leg0 坐标系的齐次矩阵
        T_b_l = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, r_yaw * math.cos(yaw)],
            [math.sin(yaw),  math.cos(yaw), 0, r_yaw * math.sin(yaw)],
            [0,              0,             1, 0],   # 图注明确同一平面对应Z原点重合
            [0,              0,             0, 1]
        ])
        
        T_l_b = np.linalg.inv(T_b_l)
        
        # P_nom_leg: 在每条腿系下定义的足端基准常态位置
        P_nom_leg = np.array([self.nominal_x, self.nominal_y, self.nominal_z, 1.0])
        
        # 映射到 Base 原点坐标系进行世界步态偏移叠加
        P_nom_base = np.dot(T_b_l, P_nom_leg)
        
        P_target_base = P_nom_base.copy()
        P_target_base[0] += dx_base
        P_target_base[1] += dy_base
        P_target_base[2] = tz_base  # 替换Z方向高度
        
        # 再由 Base 逆映射回 Leg 独立局部标系进行逆解
        P_target_leg = np.dot(T_l_b, P_target_base)
        
        return P_target_leg[0], P_target_leg[1], P_target_leg[2]

    def _ik_transform_matrix_solve(self, x, y, z):
        """
        基于万向节中心为目标点的代数反解法。
        既然关节4和5原点在万向节中心，且地面行走时万向节以下顺应地面，
        那么系统真正该由主动关节决定的是"万向节中心点"。
        """
        # 在 Leg 坐标系里，我们要把总的足端控制目标 (x, y, z)
        # 上升一个万向节下方的垂直机构补偿高度，回推到“万向节中心”应处于的目标点。
        
        universal_joint_h = abs(self.l_d6 + self.l_d7)  # 13.5 + 106.7 = 120.2
        p_z = z + universal_joint_h
        
        # 1. 关节1控制平面偏航角
        q1 = math.atan2(y, x)
        
        # 2. 算到万向节中心点的极轴半径降维
        R_total = math.hypot(x, y)
        
        # 按照图示，横向距离中的 74 是受 q1 旋转但始终水平的固定偏距。
        # R_prime = R_total - 74，是留给 150 和 41.5 这两根主动连杆去折叠的长度。
        R_prime = max(R_total - self.l_femur, 1.0)
        
        # 3. 二段主动连杆求交 (以 l_tibia=150, l_a3=41.5 为二连杆) 对 (R_prime, p_z)
        L1 = self.l_tibia
        L2 = self.l_a3
        
        D_sq = R_prime**2 + p_z**2
        cos_theta3 = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_theta3 = self._clamp(cos_theta3, -1.0, 1.0)
        
        # IK通常取负根（膝关节“外翻/下折”）
        theta3 = math.atan2(-math.sqrt(max(0.0, 1.0 - cos_theta3**2)), cos_theta3)
        
        # 补偿 MDH 模型中的偏移 (q3 = theta3 + 90)
        q3 = theta3 + math.radians(90.0)
        
        # q2 角度
        q2 = math.atan2(p_z, R_prime) - math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3))

        q1_deg = math.degrees(q1)
        q2_deg = math.degrees(q2)
        q3_deg = math.degrees(q3)

        q1_deg = self._clamp(q1_deg, self.joint_limit_deg["j1"][0], self.joint_limit_deg["j1"][1])
        q2_deg = self._clamp(q2_deg, self.joint_limit_deg["j2"][0], self.joint_limit_deg["j2"][1])
        q3_deg = self._clamp(q3_deg, self.joint_limit_deg["j3"][0], self.joint_limit_deg["j3"][1])

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
            motor_deg = output_deg
            ticks = home + direction * motor_deg * unit
            ticks = self._clamp(ticks, 0.0, 4095.0)
        else:
            motor_deg = output_deg * self.gear_ratio_joint23
            ticks = home + direction * motor_deg * unit

        return int(round(ticks))

    def _publish_leg(self, leg, t_now):
        phase = (t_now * self.freq_hz + leg.phase_offset) % 1.0
        
        # 根据相频得出在 base_link 中应有的相对平移量和Z方向抬腿量
        dx_base, dy_base, tz_base = self._foot_delta_base(phase)
        
        # 利用齐次变换矩阵进行原点偏心和姿态的正逆映射
        tx_leg, ty_leg, tz_leg = self._base_to_leg_transform(dx_base, dy_base, tz_base, leg)

        # 运动学几何矩阵反解得到各关节输入角
        q1, q2, q3 = self._ik_transform_matrix_solve(tx_leg, ty_leg, tz_leg)

        q1_cmd = q1 + float(self.joint_zero_deg["j1"])
        q2_cmd = q2 + float(self.joint_zero_deg["j2"])
        q3_cmd = q3 + float(self.joint_zero_deg["j3"])

        cmd = [q1_cmd, q2_cmd, q3_cmd]
        for i in range(3):
            mid = leg.motor_ids[i]
            single_turn = mid in self.single_turn_ids
            ticks = self._output_deg_to_ticks(mid, cmd[i], single_turn)
            msg = SetPosition()
            msg.id = int(mid)
            msg.position = int(ticks)
            self.pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            for leg in self.legs:
                self._publish_leg(leg, t)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = QuadrupedGaitController()
        node.spin()
    except rospy.ROSInterruptException:
        pass
