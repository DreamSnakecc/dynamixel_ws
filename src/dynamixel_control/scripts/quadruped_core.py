#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
四足机器人底层核心运动学与初始化节点 (Quadruped Core)
负责：读取电机及连杆配置、平滑上电归零(插值到名义预备姿态)、进行运动学逆解（IK）、并将角度指令下发给舵机底层的C++节点。
"""

from __future__ import division

import math

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

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/gait_controller/" + name, default))

        self.pub = rospy.Publisher("set_position", SetPosition, queue_size=200)
        self.calibration_ready_pub = rospy.Publisher("calibration_ready", Bool, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher("core_ready", Bool, queue_size=1, latch=True)
        rospy.Subscriber("start_nominal_pose", Bool, self.start_nominal_pose_callback)

        # Nominal foot position in each leg local frame.
        self.nominal_x = get_cfg("nominal_x", 200.0)
        self.nominal_y = get_cfg("nominal_y", 0.0)
        self.nominal_z = get_cfg("nominal_z", -180.0)
        self.initial_pose_transition_time = max(float(get_cfg("initial_pose_transition_time", 3.0)), 0.1)
        self.nominal_pose_transition_time = max(float(get_cfg("nominal_pose_transition_time", 6.0)), 0.1)
        self.transition_rate_hz = max(float(get_cfg("transition_rate_hz", 50.0)), 1.0)

        self.l_coxa = get_cfg("link_coxa", 44.75)
        self.l_femur = get_cfg("link_femur", 74.0)
        self.l_tibia = get_cfg("link_tibia", 150.0)

        # 提取被动关节和基础配置参数
        self.l_a3 = get_cfg("link_a3", 41.5)
        self.l_d6 = get_cfg("link_d6", -13.5)
        self.l_d7 = get_cfg("link_d7", -106.7)
        self.base_r = get_cfg("base_radius", 203.06)

        legacy_joint23_ratio = get_cfg("gear_ratio_joint23", 4.421)
        self.gear_ratio_joint2 = get_cfg("gear_ratio_joint2", legacy_joint23_ratio)
        self.gear_ratio_joint3 = get_cfg("gear_ratio_joint3", legacy_joint23_ratio)

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
                "1": 1, "2": 1, "3": -1,
                "4": -1, "5": 1, "6": 1,
                "7": -1, "8": -1, "11": -1,
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

        self.left_board_motor_ids = set([int(v) for v in rospy.get_param("/left_board/motor_ids", [11, 1, 2, 14, 7, 8])])
        self.right_board_motor_ids = set([int(v) for v in rospy.get_param("/right_board/motor_ids", [12, 3, 4, 13, 5, 6])])
        self.left_board_multi_turn_ids = set([int(v) for v in rospy.get_param("/left_board/multi_turn_ids", [1, 2, 7, 8])])
        self.right_board_multi_turn_ids = set([int(v) for v in rospy.get_param("/right_board/multi_turn_ids", [3, 4, 5, 6])])
        derived_position_mode_ids = sorted(
            list(
                (self.left_board_motor_ids | self.right_board_motor_ids)
                - self.left_board_multi_turn_ids
                - self.right_board_multi_turn_ids
            )
        )
        self.position_mode_ids = set([int(v) for v in get_cfg("position_mode_ids", derived_position_mode_ids)])
        self.direct_drive_ids = set([int(v) for v in get_cfg("single_turn_ids", [11, 12, 13, 14])])
        self.startup_pose_mode = self._normalize_startup_pose_mode(get_cfg("startup_pose_mode", "configured_home"))
        
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
        self.calibration_complete = False
        self.start_nominal_requested = False
        rospy.loginfo("quadruped_core started. Preparing initialization with startup_pose_mode=%s", self.startup_pose_mode)

    @staticmethod
    def _normalize_startup_pose_mode(mode):
        mode_aliases = {
            "configured_home": "configured_home",
            "configured": "configured_home",
            "home": "configured_home",
            "capture_current_as_home": "capture_current_as_home",
            "capture_current": "capture_current_as_home",
            "current_as_home": "capture_current_as_home",
            "current": "capture_current_as_home",
        }
        normalized = str(mode).strip().lower()
        if normalized not in mode_aliases:
            rospy.logwarn("Unknown startup_pose_mode '%s', fallback to configured_home", mode)
            return "configured_home"
        return mode_aliases[normalized]

    def _base_delta_to_leg_delta(self, dx_base, dy_base, leg):
        yaw = leg.hip_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx_leg = cos_yaw * dx_base + sin_yaw * dy_base
        dy_leg = -sin_yaw * dx_base + cos_yaw * dy_base
        return dx_leg, dy_leg

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
        
        # 按照新版MDH表: theta_3 = q_3, theta_2 = q_2 - 90
        q3 = theta3
        q2 = math.atan2(p_z, R_prime) - math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3)) + math.radians(90.0)

        q1_deg = self._clamp(math.degrees(q1), self.joint_limit_deg["j1"][0], self.joint_limit_deg["j1"][1])
        q2_deg = self._clamp(math.degrees(q2), self.joint_limit_deg["j2"][0], self.joint_limit_deg["j2"][1])
        q3_deg = self._clamp(math.degrees(q3), self.joint_limit_deg["j3"][0], self.joint_limit_deg["j3"][1])

        return (q1_deg, q2_deg, q3_deg)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _smoothstep5(u):
        u = min(max(u, 0.0), 1.0)
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

    def _joint_gear_ratio(self, joint_index):
        if joint_index == 1:
            return self.gear_ratio_joint2
        if joint_index == 2:
            return self.gear_ratio_joint3
        return 1.0

    def _is_position_mode(self, motor_id):
        return int(motor_id) in self.position_mode_ids

    def _normalize_target_for_mode(self, motor_id, target_ticks):
        if self._is_position_mode(motor_id):
            normalized = int(round(float(target_ticks))) % 4096
            return self._clamp(normalized, 0, 4095)
        return self._clamp(int(round(float(target_ticks))), -1048575, 1048575)

    def _output_deg_to_ticks(self, motor_id, output_deg, joint_index):
        unit = 4096.0 / 360.0
        key = str(int(motor_id))
        home = float(self.motor_home.get(key, 0.0))
        direction = float(self.motor_dir.get(key, 1.0))
        ratio = self._joint_gear_ratio(joint_index)

        if int(motor_id) in self.direct_drive_ids:
            ratio = 1.0

        if self._is_position_mode(motor_id):
            home = home % 4096.0

        ticks = home + direction * (output_deg * ratio) * unit
        return int(self._normalize_target_for_mode(motor_id, ticks))

    def compute_and_output_leg_ik(self, dx_base, dy_base, dz_delta_base, leg):
        """进行IK解算并可将数值转化为发送端所需的控制帧(或立即下发)"""
        dx_leg, dy_leg = self._base_delta_to_leg_delta(dx_base, dy_base, leg)
        tx = self.nominal_x + dx_leg
        ty = self.nominal_y + dy_leg
        tz_leg = self.nominal_z + dz_delta_base
        q1, q2, q3 = self._ik_transform_matrix_solve(tx, ty, tz_leg)

        q1_cmd = q1 + float(self.joint_zero_deg["j1"])
        q2_cmd = q2 + float(self.joint_zero_deg["j2"])
        q3_cmd = q3 + float(self.joint_zero_deg["j3"])
        
        cmd = [q1_cmd, q2_cmd, q3_cmd]
        tick_results = {}
        for i in range(3):
            mid = leg.motor_ids[i]
            ticks = self._output_deg_to_ticks(mid, cmd[i], i)
            tick_results[mid] = ticks
            
            # 如果核心已就位准备接收高层命令，则立即发布给底层
            if self.ready_for_commands:
                msg = SetPosition()
                msg.id = int(mid)
                msg.position = int(ticks)
                self.pub.publish(msg)
                
        return tick_results

    @staticmethod
    def _normalize_present_position(raw_pos):
        if raw_pos > 2147483647:
            raw_pos -= 4294967296
        return int(raw_pos)

    def _read_current_positions(self):
        rospy.loginfo("Waiting for get_position services to become available...")
        rospy.wait_for_service('/left_board/get_position')
        rospy.wait_for_service('/right_board/get_position')

        get_pos_left = rospy.ServiceProxy('/left_board/get_position', GetPosition)
        get_pos_right = rospy.ServiceProxy('/right_board/get_position', GetPosition)

        rospy.loginfo("Detecting current physical positions...")
        start_positions = {}

        for leg in self.legs:
            for mid in leg.motor_ids:
                try:
                    if mid in self.left_board_motor_ids:
                        res = get_pos_left(mid)
                    elif mid in self.right_board_motor_ids:
                        res = get_pos_right(mid)
                    else:
                        raise rospy.ServiceException("motor id %d is not assigned to left_board or right_board" % mid)

                    start_positions[mid] = self._normalize_present_position(res.position)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed for ID %d: %s" % (mid, e))
                    start_positions[mid] = int(self.motor_home.get(str(mid), 2048))

        return start_positions

    def _publish_target_positions(self, target_positions):
        for leg in self.legs:
            for mid in leg.motor_ids:
                target = self._normalize_target_for_mode(mid, target_positions[mid])

                msg = SetPosition()
                msg.id = int(mid)
                msg.position = target
                self.pub.publish(msg)

    def _build_nominal_stance_targets(self):
        target_positions = {}
        for leg in self.legs:
            target_positions.update(self.compute_and_output_leg_ik(0.0, 0.0, 0.0, leg))
        return target_positions

    def _interpolate_positions(self, start_positions, target_positions, description, duration):
        rospy.loginfo("Starting smooth interpolation to %s...", description)
        hz = self.transition_rate_hz
        steps = max(int(duration * hz), 1)
        rate = rospy.Rate(hz)

        for step in range(1, steps + 1):
            blended_targets = {}
            alpha = self._smoothstep5(step / float(steps))
            for leg in self.legs:
                for mid in leg.motor_ids:
                    start_pos = float(start_positions[mid])
                    target_pos = float(target_positions[mid])
                    blended_targets[mid] = int(start_pos + (target_pos - start_pos) * alpha)
            self._publish_target_positions(blended_targets)
            rate.sleep()

    def _capture_current_pose_as_home(self, start_positions):
        captured_home = {}
        for leg in self.legs:
            for mid in leg.motor_ids:
                captured_home[str(mid)] = int(start_positions[mid])

        self.motor_home = captured_home
        rospy.set_param("~motor_home_ticks", captured_home)
        rospy.set_param("/gait_controller/motor_home_ticks", captured_home)

        rospy.loginfo("Captured current motor positions as runtime motor_home_ticks.")
        rospy.loginfo("Persist the following YAML if you want to keep this calibration:")
        rospy.loginfo("motor_home_ticks:")
        for motor_id in sorted(captured_home.keys(), key=lambda value: int(value)):
            rospy.loginfo("  \"%s\": %d", motor_id, captured_home[motor_id])

    def _publish_core_ready(self):
        self.ready_for_commands = True
        self.ready_pub.publish(Bool(data=True))

    def _publish_calibration_ready(self):
        self.calibration_complete = True
        self.calibration_ready_pub.publish(Bool(data=True))

    def start_nominal_pose_callback(self, msg):
        if not msg.data:
            return
        self.start_nominal_requested = True
        if self.calibration_complete and not self.ready_for_commands:
            rospy.loginfo("Received start_nominal_pose trigger. Proceeding to nominal stance phase.")

    def _build_initial_pose_targets(self):
        # 使用基于MDH正向运动学 q=[0,0,0] 作为基准标定初始姿态
        # 或者在 capture_current_as_home 模式下，保持在原地作为初始位姿
        target_positions = {}
        for leg in self.legs:
            # 此时 q1=0, q2=0, q3=0
            cmd = [
                0.0 + float(self.joint_zero_deg["j1"]),
                0.0 + float(self.joint_zero_deg["j2"]),
                0.0 + float(self.joint_zero_deg["j3"])
            ]
            for i in range(3):
                mid = leg.motor_ids[i]
                
                # 若启动模式为捕捉当前，并且这是初始化上电阶段，将直接获取已更新为当前刻度的 motor_home
                # 此时机器人不会做任何绝对位置的移动(在原地硬化)
                if self.startup_pose_mode == "capture_current_as_home":
                    target_positions[mid] = self._normalize_target_for_mode(mid, self.motor_home.get(str(mid), 2048))
                else:
                    # 按照 MDH q=0 标准零位移动
                    target_positions[mid] = self._output_deg_to_ticks(mid, cmd[i], i)
                    
        return target_positions

    def _run_initial_pose_calibration(self):
        start_positions = self._read_current_positions()

        if self.startup_pose_mode == "capture_current_as_home":
            self._capture_current_pose_as_home(start_positions)
            rospy.loginfo("Current pose captured as runtime motor_home_ticks.")

        target_positions = self._build_initial_pose_targets()
        for leg in self.legs:
            for mid in leg.motor_ids:
                rospy.loginfo(
                    "Joint %d: Current Position %d -> Initial Pose %d",
                    mid,
                    start_positions[mid],
                    target_positions[mid],
                )

        self._interpolate_positions(
            start_positions,
            target_positions,
            "initial pose",
            self.initial_pose_transition_time,
        )
        rospy.loginfo("Initial pose calibration completed. Waiting for start_nominal_pose trigger.")
        self._publish_calibration_ready()

    def _wait_for_nominal_start(self):
        if self.start_nominal_requested:
            return

        rospy.loginfo("Waiting for start_nominal_pose trigger before moving to nominal stance...")
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.start_nominal_requested:
            rate.sleep()

    def _move_to_nominal_stance(self):
        start_positions = self._read_current_positions()

        target_positions = self._build_nominal_stance_targets()
        for leg in self.legs:
            for mid in leg.motor_ids:
                rospy.loginfo(
                    "Joint %d: Current Position %d -> Nominal Stance %d",
                    mid,
                    start_positions[mid],
                    target_positions[mid],
                )

        self._interpolate_positions(
            start_positions,
            target_positions,
            "nominal stance",
            self.nominal_pose_transition_time,
        )
        rospy.loginfo("Nominal stance reached. Core is ready to receive foot trajectories.")
        self._publish_core_ready()

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
        # 1. 先执行初始位姿标定，只平滑运动到初始位
        self._run_initial_pose_calibration()

        # 2. 等待显式触发后，再从初始位平滑运动到名义位
        self._wait_for_nominal_start()
        if rospy.is_shutdown():
            return

        self._move_to_nominal_stance()
        
        # 3. 挂载订阅话题，准备接收高层四足运动指令
        rospy.Subscriber("cmd_foot_pose", Float64MultiArray, self.gait_command_callback)
        
        # 4. 阻塞维护
        rospy.spin()

if __name__ == "__main__":
    try:
        node = QuadrupedCoreNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass