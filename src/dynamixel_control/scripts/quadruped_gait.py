#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
四足机器人步态生成器 (Quadruped Gait Generator)
负责：根据设定的运行频率和步态模式（如Crawl/Trot），生成足端在Base平面内的相对轨迹(dx, dy, dz_delta)，并下发给Core节点解析。
"""

import math
import rospy
from std_msgs.msg import Bool, Float64MultiArray


class QuadrupedGaitGenerator(object):
    def __init__(self):
        rospy.init_node("quadruped_gait_generator", anonymous=False)
        self.pub = rospy.Publisher("cmd_foot_pose", Float64MultiArray, queue_size=10)
        rospy.Subscriber("start_gait_motion", Bool, self.start_gait_callback)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/gait_controller/" + name, default))

        # 步态参数配置
        self.rate_hz = get_cfg("rate_hz", 100.0)
        self.gait = get_cfg("gait", "crawl")  # 默认爬行步态
        self.freq_hz = get_cfg("gait_frequency", 0.35)

        self.step_x = get_cfg("step_x", 35.0)
        self.step_y = get_cfg("step_y", 0.0)
        self.swing_height = get_cfg("swing_height", 25.0)
        self.gait_ramp_time = max(float(get_cfg("gait_ramp_time", 1.5)), 0.0)
        self.output_smoothing_alpha = min(max(float(get_cfg("output_smoothing_alpha", 0.35)), 0.0), 1.0)

        # 定义每条腿的时序相差 (对应序列顺序 [lf, rf, lr, rr])
        if self.gait == "trot":
            self.phases = {"lf": 0.0, "rf": 0.5, "lr": 0.5, "rr": 0.0}
        else:
            # Crawl 步态：同一时刻仅1条腿摆动
            self.phases = {"lf": 0.00, "rf": 0.25, "lr": 0.75, "rr": 0.50}

        # 限定统一生成序列顺序
        self.leg_order = ["lf", "rf", "lr", "rr"]
        self.filtered_deltas = dict((leg_name, [0.0, 0.0, 0.0]) for leg_name in self.leg_order)
        self.gait_start_requested = False
        
        rospy.loginfo(
            "Gait Generator started: gait=%s, freq=%.3f Hz, ramp_time=%.2f s, smoothing_alpha=%.2f",
            self.gait,
            self.freq_hz,
            self.gait_ramp_time,
            self.output_smoothing_alpha,
        )

    @staticmethod
    def _smoothstep5(u):
        u = min(max(u, 0.0), 1.0)
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

    @staticmethod
    def _raised_cosine(u):
        u = min(max(u, 0.0), 1.0)
        return 0.5 - 0.5 * math.cos(2.0 * math.pi * u)

    def _ramp_scale(self, elapsed_time):
        if self.gait_ramp_time <= 1e-6:
            return 1.0
        return self._smoothstep5(elapsed_time / self.gait_ramp_time)

    def _apply_output_smoothing(self, leg_name, dx, dy, dz):
        alpha = self.output_smoothing_alpha
        previous = self.filtered_deltas[leg_name]
        if alpha >= 1.0:
            filtered = [dx, dy, dz]
        else:
            filtered = [
                previous[0] + alpha * (dx - previous[0]),
                previous[1] + alpha * (dy - previous[1]),
                previous[2] + alpha * (dz - previous[2]),
            ]
        self.filtered_deltas[leg_name] = filtered
        return filtered[0], filtered[1], filtered[2]

    def _swing_ratio(self):
        if self.gait == "trot":
            return rospy.get_param("~trot_swing_ratio", rospy.get_param("/gait_controller/trot_swing_ratio", 0.5))
        return rospy.get_param("~crawl_swing_ratio", rospy.get_param("/gait_controller/crawl_swing_ratio", 0.25))

    def start_gait_callback(self, msg):
        if not msg.data:
            return
        self.gait_start_requested = True
        rospy.loginfo("Received start_gait_motion trigger. Starting gait output.")

    def generate_foot_delta(self, phase, swing_ratio, amplitude_scale):
        """生成摆放或支撑相的连续步态基线偏差增量"""
        step_x = self.step_x * amplitude_scale
        step_y = self.step_y * amplitude_scale
        swing_height = self.swing_height * amplitude_scale

        if phase < swing_ratio:
            u = phase / max(swing_ratio, 1e-6)
            s = self._smoothstep5(u)
            x = -step_x * 0.5 + step_x * s
            y = -step_y * 0.5 + step_y * s
            z = swing_height * self._raised_cosine(u)
        else:
            u = (phase - swing_ratio) / max(1.0 - swing_ratio, 1e-6)
            s = self._smoothstep5(u)
            x = step_x * 0.5 - step_x * s
            y = step_y * 0.5 - step_y * s
            z = 0.0
        return x, y, z

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Waiting for Quadruped Core ready signal...")
        while not rospy.is_shutdown():
            ready_msg = rospy.wait_for_message("core_ready", Bool)
            if ready_msg.data:
                break

        rospy.loginfo("Nominal pose reached. Waiting for start_gait_motion trigger...")
        wait_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.gait_start_requested:
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        t0 = rospy.Time.now().to_sec()
        rospy.loginfo("Starting %s gait generation!", self.gait)

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            ramp_scale = self._ramp_scale(t)
            msg = Float64MultiArray()
            # 数组存储空间依次写入 [lf_dx, lf_dy, lf_dz, rf_dx, rf_dy, rf_dz... ]
            
            for leg_name in self.leg_order:
                # 计算当前腿的周期相位
                current_phase = (t * self.freq_hz + self.phases[leg_name]) % 1.0
                swing_ratio = self._swing_ratio()
                
                # 取得当前时间增量
                dx, dy, dz = self.generate_foot_delta(current_phase, swing_ratio, ramp_scale)
                dx, dy, dz = self._apply_output_smoothing(leg_name, dx, dy, dz)
                
                # 附加到数组列表
                msg.data.extend([dx, dy, dz])

            self.pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        generator = QuadrupedGaitGenerator()
        generator.spin()
    except rospy.ROSInterruptException:
        pass