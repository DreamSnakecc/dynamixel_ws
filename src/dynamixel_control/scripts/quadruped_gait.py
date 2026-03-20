#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
四足机器人步态生成器 (Quadruped Gait Generator)
负责：根据设定的运行频率和步态模式（如Crawl/Trot），生成足端在Base平面内的相对轨迹(dx, dy, dz_delta)，并下发给Core节点解析。
"""

import math
import rospy
from std_msgs.msg import Float64MultiArray

class QuadrupedGaitGenerator(object):
    def __init__(self):
        rospy.init_node("quadruped_gait_generator", anonymous=False)
        self.pub = rospy.Publisher("cmd_foot_pose", Float64MultiArray, queue_size=10)

        # 步态参数配置
        self.rate_hz = rospy.get_param("~rate_hz", 100.0)
        self.gait = rospy.get_param("~gait", "crawl")  # 默认爬行步态
        self.freq_hz = rospy.get_param("~gait_frequency", 0.35)

        self.step_x = rospy.get_param("~step_x", 35.0)
        self.step_y = rospy.get_param("~step_y", 0.0)
        self.swing_height = rospy.get_param("~swing_height", 25.0)

        # 定义每条腿的时序相差 (对应序列顺序 [lf, rf, lr, rr])
        if self.gait == "trot":
            self.phases = {"lf": 0.0, "rf": 0.5, "lr": 0.5, "rr": 0.0}
        else:
            # Crawl 步态：同一时刻仅1条腿摆动
            self.phases = {"lf": 0.00, "rf": 0.25, "lr": 0.75, "rr": 0.50}

        # 限定统一生成序列顺序
        self.leg_order = ["lf", "rf", "lr", "rr"]
        
        rospy.loginfo("Gait Generator started: gait=%s, freq=%.3f Hz", self.gait, self.freq_hz)

    def _swing_ratio(self):
        if self.gait == "trot":
            return rospy.get_param("~trot_swing_ratio", 0.5)
        return rospy.get_param("~crawl_swing_ratio", 0.25)

    def generate_foot_delta(self, phase, swing_ratio):
        """生成摆放或支撑相的连续步态基线偏差增量"""
        if phase < swing_ratio:
            u = phase / max(swing_ratio, 1e-6)
            x = -self.step_x * 0.5 + self.step_x * u
            y = -self.step_y * 0.5 + self.step_y * u
            z = self.swing_height * math.sin(math.pi * u)
        else:
            u = (phase - swing_ratio) / max(1.0 - swing_ratio, 1e-6)
            x = self.step_x * 0.5 - self.step_x * u
            y = self.step_y * 0.5 - self.step_y * u
            z = 0.0
        return x, y, z

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        
        # 等待 Core 节点完成其 3 秒的归零插值
        # 最好由用户参数触发，此处留出富裕缓冲（也可以通过ROS Service同步，为简化就加个纯延时提示）
        rospy.loginfo("Waiting 4 seconds for Quadruped Core to finish homing...")
        rospy.sleep(4.0)

        t0 = rospy.Time.now().to_sec()
        rospy.loginfo("Starting %s gait generation!", self.gait)

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            msg = Float64MultiArray()
            # 数组存储空间依次写入 [lf_dx, lf_dy, lf_dz, rf_dx, rf_dy, rf_dz... ]
            
            for leg_name in self.leg_order:
                # 计算当前腿的周期相位
                current_phase = (t * self.freq_hz + self.phases[leg_name]) % 1.0
                swing_ratio = self._swing_ratio()
                
                # 取得当前时间增量
                dx, dy, dz = self.generate_foot_delta(current_phase, swing_ratio)
                
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