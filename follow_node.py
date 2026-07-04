#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：人体跟随决策节点，融合视觉人体坐标+雷达距离，采用P比例控制生成运动指令
订阅话题：/person_detection(String)、/lidar_front_dist(Float32)
发布话题：/car_cmd(String)
"""

import json
import signal
import sys
import os
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Float32
import Hobot.GPIO as GPIO

# ============ 运动状态枚举定义 ============
STATE_STOP = "stop"        # 停车状态
STATE_FORWARD = "forward"  # 直行状态
STATE_LEFT = "left"        # 左转状态
STATE_RIGHT = "right"      # 右转状态

class FollowNode(Node):
    """视觉P比例跟随控制节点，增加坐标滑动滤波、直行死区抑制车身抖动"""
    def __init__(self):
        super().__init__("follow_node")

        # 安全停车距离参数，单位毫米
        self.declare_parameter("stop_dist_mm", 500.0)
        self.stop_dist_mm = self.get_parameter("stop_dist_mm").value

        # 直行死区：画面0.25~0.75区间判定目标居中，不转向
        self.left_zone_max = 0.25
        self.right_zone_min = 0.75

        # P比例控制参数，Kp越大转向修正力度越强
        self.Kp = 1.2
        # 大幅偏移阈值，超过则持续转向修正
        self.big_offset_thr = 0.10

        # 坐标滑动平均滤波缓存帧数
        self.filter_frame_num = 3
        self.cx_buffer = []

        # 检测状态全局变量
        self.has_person = False
        self.person_center_x = 0.5
        self.front_dist_mm = 9999.0
        self.run_state = STATE_STOP
        self.person_lost_time = None
        # 目标丢失超时3秒自动停车
        self.person_timeout = 3.0

        # ROS话题订阅与发布创建
        self.sub_det = self.create_subscription(
            String, "/person_detection", self.detection_callback, 10)
        self.sub_dist = self.create_subscription(
            Float32, "/lidar_front_dist", self.dist_callback, 10)
        self.cmd_pub = self.create_publisher(String, "/car_cmd", 10)

        # 50ms周期定时执行控制逻辑
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("===== 视觉P比例跟随启动（无编码器） =====")
        self.get_logger().info(f"停车距离:{self.stop_dist_mm}mm | 直行死区:{self.left_zone_max}~{self.right_zone_min}")
        self.get_logger().info(f"开启{self.filter_frame_num}帧坐标平滑，比例系数Kp={self.Kp}")
        self.get_logger().info("等待 /person_detection /lidar_front_dist 数据...")

    def detection_callback(self, msg):
        """接收人体检测结果，滑动平均平滑目标中心坐标，消除画面抖动"""
        try:
            data = json.loads(msg.data)
            self.has_person = data.get("has_person", False)
            boxes = data.get("boxes", [])
            raw_cx = 0.5

            # 检测到人体时计算目标框中心归一化横坐标
            if self.has_person and len(boxes) > 0:
                x1, y1, x2, y2 = boxes[0]
                img_w = 640.0
                cx_pixel = (x1 + x2) / 2.0
                raw_cx = cx_pixel / img_w

            # 滑动窗口滤波，超出帧数丢弃最早数据
            self.cx_buffer.append(raw_cx)
            if len(self.cx_buffer) > self.filter_frame_num:
                self.cx_buffer.pop(0)
            # 取均值作为平滑后的目标中心
            self.person_center_x = sum(self.cx_buffer) / len(self.cx_buffer)

        except json.JSONDecodeError:
            pass

    def dist_callback(self, msg):
        """接收雷达前方平均距离，更新全局距离变量"""
        self.front_dist_mm = msg.data

    def timer_callback(self):
        """定时主控制逻辑，根据人体位置、距离生成运动指令"""
        cmd = String()
        # 无传感器数据时直接停车
        if self.front_dist_mm == 9999.0 and not self.has_person:
            self.get_logger().info("等待话题数据...", throttle_duration_sec=3.0)
            cmd.data = "x"
            self.cmd_pub.publish(cmd)
            return

        # 场景1：画面未检测到人体
        if not self.has_person:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.person_lost_time is None:
                self.person_lost_time = now
                self.get_logger().info(f"无人，等待{self.person_timeout}秒后停车")
            elapsed = now - self.person_lost_time
            # 丢失超时则下发停车指令
            if elapsed >= self.person_timeout:
                cmd.data = "x"
                if self.run_state != STATE_STOP:
                    self.get_logger().info(f"无人超时{elapsed:.1f}s，停车")
                    self.run_state = STATE_STOP
            else:
                # 未超时保持原有运动状态
                cmd.data = self._last_cmd if hasattr(self, '_last_cmd') else "x"
                self.get_logger().info(f"无人{elapsed:.1f}s，保持原有运动", throttle_duration_sec=1.0)

        # 场景2：画面检测到人体
        else:
            self.person_lost_time = None
            # 距离小于安全阈值，直接停车避障
            if self.front_dist_mm <= self.stop_dist_mm:
                cmd.data = "x"
                if self.run_state != STATE_STOP:
                    self.get_logger().info(f"距离{self.front_dist_mm:.0f}mm，近距离停车")
                    self.run_state = STATE_STOP

            # 距离安全，执行P比例居中跟随逻辑
            else:
                cx = self.person_center_x
                err = cx - 0.5  # 坐标误差：负数偏左、正数偏右
                abs_err = abs(err)

                # 目标落在直行死区，保持前进
                if self.left_zone_max <= cx <= self.right_zone_min:
                    cmd.data = "w"
                    if self.run_state != STATE_FORWARD:
                        self.get_logger().info(f"居中，平滑cx={cx:.2f}，直行w")
                        self.run_state = STATE_FORWARD

                # 目标偏左，执行左转修正
                elif cx < self.left_zone_max:
                    if abs_err > self.big_offset_thr:
                        cmd.data = "a"
                        if self.run_state != STATE_LEFT:
                            self.get_logger().info(f"大幅左偏 err={err:.2f}，持续左转a")
                            self.run_state = STATE_LEFT
                    else:
                        cmd.data = "a"
                        self.run_state = STATE_LEFT

                # 目标偏右，执行右转修正
                elif cx > self.right_zone_min:
                    if abs_err > self.big_offset_thr:
                        cmd.data = "d"
                        if self.run_state != STATE_RIGHT:
                            self.get_logger().info(f"大幅右偏 err={err:.2f}，持续右转d")
                            self.run_state = STATE_RIGHT
                    else:
                        cmd.data = "d"
                        self.run_state = STATE_RIGHT

        # 保存上一条指令并发布
        self._last_cmd = cmd.data
        self.cmd_pub.publish(cmd)

def signal_handler(sig, frame):
    """捕获终止信号，抛出中断退出程序"""
    sig_name = signal.Signals(sig).name
    print(f"\n检测到{ sig_name }，程序退出")
    raise KeyboardInterrupt()

def main():
    # 底层急停GPIO引脚定义，与电机驱动节点一致
    PWM1 = 29; PWM2 = 31; PWM3 = 32; PWM4 = 33
    M1_IN1 = 11; M1_IN2 = 13; M2_IN1 = 16; M2_IN2 = 15
    M3_IN1 = 26; M3_IN2 = 28; M4_IN1 = 24; M4_IN2 = 23

    def _gpio_stop():
        """底层硬件强制停车，不依赖ROS通信，防止程序异常失控"""
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            all_motor_pins = [M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2]
            for pin in all_motor_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            # 关闭全部PWM硬件通道
            pwm_paths = [
                "/sys/class/pwm/pwmchip0/pwm0/enable",
                "/sys/class/pwm/pwmchip0/pwm1/enable",
                "/sys/class/pwm/pwmchip2/pwm0/enable",
                "/sys/class/pwm/pwmchip2/pwm1/enable",
            ]
            for path in pwm_paths:
                try:
                    with open(path, "w") as f:
                        f.write("0")
                except:
                    pass
            GPIO.cleanup()
        except Exception:
            pass

    # 绑定终止信号处理函数
    signal.signal(signal.SIGTERM, signal_handler)
    rclpy.init(args=sys.argv)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n手动终止程序，执行停车")
        try:
            stop_msg = String()
            stop_msg.data = "x"
            node.cmd_pub.publish(stop_msg)
            print("ROS下发停车指令x")
        except Exception as e:
            print(f"ROS下发停车失败: {e}")
        print("底层GPIO强制停机...")
        _gpio_stop()
        print("GPIO停车完成")
    finally:
        # 释放ROS资源
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("程序完全退出")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    main()