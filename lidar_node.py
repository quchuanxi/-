#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：雷达数据预处理节点，截取前方±30°测距值，滑动窗口滤波输出平均距离
订阅话题：/scan(sensor_msgs/LaserScan)
发布话题：/lidar_front_dist(std_msgs/Float32)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class LidarFrontDistNode(Node):
    def __init__(self):
        super().__init__("lidar_front_dist_node")
        # 创建前方距离发布器
        self.pub_dist = self.create_publisher(Float32, "/lidar_front_dist", 10)
        # 订阅雷达原始点云话题
        self.sub_scan = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        # 默认无效距离值
        self.default_dist = 9999.0
        # 滑动滤波窗口缓存
        self.filter_window = []
        self.window_size = 5  # 缓存5帧距离数据做中值滤波
        self.min_valid_mm = 200.0   # 最小有效测距，低于视为噪点丢弃
        self.max_valid_mm = 6000.0  # 最大有效测距
        self.get_logger().info("Lidar processing node started, listening to /scan, publishing to /lidar_front_dist")

    def scan_callback(self, scan):
        """雷达原始点云回调：筛选前方角度、过滤噪点、滑动窗口平滑距离"""
        valid_mm = []
        meter2mm = 1000.0  # 单位转换：米转毫米
        angle_inc = scan.angle_increment

        # 遍历全部雷达测距点
        for idx, range_m in enumerate(scan.ranges):
            # 计算当前点角度，转为0~360度
            rad = scan.angle_min + idx * angle_inc
            deg = math.degrees(rad)
            if deg < 0:
                deg += 360.0
            # 仅保留前方±30°范围数据（330°~360°、0°~30°）
            if 330 <= deg <= 360 or 0 <= deg <= 30:
                dist_mm = range_m * meter2mm
                # 过滤超近、超远无效噪点
                if self.min_valid_mm < dist_mm < self.max_valid_mm:
                    valid_mm.append(dist_mm)

        # 当前帧存在有效测距数据时做滤波
        if len(valid_mm) > 0:
            frame_avg = sum(valid_mm) / len(valid_mm)
            self.filter_window.append(frame_avg)
            # 窗口满5帧丢弃最早数据
            if len(self.filter_window) > self.window_size:
                self.filter_window.pop(0)
            # 取窗口中值，抗突发跳变噪点
            sorted_win = sorted(self.filter_window)
            self.default_dist = sorted_win[len(sorted_win) // 2]
            self.get_logger().info(
                f"前方距离: {self.default_dist:.0f}mm (原始帧均值: {frame_avg:.0f}mm, 有效点: {len(valid_mm)})",
                throttle_duration_sec=1.0
            )

        # 发布平滑后的前方平均距离
        msg = Float32()
        msg.data = self.default_dist
        self.pub_dist.publish(msg)

def main():
    rclpy.init()
    node = LidarFrontDistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nLidar processing node stopped")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()