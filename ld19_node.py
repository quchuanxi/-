#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：ROS2启动文件，加载LD19激光雷达驱动、雷达坐标TF转换节点
订阅话题：无
发布话题：/scan(LaserScan)
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LD19雷达驱动节点，读取TTL转USB串口原始雷达数据，发布scan话题
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},       # 雷达型号
            {'topic_name': 'scan'},                 # 雷达原始点云话题名
            {'frame_id': 'base_laser'},             # 雷达坐标系ID
            {'port_name': '/dev/ttyUSB0'},          # TTL转USB串口设备号
            {'port_baudrate': 230400},              # 串口波特率
            {'laser_scan_dir': True},               # 雷达扫描方向
            {'enable_angle_crop_func': False},      # 关闭角度裁剪
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # 静态TF坐标变换：底盘base_link到雷达base_laser坐标转换
    base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld19',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
    )

    # 创建启动描述对象，添加两个节点
    ld = LaunchDescription()
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)

    return ld