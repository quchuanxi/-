#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：底层电机驱动ROS2节点，接收遥控/跟随指令，控制四路电机PWM与方向引脚
订阅话题：/car_cmd (std_msgs/String)
发布话题：无
"""

import sys
import signal
import Hobot.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---------------------- 硬件引脚定义 ----------------------
# 四路电机PWM调速引脚
PWM1 = 29
PWM2 = 31
PWM3 = 32
PWM4 = 33

# 电机1方向控制引脚
M1_IN1 = 11
M1_IN2 = 13
# 电机2方向控制引脚
M2_IN1 = 16
M2_IN2 = 15
# 电机3方向控制引脚
M3_IN1 = 26
M3_IN2 = 28
# 电机4方向控制引脚
M4_IN1 = 24
M4_IN2 = 23

# 关闭GPIO多余警告输出
GPIO.setwarnings(False)

# 全局电机速度变量
speed = 15
# 四路PWM对象全局变量
p1 = p2 = p3 = p4 = None
# 速度最大、最小限制
SPEED_MAX = 100
SPEED_MIN = 0

# ---------------------- GPIO底层输出控制函数 ----------------------
def ctrl_GPIO(m1_in1, m1_in2,
              m2_in1, m2_in2,
              m3_in1, m3_in2,
              m4_in1, m4_in2):
    """统一设置四路电机方向电平"""
    GPIO.output(M1_IN1, m1_in1)
    GPIO.output(M1_IN2, m1_in2)
    GPIO.output(M2_IN1, m2_in1)
    GPIO.output(M2_IN2, m2_in2)
    GPIO.output(M3_IN1, m3_in1)
    GPIO.output(M3_IN2, m3_in2)
    GPIO.output(M4_IN1, m4_in1)
    GPIO.output(M4_IN2, m4_in2)

def refresh_pwm_speed():
    """刷新四路电机PWM占空比，同步更新行驶速度"""
    global speed
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(speed)
    p3.ChangeDutyCycle(speed)
    p4.ChangeDutyCycle(speed)

# ---------------------- 小车运动控制函数 ----------------------
def forward():
    """小车前进，设置四路电机正转电平并刷新速度"""
    global speed
    ctrl_GPIO(
        GPIO.LOW, GPIO.HIGH,
        GPIO.HIGH, GPIO.LOW,
        GPIO.HIGH, GPIO.LOW,
        GPIO.LOW, GPIO.HIGH
    )
    refresh_pwm_speed()

def back():
    """小车后退，设置四路电机反转电平并刷新速度"""
    global speed
    ctrl_GPIO(
        GPIO.HIGH, GPIO.LOW,
        GPIO.LOW, GPIO.HIGH,
        GPIO.LOW, GPIO.HIGH,
        GPIO.HIGH, GPIO.LOW
    )
    refresh_pwm_speed()

def right():
    """原地右转，全部电机同侧转向"""
    global speed
    ctrl_GPIO(
        GPIO.LOW, GPIO.HIGH,
        GPIO.LOW, GPIO.HIGH,
        GPIO.LOW, GPIO.HIGH,
        GPIO.LOW, GPIO.HIGH
    )
    refresh_pwm_speed()

def left():
    """原地左转，全部电机反向转向"""
    global speed
    ctrl_GPIO(
        GPIO.HIGH, GPIO.LOW,
        GPIO.HIGH, GPIO.LOW,
        GPIO.HIGH, GPIO.LOW,
        GPIO.HIGH, GPIO.LOW
    )
    refresh_pwm_speed()

def stop_car():
    """小车急停，全部方向引脚置低，PWM占空比清零"""
    ctrl_GPIO(
        GPIO.LOW, GPIO.LOW,
        GPIO.LOW, GPIO.LOW,
        GPIO.LOW, GPIO.LOW,
        GPIO.LOW, GPIO.LOW
    )
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    p3.ChangeDutyCycle(0)
    p4.ChangeDutyCycle(0)

# ---------------------- 速度增减控制函数 ----------------------
def SPEED_HIGH():
    """加速，单次+5，上限100"""
    global speed
    speed = min(speed + 5, SPEED_MAX)
    refresh_pwm_speed()

def SPEED_LOW():
    """减速，单次-5，下限0"""
    global speed
    speed = max(speed - 5, SPEED_MIN)
    refresh_pwm_speed()

# ---------------------- ROS2电机驱动节点类 ----------------------
class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")
        # 订阅小车运动指令话题/car_cmd
        self.cmd_sub = self.create_subscription(
            String,
            "/car_cmd",
            self.keyboard_callback,
            qos_profile=10
        )

    def keyboard_callback(self, msg):
        """话题消息回调，解析指令并执行对应动作"""
        cmd = msg.data.strip().lower()
        if cmd == "w":
            forward()
        elif cmd == "s":
            back()
        elif cmd == "a":
            left()
        elif cmd == "d":
            right()
        elif cmd == "x":
            stop_car()
        elif cmd == "i":
            SPEED_HIGH()
        elif cmd == "o":
            SPEED_LOW()

# ---------------------- Ctrl+C安全退出处理函数 ----------------------
def signal_handler(signal_num, frame):
    """捕获终止信号，先停车、关闭PWM、释放GPIO资源"""
    stop_car()
    global p1, p2, p3, p4
    p1.stop()
    p2.stop()
    p3.stop()
    p4.stop()
    # 底层sysfs强制关闭PWM通道，防止硬件持续输出
    import os
    for path in [
        "/sys/class/pwm/pwmchip0/pwm0/enable",
        "/sys/class/pwm/pwmchip0/pwm1/enable",
        "/sys/class/pwm/pwmchip2/pwm0/enable",
        "/sys/class/pwm/pwmchip2/pwm1/enable",
    ]:
        try:
            with open(path, "w") as f:
                f.write("0")
        except Exception:
            pass
    GPIO.cleanup()
    sys.exit(0)

# ---------------------- 硬件初始化 & ROS主函数 ----------------------
def main():
    global speed, p1, p2, p3, p4
    # 绑定Ctrl+C终止信号
    signal.signal(signal.SIGINT, signal_handler)

    # 设置GPIO编号模式为BOARD物理引脚编号
    GPIO.setmode(GPIO.BOARD)

    # 创建四路PWM对象，频率48000Hz，并初始化方向引脚为输出模式
    p1 = GPIO.PWM(PWM1, 48000)
    GPIO.setup(M1_IN1, GPIO.OUT)
    GPIO.setup(M1_IN2, GPIO.OUT)

    p2 = GPIO.PWM(PWM2, 48000)
    GPIO.setup(M2_IN1, GPIO.OUT)
    GPIO.setup(M2_IN2, GPIO.OUT)

    p3 = GPIO.PWM(PWM3, 48000)
    GPIO.setup(M3_IN1, GPIO.OUT)
    GPIO.setup(M3_IN2, GPIO.OUT)

    p4 = GPIO.PWM(PWM4, 48000)
    GPIO.setup(M4_IN1, GPIO.OUT)
    GPIO.setup(M4_IN2, GPIO.OUT)

    # 启动PWM，初始速度15
    init_val = 15
    p1.start(init_val)
    p2.start(init_val)
    p3.start(init_val)
    p4.start(init_val)
    # 底层文件强制开启PWM输出通道，适配RDK系统内核问题
    import os
    pwm_map = {
        PWM1: "/sys/class/pwm/pwmchip0/pwm0/enable",
        PWM2: "/sys/class/pwm/pwmchip0/pwm1/enable",
        PWM3: "/sys/class/pwm/pwmchip2/pwm0/enable",
        PWM4: "/sys/class/pwm/pwmchip2/pwm1/enable",
    }
    for pin, path in pwm_map.items():
        try:
            with open(path, "w") as f:
                f.write("1")
        except Exception:
            pass
    # 开机默认停止小车，防止上电误动
    stop_car()

    # 初始化ROS2并循环运行节点
    rclpy.init(args=sys.argv)
    motor_node = MotorDriverNode()
    try:
        rclpy.spin(motor_node)
    except Exception:
        pass
    finally:
        # 程序退出资源释放流程
        motor_node.destroy_node()
        rclpy.shutdown()
        stop_car()
        p1.stop()
        p2.stop()
        p3.stop()
        p4.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()