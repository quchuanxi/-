#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardPubNode(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        # 话题同步改为 /car_cmd 和X5匹配
        self.pub = self.create_publisher(String, "/car_cmd", 10)
        self.current_speed = 15
        self.current_task = "Stop"
        self.cmd_task_map = {
            "w": "Move Forward",
            "s": "Move Backward",
            "a": "Turn Left In Place",
            "d": "Turn Right In Place",
            "x": "Stop Car",
            "i": "Speed Up +5",
            "o": "Speed Down -5"
        }
        print("==== Real-Time Keyboard Controller ====")
        print("Key: w/s/a/d/x/i/o | Press q to quit")
        print(f"Initial State | Task: {self.current_task} | Speed: {self.current_speed}\n")

    def send_cmd(self, char):
        msg = String()
        msg.data = char
        self.pub.publish(msg)
        # 本地显示同步上下限逻辑，和X5保持一致
        if char == "i":
            self.current_speed = min(self.current_speed + 5, 100)
        elif char == "o":
            self.current_speed = max(self.current_speed - 5, 0)
        if char in self.cmd_task_map:
            self.current_task = self.cmd_task_map[char]
        print(f"[Task]: {self.current_task:20} | [Speed]: {self.current_speed}")

def get_single_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    pub_node = KeyboardPubNode()
    try:
        while True:
            key = get_single_key()
            if key == "q":
                print("\nExit Controller")
                break
            if key in ["w", "a", "s", "d", "x", "i", "o"]:
                pub_node.send_cmd(key)
    finally:
        pub_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()