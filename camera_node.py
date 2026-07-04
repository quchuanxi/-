#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：USB摄像头采集图像，RDK X5 BPU加速YOLOv5人体检测，输出人体目标框坐标
订阅话题：无
发布话题：/person_detection(std_msgs/String)
"""
import sys
import cv2
import argparse
import numpy as np
import time
import signal
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ============ RDK BPU YOLO推理工具库导入 ============
import hbm_runtime
sys.path.append(os.path.abspath("pydev_demo"))
sys.path.append("/app/pydev_demo")
import utils.preprocess_utils as pre_utils
import utils.postprocess_utils as post_utils
import utils.common_utils as common
import utils.draw_utils as draw

# YOLO模型锚框、步长配置
STRIDES = np.array([8, 16, 32], dtype=np.int32)
ANCHORS = np.array([
    [10, 13], [16, 30], [33, 23],
    [30, 61], [62, 45], [59, 119],
    [116, 90], [156, 198], [373, 326]
], dtype=np.float32).reshape(3, 3, 2)

class YoloV5X:
    """YOLOv5检测推理封装类，调用RDK X5 BPU硬件加速"""
    def __init__(self, opt):
        # 加载量化推理模型
        self.model = hbm_runtime.HB_HBMRuntime(opt.model_path)
        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]
        self.output_quants = self.model.output_quants[self.model_name]
        self.input_H = self.input_shapes[self.input_names[0]][2]
        self.input_W = self.input_shapes[self.input_names[0]][3]
        self.score_thres = opt.score_thres
        self.nms_thres = opt.nms_thres
        self.resize_type = 1
        self.classes_num = 80

    def set_scheduling_params(self, priority=None, bpu_cores=None):
        """设置BPU运算优先级、使用核心数量"""
        kwargs = {}
        if priority is not None:
            kwargs["priority"] = {self.model_name: priority}
        if bpu_cores is not None:
            kwargs["bpu_cores"] = {self.model_name: bpu_cores}
        if kwargs:
            self.model.set_scheduling_params(**kwargs)

    def pre_process(self, img):
        """图像预处理：缩放、转NV12格式适配BPU输入"""
        resize_img = pre_utils.resized_image(img, self.input_W, self.input_H, self.resize_type)
        y, uv = pre_utils.bgr_to_nv12_planes(resize_img)
        nv12 = np.concatenate((y.reshape(-1), uv.reshape(-1)), axis=0)
        nv12 = nv12.reshape((1, self.input_H * 3 // 2, self.input_W, 1))
        return {self.model_name: {self.input_names[0]: nv12}}

    def forward(self, input_tensor):
        """BPU模型前向推理"""
        outputs = self.model.run(input_tensor)
        return outputs[self.model_name]

    def post_process(self, outputs, img_w, img_h):
        """推理后处理：反量化、解码、NMS、还原原图坐标"""
        fp32_outputs = post_utils.dequantize_outputs(outputs, self.output_quants)
        pred = post_utils.decode_outputs(self.output_names, fp32_outputs,
                                         STRIDES, ANCHORS, self.classes_num)
        xyxy_boxes, score, cls = post_utils.filter_predictions(pred, self.score_thres)
        keep = post_utils.NMS(xyxy_boxes, score, cls, self.nms_thres)
        xyxy = post_utils.scale_coords_back(xyxy_boxes[keep], img_w, img_h,
                                            self.input_W, self.input_H, self.resize_type)
        return xyxy, cls[keep], score[keep]

# ============ 工具函数 ============
def is_usb_camera(device):
    """检测指定视频设备是否为可用USB摄像头"""
    try:
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            return False
        cap.release()
        return True
    except Exception:
        return False

def find_first_usb_camera():
    """自动查找系统内USB摄像头设备节点/dev/videoX"""
    import subprocess
    try:
        result = subprocess.run(["v4l2-ctl", "--list-devices"],
                                capture_output=True, text=True, timeout=5)
        lines = result.stdout.strip().split("\n")
        for i, line in enumerate(lines):
            if "usb" in line.lower() or "camera" in line.lower() or "webcam" in line.lower():
                for j in range(i + 1, len(lines)):
                    dev = lines[j].strip()
                    if dev.startswith("/dev/video"):
                        return dev
                    if not dev.startswith("/dev/"):
                        break
    except Exception:
        pass
    # 兜底默认video0
    if os.path.exists("/dev/video0"):
        return "/dev/video0"
    return None

def check_person_detected(cls, coco_names):
    """判断检测结果中是否包含人体类别"""
    for cls_id in cls:
        class_name = coco_names[int(cls_id)]
        if class_name.lower() == "person":
            return True
    return False

def boxes_to_list(boxes):
    """numpy框数组转为JSON可序列化列表"""
    if boxes is None or len(boxes) == 0:
        return []
    return boxes.tolist()

# ============ 信号中断处理 ============
def signal_handler(signal, frame):
    print("\n检测到Ctrl+C，程序退出")
    sys.exit(0)

# ============ 主程序入口 ============
def main():
    parser = argparse.ArgumentParser(
        description='USB摄像头人体检测 + 图传画框')
    parser.add_argument('--model-path', type=str,
                        default='/app/model/basic/yolov5s_672x672_nv12.bin',
                        help='BPU量化模型路径')
    parser.add_argument('--score-thres', type=float, default=0.25,
                        help='检测置信度阈值')
    parser.add_argument('--nms-thres', type=float, default=0.45,
                        help='非极大抑制阈值')
    parser.add_argument('--label-file', type=str,
                        default='/app/pydev_demo/07_usb_camera_sample/coco_classes.names',
                        help='COCO类别标签文件')
    parser.add_argument('--priority', type=int, default=0,
                        help='BPU模型运算优先级0~255')
    parser.add_argument('--bpu-cores', nargs='+', type=int, default=[0],
                        help='使用BPU核心编号')
    parser.add_argument('--no-display', action='store_true',
                        help='关闭可视化窗口，纯后台推理模式')

    opt = parser.parse_args()

    # 加载YOLOv5推理模型
    print("=" * 50)
    print("加载YOLOv5推理模型")
    print("=" * 50)
    if not os.path.exists(opt.model_path):
        print(f"错误：模型不存在 {opt.model_path}")
        sys.exit(-1)

    yolov5x = YoloV5X(opt)
    yolov5x.set_scheduling_params(priority=opt.priority, bpu_cores=opt.bpu_cores)
    common.print_model_info(yolov5x.model)
    coco_names = common.load_class_names(opt.label_file)

    # ROS2节点初始化，创建人体检测发布器
    rclpy.init(args=sys.argv)
    node = Node("camera_node")
    det_pub = node.create_publisher(String, "/person_detection", 10)
    print("ROS节点启动，发布话题 /person_detection")

    # 自动查找并打开USB摄像头
    video_device = find_first_usb_camera()
    if video_device is None:
        print("未检测到USB摄像头")
        rclpy.shutdown()
        sys.exit(-1)
    cap = cv2.VideoCapture(video_device)
    if not cap.isOpened():
        print(f"摄像头 {video_device} 打开失败")
        rclpy.shutdown()
        sys.exit(-1)
    # 设置摄像头分辨率、帧率、编码格式
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"\n摄像头节点启动成功，开始发布 /person_detection\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("读取摄像头图像失败")
                break
            img_h, img_w = frame.shape[:2]

            # BPU执行YOLO推理
            input_tensor = yolov5x.pre_process(frame)
            outputs = yolov5x.forward(input_tensor)
            boxes, cls_ids, scores = yolov5x.post_process(outputs, img_w, img_h)

            # 可视化画面绘制人体检测框
            vis_img = frame.copy()
            if not opt.no_display:
                person_mask = (cls_ids == 0)
                vis_img = draw.draw_boxes(vis_img, boxes[person_mask], cls_ids[person_mask],
                                          scores[person_mask], coco_names, common.rdk_colors)

            # 封装检测结果JSON消息并发布
            has_person = check_person_detected(cls_ids, coco_names)
            det_msg = {
                "has_person": has_person,
                "boxes": boxes_to_list(boxes[cls_ids == 0]) if has_person else []
            }
            msg = String()
            msg.data = json.dumps(det_msg)
            det_pub.publish(msg)

            # 窗口显示检测状态文字
            if not opt.no_display:
                status_txt = f"Person: {'YES' if has_person else 'NO'}"
                cv2.putText(vis_img, status_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Human Follow Camera", vis_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # 单次处理ROS回调，防止消息堆积
            rclpy.spin_once(node, timeout_sec=0.001)

    except KeyboardInterrupt:
        print("\n程序手动终止")
    finally:
        # 释放摄像头、ROS资源、窗口
        cap.release()
        node.destroy_node()
        rclpy.shutdown()
        if not opt.no_display:
            cv2.destroyAllWindows()
        print("程序完全退出")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()