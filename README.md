# 逐影——ROS2人体跟随智能小车
基于RDK X5开发板与ROS2框架开发的家庭随行机器人，融合USB视觉人体检测、LD19激光雷达环境感知，支持键盘手动控制、自主人体恒距跟随两种工作模式。

## 一、项目功能
1. 视觉感知：USB摄像头搭载YOLOv5算法，依托RDK X5 BPU硬件加速识别人体，输出目标坐标信息
2. 雷达感知：LD19激光雷达采集360°原始点云，数据预处理后提取前方障碍物距离
3. 跟随决策：采用P比例控制融合视觉、雷达两路数据，实现平稳人体跟随、近距离自动停车避障
4. 底层电机驱动：接收运动指令输出PWM调速信号，控制四轮减速电机完成行驶动作
5. 键盘手动控制：通过终端按键下发运动指令，可控制小车前进、后退、转向、调速、急停

## 二、硬件清单
- 主控：RDK X5开发板
- 感知设备：USB摄像头、LD19激光雷达（TTL转USB串口）
- 运动结构：金属四轮底盘、四路12V减速电机
- 供电方案：12V锂电池独立给电机供电，单块5V充电宝为主控、传感器供电，强弱电隔离抑制电磁干扰

## 三、代码文件说明
| 文件名 | 节点功能 | 订阅话题 | 发布话题 | 启动方式 |
|--------|----------|----------|----------|----------|
| camera_node.py | USB摄像头人体检测推理 | 无 | /person_detection | python3 camera_node.py |
| ld19_node.py | LD19激光雷达驱动 | 无 | /scan | ros2 launch ld19.launch.py |
| lidar_node.py | 雷达数据滤波、截取前方测距 | /scan | /lidar_front_dist | python3 lidar_node.py |
| follow_node.py | 人体跟随决策控制 | /person_detection、/lidar_front_dist | /car_cmd | python3 follow_node.py |
| ctrl_node.py | 底层电机硬件驱动 | /car_cmd | 无 | python3 ctrl_node.py |
| key_ctrl.py | 终端键盘手动控制 | 无 | /car_cmd | python3 key_ctrl.py |

## 四、运行环境依赖
1. 操作系统：Ubuntu 22.04
2. 机器人框架：ROS2 Humble
3. 推理库：RDK X5配套BPU加速库
4. 雷达驱动包：ldlidar_stl_ros2

## 五、完整启动步骤
1. 启动激光雷达：`ros2 launch ld19_node.py`
2. 启动雷达预处理节点：`python3 lidar_node.py`
3. 启动摄像头人体检测：`python3  camera_node.py`
4. 启动跟随决策节点：`python3  follow_node.py`
5. 启动电机驱动节点：`python3  ctrl_node.py`
6. 启动远程键盘控制节点：`python3  key_ctrl.py`

## 六、拓展方向
1. 接入云端，手机APP远程监控画面与小车状态
2. 集成SLAM算法，实现自主建图导航、自动回充
3. 增加跌倒检测、异常行为预警AI功能
4. 拓展语音交互模块，实现人机语音互动
