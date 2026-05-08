# -*- coding: utf-8 -*-
import uptech
import cv2
import numpy as np
import time
import apriltag
from math import atan2, degrees
import warnings
import threading
from queue import Queue

warnings.filterwarnings("ignore", message="count < 8")


class TagTracker:
    def __init__(self):
        # 初始化硬件
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()

        # 初始化摄像头
        self.cap = self._init_camera()

        # 最简化的AprilTag检测器配置（兼容旧版本）
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

        # 运动参数
        self.speed_max = 420
        self.min_speed = 380
        self.turn_speed = 400
        self.tag_size = 0.1
        self.target_distance = 0.5
        self.angle_threshold = 5
        self.motor2_reverse = True

        # 新增参数：触发距离和组定义
        self.trigger_distance = 1.4  # 触发命令的距离阈值
        self.group1_ids = [1, 4]  # 第一组ID
        self.group2_ids = [2, 3]  # 第二组ID
        self.command_executed = False  # 防止重复执行命令

        # 相机内参（降采样后的值）
        self.Kmat = np.array([[350, 0, 160],
                              [0, 350, 120],
                              [0, 0, 1]])

        # 多线程相关
        self.frame_queue = Queue(maxsize=2)
        self.result_queue = Queue(maxsize=2)
        self.running = False

        # 启动检测线程
        self.detection_thread = threading.Thread(
            target=self._detection_worker,
            daemon=True
        )

    def _init_camera(self):
        """初始化摄像头"""
        for i in range(5):
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                # 降低分辨率提高帧率
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                return cap
            print(f"摄像头初始化失败，尝试 {i + 1}/5")
            time.sleep(1)
        print("无法初始化摄像头，请检查连接")
        exit(1)

    def set_motor_speed(self, left, right):
        """设置电机速度"""
        if self.motor2_reverse:
            right = -right
        self.up.CDS_SetSpeed(1, int(left))
        self.up.CDS_SetSpeed(2, int(right))
        return left, right

    def initialize_movement(self):
        """初始化移动"""
        print("执行初始化移动...")
        self.set_motor_speed(600, -600)  # 右转
        time.sleep(0.5)
        self.set_motor_speed(0, 0)
        time.sleep(0.1)
        self.set_motor_speed(410, 410)  # 直行
        time.sleep(1.0)
        self.set_motor_speed(0, 0)
        print("初始化移动完成")

    def _detection_worker(self):
        """检测线程工作函数"""
        while self.running:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                if frame is None:
                    break

                # 快速灰度转换
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # 检测AprilTag
                results = self.detector.detect(gray)

                if len(results) > 0:
                    best_tag = None
                    best_score = -float('inf')

                    for tag in results:
                        try:
                            distance = self._calculate_distance(tag.homography)
                            angle = self._calculate_angle(tag.corners)

                            # 评分标准
                            distance_score = 1 - abs(distance - self.target_distance) / self.target_distance
                            angle_score = 1 - abs(angle) / 45
                            total_score = distance_score * 0.7 + angle_score * 0.3

                            if total_score > best_score:
                                best_score = total_score
                                best_tag = {
                                    'distance': distance,
                                    'angle': angle,
                                    'tag': tag
                                }
                        except Exception as e:
                            print(f"计算Tag{tag.tag_id}时出错: {e}")

                    if best_tag:
                        self.result_queue.put((
                            True,
                            best_tag['distance'],
                            best_tag['angle'],
                            best_tag['tag']
                        ))
                        continue

                # 没有检测到标签
                self.result_queue.put((False, 0, 0, None))

    def _calculate_distance(self, H):
        """计算距离"""
        opoints = np.array([[-1.0, -1.0, 0.0],
                            [1.0, -1.0, 0.0],
                            [1.0, 1.0, 0.0],
                            [-1.0, 1.0, 0.0]], dtype=np.float32) * (self.tag_size / 2)

        ipoints = []
        for point in [[-1, -1], [1, -1], [1, 1], [-1, 1]]:
            x, y = point
            z = H[2, 0] * x + H[2, 1] * y + H[2, 2]
            u = (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / z
            v = (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / z
            ipoints.append([u, v])

        ipoints = np.array(ipoints, dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(opoints, ipoints, self.Kmat, None, flags=cv2.SOLVEPNP_ITERATIVE)
        return tvec[2][0]

    def _calculate_angle(self, corners):
        """计算角度"""
        center_x = corners[:, 0].mean()
        image_center = self.Kmat[0, 2]
        offset_pixels = center_x - image_center
        focal_length = self.Kmat[0, 0]
        return degrees(atan2(offset_pixels, focal_length))

    def execute_group_command(self, group_id):
        """根据组ID执行不同的命令"""
        if group_id == 1:
            print("执行组1命令(ID1或ID4)")
            # 示例动作：右转1秒，然后停止
            self.set_motor_speed(400, -400)
            time.sleep(1.0)
            self.set_motor_speed(0, 0)
        elif group_id == 2:
            print("执行组2命令(ID2或ID3)")
            # 示例动作：左转1秒，然后停止
            # self.set_motor_speed(-400, 400)
            # time.sleep(1.0)
            self.set_motor_speed(0, 0)
        self.command_executed = True
        # 启动一个线程在5秒后重置标志
        threading.Thread(
            target=self._reset_command_flag,
            daemon=True
        ).start()

    def _reset_command_flag(self):
        """5秒后重置命令执行标志"""
        time.sleep(1)
        self.command_executed = False
        print("命令执行标志已重置")

    def control_motors(self, distance, angle, tag_id):
        """电机控制逻辑"""
        # 检查是否达到触发距离且未执行过命令
        if distance < self.trigger_distance and not self.command_executed:
            if tag_id in self.group1_ids:
                self.execute_group_command(1)
            elif tag_id in self.group2_ids:
                self.execute_group_command(2)
            return 0, 0  # 执行命令时停止运动

        # 移除了原来的距离判断重置逻辑

        # 正常追踪逻辑
        if abs(angle) > self.angle_threshold:
            if angle > 0:  # 右转
                left, right = -self.turn_speed, self.turn_speed
            else:  # 左转
                left, right = self.turn_speed, -self.turn_speed
        else:
            speed = min(self.speed_max,
                        max(self.min_speed,
                            int((distance - self.target_distance) * 200)))
            left = right = speed
        return self.set_motor_speed(left, right)

    def track_tag(self):
        """主追踪循环"""
        try:
            self.initialize_movement()
            self.running = True
            self.detection_thread.start()

            last_time = time.time()
            frame_count = 0
            fps = 0

            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("摄像头读取失败")
                    break

                # 计算FPS
                frame_count += 1
                if frame_count % 10 == 0:
                    now = time.time()
                    fps = 10 / (now - last_time)
                    last_time = now

                # 非阻塞式放入队列
                if self.frame_queue.empty():
                    self.frame_queue.put(frame.copy())

                # 获取最新结果
                if not self.result_queue.empty():
                    found, distance, angle, tag = self.result_queue.get()
                    if found:
                        print(f"[FPS: {fps:.1f}] 追踪Tag {tag.tag_id} | 距离: {distance:.2f}m | 角度: {angle:.1f}°")
                        left, right = self.control_motors(distance, angle, tag.tag_id)
                    else:
                        print(f"[FPS: {fps:.1f}] 未检测到AprilTag")
                        self.set_motor_speed(0, 0)

                # 控制循环频率
                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.frame_queue.put(None)  # 通知线程退出
            self.detection_thread.join()
            self.set_motor_speed(0, 0)
            self.cap.release()
            print("程序安全退出")


if __name__ == "__main__":
    tracker = TagTracker()
    tracker.track_tag()