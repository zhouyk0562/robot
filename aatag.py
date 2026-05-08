# -*- coding: utf-8 -*-
import uptech
import cv2
import numpy as np
import time
import apriltag
from math import atan2, degrees
import warnings

warnings.filterwarnings("ignore", message="count < 8")


class TagTracker:
    def __init__(self):
        # 初始化硬件
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()

        # 初始化摄像头（尝试多次）
        self.cap = None
        for i in range(5):
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                break
            print(f"摄像头初始化失败，尝试 {i + 1}/5")
            time.sleep(1)

        if not self.cap or not self.cap.isOpened():
            print("无法初始化摄像头，请检查连接")
            exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # AprilTag检测器
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

        # 运动参数（已调整最大速度为400）
        self.speed_max = 300  # 最大速度限制为400
        self.min_effective_speed = 250  # 电机最低有效速度
        self.tag_size = 0.1  # AprilTag实际尺寸(米)
        self.target_distance = 0.5  # 目标距离(米)
        self.angle_gain = 6.0  # 角度增益系数（适当降低）
        self.base_speed_gain = 1.0  # 基础速度增益（调整为1.0）

        # 相机内参
        self.Kmat = np.array([[700, 0, 320],
                              [0, 700, 240],
                              [0, 0, 1]])

    def get_tag_info(self, frame):
        """检测所有AprilTag并返回最佳目标信息"""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 添加图像预处理
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY, 11, 2)

            results = self.detector.detect(gray)

            if len(results) > 0:
                best_tag = None
                best_score = -float('inf')

                for tag in results:
                    try:
                        distance = self.calculate_distance(tag.homography)
                        angle = self.calculate_angle(tag.corners)

                        # 评分标准（增加对近距离的权重）
                        distance_score = 1 - abs(distance - self.target_distance) / self.target_distance
                        angle_score = 1 - abs(angle) / 45  # 将角度范围缩小到45度
                        total_score = distance_score * 0.7 + angle_score * 0.3  # 增加距离权重

                        if total_score > best_score:
                            best_score = total_score
                            best_tag = {
                                'distance': distance,
                                'angle': angle,
                                'corners': tag.corners,
                                'tag': tag
                            }

                    except Exception as e:
                        print(f"计算Tag{tag.tag_id}时出错: {e}")

                if best_tag:
                    return True, best_tag['distance'], best_tag['angle'], best_tag['corners'], best_tag['tag']

            return False, 0, 0, None, None
        except Exception as e:
            print(f"检测AprilTag时出错: {e}")
            return False, 0, 0, None, None

    def calculate_distance(self, H):
        """计算与AprilTag的距离"""
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

        if ipoints.shape[0] < 4:
            return 0.0

        _, rvec, tvec = cv2.solvePnP(opoints, ipoints, self.Kmat, None, flags=cv2.SOLVEPNP_ITERATIVE)
        return tvec[2][0]

    def calculate_angle(self, corners):
        """计算AprilTag中心相对于图像中心的水平偏角"""
        center_x = corners[:, 0].mean()
        image_center = self.Kmat[0, 2]
        offset_pixels = center_x - image_center
        focal_length = self.Kmat[0, 0]
        return degrees(atan2(offset_pixels, focal_length)) * 1.5  # 降低角度响应系数

    def ensure_speed_range(self, speed):
        """确保速度在有效范围内"""
        if speed > 0:
            return max(self.min_effective_speed, min(self.speed_max, speed))
        elif speed < 0:
            return min(-self.min_effective_speed, max(-self.speed_max, speed))
        return 0

    def smooth_stop(self):
        """减速停止"""
        current_speed = self.speed_max
        while current_speed > 0:
            self.up.CDS_SetSpeed(1, current_speed)
            self.up.CDS_SetSpeed(2, -current_speed)
            current_speed -= 50
            time.sleep(0.05)
        self.up.CDS_SetSpeed(1, 0)
        self.up.CDS_SetSpeed(2, 0)

    def track_tag(self):
        """持续追踪最佳AprilTag"""
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("摄像头读取失败")
                    break

                found, distance, angle, _, tag = self.get_tag_info(frame)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                tag_count = len(self.detector.detect(gray)) if found else 0

                if found:
                    print(
                        f"追踪Tag {tag.tag_id} | 距离: {distance:.2f}m | 角度: {angle:.1f}° | 视野中Tag数: {tag_count}")

                    # 运动控制逻辑（已调整速度计算）
                    if distance > self.target_distance * 1.2:
                        # 前进速度计算（限制在400以内）
                        speed_ratio = min((distance - self.target_distance) / 1.0, 1)
                        base_speed = int(
                            self.min_effective_speed + (self.speed_max - self.min_effective_speed) * speed_ratio)
                        left_speed = base_speed - int(angle * self.angle_gain * 0.8)  # 降低转向灵敏度
                        right_speed = -base_speed - int(angle * self.angle_gain * 0.8)
                    elif distance < self.target_distance * 0.8:
                        # 后退速度计算（限制在400以内）
                        speed_ratio = min((self.target_distance - distance) / 0.5, 1)
                        base_speed = int(
                            self.min_effective_speed + (self.speed_max - self.min_effective_speed) * speed_ratio)
                        left_speed = -base_speed - int(angle * self.angle_gain * 0.8)
                        right_speed = base_speed - int(angle * self.angle_gain * 0.8)
                    else:
                        # 仅转向控制（限制在300以内）
                        turn_speed = min(300, max(self.min_effective_speed, abs(int(angle * self.angle_gain))))
                        left_speed = -turn_speed if angle > 0 else turn_speed
                        right_speed = -turn_speed if angle > 0 else turn_speed

                    # 设置电机速度（确保不超过400）
                    left_speed = max(-self.speed_max, min(self.speed_max, left_speed))
                    right_speed = max(-self.speed_max, min(self.speed_max, right_speed))

                    print(f"电机速度: 左轮={left_speed}, 右轮={right_speed}")
                    self.up.CDS_SetSpeed(1, left_speed)
                    self.up.CDS_SetSpeed(2, right_speed)

                else:
                    self.up.CDS_SetSpeed(1, 0)
                    self.up.CDS_SetSpeed(2, 0)
                    if tag_count > 0:
                        print(f"未选择任何Tag | 视野中Tag数: {tag_count}")
                    else:
                        print("未检测到AprilTag")

                time.sleep(0.15)  # 增加控制周期，降低响应频率

        except KeyboardInterrupt:
            pass
        finally:
            self.smooth_stop()
            self.cap.release()
            print("程序安全退出")


if __name__ == "__main__":
    tracker = TagTracker()
    tracker.track_tag()