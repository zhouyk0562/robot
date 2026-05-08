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
import sys
import select
import tty
import termios

warnings.filterwarnings("ignore", message="count < 8")


class DualCameraTagTracker:
    def __init__(self):
        # 初始化硬件
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()

        # 初始化主副摄像头
        self.main_cam = self._init_camera(0)  # 主摄像头
        self.aux_cam = self._init_camera(2)  # 副摄像头
        self.current_cam = self.main_cam  # 当前使用的摄像头

        # AprilTag检测器配置
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

        # 运动参数 (与KeyboardControl一致)
        self.base_speed = 350  # 常规移动速度(PWM值)
        self.turn_speed = 350  # 转向速度
        self.speed_step = 50  # 每次加减速幅度
        self.speed_offset = 75  # 左右电机速度差
        self.min_speed = 330  # 最小速度
        self.search_speed = 350  # 搜索模式下的旋转速度

        # 追踪参数
        self.tag_size = 0.1
        self.target_distance = 0.5
        self.angle_threshold = 2
        self.motor2_reverse = True

        # 触发参数
        self.trigger_distance = 1.4
        self.all_tag_ids = [1, 2, 3, 4]  # 所有tag ID
        self.command_executed = False
        self.group2_executing = False  # 动作组二是否正在执行
        self.last_group2_time = 0  # 上次执行动作组二的时间
        self.group2_interval = 0.8  # 动作组二执行间隔(秒)
        self.group2_lock = threading.Lock()  # 动作组二线程锁

        # 相机内参 (需要分别校准主副摄像头)
        self.main_cam_matrix = np.array([[350, 0, 160], [0, 350, 120], [0, 0, 1]])
        self.aux_cam_matrix = np.array([[350, 0, 160], [0, 350, 120], [0, 0, 1]])

        # 状态变量
        self.last_detection_time = time.time()
        self.search_mode = False
        self.main_cam_active = True

        # 多线程相关
        self.frame_queue = Queue(maxsize=2)
        self.result_queue = Queue(maxsize=2)
        self.running = False

        # 初始化舵机位置
        self._init_servos()

    def _init_camera(self, cam_id):
        """初始化指定ID的摄像头"""
        for i in range(5):
            cap = cv2.VideoCapture(cam_id)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                print(f"摄像头{cam_id}初始化成功")
                return cap
            print(f"摄像头{cam_id}初始化失败，尝试 {i + 1}/5")
            time.sleep(1)
        print(f"无法初始化摄像头{cam_id}，请检查连接")
        return None

    def _init_servos(self):
        """初始化舵机位置"""
        self.up.CDS_SetAngle(4, 100, 500)
        self.up.CDS_SetAngle(8, 235, 500)
        self.up.CDS_SetAngle(7, 900, 500)
        time.sleep(0.1)
        self.up.CDS_SetAngle(10, 650, 500)
        self.up.CDS_SetAngle(9, 430, 500)
        self.up.CDS_SetAngle(3, 300, 500)

    def switch_camera(self, use_main_cam):
        """切换主副摄像头"""
        if use_main_cam and not self.main_cam_active:
            print("切换到主摄像头")
            self.current_cam = self.main_cam
            self.main_cam_active = True
        elif not use_main_cam and self.main_cam_active:
            print("切换到副摄像头")
            self.current_cam = self.aux_cam
            self.main_cam_active = False

    def set_motor_speed(self, left, right):
        """
        设置电机速度，与KeyboardControl一致
        左电机速度比右电机绝对值大speed_offset
        """
        # 根据方向调整速度差
        if left > 0:
            adjusted_left = abs(left) + self.speed_offset
        elif left < 0:
            adjusted_left = -(abs(left) + self.speed_offset)
        else:
            adjusted_left = 0

        if right > 0:
            adjusted_right = abs(right)
        elif right < 0:
            adjusted_right = -abs(right)
        else:
            adjusted_right = 0

        # 限幅处理 (确保速度值在-1000到1000之间)
        adjusted_left = max(min(adjusted_left, 1000), -1000)
        adjusted_right = max(min(adjusted_right, 1000), -1000)

        # 注意第二个电机可能需要反向
        if self.motor2_reverse:
            adjusted_right = -adjusted_right

        self.up.CDS_SetSpeed(1, int(adjusted_left))
        self.up.CDS_SetSpeed(2, int(adjusted_right))

        return adjusted_left, adjusted_right

    def initialize_movement(self):
        """初始化移动"""

        print("执行初始化移动...")
        self.set_motor_speed(600, -600)  # 右转
        time.sleep(0.4)
        self.set_motor_speed(0, 0)
        time.sleep(0.2)
        self.set_motor_speed(800, 800)  # 直行
        time.sleep(0.8)
        self.set_motor_speed(0, 0)

    def _detection_worker(self):
        """检测线程工作函数"""
        while self.running:
            if not self.frame_queue.empty():
                frame, is_main_cam = self.frame_queue.get()
                if frame is None:
                    break

                # 处理不同通道数的图像
                if len(frame.shape) == 3:  # 彩色图像 (3或4通道)
                    if frame.shape[2] == 3:  # BGR图像
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    elif frame.shape[2] == 4:  # BGRA图像
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
                    else:
                        gray = frame[:, :, 0]  # 取第一个通道
                elif len(frame.shape) == 2:  # 已经是灰度图像
                    gray = frame
                else:  # 其他情况
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 尝试转换

                results = self.detector.detect(gray)

                if len(results) > 0:
                    best_tag = None
                    best_score = -float('inf')

                    for tag in results:
                        try:
                            # 根据当前摄像头选择对应的相机矩阵
                            cam_matrix = self.main_cam_matrix if is_main_cam else self.aux_cam_matrix

                            distance = self._calculate_distance(tag.homography, cam_matrix)
                            angle = self._calculate_angle(tag.corners, cam_matrix)

                            # 评分机制
                            distance_score = 1 - abs(distance - self.target_distance) / self.target_distance
                            angle_score = 1 - abs(angle) / 45
                            total_score = distance_score * 0.7 + angle_score * 0.3

                            if total_score > best_score:
                                best_score = total_score
                                best_tag = {
                                    'distance': distance,
                                    'angle': angle,
                                    'tag': tag,
                                    'from_main_cam': is_main_cam
                                }
                        except Exception as e:
                            print(f"计算Tag{tag.tag_id}时出错: {e}")

                    if best_tag:
                        self.result_queue.put((
                            True,
                            best_tag['distance'],
                            best_tag['angle'],
                            best_tag['tag'],
                            best_tag['from_main_cam']
                        ))
                        continue

                self.result_queue.put((False, 0, 0, None, is_main_cam))

    def _calculate_distance(self, H, camera_matrix):
        """计算距离，使用指定的相机内参"""
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
        _, rvec, tvec = cv2.solvePnP(opoints, ipoints, camera_matrix, None, flags=cv2.SOLVEPNP_ITERATIVE)
        return tvec[2][0]

    def _calculate_angle(self, corners, camera_matrix):
        """计算角度，使用指定的相机内参"""
        center_x = corners[:, 0].mean()
        image_center = camera_matrix[0, 2]
        offset_pixels = center_x - image_center
        focal_length = camera_matrix[0, 0]
        return degrees(atan2(offset_pixels, focal_length))

    def _execute_group2_actions(self):
        """动作组二的实际执行函数"""
        with self.group2_lock:
            self.group2_executing = True
            print("开始执行动作组二")

            # 执行动作组二
            self.up.CDS_SetAngle(4, 120, 500)
            self.up.CDS_SetAngle(10, 650, 500)
            self.up.CDS_SetAngle(9, 750, 500)
            time.sleep(0.3)
            self.up.CDS_SetAngle(8, 570, 500)
            self.up.CDS_SetAngle(3, 230, 800)
            self.up.CDS_SetAngle(7, 900, 800)
            time.sleep(0.4)
            self.up.CDS_SetAngle(3, 430, 800)
            self.up.CDS_SetAngle(7, 700, 800)

            print("动作组二执行完成")
            self.group2_executing = False
            self.last_group2_time = time.time()

    def execute_group_command(self, group_id):
        """根据组ID执行不同的命令"""
        if group_id == 1:
            # 动作组一：所有ID都执行这个动作
            print("执行动作组一(所有ID)")
            self.set_motor_speed(-500, 500)  # 左转
            time.sleep(2.5)
            self.set_motor_speed(0, 0)
            self.command_executed = True
            threading.Thread(
                target=self._reset_command_flag,
                daemon=True
            ).start()

        elif group_id == 2 and not self.group2_executing:
            # 动作组二：在单独线程中执行
            threading.Thread(
                target=self._execute_group2_actions,
                daemon=True
            ).start()

    def _reset_command_flag(self):
        """1秒后重置命令执行标志"""
        time.sleep(1)
        self.command_executed = False
        print("动作组一执行标志已重置")

    def check_group2_condition(self):
        """检查是否满足执行动作组二的条件"""
        current_time = time.time()
        return not self.group2_executing and (current_time - self.last_group2_time) >= self.group2_interval

    def control_motors(self, distance, angle, tag_id, from_main_cam):
        """电机控制逻辑，使用新的速度控制方法"""
        # 只有主摄像头检测到tag时才检查触发条件
        if from_main_cam and tag_id in self.all_tag_ids and distance < self.trigger_distance and not self.command_executed:
            self.execute_group_command(1)
            return 0, 0

        # 副摄像头检测到tag只进行追踪
        if self.group2_executing:
            return 0, 0

        if abs(angle) > self.angle_threshold:
            # 转向控制
            if angle > 0:
                left, right = -self.turn_speed, self.turn_speed
            else:
                left, right = self.turn_speed, -self.turn_speed
        else:
            # 前进/后退控制
            speed = min(self.base_speed + 100,  # 上限为基础速度+100
                        max(self.min_speed,
                            int((distance - self.target_distance) * 200)))
            left = right = speed

        return self.set_motor_speed(left, right)

    def is_key_pressed(self):
        """检测按键是否按下（非阻塞）"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def wait_for_key_press(self):
        """等待键盘按键按下"""
        print("等待按键按下以开始程序...")
        print("按任意键开始执行，按ESC键退出")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while True:
                if self.is_key_pressed():
                    key = sys.stdin.read(1)
                    if key == '\x1b':  # ESC键
                        print("ESC键已按下，退出程序")
                        exit(0)
                    else:
                        print(f"按键 {key} 已按下，开始执行程序")
                        time.sleep(0.5)
                        return
                time.sleep(0.1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def track_tag(self):
        """主追踪循环"""
        try:
            self.wait_for_key_press()
            self.initialize_movement()
            self.running = True
            detection_thread = threading.Thread(
                target=self._detection_worker,
                daemon=True
            )
            detection_thread.start()

            last_time = time.time()
            frame_count = 0
            fps = 0
            last_switch_time = time.time()
            switch_interval = 0.3

            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())

                while True:
                    if self.current_cam is None or not self.current_cam.isOpened():
                        print("摄像头未就绪，尝试重新初始化...")
                        self.current_cam = self._init_camera(0 if self.main_cam_active else 2)
                        time.sleep(0.1)
                        continue

                    ret, frame = self.current_cam.read()
                    if not ret:
                        print(f"{'主' if self.main_cam_active else '副'}摄像头读取失败")
                        continue

                    frame_count += 1
                    if frame_count % 10 == 0:
                        now = time.time()
                        fps = 10 / (now - last_time)
                        last_time = now

                    if self.frame_queue.empty():
                        self.frame_queue.put((frame, self.main_cam_active))

                    current_time = time.time()

                    if self.check_group2_condition():
                        self.execute_group_command(2)

                    if not self.result_queue.empty():
                        found, distance, angle, tag, from_main_cam = self.result_queue.get()

                        if found:
                            self.search_mode = False
                            self.last_detection_time = current_time
                            print(
                                f"[FPS: {fps:.1f}] [{'主' if from_main_cam else '副'}摄像头] 追踪Tag {tag.tag_id} | 距离: {distance:.2f}m | 角度: {angle:.1f}°")
                            left, right = self.control_motors(distance, angle, tag.tag_id, from_main_cam)
                            continue

                    if not self.search_mode:
                        if current_time - self.last_detection_time > 0.5:
                            self.search_mode = True
                            print("进入搜索模式")
                            self.set_motor_speed(-self.search_speed, self.search_speed)
                            last_switch_time = current_time
                            self.switch_camera(not self.main_cam_active)
                    else:
                        if current_time - last_switch_time > switch_interval:
                            self.switch_camera(not self.main_cam_active)
                            last_switch_time = current_time
                            print(f"搜索模式下切换至{'主' if self.main_cam_active else '副'}摄像头")

                    if self.is_key_pressed():
                        key = sys.stdin.read(1)
                        if key == '\x1b':
                            print("检测到ESC键，退出程序")
                            break

                    time.sleep(0.01)

            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.frame_queue.put((None, False))
            self.set_motor_speed(0, 0)
            if self.main_cam is not None:
                self.main_cam.release()
            if self.aux_cam is not None:
                self.aux_cam.release()
            print("程序安全退出")


if __name__ == "__main__":
    tracker = DualCameraTagTracker()
    tracker.track_tag()