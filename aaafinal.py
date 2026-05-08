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


class SingleCameraTagTracker:
    def __init__(self):
        # 初始化硬件
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()

        # 初始化主摄像头 - 使用自动探测
        self.cam = self._init_camera()  # 自动探测可用摄像头

        # AprilTag检测器配置
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

        # 运动参数 - 修改为固定值
        self.base_speed = 390  # 追踪模式直行速度(PWM值)
        self.turn_speed = 440  # 追踪模式转向速度
        self.speed_offset = -25  # 左右电机速度差
        self.min_speed = 400  # 最小速度
        self.search_speed = 440  # 搜索模式下的旋转速度

        # 追踪参数
        self.tag_size = 0.1
        self.target_distance = 0.5
        self.angle_threshold = 2
        self.motor2_reverse = True

        # 触发参数
        self.trigger_distance = 1.0
        self.all_tag_ids = [1, 2, 3, 4]  # 所有tag ID
        self.command_executed = False
        self.group2_executing = False  # 动作组二是否正在执行
        self.last_group2_time = 0  # 上次执行动作组二的时间
        self.group2_interval = 0.8  # 动作组二执行间隔(秒)
        self.group2_lock = threading.Lock()  # 动作组二线程锁

        # 相机内参
        self.cam_matrix = np.array([[350, 0, 160], [0, 350, 120], [0, 0, 1]])

        # 状态变量
        self.last_detection_time = time.time()
        self.search_mode = False
        self.search_stage = "grayscale"  # 搜索阶段: grayscale/rotation
        self.last_grayscale_time = 0  # 上次执行灰度导航的时间
        self.grayscale_interval = 0.1  # 灰度导航执行间隔(秒)

        # 新增：旋转搜索开始时间
        self.rotation_start_time = 0  # 记录开始旋转搜索的时间
        self.min_stage_duration = 2.0  # 最小状态持续时间(秒)
        self.last_stage_change = 0  # 上次状态变更时间

        # 灰度传感器参数
        self.center_threshold = 1800
        self.edge_threshold = 900

        # 多线程相关
        self.frame_queue = Queue(maxsize=2)
        self.result_queue = Queue(maxsize=2)
        self.running = False

        # 初始化舵机位置
        self._init_servos()

    def _init_camera(self):
        """自动探测并初始化可用的摄像头"""
        camera_ids = [0, 1, 2, 3, 4]  # 尝试所有可能的摄像头ID
        for cam_id in camera_ids:
            cap = cv2.VideoCapture(cam_id)
            if cap.isOpened():
                print(f"尝试打开摄像头 {cam_id}: 成功")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                return cap
            print(f"尝试打开摄像头 {cam_id}: 失败")
            cap.release()

        print("错误: 未找到可用摄像头")
        return None

    def _init_servos(self):
        """初始化舵机位置"""
        self.up.CDS_SetAngle(4, 100, 500)
        self.up.CDS_SetAngle(8, 235, 500)
        self.up.CDS_SetAngle(7, 900, 500)
        time.sleep(0.1)
        self.up.CDS_SetAngle(10, 650, 500)
        self.up.CDS_SetAngle(9, 150, 500)
        self.up.CDS_SetAngle(3, 150, 500)

    def set_motor_speed(self, left, right):
        """
        设置电机速度
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
        time.sleep(0.5)
        self.set_motor_speed(0, 0)
        time.sleep(0.2)
        self.set_motor_speed(700, 700)  # 直行
        time.sleep(0.6)
        self.set_motor_speed(0, 0)

    def _detection_worker(self):
        """检测线程工作函数"""
        while self.running:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                if frame is None:
                    break

                # 统一处理图像为灰度格式
                if len(frame.shape) == 3:  # 彩色图像
                    if frame.shape[2] == 3:  # BGR图像
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    elif frame.shape[2] == 4:  # BGRA图像
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
                    else:  # 其他格式
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 尝试转换
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
                            distance = self._calculate_distance(tag.homography, self.cam_matrix)
                            angle = self._calculate_angle(tag.corners, self.cam_matrix)

                            # 评分机制
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

                self.result_queue.put((False, 0, 0, None))

            # 添加帧率控制
            time.sleep(0.02)  # 约50FPS

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
        """动作组二的实际执行函数（改进版）"""
        with self.group2_lock:
            # 双重检查防止重复执行
            if self.group2_executing:
                return

            self.group2_executing = True
            print("开始执行动作组二")

            try:
                # 执行动作组二 - 只控制舵机
                self.up.CDS_SetAngle(4, 120, 500)
                self.up.CDS_SetAngle(10, 650, 500)
                self.up.CDS_SetAngle(9, 520, 500)
                time.sleep(0.4)

                self.up.CDS_SetAngle(8, 630, 500)
                self.up.CDS_SetAngle(3, 350, 800)
                self.up.CDS_SetAngle(7, 900, 800)
                time.sleep(0.4)

                self.up.CDS_SetAngle(3, 130, 800)
                self.up.CDS_SetAngle(7, 700, 800)

                print("动作组二执行完成")
            except Exception as e:
                print(f"动作组二执行出错: {e}")
            finally:
                self.group2_executing = False
                self.last_group2_time = time.time()

    def execute_group_command(self, group_id):
        """根据组ID执行不同的命令"""
        if group_id == 1:
            # 动作组一：所有ID都执行这个动作
            print("执行动作组一(所有ID)")
            self.set_motor_speed(-600, 600)  # 左转
            time.sleep(1.8)
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

    def get_grayscale(self):
        """获取前后灰度传感器值"""
        adc_values = self.up.ADC_Get_All_Channle()
        return adc_values[0], adc_values[1]

    def approach_center(self):
        """改进的中心接近方法（非阻塞式）"""
        current_time = time.time()

        # 控制执行频率
        if current_time - self.last_grayscale_time < self.grayscale_interval:
            return

        self.last_grayscale_time = current_time

        # 如果检测到tag，立即退出并停止电机
        if not self.result_queue.empty():
            found, _, _, _ = self.result_queue.get()
            if found:
                print("检测到标签，退出灰度导航")
                self.search_mode = False
                self.set_motor_speed(0, 0)  # 停止电机
                return

        front, rear = self.get_grayscale()
        diff = front - rear

        if front >= self.center_threshold:
            print("已到达中心区域，切换到旋转搜索")
            self.change_stage("rotation")  # 使用状态变更方法
            self.rotation_start_time = time.time()  # 记录旋转搜索开始时间
            self.set_motor_speed(0, 0)  # 停止电机
            return

        # 计算基础速度 - 搜索模式下直行和转弯速度设为400
        base_speed = 440  # 搜索模式下直行速度

        # 方向控制 - 搜索模式下转弯速度设为400
        if diff > 900:  # 正对中心
            left_speed = base_speed
            right_speed = base_speed  # 右转
        elif diff < -900:  # 方向反了
            left_speed = base_speed  # 左转
            right_speed = base_speed
        else:  # 近似垂直
            left_speed = -base_speed
            right_speed = base_speed

        # 应用电机控制
        self.set_motor_speed(left_speed, right_speed)

    def control_motors(self, distance, angle, tag_id):
        """电机控制逻辑，使用固定速度值"""
        # 检查触发条件
        if tag_id in self.all_tag_ids and distance < self.trigger_distance and not self.command_executed:
            self.execute_group_command(1)
            return 0, 0

        if self.group2_executing:
            return 0, 0

        if abs(angle) > self.angle_threshold:
            # 转向控制 - 使用固定转弯速度400
            if angle > 0:
                left, right = -self.turn_speed, self.turn_speed  # 右转
            else:
                left, right = self.turn_speed, -self.turn_speed  # 左转
        else:
            # 前进/后退控制 - 使用固定直行速度380
            left = right = self.base_speed

        return self.set_motor_speed(left, right)

    def change_stage(self, new_stage):
        """安全地变更搜索阶段，避免快速切换"""
        if new_stage != self.search_stage:
            current_time = time.time()
            # 检查是否满足最小状态持续时间
            if current_time - self.last_stage_change > self.min_stage_duration:
                print(f"状态变更: {self.search_stage} -> {new_stage}")
                self.search_stage = new_stage
                self.last_stage_change = current_time
                return True
            return False
        return False

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

            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())

                while True:
                    # 摄像头重新初始化逻辑
                    if self.cam is None or not self.cam.isOpened():
                        print("摄像头未就绪，尝试重新初始化...")
                        self.cam = self._init_camera()  # 使用自动探测
                        if self.cam is None or not self.cam.isOpened():
                            print("摄像头初始化失败，等待0.5秒后重试")
                            time.sleep(0.5)
                        continue

                    ret, frame = self.cam.read()
                    if not ret:
                        print("主摄像头读取失败，尝试重新初始化")
                        self.cam.release()
                        self.cam = self._init_camera()  # 使用自动探测
                        continue

                    frame_count += 1
                    if frame_count % 10 == 0:
                        now = time.time()
                        fps = 10 / (now - last_time)
                        last_time = now

                    if self.frame_queue.empty():
                        self.frame_queue.put(frame)

                    current_time = time.time()

                    # 定期检查并执行动作组二 - 在任何状态下都会执行
                    if self.check_group2_condition():
                        self.execute_group_command(2)

                    if not self.result_queue.empty():
                        found, distance, angle, tag = self.result_queue.get()

                        if found:
                            self.search_mode = False
                            self.search_stage = "grayscale"
                            self.last_detection_time = current_time
                            print(f"[FPS: {fps:.1f}] 追踪Tag {tag.tag_id} | 距离: {distance:.2f}m | 角度: {angle:.1f}°")
                            left, right = self.control_motors(distance, angle, tag.tag_id)
                            continue

                    if not self.search_mode:
                        if current_time - self.last_detection_time > 0.5:
                            self.search_mode = True
                            self.search_stage = "grayscale"
                            self.last_stage_change = current_time  # 记录状态变更时间
                            print("进入搜索模式")

                    if self.search_mode:
                        if self.search_stage == "grayscale":
                            # 非阻塞式灰度导航
                            self.approach_center()
                        elif self.search_stage == "rotation":
                            # 旋转搜索 - 使用固定旋转速度410
                            self.set_motor_speed(-self.search_speed, self.search_speed)

                            # 添加调试输出
                            rotation_time = current_time - self.rotation_start_time
                            print(f"旋转搜索中... 已持续: {rotation_time:.1f}秒")

                            # 使用专门的旋转开始时间检查超时
                            if current_time - self.rotation_start_time > 10.0:
                                print("旋转搜索超时，返回灰度导航")
                                self.change_stage("grayscale")
                                # 重置灰度导航相关状态
                                self.last_grayscale_time = 0

                    if self.is_key_pressed():
                        key = sys.stdin.read(1)
                        if key == '\x1b':
                            print("检测到ESC键，退出程序")
                            break

                    # 保持主循环流畅运行
                    time.sleep(0.01)

            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.frame_queue.put(None)
            self.set_motor_speed(0, 0)
            if self.cam is not None and self.cam.isOpened():
                self.cam.release()
            print("程序安全退出")


if __name__ == "__main__":
    tracker = SingleCameraTagTracker()
    tracker.track_tag()
    # tracker.initialize_movement()