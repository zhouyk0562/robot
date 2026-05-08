# -*- coding: utf-8 -*-
import uptech
import time
import cv2
import numpy as np
from pupil_apriltags import Detector


class TagTracker:
    def __init__(self):
        """初始化电机、摄像头和AprilTag检测器"""
        self.board = uptech.UpTech()
        self.board.CDS_Open()  # 开启电机驱动
        self.board.ADC_IO_Open()  # 开启ADC输入

        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # AprilTag检测器配置
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # 运动参数
        self.base_speed = 300
        self.max_speed = 800
        self.k_p = 0.8  # 比例控制系数
        self.target_tag_id = 0  # 默认追踪的Tag ID
        self.tag_size = 0.1  # Tag的实际大小(米)

        # 相机内参(需要根据实际摄像头校准)
        self.camera_params = [
            500,  # fx
            500,  # fy
            320,  # cx
            240  # cy
        ]

        # 当前状态
        self.tracking = False
        self.last_seen = 0

    def set_speed(self, left, right):
        """设置电机速度"""
        left = max(min(left, self.max_speed), -self.max_speed)
        right = max(min(right, self.max_speed), -self.max_speed)
        self.board.CDS_SetSpeed(1, left)
        self.board.CDS_SetSpeed(2, -right)  # 第二个电机可能需要反向

    def stop(self):
        """停止机器人"""
        self.set_speed(0, 0)

    def detect_tags(self):
        """检测图像中的AprilTags"""
        ret, frame = self.cap.read()
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        return frame, tags

    def visualize(self, frame, tags):
        """可视化检测结果"""
        for tag in tags:
            # 绘制Tag边框
            for idx in range(len(tag.corners)):
                cv2.line(frame,
                         tuple(tag.corners[idx - 1].astype(int)),
                         tuple(tag.corners[idx].astype(int)),
                         (0, 255, 0), 2)

            # 绘制Tag中心点和ID
            center = tag.center.astype(int)
            cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
            cv2.putText(frame, str(tag.tag_id),
                        (center[0] + 10, center[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 绘制距离和角度信息
            if hasattr(tag, 'pose_t'):
                distance = np.linalg.norm(tag.pose_t)
                cv2.putText(frame, f"Dist: {distance:.2f}m",
                            (center[0] + 10, center[1] + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        cv2.imshow("Tag Tracking", frame)
        cv2.waitKey(1)

    def track_tag(self, tag):
        """根据检测到的Tag控制机器人运动"""
        center_x = tag.center[0]
        image_center = self.camera_params[2]  # cx

        # 计算Tag中心与图像中心的偏差
        error = center_x - image_center
        turn = int(error * self.k_p)

        # 根据距离调整速度
        if hasattr(tag, 'pose_t'):
            distance = np.linalg.norm(tag.pose_t)
            speed = min(int(self.base_speed * (1 / distance)), self.max_speed)
        else:
            speed = self.base_speed

        # 设置电机速度
        left_speed = speed - turn
        right_speed = speed + turn

        self.set_speed(left_speed, right_speed)
        self.last_seen = time.time()

    def run(self):
        """主控制循环"""
        print("Tag追踪程序启动...")
        print(f"追踪目标Tag ID: {self.target_tag_id}")
        print("按'q'退出程序")

        try:
            while True:
                frame, tags = self.detect_tags()
                if frame is None:
                    continue

                target_tag = None
                for tag in tags:
                    if tag.tag_id == self.target_tag_id:
                        target_tag = tag
                        break

                if target_tag:
                    self.tracking = True
                    self.track_tag(target_tag)
                    self.visualize(frame, [target_tag])
                else:
                    self.visualize(frame, tags)
                    if self.tracking and time.time() - self.last_seen > 1.0:
                        # 超过1秒没看到Tag，停止机器人
                        self.stop()
                        self.tracking = False
                        print("目标Tag丢失，已停止")

                # 检查退出键
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            self.board.CDS_Close()
            self.board.ADC_IO_Close()
            self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    tracker = TagTracker()
    tracker.run()