# -*- coding: utf-8 -*-
import uptech
import time
import math


class CenterPatrol:
    def __init__(self):
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()

        # 传感器参数（需实际校准）
        self.center_threshold = 1650  # 中心区域灰度阈值
        self.edge_threshold = 1400  # 边缘灰度阈值
        self.speed_max = 600  # 最大速度
        self.speed_min = 200  # 巡逻速度

        # 巡逻参数
        self.patrol_radius = 300  # 虚拟巡逻半径（PWM*时间等效值）
        self.last_angle = 0  # 记录上次转向角度

    def get_grayscale(self):
        """获取前后灰度传感器值（后, 前）"""
        adc_values = self.up.ADC_Get_All_Channle()
        return adc_values[0], adc_values[1]

    def smooth_stop(self):
        """减速停止"""
        for speed in range(self.speed_max, 0, -50):
            self.up.CDS_SetSpeed(1, speed)
            self.up.CDS_SetSpeed(2, -speed)
            time.sleep(0.05)
        self.up.CDS_SetSpeed(1, 0)
        self.up.CDS_SetSpeed(2, 0)

    def approach_center(self):
        """向中心移动直到到达"""
        while True:
            rear, front = self.get_grayscale()
            print(f"前:{front} 后:{rear} 差值:{front - rear}")

            # 动态速度控制（越接近中心越慢）
            speed = int(self.speed_max * (1 - front / self.center_threshold))
            speed = max(min(speed, self.speed_max), self.speed_min)

            if front >= self.center_threshold:
                self.smooth_stop()
                print("---已到达中心---")
                break
            elif rear < self.edge_threshold:  # 防止从后方离开中心
                self.up.CDS_SetSpeed(1, -speed // 2)
                self.up.CDS_SetSpeed(2, speed // 2)
                time.sleep(0.3)
            else:
                # 基本朝向控制
                if front - rear > 50:  # 正对中心
                    self.up.CDS_SetSpeed(1, speed)
                    self.up.CDS_SetSpeed(2, -speed)
                elif rear - front > 50:  # 方向反了
                    self.up.CDS_SetSpeed(1, -speed // 2)
                    self.up.CDS_SetSpeed(2, speed // 2)
                    time.sleep(0.5)
                else:  # 近似垂直中心线
                    self.up.CDS_SetSpeed(1, speed)
                    self.up.CDS_SetSpeed(2, speed // 3)

            time.sleep(0.1)

    def circular_patrol(self):
        """在中心区域环形巡逻"""
        patrol_seq = [
            (self.speed_min, -self.speed_min),  # 前进
            (self.speed_min, self.speed_min // 2),  # 右弧线
            (self.speed_min // 2, self.speed_min),  # 右转
            (-self.speed_min, -self.speed_min),  # 后退
            (-self.speed_min // 2, -self.speed_min)  # 左弧线
        ]

        while True:
            rear, front = self.get_grayscale()

            # 中心区域保持巡逻
            if front >= self.center_threshold * 0.9:
                for left_speed, right_speed in patrol_seq:
                    self.up.CDS_SetSpeed(1, left_speed)
                    self.up.CDS_SetSpeed(2, right_speed)
                    time.sleep(0.8)

                    # 实时检测是否偏离中心
                    if front < self.center_threshold * 0.7:
                        break
            else:
                print("偏离中心，重新校正...")
                self.approach_center()

    def run(self):
        try:
            self.approach_center()
            self.circular_patrol()
        except KeyboardInterrupt:
            self.up.CDS_SetSpeed(1, 0)
            self.up.CDS_SetSpeed(2, 0)
            print("程序终止")


if __name__ == "__main__":
    patrol = CenterPatrol()
    patrol.run()