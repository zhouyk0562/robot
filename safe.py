#!/usr/bin/env python3
import os
import time
import RPi.GPIO as GPIO
from typing import Tuple, Optional


class RobustTagTracker(TagTracker):
    def __init__(self):
        # 硬件复位引脚配置
        self.RESET_PIN = 18
        self.POWER_RELAY_PIN = 23
        self._init_hardware_protection()

        super().__init__()

    def _init_hardware_protection(self):
        """初始化硬件保护电路"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RESET_PIN, GPIO.OUT)
        GPIO.setup(self.POWER_RELAY_PIN, GPIO.OUT)
        GPIO.output(self.RESET_PIN, GPIO.LOW)
        GPIO.output(self.POWER_RELAY_PIN, GPIO.HIGH)  # 默认通电

    def _cut_power_relay(self):
        """切断执行器电源"""
        GPIO.output(self.POWER_RELAY_PIN, GPIO.LOW)
        self.logger.info("已切断执行器电源")

    def _restore_power(self):
        """恢复执行器电源"""
        GPIO.output(self.POWER_RELAY_PIN, GPIO.HIGH)
        time.sleep(1)  # 等待电源稳定

    def _software_reset(self):
        """尝试软件复位控制板"""
        try:
            self.up.CDS_Close()
            time.sleep(1)
            self.up.CDS_Open()
            return True
        except:
            return False

    def emergency_stop(self):
        """增强版紧急停止"""
        # 1. 停止所有信号输出
        super().emergency_stop()

        # 2. 切断电源
        self._cut_power_relay()

        # 3. 记录崩溃状态
        with open("/tmp/robot_crash.log", "a") as f:
            f.write(f"Crash at {time.ctime()}\n")

    def recover_from_crash(self):
        """系统恢复流程"""
        self.logger.info("尝试恢复系统...")

        # 阶段1：基础检查
        if self._check_controller_alive():
            return True

        # 阶段2：软件复位
        if self._software_reset():
            return True

        # 阶段3：硬件复位
        self._hardware_reset()
        time.sleep(3)  # 等待硬件初始化

        # 阶段4：电源循环
        self._cut_power_relay()
        time.sleep(2)
        self._restore_power()

        return self._check_controller_alive()


if __name__ == "__main__":
    max_retries = 3
    for attempt in range(max_retries):
        try:
            tracker = RobustTagTracker()
            if tracker.recover_from_crash():
                tracker.track_tag()
            else:
                raise RuntimeError("无法恢复硬件连接")
            break
        except Exception as e:
            print(f"尝试 {attempt + 1}/{max_retries} 失败: {e}")
            time.sleep(5)
    else:
        print("所有恢复尝试失败，请人工检查硬件")
        os.system("sudo shutdown -h now")  # 安全关机