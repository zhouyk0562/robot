#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机基础功能测试 —— 仅验证左右电机是否能正反转
前提：将 UpTech 类所在的文件命名为 uptech.py 并放在同一目录下
"""

import uptech
import time

# 测试速度（0~1000，根据实际电池电压调整）
TEST_SPEED = 300

def main():
    # 1. 初始化
    up = uptech.UpTech()
    up.CDS_Open()          # 开启电机控制
    up.ADC_IO_Open()       # 如果只测电机，这句也可以省略（但保留不影响）

    def set_motors(left, right):
        """左右电机速度设置，CDS ID 1=左电机，2=右电机"""
        up.CDS_SetSpeed(2, left)
        up.CDS_SetSpeed(1, right)

    def stop():
        set_motors(0, 0)

    try:
        print("电机测试开始，请确保小车悬空或放在宽敞地面。")
        print("按 Ctrl+C 可随时停止。\n")

        # 2. 前进（两轮同向正转）
        print("[1/4] 前进 2 秒...")
        set_motors(TEST_SPEED, TEST_SPEED)
        time.sleep(2)
        stop()
        time.sleep(0.5)

        # 3. 后退（两轮同向反转）
        print("[2/4] 后退 2 秒...")
        set_motors(-TEST_SPEED, -TEST_SPEED)
        time.sleep(2)
        stop()
        time.sleep(0.5)

        # 4. 左转（左轮反转，右轮正转）
        print("[3/4] 左转 1.5 秒...")
        set_motors(-TEST_SPEED, TEST_SPEED)
        time.sleep(1.5)
        stop()
        time.sleep(0.5)

        # 5. 右转（左轮正转，右轮反转）
        print("[4/4] 右转 1.5 秒...")
        set_motors(TEST_SPEED, -TEST_SPEED)
        time.sleep(1.5)
        stop()

        print("\n电机测试完成！所有动作均已执行。")

    except KeyboardInterrupt:
        stop()
        print("\n测试被用户中断。")
    finally:
        stop()
        # 可选关闭外设
        # up.CDS_Close()
        # up.ADC_IO_Close()

if __name__ == "__main__":
    main()