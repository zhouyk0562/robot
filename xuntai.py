#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# 初始化
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ---------- 参数配置 ----------
THRESHOLD = 650  # 灰度小于此值认为是黑线（你的黑色约560，白色应该更大）

# 前进速度（根据你实测的直线匹配）
LEFT_SPEED_FWD = 270  # 1号电机
RIGHT_SPEED_FWD = 300  # 2号电机

# 后退速度（可适当减慢）
LEFT_SPEED_BACK = -270
RIGHT_SPEED_BACK = -300

# 转弯速度（左转：左侧减速/倒车，右侧正转；方向可根据实际调整）
TURN_SPEED_LEFT = -300  # 左轮速度
TURN_SPEED_RIGHT = 300  # 右轮速度

# 动作持续时间（秒）
BACK_DURATION = 0.5
TURN_DURATION = 0.5


def get_front_grayscale():
    """返回前方灰度传感器数值（假设 ADC 通道1 是前面）"""
    vals = up.ADC_Get_All_Channle()
    # 根据你的描述：ADC_Get_All_Channle() 返回 [后, 前]
    return vals[1]  # 如果实际接线相反，改为 vals[0]


def set_motor(left_speed, right_speed):
    up.CDS_SetSpeed(1, left_speed)
    up.CDS_SetSpeed(2, right_speed)


def stop():
    set_motor(0, 0)


def forward():
    set_motor(LEFT_SPEED_FWD, RIGHT_SPEED_FWD)


def backward():
    set_motor(LEFT_SPEED_BACK, RIGHT_SPEED_BACK)
    time.sleep(BACK_DURATION)
    stop()
    time.sleep(0.3)


def turn_left():
    set_motor(TURN_SPEED_LEFT, TURN_SPEED_RIGHT)
    time.sleep(TURN_DURATION)
    stop()
    time.sleep(0.3)


# 可选右转函数，如果不需要可注释
def turn_right():
    set_motor(TURN_SPEED_RIGHT, TURN_SPEED_LEFT)
    time.sleep(TURN_DURATION)
    stop()
    time.sleep(0.3)


try:
    print("开始运行：前方遇黑线则后退并左转，按 Ctrl+C 退出")
    while True:
        gray = get_front_grayscale()
        print(f"前灰度: {gray}")

        if gray < THRESHOLD:  # 检测到黑线
            print("检测到黑线！")
            stop()
            time.sleep(0.5)
            backward()  # 后退
            turn_left()  # 左转（可改成 turn_right()）
            # 转弯后自动继续前进（循环继续）
        else:
            forward()

        time.sleep(0.05)  # 控制循环速度

except KeyboardInterrupt:
    stop()
    print("程序已停止")