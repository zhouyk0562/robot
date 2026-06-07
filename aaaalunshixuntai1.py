#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()          # 电机控制
up.ADC_IO_Open()       # ADC 扩展板（灰度传感器）

# ========== 可调参数区 ==========
# 四个方向独立的黑线阈值（小于对应值视为检测到边缘）
THRESHOLD_FRONT = 1400  # 前方灰度阈值
THRESHOLD_REAR  = 2200   # 后方灰度阈值
THRESHOLD_LEFT  = 1080   # 左侧灰度阈值
THRESHOLD_RIGHT = 1620   # 右侧灰度阈值

# 基础前进速度（左轮通道2，右轮通道1）
LEFT_SPEED_FWD  = 500
RIGHT_SPEED_FWD = 500

# 后退速度
LEFT_SPEED_BACK  = -500
RIGHT_SPEED_BACK = -500

# 原地转弯速度（左转：左轮倒转，右轮正转；右转相反）
TURN_LEFT_SPEED  = -450   # 转弯时左轮速度
TURN_RIGHT_SPEED = 450    # 转弯时右轮速度

# 动作持续时间（秒）
BACK_DURATION  = 0.5      # 后退时长
TURN_DURATION  = 0.5      # 转弯时长（角度微调）
# ================================

def read_all_grays():
    """
    读取四个灰度传感器
    接线映射：
        adc[0] -> 后方灰度
        adc[1] -> 前方灰度
        adc[2] -> 左侧灰度
        adc[3] -> 右侧灰度
    返回: (前, 后, 左, 右)
    """
    adc = up.ADC_Get_All_Channle()
    front = adc[1]   # 前方
    rear  = adc[0]   # 后方
    left  = adc[2]   # 左侧
    right = adc[3]   # 右侧
    return front, rear, left, right

def set_motors(left_speed, right_speed):
    """
    设置电机速度
    通道对应：
        左电机 -> CDS通道2
        右电机 -> CDS通道1
    """
    up.CDS_SetSpeed(2, left_speed)   # 左电机
    up.CDS_SetSpeed(1, right_speed)  # 右电机

def stop():
    """停车"""
    set_motors(0, 0)

def forward():
    """直行"""
    set_motors(LEFT_SPEED_FWD, RIGHT_SPEED_FWD)

def backward():
    """后退一段距离"""
    set_motors(LEFT_SPEED_BACK, RIGHT_SPEED_BACK)
    time.sleep(BACK_DURATION)
    stop()
    time.sleep(0.1)

def turn_left():
    """原地左转：左轮倒转，右轮正转"""
    set_motors(TURN_LEFT_SPEED, TURN_RIGHT_SPEED)
    time.sleep(TURN_DURATION)
    stop()
    time.sleep(0.1)

def turn_right():
    """原地右转：左轮正转，右轮倒转"""
    set_motors(TURN_RIGHT_SPEED, TURN_LEFT_SPEED)
    time.sleep(TURN_DURATION)
    stop()
    time.sleep(0.1)

# ---------- 主循环 ----------
try:
    print("巡台程序启动，按 Ctrl+C 退出")
    while True:
        front, rear, left, right = read_all_grays()
        print(f"前:{front:4d} 后:{rear:4d} 左:{left:4d} 右:{right:4d}")

        # 边缘检测：使用各自独立的阈值
        if front < THRESHOLD_FRONT:          # 前方边缘 → 后退再左转
            print("前方边缘！后退→左转")
            stop()
            time.sleep(0.1)
            backward()
            turn_left()

        elif left < THRESHOLD_LEFT:          # 左侧边缘 → 右转
            print("左侧边缘！右转")
            stop()
            time.sleep(0.1)
            turn_right()

        elif right < THRESHOLD_RIGHT:        # 右侧边缘 → 左转
            print("右侧边缘！左转")
            stop()
            time.sleep(0.1)
            turn_left()

        elif rear < THRESHOLD_REAR:          # 后方边缘 → 前进脱离
            print("后方边缘！前进")
            stop()
            time.sleep(0.1)
            forward()
            time.sleep(0.3)

        else:                                # 安全区域 → 直行
            forward()

        time.sleep(0.05)                     # 循环节拍

except KeyboardInterrupt:
    stop()
    print("程序已停止")