#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
转弯时间调试工具
- 仅使用前向红外（IO0, IO1）和侧向红外（IO2, IO3）
- 车原地不动，检测到目标后执行一次转弯即停
- 转弯后等待固定延时再重新检测，避免连续触发
- 不含边缘保护、灰度巡台等其他逻辑
"""

import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()          # 电机控制
up.ADC_IO_Open()       # ADC 扩展板（IO）

# ========== 可调参数区 ==========
# 转弯速度（与原代码一致）
TURN_RIGHT_SPEED = 450   # 右轮正转速度（左转时使用）
# 原地左转：左轮 = -TURN_RIGHT_SPEED，右轮 = TURN_RIGHT_SPEED
# 原地右转：左轮 =  TURN_RIGHT_SPEED，右轮 = -TURN_RIGHT_SPEED

# ★ 以下三个时间就是你需要重点调试的 ★
TURN_DURATION    = 0.7   # 侧向红外触发时转弯时间（原 TURN_SIDE_DUR 也是0.5，这里统一用这个）
FRONT_TURN_TIME  = 0.2   # 前向单边触发时的微调转弯时间（原代码中为 0.2 秒）
WAIT_AFTER_TURN  = 0.5   # 转弯结束后等待的延时（秒），防止重复触发

# 注：原代码中侧向触发也用 TURN_SIDE_DUR=0.5，前向单边触发用 0.2，
#     这里用 FRONT_TURN_TIME 对应前向单边微调，TURN_DURATION 对应侧向大转角。
#     你可以根据实际效果分别修改它们。
# ================================

# ---------- 传感器读取 ----------
def read_io_bit(bit_index):
    """读取指定 IO 口的电平（0或1）"""
    val = up.ADC_IO_GetAllInputLevel()
    return (val >> bit_index) & 1

def read_front_ir():
    """返回左前(IO0), 右前(IO1) 的电平，0 表示检测到目标"""
    left  = read_io_bit(0)
    right = read_io_bit(1)
    return left, right

def read_side_ir():
    """返回左侧(IO2), 右侧(IO3) 的电平，0 表示检测到目标"""
    left  = read_io_bit(2)
    right = read_io_bit(3)
    return left, right

# ---------- 电机控制 ----------
def set_motors(left_speed, right_speed):
    """左电机通道2，右电机通道1"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

def turn_left():
    """原地左转：左轮倒转，右轮正转"""
    set_motors(-TURN_RIGHT_SPEED, TURN_RIGHT_SPEED)

def turn_right():
    """原地右转：左轮正转，右轮倒转"""
    set_motors(TURN_RIGHT_SPEED, -TURN_RIGHT_SPEED)

# ---------- 主测试循环 ----------
print("转弯调试程序启动")
print("前向双亮：不做动作")
print("左前亮：左转 {:.2f}s".format(FRONT_TURN_TIME))
print("右前亮：右转 {:.2f}s".format(FRONT_TURN_TIME))
print("左侧亮：左转 {:.2f}s".format(TURN_DURATION))
print("右侧亮：右转 {:.2f}s".format(TURN_DURATION))
print("转弯后等待 {:.2f}s 再检测".format(WAIT_AFTER_TURN))
print("按 Ctrl+C 停止程序")

try:
    while True:
        # 读取四个红外传感器
        fl, fr = read_front_ir()   # 0 = 有目标
        sl, sr = read_side_ir()    # 0 = 有目标

        # ---------- 按原代码优先级处理 ----------
        # 1. 前向双亮 → 不做动作（原代码会冲撞，这里忽略）
        if fl == 0 and fr == 0:
            print("前向双亮，忽略（调车时不动作）")
            time.sleep(WAIT_AFTER_TURN)   # 短暂延时避免刷屏
            continue

        # 2. 左前亮，右前不亮 → 左转微调
        if fl == 0 and fr == 1:
            print("左前触发 → 左转 {:.2f}s".format(FRONT_TURN_TIME))
            stop()
            turn_left()
            time.sleep(FRONT_TURN_TIME)
            stop()
            time.sleep(WAIT_AFTER_TURN)
            continue

        # 3. 右前亮，左前不亮 → 右转微调
        if fl == 1 and fr == 0:
            print("右前触发 → 右转 {:.2f}s".format(FRONT_TURN_TIME))
            stop()
            turn_right()
            time.sleep(FRONT_TURN_TIME)
            stop()
            time.sleep(WAIT_AFTER_TURN)
            continue

        # 4. 前向无目标时，检查侧面
        if fl == 1 and fr == 1:
            if sl == 0:   # 左侧有目标
                print("左侧触发 → 左转 {:.2f}s".format(TURN_DURATION))
                stop()
                turn_left()
                time.sleep(TURN_DURATION)
                stop()
                time.sleep(WAIT_AFTER_TURN)
                continue
            if sr == 0:   # 右侧有目标
                print("右侧触发 → 右转 {:.2f}s".format(TURN_DURATION))
                stop()
                turn_right()
                time.sleep(TURN_DURATION)
                stop()
                time.sleep(WAIT_AFTER_TURN)
                continue

        # 5. 完全没有目标 → 保持停止，短暂延时
        time.sleep(0.05)

except KeyboardInterrupt:
    stop()
    print("程序已停止")