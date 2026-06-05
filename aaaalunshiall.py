#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ========== 可调参数区 ==========
# 灰度传感器阈值（小于对应值视为检测到边缘）
THRESHOLD_FRONT = 1450
THRESHOLD_REAR  = 2200
THRESHOLD_LEFT  = 1080
THRESHOLD_RIGHT = 1620

# ★ 掉台判断阈值：四个灰度值全部小于此值，认为车身已掉下台面
THRESHOLD_OFF_TABLE = 1750

# 电机速度
LEFT_SPEED_FWD   = 500
RIGHT_SPEED_FWD  = 500
LEFT_SPEED_BACK  = -500
RIGHT_SPEED_BACK = -500
TURN_LEFT_SPEED  = -450
TURN_RIGHT_SPEED = 450

# 动作时间
BACK_DURATION   = 0.5
TURN_DURATION   = 0.5
TURN_SIDE_DUR   = 0.5
RAM_DURATION    = 0.1
CYCLE_DELAY     = 0.05

# 边缘恢复参数
RECOVER_TURN_SPEED = 500
RECOVER_CHECK_DELAY = 0.05

# ★ 掉台恢复参数
OFF_TABLE_FORWARD_TIME = 0.8    # 双亮后前进时长
OFF_TABLE_RUSH_TIME    = 1.5    # 全力后退冲台时长
OFF_TABLE_STEP_TURN_TIME = 0.3  # 步进旋转每次转动的时间
OFF_TABLE_STEP_BACK_TIME = 0.3  # 步进旋转每次后退的时间
# ================================

# ---------- 传感器读取 ----------
def read_all_grays():
    adc = up.ADC_Get_All_Channle()
    return adc[1], adc[0], adc[2], adc[3]  # 前,后,左,右

def read_io_bit(bit_index):
    val = up.ADC_IO_GetAllInputLevel()
    return (val >> bit_index) & 1

def read_edge_ir():
    return read_io_bit(4), read_io_bit(6)

def read_front_ir():
    return read_io_bit(0), read_io_bit(1)

def read_side_ir():
    return read_io_bit(2), read_io_bit(3)

# ---------- 电机控制 ----------
def set_motors(left_speed, right_speed):
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

def forward():
    set_motors(LEFT_SPEED_FWD, RIGHT_SPEED_FWD)

def backward():
    set_motors(LEFT_SPEED_BACK, RIGHT_SPEED_BACK)

def turn_left():
    set_motors(-TURN_RIGHT_SPEED, TURN_RIGHT_SPEED)

def turn_right():
    set_motors(TURN_RIGHT_SPEED, -TURN_RIGHT_SPEED)

# ---------- 掉台恢复（步进旋转+后退）----------
def off_table_recovery():
    """
    掉台恢复（步进旋转+后退）：
    1. 左转一小段时间，检查前向双红外
    2. 未对准则后退一小段，重复旋转
    3. 对准后前进靠近台面，全速后退冲台
    """
    print("掉台！开始步进旋转扫描...")
    stop()
    time.sleep(0.1)

    while True:
        # 左转一个步进角度
        turn_left()
        time.sleep(OFF_TABLE_STEP_TURN_TIME)
        stop()
        time.sleep(0.05)

        # 检查前向红外是否双亮
        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:
            print("前向双亮，对准完成")
            break

        # 后退一小段，调整位置
        backward()
        time.sleep(OFF_TABLE_STEP_BACK_TIME)
        stop()
        time.sleep(0.1)

    # 2. 前进一小段（靠近台面）
    print(f"前进 {OFF_TABLE_FORWARD_TIME} 秒")
    forward()
    time.sleep(OFF_TABLE_FORWARD_TIME)
    stop()
    time.sleep(0.1)

    # 3. 全速后退冲台
    print(f"全力后退冲台 {OFF_TABLE_RUSH_TIME} 秒")
    set_motors(-1000, -1000)
    time.sleep(OFF_TABLE_RUSH_TIME)
    stop()
    time.sleep(0.2)

    # 验证是否回到台上
    fg, rg, lg, rgg = read_all_grays()
    if (fg >= THRESHOLD_FRONT and rg >= THRESHOLD_REAR and
        lg >= THRESHOLD_LEFT and rgg >= THRESHOLD_RIGHT):
        print("成功上台，恢复巡台")
    else:
        print("冲台未完全到位，但仍继续正常流程")

# ---------- 灰度巡台逻辑 ----------
def grayscale_follow(front, rear, left, right):
    if front < THRESHOLD_FRONT:
        stop()
        backward()
        time.sleep(BACK_DURATION)
        turn_left()
        time.sleep(TURN_DURATION)
    elif right < THRESHOLD_RIGHT:
        stop()
        turn_left()
        time.sleep(TURN_DURATION)
        stop()
    elif rear < THRESHOLD_REAR:
        stop()
        forward()
        time.sleep(0.3)
    else:
        forward()

# ---------- 边缘恢复 ----------
def recover_from_edge():
    print("边缘触发！开始恢复...")
    stop()
    backward()
    time.sleep(0.7)
    stop()
    time.sleep(0.1)

    turn_left()
    while True:
        front, rear, left, right = read_all_grays()
        if (front >= THRESHOLD_FRONT and rear >= THRESHOLD_REAR and
            left >= THRESHOLD_LEFT and right >= THRESHOLD_RIGHT):
            stop()
            print("已恢复到安全位置")
            break
        time.sleep(RECOVER_CHECK_DELAY)
    time.sleep(0.2)

# ---------- 冲撞（带边缘保护）----------
def ram_forward_until_safe():
    forward()
    while True:
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge()
            return
        fl, fr = read_front_ir()
        if fl == 1 and fr == 1:
            stop()
            print("目标丢失，停止冲撞")
            return
        time.sleep(RAM_DURATION)

# ---------- 主循环 ----------
print("程序启动：灰度巡台 + 红外防边缘/目标追踪 + 掉台恢复（步进旋转+后退）")
try:
    while True:
        # ★ 最高优先级：掉台判断
        fg, rg, lg, rgg = read_all_grays()
        if (fg < THRESHOLD_OFF_TABLE and rg < THRESHOLD_OFF_TABLE and
            lg < THRESHOLD_OFF_TABLE and rgg < THRESHOLD_OFF_TABLE):
            off_table_recovery()
            continue

        # 第二优先级：边缘悬空
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge()
            continue

        # 读取红外目标
        fl, fr = read_front_ir()
        sl, sr = read_side_ir()

        # 目标追踪
        if fl == 0 and fr == 0:
            print("前向锁定目标，冲撞！")
            ram_forward_until_safe()
            continue

        if fl == 0 and fr == 1:
            print("左前发现目标，左转调整")
            stop()
            turn_left()
            time.sleep(0.2)
            stop()
            continue

        if fl == 1 and fr == 0:
            print("右前发现目标，右转调整")
            stop()
            turn_right()
            time.sleep(0.2)
            stop()
            continue

        if fl == 1 and fr == 1:
            if sl == 0:
                print("左侧发现目标，左转引入")
                stop()
                turn_left()
                time.sleep(TURN_SIDE_DUR)
                stop()
                continue
            if sr == 0:
                print("右侧发现目标，右转引入")
                stop()
                turn_right()
                time.sleep(TURN_SIDE_DUR)
                stop()
                continue

        # 无红外目标 → 灰度巡台
        grayscale_follow(fg, rg, lg, rgg)
        time.sleep(CYCLE_DELAY)

except KeyboardInterrupt:
    stop()
    print("程序已停止")