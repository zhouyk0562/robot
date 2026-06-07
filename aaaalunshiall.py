#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整巡台 + 目标追踪程序
功能：
- 灰度巡台（前/后/左/右边缘检测）
- 红外目标追踪（前向双亮冲撞，单边慢速引入，侧向快速引入并可被打断）
- 边缘悬空恢复（独立速度，左悬空右转，右悬空左转，双悬空仅后退）
- 掉台恢复（步进扫描 + 冲台）
- 所有关键速度/时间均可独立调节
- 启动时等待左右红外同时检测到目标，然后全速后退固定时长后进入主循环
"""

import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ========== 可调参数区（详细注释） ==========

# ---- 灰度传感器阈值（小于阈值视为检测到边缘/黑线） ----
THRESHOLD_FRONT = 1480
THRESHOLD_REAR  = 2278
THRESHOLD_LEFT  = 1280
THRESHOLD_RIGHT = 1620

# ---- 掉台判断阈值 ----
THRESHOLD_OFF_TABLE = 1500

# ---- 基础行驶速度 ----
LEFT_SPEED_FWD   = 600
RIGHT_SPEED_FWD  = 600
LEFT_SPEED_BACK  = -600
RIGHT_SPEED_BACK = -600

# ---- 通用转向速度（灰度巡台、掉台扫描等） ----
TURN_RIGHT_SPEED = 600

# ---- 目标追踪专用转向速度 ----
TRACK_TURN_SPEED = 450
SIDE_TRACK_SPEED = 600

# ---- 边缘悬空恢复专用速度及时间（独立可调） ----
EDGE_TURN_SPEED = 600
EDGE_BACK_TIME   = 0.5
EDGE_TURN_TIME   = 0.5

# ---- 动作持续时间 ----
BACK_DURATION   = 0.5
TURN_DURATION   = 0.5
TURN_SIDE_DUR   = 0.7
RAM_DURATION    = 0.1
CYCLE_DELAY     = 0.02

# ---- 掉台恢复步进扫描参数 ----
OFF_TABLE_FORWARD_TIME  = 0.8
OFF_TABLE_RUSH_TIME     = 1.5
OFF_TABLE_STEP_TURN_TIME = 0.4
OFF_TABLE_STEP_BACK_TIME = 0.4

# ---- 启动阶段参数（固定时长后退） ----
START_BACK_SPEED = -1000        # 全速后退速度（负值）
START_BACK_DURATION = 2.0       # 固定后退时长（秒），可根据需要调整
# ================================

# ---------- 传感器读取 ----------
def read_all_grays():
    adc = up.ADC_Get_All_Channle()
    return adc[1], adc[0], adc[2], adc[3]

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

def track_turn_left():
    set_motors(-TRACK_TURN_SPEED, TRACK_TURN_SPEED)

def track_turn_right():
    set_motors(TRACK_TURN_SPEED, -TRACK_TURN_SPEED)

def side_fast_left():
    set_motors(-SIDE_TRACK_SPEED, SIDE_TRACK_SPEED)

def side_fast_right():
    set_motors(SIDE_TRACK_SPEED, -SIDE_TRACK_SPEED)

def edge_turn_left():
    set_motors(-EDGE_TURN_SPEED, EDGE_TURN_SPEED)

def edge_turn_right():
    set_motors(EDGE_TURN_SPEED, -EDGE_TURN_SPEED)

# ---------- 可中断的侧向引入 ----------
def side_track_interruptible(direction):
    if direction == 'left':
        side_fast_left()
    else:
        side_fast_right()

    start = time.time()
    while time.time() - start < TURN_SIDE_DUR:
        fl, fr = read_front_ir()
        if fl == 0 or fr == 0:
            stop()
            return True
        time.sleep(0.02)
    stop()
    return False

# ---------- 前向持续引入 ----------
def track_front_until_double(direction):
    if direction == 'left':
        track_turn_left()
    else:
        track_turn_right()

    while True:
        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:
            stop()
            return True
        if fl == 1 and fr == 1:
            stop()
            return False
        time.sleep(0.03)

# ---------- 灰度巡台 ----------
def grayscale_follow(front, rear, left, right):
    if front < THRESHOLD_FRONT:
        stop()
        backward()
        time.sleep(BACK_DURATION)
        turn_left()
        time.sleep(TURN_DURATION)
        stop()
    elif left < THRESHOLD_LEFT:
        stop()
        turn_right()
        time.sleep(TURN_DURATION)
        stop()
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

# ---------- 边缘悬空恢复 ----------
def recover_from_edge(l_edge, r_edge):
    print(f"边缘触发！ L={l_edge} R={r_edge}")
    stop()
    backward()
    time.sleep(EDGE_BACK_TIME)
    stop()
    time.sleep(0.1)

    if l_edge == 1 and r_edge == 0:
        print("左侧悬空，向右大角度转")
        edge_turn_right()
        time.sleep(EDGE_TURN_TIME)
        stop()
    elif l_edge == 0 and r_edge == 1:
        print("右侧悬空，向左大角度转")
        edge_turn_left()
        time.sleep(EDGE_TURN_TIME)
        stop()
    elif l_edge == 1 and r_edge == 1:
        print("双悬空，仅后退，不转向")
    time.sleep(0.2)

# ---------- 掉台判断辅助函数（主循环继续使用）----------
def is_off_table(fg, rg, lg, rgg):
    """四路灰度全部低于掉台阈值则视为掉台"""
    return (fg < THRESHOLD_OFF_TABLE and rg < THRESHOLD_OFF_TABLE and
            lg < THRESHOLD_OFF_TABLE and rgg < THRESHOLD_OFF_TABLE)

# ---------- 掉台恢复 ----------
def off_table_recovery():
    print("掉台！开始步进旋转扫描...")
    stop()
    time.sleep(0.1)

    while True:
        turn_left()
        time.sleep(OFF_TABLE_STEP_TURN_TIME)
        stop()
        time.sleep(0.05)

        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:
            print("前向双亮，对准完成")
            break

        backward()
        time.sleep(OFF_TABLE_STEP_BACK_TIME)
        stop()
        time.sleep(0.1)

    print(f"前进 {OFF_TABLE_FORWARD_TIME} 秒靠近台面")
    forward()
    time.sleep(OFF_TABLE_FORWARD_TIME)
    stop()
    time.sleep(0.1)

    print(f"全力后退冲台 {OFF_TABLE_RUSH_TIME} 秒")
    set_motors(-1000, -1000)
    time.sleep(OFF_TABLE_RUSH_TIME)
    stop()
    time.sleep(0.2)

    fg, rg, lg, rgg = read_all_grays()
    if not is_off_table(fg, rg, lg, rgg):
        print("成功上台，恢复巡台")
    else:
        print("冲台未完全到位，继续主循环")

# ---------- 冲撞 ----------
def ram_forward_until_safe():
    forward()
    while True:
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge(edge_l, edge_r)
            return
        fl, fr = read_front_ir()
        if fl == 1 and fr == 1:
            stop()
            print("目标丢失，停止冲撞")
            return
        time.sleep(RAM_DURATION)

# ========== 主程序 ==========
print("程序启动：等待左右红外同时检测到目标...")
try:
    # ------- 启动等待与固定时长后退 -------
    while True:
        sl_start, sr_start = read_side_ir()
        if sl_start == 0 and sr_start == 0:
            print(f"左右同时检测到目标，全速后退 {START_BACK_DURATION} 秒")
            set_motors(START_BACK_SPEED, START_BACK_SPEED)
            time.sleep(START_BACK_DURATION)      # 固定时长后退
            stop()
            print("后退完成，进入主循环")
            break
        else:
            # 未同时检测到，原地等待
            time.sleep(0.05)

    # ------- 主循环 -------
    while True:
        fg, rg, lg, rgg = read_all_grays()
        # 最高优先级：掉台判断（调用统一函数）
        if is_off_table(fg, rg, lg, rgg):
            off_table_recovery()
            continue

        # 第二优先级：边缘悬空
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge(edge_l, edge_r)
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
            print("左前发现目标，慢速左转引入")
            track_front_until_double('left')
            continue

        if fl == 1 and fr == 0:
            print("右前发现目标，慢速右转引入")
            track_front_until_double('right')
            continue

        if fl == 1 and fr == 1:
            if sl == 0:
                print("左侧发现目标，快速左转（可打断）")
                interrupted = side_track_interruptible('left')
                if interrupted:
                    continue
                else:
                    time.sleep(0.2)
                    continue

            if sr == 0:
                print("右侧发现目标，快速右转（可打断）")
                interrupted = side_track_interruptible('right')
                if interrupted:
                    continue
                else:
                    time.sleep(0.2)
                    continue

        # 无红外目标，灰度巡台
        grayscale_follow(fg, rg, lg, rgg)
        time.sleep(CYCLE_DELAY)

except KeyboardInterrupt:
    stop()
    print("程序已停止")