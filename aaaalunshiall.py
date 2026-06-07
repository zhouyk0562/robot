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
"""

import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ========== 可调参数区（详细注释） ==========

# ---- 灰度传感器阈值（小于阈值视为检测到边缘/黑线） ----
THRESHOLD_FRONT = 1480          # 前方灰度阈值（越小越敏感，检测到黑线会后退左转）
THRESHOLD_REAR  = 2278          # 后方灰度阈值（检测到后边缘则前进）
THRESHOLD_LEFT  = 1280          # 左侧灰度阈值（检测到左边缘则右转）
THRESHOLD_RIGHT = 1620          # 右侧灰度阈值（检测到右边缘则左转）

# ---- 掉台判断阈值 ----
THRESHOLD_OFF_TABLE = 1500      # 四路灰度全部低于此值，认为车身已掉下台面（触发掉台恢复）

# ---- 基础行驶速度 ----
LEFT_SPEED_FWD   = 600          # 前进时左轮速度
RIGHT_SPEED_FWD  = 600          # 前进时右轮速度
LEFT_SPEED_BACK  = -600         # 后退时左轮速度（负值）
RIGHT_SPEED_BACK = -600         # 后退时右轮速度

# ---- 通用转向速度（灰度巡台、掉台扫描等） ----
TURN_RIGHT_SPEED = 600         # 原地转向时两轮速度的绝对值

# ---- 目标追踪专用转向速度 ----
TRACK_TURN_SPEED = 500          # 前向单边触发时慢速对准的速度（精确调整，不宜过大）
SIDE_TRACK_SPEED = 600          # 侧向红外触发时快速引入的速度（可被打断）

# ---- 边缘悬空恢复专用速度及时间（独立可调） ----
EDGE_TURN_SPEED = 600           # 单侧悬空时转向脱困的速度
EDGE_BACK_TIME   = 0.5          # 触发边缘后先后退的时间（远离边缘）
EDGE_TURN_TIME   = 0.5          # 单侧悬空时转向的大角度时间（左悬空右转，右悬空左转）

# ---- 动作持续时间 ----
BACK_DURATION   = 0.5           # 灰度巡台检测到前边缘时后退的时间
TURN_DURATION   = 0.5           # 灰度巡台检测到边缘后转弯的时间
TURN_SIDE_DUR   = 0.7           # 侧向红外触发后快速转弯的最大持续时间（超时则停）
RAM_DURATION    = 0.1           # 冲撞过程中循环检测的间隔时间（秒）
CYCLE_DELAY     = 0.02          # 主循环无红外目标时的节拍延时（秒）

# ---- 掉台恢复步进扫描参数 ----
OFF_TABLE_FORWARD_TIME  = 0.8   # 对准台面后前进靠近的时间
OFF_TABLE_RUSH_TIME     = 1.5   # 全力后退冲上台面的时间
OFF_TABLE_STEP_TURN_TIME = 0.3  # 掉台扫描时每次旋转的时间
OFF_TABLE_STEP_BACK_TIME = 0.4  # 掉台扫描时每次后退的时间
# ================================

# ---------- 传感器读取 ----------
def read_all_grays():
    """返回：前, 后, 左, 右（adc[1], adc[0], adc[2], adc[3]）"""
    adc = up.ADC_Get_All_Channle()
    return adc[1], adc[0], adc[2], adc[3]

def read_io_bit(bit_index):
    val = up.ADC_IO_GetAllInputLevel()
    return (val >> bit_index) & 1

def read_edge_ir():
    """返回左下(IO4), 右下(IO6) 电平，1=悬空"""
    return read_io_bit(4), read_io_bit(6)

def read_front_ir():
    """返回左前(IO0), 右前(IO1) 电平，0=检测到目标"""
    return read_io_bit(0), read_io_bit(1)

def read_side_ir():
    """返回左侧(IO2), 右侧(IO3) 电平，0=检测到目标"""
    return read_io_bit(2), read_io_bit(3)

# ---------- 电机控制 ----------
def set_motors(left_speed, right_speed):
    """左电机通道2，右电机通道1"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

def forward():
    set_motors(LEFT_SPEED_FWD, RIGHT_SPEED_FWD)

def backward():
    set_motors(LEFT_SPEED_BACK, RIGHT_SPEED_BACK)

# ---- 通用转向（灰度巡台、掉台扫描等） ----
def turn_left():
    set_motors(-TURN_RIGHT_SPEED, TURN_RIGHT_SPEED)

def turn_right():
    set_motors(TURN_RIGHT_SPEED, -TURN_RIGHT_SPEED)

# ---- 目标追踪专用转向 ----
def track_turn_left():
    """慢速左转（前向对准）"""
    set_motors(-TRACK_TURN_SPEED, TRACK_TURN_SPEED)

def track_turn_right():
    """慢速右转（前向对准）"""
    set_motors(TRACK_TURN_SPEED, -TRACK_TURN_SPEED)

def side_fast_left():
    """快速左转（侧向引入）"""
    set_motors(-SIDE_TRACK_SPEED, SIDE_TRACK_SPEED)

def side_fast_right():
    """快速右转（侧向引入）"""
    set_motors(SIDE_TRACK_SPEED, -SIDE_TRACK_SPEED)

# ---- 边缘悬空专用转向 ----
def edge_turn_left():
    set_motors(-EDGE_TURN_SPEED, EDGE_TURN_SPEED)

def edge_turn_right():
    set_motors(EDGE_TURN_SPEED, -EDGE_TURN_SPEED)

# ---------- 可中断的侧向引入 ----------
def side_track_interruptible(direction):
    """
    快速侧向转弯，同时持续监测前向红外。
    任意一个前向红外检测到目标（0）立即停止。
    direction: 'left' 或 'right'
    返回: True 表示被前向目标打断，False 表示超时结束。
    """
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

# ---------- 前向持续引入（慢速，直到双亮或丢失）----------
def track_front_until_double(direction):
    """
    慢速持续转弯，直到前向双亮或完全丢失。
    direction: 'left' 或 'right'
    返回: True 表示成功引入（双亮），False 表示目标丢失。
    """
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

# ---------- 边缘悬空恢复（新逻辑，速度/时间独立）----------
def recover_from_edge(l_edge, r_edge):
    """
    根据下边缘红外状态执行脱困。
    - 左侧悬空 -> 后退 + 向右大角度转
    - 右侧悬空 -> 后退 + 向左大角度转
    - 双悬空   -> 仅后退
    """
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

# ---------- 掉台恢复（步进旋转扫描 + 冲台）----------
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
    if (fg >= THRESHOLD_FRONT and rg >= THRESHOLD_REAR and
        lg >= THRESHOLD_LEFT and rgg >= THRESHOLD_RIGHT):
        print("成功上台，恢复巡台")
    else:
        print("冲台未完全到位，继续主循环")

# ---------- 冲撞（带边缘保护）----------
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

# ========== 主循环 ==========
print("程序启动：掉台恢复 | 边缘独立速度 | 侧向快速可打断 | 前向慢速对准")
try:
    while True:
        # 最高优先级：掉台判断
        fg, rg, lg, rgg = read_all_grays()
        if (fg < THRESHOLD_OFF_TABLE and rg < THRESHOLD_OFF_TABLE and
            lg < THRESHOLD_OFF_TABLE and rgg < THRESHOLD_OFF_TABLE):
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

        # ---- 目标追踪 ----
        # 前向双亮 → 冲撞
        if fl == 0 and fr == 0:
            print("前向锁定目标，冲撞！")
            ram_forward_until_safe()
            continue

        # 左前单亮 → 慢速左转直到双亮或丢失
        if fl == 0 and fr == 1:
            print("左前发现目标，慢速左转引入")
            track_front_until_double('left')
            continue

        # 右前单亮 → 慢速右转直到双亮或丢失
        if fl == 1 and fr == 0:
            print("右前发现目标，慢速右转引入")
            track_front_until_double('right')
            continue

        # 前向无目标，检查侧面
        if fl == 1 and fr == 1:
            if sl == 0:
                print("左侧发现目标，快速左转（可打断）")
                interrupted = side_track_interruptible('left')
                if interrupted:
                    continue   # 被前向打断，直接下一轮循环（前向逻辑接管）
                else:
                    time.sleep(0.2)  # 超时消抖
                    continue

            if sr == 0:
                print("右侧发现目标，快速右转（可打断）")
                interrupted = side_track_interruptible('right')
                if interrupted:
                    continue
                else:
                    time.sleep(0.2)
                    continue

        # 无红外目标，执行灰度巡台
        grayscale_follow(fg, rg, lg, rgg)
        time.sleep(CYCLE_DELAY)

except KeyboardInterrupt:
    stop()
    print("程序已停止")