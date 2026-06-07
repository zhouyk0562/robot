#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红外边缘巡台 + 红外目标追踪 + 自动发车 + 掉台恢复程序
功能：
- 发车：启动后等待左右侧向红外同时检测到目标，全速后退固定时长上台
- 掉台恢复：灰度四路低于阈值视为掉台，扫描搜索台面，找到后前进冲撞再全力后退上台
- 巡台：底部红外（IO4/IO6）边缘悬空脱困
- 追踪：前向红外（IO0/IO1）和侧向红外（IO2/IO3）目标追踪
- 所有速度、时间、方向参数均可独立调节
"""

import uptech
import time

# ---------- 硬件初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ==================== 可调参数区（所有参数独立控制） ====================

# ======== 发车参数 ========
START_SIDE_DETECT_TIMEOUT = 0.05    # 发车等待中，每次检测的间隔（秒）
START_BACK_SPEED = -1000            # 发车全速后退速度（负值，表示全力后退）
START_BACK_DURATION =0.7            # 发车后退持续时间（秒）

# ======== 掉台恢复参数 ========
# ---- 灰度传感器掉台阈值（四路均低于此值视为掉台） ----
OFF_TABLE_THRESHOLD = 1500          # 掉台灰度阈值

# ---- 掉台恢复动作速度 ----
RECOVERY_FORWARD_SPEED = 700        # 掉台恢复时前进撞台速度（正值）
RECOVERY_BACK_SPEED = -1000         # 掉台恢复时全力后退速度（负值）

# ---- 掉台恢复动作时间 ----
RECOVERY_FORWARD_TIME = 0.8         # 前进撞台时间（秒）
RECOVERY_BACK_TIME =1.0            # 全力后退上台时间（秒）

# ---- 掉台扫描参数 ----
SCAN_FORWARD_TIME = 0.5             # 每次扫描前的前进时间（秒）
SCAN_TURN_SPEED = 600               # 扫描旋转速度（绝对值）
SCAN_TURN_DIRECTION = 'right'       # 扫描旋转固定方向：'left' 或 'right'
SCAN_TURN_STEP_TIME = 0.4           # 每次旋转步进时间（秒），控制扫描角度
SCAN_CHECK_DELAY = 0.02             # 旋转过程中检测间隔（秒）

# ======== 巡台直行速度 ========
PATROL_FORWARD_LEFT  = 600
PATROL_FORWARD_RIGHT = 600

# ======== 巡台通用后退速度 ========
PATROL_BACK_LEFT  = -800
PATROL_BACK_RIGHT = -800

# ======== 巡台转向速度 ========
PATROL_TURN_SPEED = 700

# ======== 巡台边缘悬空恢复时间参数 ========
PATROL_BACK_TIME = 0.4
LEFT_SUSPEND_TURN_TIME = 0.7
RIGHT_SUSPEND_TURN_TIME = 0.7
DOUBLE_SUSPEND_TURN_TIME = 0.9
DOUBLE_SUSPEND_TURN_DIR = 'right'

# ======== 巡台动作间隔延时 ========
PATROL_STOP_PAUSE = 0.1
CYCLE_DELAY = 0.02

# ======== 前向目标慢速引入转向速度 ========
SLOW_INTRO_SPEED = 500

# ======== 侧向目标快速引入速度 ========
SIDE_FAST_SPEED = 700

# ======== 侧向快速引入持续时间 ========
SIDE_TRACK_DUR = 0.7

# ======== 前向双亮冲撞速度 ========
RAM_SPEED_LEFT  = 800
RAM_SPEED_RIGHT = 800

# ======== 冲撞专用边缘恢复参数（独立） ========
RAM_BACK_SPEED_LEFT  = -600
RAM_BACK_SPEED_RIGHT = -600
RAM_BACK_TIME = 0.5
RAM_LEFT_SUSPEND_TURN_TIME = 0.7
RAM_RIGHT_SUSPEND_TURN_TIME = 0.7
RAM_DOUBLE_SUSPEND_TURN_TIME = 0.9
RAM_DOUBLE_SUSPEND_TURN_DIR = 'right'
RAM_TURN_SPEED = 700
RAM_CHECK_DELAY = 0.03

# ==================== 传感器读取 ====================
def read_edge_ir():
    """底部红外，返回 (左悬空, 右悬空)  1=悬空, 0=在台面"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 4) & 1   # IO4
    right = (all_in >> 6) & 1   # IO6
    return left, right

def read_front_ir():
    """前向红外，返回 (左前, 右前)  0=检测到目标, 1=未检测到"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 0) & 1   # IO0
    right = (all_in >> 1) & 1   # IO1
    return left, right

def read_side_ir():
    """侧向红外，返回 (左侧, 右侧)  0=检测到目标, 1=未检测到"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 2) & 1   # IO2
    right = (all_in >> 3) & 1   # IO3
    return left, right

def read_all_grays():
    """灰度传感器，返回 (前, 后, 左, 右) 灰度值"""
    adc = up.ADC_Get_All_Channle()
    return adc[1], adc[0], adc[2], adc[3]

# ==================== 电机控制 ====================
def set_motors(left_speed, right_speed):
    """设置左右电机（左通道2，右通道1）"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

# ----- 巡台基本动作 -----
def patrol_forward():
    set_motors(PATROL_FORWARD_LEFT, PATROL_FORWARD_RIGHT)

def patrol_backward():
    set_motors(PATROL_BACK_LEFT, PATROL_BACK_RIGHT)

def patrol_turn_left(duration):
    set_motors(-PATROL_TURN_SPEED, PATROL_TURN_SPEED)
    time.sleep(duration)
    stop()

def patrol_turn_right(duration):
    set_motors(PATROL_TURN_SPEED, -PATROL_TURN_SPEED)
    time.sleep(duration)
    stop()

# ----- 冲撞专用动作 -----
def ram_forward():
    set_motors(RAM_SPEED_LEFT, RAM_SPEED_RIGHT)

def ram_backward():
    set_motors(RAM_BACK_SPEED_LEFT, RAM_BACK_SPEED_RIGHT)

def ram_turn_left(duration):
    set_motors(-RAM_TURN_SPEED, RAM_TURN_SPEED)
    time.sleep(duration)
    stop()

def ram_turn_right(duration):
    set_motors(RAM_TURN_SPEED, -RAM_TURN_SPEED)
    time.sleep(duration)
    stop()

# ----- 目标追踪专用动作 -----
def slow_turn_left():
    set_motors(-SLOW_INTRO_SPEED, SLOW_INTRO_SPEED)

def slow_turn_right():
    set_motors(SLOW_INTRO_SPEED, -SLOW_INTRO_SPEED)

def fast_turn_left():
    set_motors(-SIDE_FAST_SPEED, SIDE_FAST_SPEED)

def fast_turn_right():
    set_motors(SIDE_FAST_SPEED, -SIDE_FAST_SPEED)

# ----- 掉台恢复专用动作 -----
def recovery_forward():
    """掉台恢复前进撞台"""
    set_motors(RECOVERY_FORWARD_SPEED, RECOVERY_FORWARD_SPEED)

def recovery_backward():
    """掉台恢复全力后退"""
    set_motors(RECOVERY_BACK_SPEED, RECOVERY_BACK_SPEED)

def scan_turn(direction):
    """扫描旋转，不延时，由调用者控制时间"""
    if direction == 'left':
        set_motors(-SCAN_TURN_SPEED, SCAN_TURN_SPEED)
    else:
        set_motors(SCAN_TURN_SPEED, -SCAN_TURN_SPEED)

# ==================== 边缘悬空恢复（巡台通用） ====================
def patrol_recover_from_edge(left, right):
    stop()
    print(f"[巡台恢复] 边缘触发 L={left} R={right}")
    patrol_backward()
    time.sleep(PATROL_BACK_TIME)
    stop()
    time.sleep(PATROL_STOP_PAUSE)

    if left == 1 and right == 0:
        print(f"  左侧悬空，向右转 {LEFT_SUSPEND_TURN_TIME} 秒")
        patrol_turn_right(LEFT_SUSPEND_TURN_TIME)
    elif left == 0 and right == 1:
        print(f"  右侧悬空，向左转 {RIGHT_SUSPEND_TURN_TIME} 秒")
        patrol_turn_left(RIGHT_SUSPEND_TURN_TIME)
    else:
        print(f"  双侧悬空，向{DOUBLE_SUSPEND_TURN_DIR}转 {DOUBLE_SUSPEND_TURN_TIME} 秒")
        if DOUBLE_SUSPEND_TURN_DIR == 'left':
            patrol_turn_left(DOUBLE_SUSPEND_TURN_TIME)
        else:
            patrol_turn_right(DOUBLE_SUSPEND_TURN_TIME)
    time.sleep(PATROL_STOP_PAUSE)

# ==================== 冲撞专用边缘恢复 ====================
def ram_recover_from_edge(left, right):
    stop()
    print(f"[冲撞恢复] 边缘触发 L={left} R={right}")
    ram_backward()
    time.sleep(RAM_BACK_TIME)
    stop()
    time.sleep(PATROL_STOP_PAUSE)

    if left == 1 and right == 0:
        print(f"  左侧悬空，向右转 {RAM_LEFT_SUSPEND_TURN_TIME} 秒")
        ram_turn_right(RAM_LEFT_SUSPEND_TURN_TIME)
    elif left == 0 and right == 1:
        print(f"  右侧悬空，向左转 {RAM_RIGHT_SUSPEND_TURN_TIME} 秒")
        ram_turn_left(RAM_RIGHT_SUSPEND_TURN_TIME)
    else:
        print(f"  双侧悬空，向{RAM_DOUBLE_SUSPEND_TURN_DIR}转 {RAM_DOUBLE_SUSPEND_TURN_TIME} 秒")
        if RAM_DOUBLE_SUSPEND_TURN_DIR == 'left':
            ram_turn_left(RAM_DOUBLE_SUSPEND_TURN_TIME)
        else:
            ram_turn_right(RAM_DOUBLE_SUSPEND_TURN_TIME)
    time.sleep(PATROL_STOP_PAUSE)

# ==================== 目标追踪函数（均含边缘检测） ====================
def slow_intro_with_edge(direction):
    if direction == 'left':
        slow_turn_left()
    else:
        slow_turn_right()

    while True:
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            patrol_recover_from_edge(l_edge, r_edge)
            return 'edge'

        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:
            stop()
            return 'double'
        if fl == 1 and fr == 1:
            stop()
            return 'lost'
        time.sleep(0.03)

def side_track_with_edge(direction):
    if direction == 'left':
        fast_turn_left()
    else:
        fast_turn_right()

    start = time.time()
    while time.time() - start < SIDE_TRACK_DUR:
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            stop()
            patrol_recover_from_edge(l_edge, r_edge)
            return 'interrupted'

        fl, fr = read_front_ir()
        if fl == 0 or fr == 0:
            stop()
            return 'interrupted'

        time.sleep(0.02)
    stop()
    return 'finished'

def ram_forward_until_safe():
    ram_forward()
    while True:
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            ram_recover_from_edge(l_edge, r_edge)
            return

        fl, fr = read_front_ir()
        if fl == 1 and fr == 1:
            stop()
            print("目标丢失，停止冲撞")
            return
        time.sleep(RAM_CHECK_DELAY)

# ==================== 掉台判断与恢复 ====================
def is_off_table():
    """四路灰度全部低于掉台阈值则返回True"""
    fg, rg, lg, rgg = read_all_grays()
    return (fg < OFF_TABLE_THRESHOLD and rg < OFF_TABLE_THRESHOLD and
            lg < OFF_TABLE_THRESHOLD and rgg < OFF_TABLE_THRESHOLD)

def off_table_recovery():
    """掉台恢复：扫描寻找台面，冲撞上台"""
    print("掉台！开始恢复...")
    stop()
    time.sleep(0.1)

    while True:
        # 先检查前向红外是否双亮
        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:
            print("发现台面（前向双亮），前进撞台后全力后退")
            recovery_forward()
            time.sleep(RECOVERY_FORWARD_TIME)
            recovery_backward()
            time.sleep(RECOVERY_BACK_TIME)
            stop()
            # 检查是否上台成功
            if not is_off_table():
                print("成功上台，恢复巡台")
                return
            else:
                print("冲台未成功，继续扫描")

        # 扫描循环：前进一段 -> 旋转一个步进角度
        print(f"前向未发现目标，前进{SCAN_FORWARD_TIME}秒后旋转扫描")
        recovery_forward()
        time.sleep(SCAN_FORWARD_TIME)
        stop()
        time.sleep(0.1)

        # 旋转扫描，检测前向红外
        print(f"向{SCAN_TURN_DIRECTION}旋转扫描...")
        scan_turn(SCAN_TURN_DIRECTION)
        scan_start = time.time()
        found = False
        while time.time() - scan_start < SCAN_TURN_STEP_TIME:
            fl, fr = read_front_ir()
            if fl == 0 and fr == 0:
                stop()
                print("扫描中发现目标，前进撞台后全力后退")
                recovery_forward()
                time.sleep(RECOVERY_FORWARD_TIME)
                recovery_backward()
                time.sleep(RECOVERY_BACK_TIME)
                stop()
                if not is_off_table():
                    print("成功上台，恢复巡台")
                    return
                else:
                    print("冲台未成功，继续扫描")
                    found = True
                    break
            time.sleep(SCAN_CHECK_DELAY)
        if found:
            continue
        stop()
        time.sleep(0.1)

# ==================== 发车程序 ====================
def start_procedure():
    """等待侧向双红外同时检测到目标，全速后退上台"""
    print("发车程序：等待左右侧向红外同时检测到目标...")
    while True:
        sl, sr = read_side_ir()
        if sl == 0 and sr == 0:
            print(f"检测到目标！全速后退 {START_BACK_DURATION} 秒")
            set_motors(START_BACK_SPEED, START_BACK_SPEED)
            time.sleep(START_BACK_DURATION)
            stop()
            print("发车完成，进入主程序")
            break
        time.sleep(START_SIDE_DETECT_TIMEOUT)

# ==================== 主循环 ====================
def main_loop():
    # 先执行发车程序
    start_procedure()

    print("红外边缘巡台 + 目标追踪 + 掉台恢复 主循环启动")
    while True:
        # ------ 最高优先级：掉台判断 ------
        if is_off_table():
            off_table_recovery()
            continue

        # ------ 次高优先级：边缘悬空检测 ------
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            patrol_recover_from_edge(l_edge, r_edge)
            continue

        # ------ 读取红外目标 ------
        fl, fr = read_front_ir()
        sl, sr = read_side_ir()

        # ------ 目标追踪逻辑 ------
        if fl == 0 and fr == 0:
            print("前向锁定目标，冲撞！")
            ram_forward_until_safe()
            continue

        if fl == 0 and fr == 1:
            print("左前发现目标，慢速左转引入")
            slow_intro_with_edge('left')
            continue

        if fl == 1 and fr == 0:
            print("右前发现目标，慢速右转引入")
            slow_intro_with_edge('right')
            continue

        if fl == 1 and fr == 1:
            if sl == 0:
                print("左侧发现目标，快速左转（可打断）")
                side_track_with_edge('left')
                continue

            if sr == 0:
                print("右侧发现目标，快速右转（可打断）")
                side_track_with_edge('right')
                continue

        # ------ 无目标，正常巡台直行 ------
        patrol_forward()
        time.sleep(CYCLE_DELAY)

# ==================== 启动入口 ====================
if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        stop()
        print("程序已停止")