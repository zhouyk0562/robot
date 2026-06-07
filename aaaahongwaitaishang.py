#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红外边缘巡台 + 红外目标追踪程序（仅台上，无灰度、无掉台恢复）
- 利用底部红外（IO4/左，IO6/右）进行边缘悬空脱困
- 利用前向红外（IO0/左，IO1/右）和侧向红外（IO2/左，IO3/右）追踪目标
- 边缘悬空优先级最高，可在任意动作中打断并恢复
- 所有速度、时间、方向参数均可独立调节
"""

import uptech
import time

# ---------- 硬件初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ==================== 可调参数区（所有参数独立控制） ====================

# ---- 巡台直行速度 ----
PATROL_FORWARD_LEFT  = 600     # 直行时左轮速度
PATROL_FORWARD_RIGHT = 600     # 直行时右轮速度

# ---- 巡台通用后退速度 ----
PATROL_BACK_LEFT  = -600       # 巡台/侧向引入恢复时后退左轮速度
PATROL_BACK_RIGHT = -600       # 巡台/侧向引入恢复时后退右轮速度

# ---- 巡台转向速度 ----
PATROL_TURN_SPEED = 700        # 原地旋转时两轮速度绝对值（左转为 (-TURN_SPEED, TURN_SPEED)）

# ---- 巡台边缘悬空恢复时间参数 ----
PATROL_BACK_TIME = 0.5                # 悬空后统一后退时间（秒）
LEFT_SUSPEND_TURN_TIME = 0.9          # 左侧悬空 -> 向右转的时间（秒）
RIGHT_SUSPEND_TURN_TIME = 0.9        # 右侧悬空 -> 向左转的时间（秒）
DOUBLE_SUSPEND_TURN_TIME = 1.3        # 双侧悬空 -> 转向时间（秒）
DOUBLE_SUSPEND_TURN_DIR = 'right'     # 双侧悬空转向方向：'left' 或 'right'

# ---- 巡台动作间隔延时 ----
PATROL_STOP_PAUSE = 0.1              # 动作间短暂停车消抖时间（秒）
CYCLE_DELAY = 0.02                   # 主循环检查间隔（秒）

# ---- 前向目标慢速引入转向速度（单侧发现目标时） ----
SLOW_INTRO_SPEED = 500                # 慢速引入时两轮速度绝对值

# ---- 侧向目标快速引入速度（两侧红外发现目标时） ----
SIDE_FAST_SPEED = 600                 # 快速引入时两轮速度绝对值

# ---- 侧向快速引入持续时间（若无打断） ----
SIDE_TRACK_DUR = 0.7                  # 侧向引入最长持续时间（秒）

# ---- 前向双亮冲撞速度 ----
RAM_SPEED_LEFT  = 600                 # 冲撞时左轮速度
RAM_SPEED_RIGHT = 600                 # 冲撞时右轮速度

# ---- 冲撞专用边缘恢复参数（独立于巡台参数） ----
RAM_BACK_SPEED_LEFT  = -600           # 冲撞中边缘悬空后退时左轮速度
RAM_BACK_SPEED_RIGHT = -600           # 冲撞中边缘悬空后退时右轮速度
RAM_BACK_TIME = 0.5                   # 冲撞中边缘恢复后退时间（秒）
RAM_LEFT_SUSPEND_TURN_TIME = 0.9      # 冲撞中左侧悬空向右转时间（秒）
RAM_RIGHT_SUSPEND_TURN_TIME = 0.9     # 冲撞中右侧悬空向左转时间（秒）
RAM_DOUBLE_SUSPEND_TURN_TIME = 1.3    # 冲撞中双侧悬空转向时间（秒）
RAM_DOUBLE_SUSPEND_TURN_DIR = 'right' # 冲撞中双侧悬空转向方向：'left' 或 'right'
RAM_TURN_SPEED = 700                  # 冲撞中边缘恢复转向速度（绝对值）

# ---- 冲撞检测间隔 ----
RAM_CHECK_DELAY = 0.03                # 冲撞循环中传感器检测间隔（秒）

# ==================== 传感器读取 ====================
def read_edge_ir():
    """读取底部红外传感器，返回 (左悬空, 右悬空)  1=悬空, 0=在台面上"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 4) & 1   # IO4
    right = (all_in >> 6) & 1   # IO6
    return left, right

def read_front_ir():
    """读取前向红外传感器，返回 (左前, 右前)  0=检测到目标, 1=未检测到"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 0) & 1   # IO0
    right = (all_in >> 1) & 1   # IO1
    return left, right

def read_side_ir():
    """读取侧向红外传感器，返回 (左侧, 右侧)  0=检测到目标, 1=未检测到"""
    all_in = up.ADC_IO_GetAllInputLevel()
    left  = (all_in >> 2) & 1   # IO2
    right = (all_in >> 3) & 1   # IO3
    return left, right

# ==================== 电机控制 ====================
def set_motors(left_speed, right_speed):
    """设置左右电机速度（左通道2，右通道1）"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

# ----- 巡台基本动作 -----
def patrol_forward():
    """巡台直行"""
    set_motors(PATROL_FORWARD_LEFT, PATROL_FORWARD_RIGHT)

def patrol_backward():
    """巡台后退（不延时，由调用者控制）"""
    set_motors(PATROL_BACK_LEFT, PATROL_BACK_RIGHT)

def patrol_turn_left(duration):
    """巡台原地左转，持续 duration 秒"""
    set_motors(-PATROL_TURN_SPEED, PATROL_TURN_SPEED)
    time.sleep(duration)
    stop()

def patrol_turn_right(duration):
    """巡台原地右转，持续 duration 秒"""
    set_motors(PATROL_TURN_SPEED, -PATROL_TURN_SPEED)
    time.sleep(duration)
    stop()

# ----- 冲撞专用动作 -----
def ram_forward():
    """冲撞前进"""
    set_motors(RAM_SPEED_LEFT, RAM_SPEED_RIGHT)

def ram_backward():
    """冲撞后退（不延时）"""
    set_motors(RAM_BACK_SPEED_LEFT, RAM_BACK_SPEED_RIGHT)

def ram_turn_left(duration):
    """冲撞恢复左转"""
    set_motors(-RAM_TURN_SPEED, RAM_TURN_SPEED)
    time.sleep(duration)
    stop()

def ram_turn_right(duration):
    """冲撞恢复右转"""
    set_motors(RAM_TURN_SPEED, -RAM_TURN_SPEED)
    time.sleep(duration)
    stop()

# ----- 目标追踪专用动作 -----
def slow_turn_left():
    """慢速左转引入（前向单目标）"""
    set_motors(-SLOW_INTRO_SPEED, SLOW_INTRO_SPEED)

def slow_turn_right():
    """慢速右转引入"""
    set_motors(SLOW_INTRO_SPEED, -SLOW_INTRO_SPEED)

def fast_turn_left():
    """侧向快速左转"""
    set_motors(-SIDE_FAST_SPEED, SIDE_FAST_SPEED)

def fast_turn_right():
    """侧向快速右转"""
    set_motors(SIDE_FAST_SPEED, -SIDE_FAST_SPEED)

# ==================== 边缘悬空恢复（巡台通用） ====================
def patrol_recover_from_edge(left, right):
    """
    巡台/侧向引入过程中的边缘恢复
    参数：当前边缘状态（left, right）
    """
    stop()
    print(f"[巡台恢复] 边缘触发 L={left} R={right}")
    # 后退
    patrol_backward()
    time.sleep(PATROL_BACK_TIME)
    stop()
    time.sleep(PATROL_STOP_PAUSE)

    # 转向
    if left == 1 and right == 0:
        print(f"  左侧悬空，向右转 {LEFT_SUSPEND_TURN_TIME} 秒")
        patrol_turn_right(LEFT_SUSPEND_TURN_TIME)
    elif left == 0 and right == 1:
        print(f"  右侧悬空，向左转 {RIGHT_SUSPEND_TURN_TIME} 秒")
        patrol_turn_left(RIGHT_SUSPEND_TURN_TIME)
    else:  # 双悬空
        print(f"  双侧悬空，向{DOUBLE_SUSPEND_TURN_DIR}转 {DOUBLE_SUSPEND_TURN_TIME} 秒")
        if DOUBLE_SUSPEND_TURN_DIR == 'left':
            patrol_turn_left(DOUBLE_SUSPEND_TURN_TIME)
        else:
            patrol_turn_right(DOUBLE_SUSPEND_TURN_TIME)
    time.sleep(PATROL_STOP_PAUSE)

# ==================== 冲撞专用边缘恢复 ====================
def ram_recover_from_edge(left, right):
    """冲撞过程中的边缘恢复（独立参数）"""
    stop()
    print(f"[冲撞恢复] 边缘触发 L={left} R={right}")
    # 后退
    ram_backward()
    time.sleep(RAM_BACK_TIME)
    stop()
    time.sleep(PATROL_STOP_PAUSE)   # 复用巡台暂停时间

    # 转向
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
    """
    前向单侧发现目标，慢速旋转引入，同时检测边缘悬空
    direction: 'left' 或 'right'
    返回: 'double' 表示成功引入双亮； 'edge' 表示被边缘悬空打断； 'lost' 表示目标丢失（双灭）
    """
    if direction == 'left':
        slow_turn_left()
    else:
        slow_turn_right()

    while True:
        # 边缘优先检测
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            patrol_recover_from_edge(l_edge, r_edge)
            return 'edge'

        fl, fr = read_front_ir()
        if fl == 0 and fr == 0:         # 双亮，引入成功
            stop()
            return 'double'
        if fl == 1 and fr == 1:         # 双灭，目标丢失
            stop()
            return 'lost'
        time.sleep(0.03)

def side_track_with_edge(direction):
    """
    侧向快速引入，可被前向目标或边缘悬空打断
    direction: 'left' 或 'right'
    返回: 'interrupted' 表示被前向目标或边缘打断并已处理； 'finished' 表示执行完毕未被打断
    """
    if direction == 'left':
        fast_turn_left()
    else:
        fast_turn_right()

    start = time.time()
    while time.time() - start < SIDE_TRACK_DUR:
        # 边缘检测
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            stop()
            patrol_recover_from_edge(l_edge, r_edge)
            return 'interrupted'

        # 前向目标打断
        fl, fr = read_front_ir()
        if fl == 0 or fr == 0:
            stop()
            # 若被打断，不额外恢复（交主循环处理）
            return 'interrupted'

        time.sleep(0.02)
    stop()
    return 'finished'

def ram_forward_until_safe():
    """
    前向双亮冲撞，持续前进，直到边缘悬空或目标丢失（双灭）
    """
    ram_forward()
    while True:
        # 边缘检测
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            ram_recover_from_edge(l_edge, r_edge)
            return  # 恢复后退出冲撞

        fl, fr = read_front_ir()
        if fl == 1 and fr == 1:   # 目标丢失
            stop()
            print("目标丢失，停止冲撞")
            return
        time.sleep(RAM_CHECK_DELAY)

# ==================== 主循环 ====================
def main_loop():
    print("红外边缘巡台 + 目标追踪程序启动")
    while True:
        # ------ 最高优先级：边缘悬空检测 ------
        l_edge, r_edge = read_edge_ir()
        if l_edge == 1 or r_edge == 1:
            patrol_recover_from_edge(l_edge, r_edge)
            continue

        # ------ 读取红外目标 ------
        fl, fr = read_front_ir()
        sl, sr = read_side_ir()

        # ------ 目标追踪逻辑 ------
        # 1) 前向双亮 -> 冲撞
        if fl == 0 and fr == 0:
            print("前向锁定目标，冲撞！")
            ram_forward_until_safe()
            continue

        # 2) 左前单亮 -> 慢速左转引入
        if fl == 0 and fr == 1:
            print("左前发现目标，慢速左转引入")
            res = slow_intro_with_edge('left')
            continue

        # 3) 右前单亮 -> 慢速右转引入
        if fl == 1 and fr == 0:
            print("右前发现目标，慢速右转引入")
            res = slow_intro_with_edge('right')
            continue

        # 4) 前向无目标，检查侧向
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