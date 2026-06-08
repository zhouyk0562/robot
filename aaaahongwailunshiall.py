#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红外边缘巡台 + 红外目标追踪 + 自动发车 + 掉台恢复程序
功能：
- 发车：启动后等待左右侧向红外同时检测到目标，全速后退固定时间（不检测上台）
- 掉台恢复：灰度加权平均低于阈值视为掉台，后退旋转扫描搜索台面，找到后前进冲撞再全力后退上台
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
# 发车时，程序会等待左右侧向红外（IO2,IO3）同时为0（检测到目标），然后全速后退
START_SIDE_DETECT_TIMEOUT = 0.05    # 发车等待中，每次检测侧向红外的间隔（秒），越小响应越快
START_BACK_SPEED = -1000            # 发车全速后退速度（负值），负值越大后退越快
START_BACK_DURATION = 2             # 发车后退固定持续时间（秒），不检测上台，时间到即停

# ======== 掉台恢复参数 ========
# 掉台恢复利用灰度传感器判断是否离开台面，并通过后退+旋转扫描寻找台面边缘，再前进冲撞+全力后退上台

# ---- 灰度校正系数 ----
# 四路灰度传感器（前、后、左、右）在相同光照下数值可能不同，系数使它们在台下（无反射）时都约等于1000
# 顺序：[前, 后, 左, 右]  —— 对应 read_all_grays() 返回顺序 adc[1], adc[0], adc[2], adc[3]
GRAY_COEFF = [1.2195, 0.625, 1.9231, 1.0]

# ---- 掉台判断阈值 ----
OFF_TABLE_AVG_THRESHOLD = 1025      # 校正后四路灰度平均值低于此值判定为掉台（阈值略高于台下典型值1000，留裕度）

# ---- 掉台恢复动作速度 ----
RECOVERY_FORWARD_SPEED = 600        # 掉台恢复时前进撞台速度（正值，冲向台面边缘）
RECOVERY_BACK_SPEED = -1000         # 掉台恢复时全力后退速度（负值，上台后退冲上场地）

# ---- 掉台恢复动作时间 ----
RECOVERY_FORWARD_TIME = 0.8         # 前进撞台持续时间（秒），确保车轮压到台面边缘
RECOVERY_BACK_TIME = 2              # 全力后退上台持续时间（秒），足够让小车完全退上台面

# ---- 掉台扫描参数（后退+旋转扫描） ----
SCAN_FORWARD_TIME = 0.3             # 扫描阶段后退时间（秒），每次旋转前先直线后退一小段，扩大搜索范围
SCAN_TURN_SPEED = 600               # 扫描旋转速度（绝对值），原地旋转时两轮速度绝对值
SCAN_TURN_DIRECTION = 'right'       # 扫描旋转固定方向，'left' 或 'right'
SCAN_TURN_STEP_TIME = 0.3           # 每次旋转步进时间（秒），控制每次转动的角度（时间越长转动越多）
SCAN_CHECK_DELAY = 0.02             # 旋转过程中检测前向红外的间隔（秒），越小检测越及时

# ======== 巡台直行速度 ========
PATROL_FORWARD_LEFT  = 500          # 正常巡逻直行时左轮速度（通道2）
PATROL_FORWARD_RIGHT = 500          # 正常巡逻直行时右轮速度（通道1）

# ======== 巡台通用后退速度 ========
PATROL_BACK_LEFT  = -600            # 巡台/侧向引入恢复时后退左轮速度
PATROL_BACK_RIGHT = -600            # 巡台/侧向引入恢复时后退右轮速度

# ======== 巡台转向速度 ========
PATROL_TURN_SPEED = 600             # 巡台原地旋转时两轮速度绝对值（左转为(-600, 600)，右转为(600, -600)）

# ======== 巡台边缘悬空恢复时间参数 ========
PATROL_BACK_TIME = 0.4              # 边缘悬空后统一后退时间（秒）
LEFT_SUSPEND_TURN_TIME = 0.7        # 仅左侧悬空时，向右转的时间（秒）
RIGHT_SUSPEND_TURN_TIME = 0.7       # 仅右侧悬空时，向左转的时间（秒）
DOUBLE_SUSPEND_TURN_TIME = 0.9      # 双侧悬空时，转向的时间（秒）
DOUBLE_SUSPEND_TURN_DIR = 'right'   # 双侧悬空时优先转向的方向，'left' 或 'right'

# ======== 巡台动作间隔延时 ========
PATROL_STOP_PAUSE = 0.1             # 动作间短暂停车消抖时间（秒）
CYCLE_DELAY = 0.02                  # 主循环无目标直行时的检查间隔（秒），越小响应越快但CPU占用稍高

# ======== 前向目标慢速引入转向速度 ========
SLOW_INTRO_SPEED = 450              # 前向单侧发现目标时，慢速旋转引入的速度（两轮绝对值）

# ======== 侧向目标快速引入速度 ========
SIDE_FAST_SPEED = 600               # 侧向发现目标时，快速旋转引入的速度（两轮绝对值）

# ======== 侧向快速引入持续时间 ========
SIDE_TRACK_DUR = 0.7                # 侧向引入最长持续时间（秒），超时未被打断则停止旋转

# ======== 前向双亮冲撞速度 ========
RAM_SPEED_LEFT  = 600               # 冲撞时左轮速度（通道2）
RAM_SPEED_RIGHT = 600               # 冲撞时右轮速度（通道1）

# ======== 冲撞专用边缘恢复参数（独立于巡台参数） ========
RAM_BACK_SPEED_LEFT  = -600         # 冲撞中边缘悬空后退时左轮速度
RAM_BACK_SPEED_RIGHT = -600         # 冲撞中边缘悬空后退时右轮速度
RAM_BACK_TIME = 0.5                 # 冲撞中边缘恢复后退时间（秒）
RAM_LEFT_SUSPEND_TURN_TIME = 0.7    # 冲撞中仅左侧悬空时向右转的时间（秒）
RAM_RIGHT_SUSPEND_TURN_TIME = 0.7   # 冲撞中仅右侧悬空时向左转的时间（秒）
RAM_DOUBLE_SUSPEND_TURN_TIME = 0.9  # 冲撞中双侧悬空时转向的时间（秒）
RAM_DOUBLE_SUSPEND_TURN_DIR = 'right' # 冲撞中双侧悬空转向方向，'left' 或 'right'
RAM_TURN_SPEED = 600                # 冲撞中边缘恢复转向速度（绝对值）
RAM_CHECK_DELAY = 0.02              # 冲撞循环中传感器检测间隔（秒）

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

# ==================== 灰度校正读取与掉台判断 ====================
def read_adjusted_grays():
    """返回校正后的四路灰度值（已乘系数）"""
    fg, rg, lg, rgg = read_all_grays()
    adj = [
        fg * GRAY_COEFF[0],
        rg * GRAY_COEFF[1],
        lg * GRAY_COEFF[2],
        rgg * GRAY_COEFF[3]
    ]
    return adj

def is_off_table():
    """校正后四路平均值低于阈值则返回True（掉台）"""
    adj = read_adjusted_grays()
    avg = sum(adj) / 4.0
    return avg < OFF_TABLE_AVG_THRESHOLD

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

def recovery_backward_scan():
    """掉台恢复扫描时用的后退（速度大小与前进撞台相同，方向相反）"""
    set_motors(-RECOVERY_FORWARD_SPEED, -RECOVERY_FORWARD_SPEED)

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

# ==================== 掉台恢复（后退旋转扫描） ====================
def off_table_recovery():
    """掉台恢复：后退+旋转扫描寻找台面，冲撞上台"""
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
            if not is_off_table():
                print("成功上台，恢复巡台")
                return
            else:
                print("冲台未成功，继续扫描")

        # 扫描循环：后退一段 → 旋转一个步进角度
        print(f"前向未发现目标，后退{SCAN_FORWARD_TIME}秒后旋转扫描")
        recovery_backward_scan()
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

# ==================== 发车程序（固定后退时间） ====================
def start_procedure():
    """等待侧向双红外同时检测到目标，全速后退固定时间，不检测上台"""
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