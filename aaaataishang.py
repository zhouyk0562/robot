#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()          # 电机控制
up.ADC_IO_Open()       # ADC 扩展板（灰度 + IO）

# ========== 可调参数区 ==========
# 灰度传感器阈值（小于对应值视为检测到边缘）
THRESHOLD_FRONT = 1400
THRESHOLD_REAR  = 2200
THRESHOLD_LEFT  = 1080
THRESHOLD_RIGHT = 1620

# 电机速度
LEFT_SPEED_FWD   = 500
RIGHT_SPEED_FWD  = 500
LEFT_SPEED_BACK  = -500
RIGHT_SPEED_BACK = -500
TURN_LEFT_SPEED  = -500   # 左转时左轮速度（这里实际未直接用，转向用下面的）
TURN_RIGHT_SPEED = 500    # 左转时右轮速度（原地左转：左轮倒转，右轮正转）

# 动作持续时间（秒）
BACK_DURATION   = 0.5
TURN_DURATION   = 0.5      # 调整用短转弯
TURN_SIDE_DUR   = 0.3    # 侧向红外触发时转弯时间
RAM_DURATION    = 0.1      # 冲撞单次循环时间
CYCLE_DELAY     = 0.01     # 主循环节拍

# ★ 边缘恢复新增参数（防止卡死）
EDGE_RECOVERY_TIMEOUT = 2   # 单次旋转最长等待时间（秒）
EDGE_FORWARD_TIME     = 0.5   # 超时后前进脱困的时间（秒）
EDGE_MAX_ATTEMPTS     = 2     # 前进重试次数（总尝试次数 = 重试次数 + 1）
# ================================

# ---------- 传感器读取 ----------
def read_all_grays():
    """
    返回四个灰度值：前, 后, 左, 右
    映射：adc[0]后，adc[1]前，adc[2]左，adc[3]右
    """
    adc = up.ADC_Get_All_Channle()
    front = adc[1]
    rear  = adc[0]
    left  = adc[2]
    right = adc[3]
    return front, rear, left, right

def read_io_bit(bit_index):
    """读取指定 IO 口的电平（0或1）"""
    val = up.ADC_IO_GetAllInputLevel()
    return (val >> bit_index) & 1

def read_edge_ir():
    """返回左下(IO4), 右下(IO6) 的电平，1表示悬空（危险）"""
    left  = read_io_bit(4)
    right = read_io_bit(6)
    return left, right

def read_front_ir():
    """返回左前(IO0), 右前(IO1) 的电平，0表示检测到目标"""
    left  = read_io_bit(0)
    right = read_io_bit(1)
    return left, right

def read_side_ir():
    """返回左侧(IO2), 右侧(IO3) 的电平，0表示检测到目标"""
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

# ---------- 基础动作（不阻塞，仅设置速度）----------
def forward():
    set_motors(LEFT_SPEED_FWD, RIGHT_SPEED_FWD)

def backward():
    set_motors(LEFT_SPEED_BACK, RIGHT_SPEED_BACK)

def turn_left():
    """原地左转：左轮倒转，右轮正转"""
    set_motors(-TURN_RIGHT_SPEED, TURN_RIGHT_SPEED)

def turn_right():
    """原地右转：左轮正转，右轮倒转"""
    set_motors(TURN_RIGHT_SPEED, -TURN_RIGHT_SPEED)

# ---------- 灰度巡台逻辑（已修复左侧缺失）----------
def grayscale_follow(front, rear, left, right):
    """
    根据四个灰度值执行一次巡台动作。
    优先级：前 > 左 > 右 > 后 > 安全前进
    """
    if front < THRESHOLD_FRONT:
        # 前方边缘 → 后退后左转
        stop()
        backward()
        time.sleep(BACK_DURATION)
        turn_left()
        time.sleep(TURN_DURATION)
        stop()
    elif left < THRESHOLD_LEFT:          # ★ 修复：新增左侧边缘处理
        # 左侧边缘 → 右转脱离
        stop()
        turn_right()
        time.sleep(TURN_DURATION)
        stop()
    elif right < THRESHOLD_RIGHT:
        # 右侧边缘 → 左转脱离
        stop()
        turn_left()
        time.sleep(TURN_DURATION)
        stop()
    elif rear < THRESHOLD_REAR:
        # 后方边缘 → 前进脱离
        stop()
        forward()
        time.sleep(0.3)
    else:
        # 安全 → 直行
        forward()

# ---------- 边缘恢复（已修复卡死问题）----------
def recover_from_edge():
    """
    边缘恢复：
    1. 先后退离开边缘；
    2. 原地左转，实时检测四路灰度是否全部恢复正常；
    3. 若在 EDGE_RECOVERY_TIMEOUT 秒内未恢复，则停止旋转，
       向前行驶 EDGE_FORWARD_TIME 秒尝试脱困，然后重新旋转；
    4. 最多尝试 EDGE_MAX_ATTEMPTS 次前进脱困，若仍失败则退出，
       由主循环下一轮重新判断。
    """
    print("边缘触发！开始恢复...")
    stop()
    # 1. 后退，远离边缘
    backward()
    time.sleep(0.7)
    stop()
    time.sleep(0.1)

    attempts = 0
    # 外层循环：最多进行 (EDGE_MAX_ATTEMPTS + 1) 次旋转尝试
    while attempts <= EDGE_MAX_ATTEMPTS:
        # 2. 开始旋转并等待灰度全部安全
        turn_left()
        start_time = time.time()
        while time.time() - start_time < EDGE_RECOVERY_TIMEOUT:
            front, rear, left, right = read_all_grays()
            # 判断是否全部安全：所有值 >= 各自阈值
            if (front >= THRESHOLD_FRONT and rear >= THRESHOLD_REAR and
                left >= THRESHOLD_LEFT and right >= THRESHOLD_RIGHT):
                stop()
                print("已恢复到安全位置")
                time.sleep(0.2)
                return  # 成功恢复，直接返回
            time.sleep(0.05)   # 检查间隔（0.05秒）

        # 3. 超时未恢复：停止旋转，尝试前进脱困
        stop()
        print(f"旋转超时（第{attempts+1}次），尝试前进脱困")
        forward()
        time.sleep(EDGE_FORWARD_TIME)
        stop()
        time.sleep(0.1)
        attempts += 1

    # 4. 所有尝试均失败，放弃本次恢复，交由主循环再次处理
    stop()
    print("边缘恢复失败，放弃等待，继续主循环")
    time.sleep(0.2)

# ---------- 带边缘检查的冲撞（持续前进直到目标丢失或边缘触发）----------
def ram_forward_until_safe():
    """
    执行冲撞：前进，同时不断检查边缘和目标。
    如果边缘触发，则立即中断并调用恢复；
    如果目标丢失（前向两个红外都丢失），则停止。
    """
    forward()
    while True:
        # 边缘优先
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge()
            return  # 恢复后返回主循环
        # 检查前向目标是否仍然存在（两个都未检测到则丢失）
        fl, fr = read_front_ir()
        if fl == 1 and fr == 1:  # 都为1表示都未检测到目标
            stop()
            print("目标丢失，停止冲撞")
            return
        time.sleep(RAM_DURATION)

# ---------- 主循环 ----------
print("程序启动：灰度巡台 + 红外防边缘/目标追踪")
try:
    while True:
        # === 最高优先级：边缘检测 ===
        edge_l, edge_r = read_edge_ir()
        if edge_l == 1 or edge_r == 1:
            recover_from_edge()
            continue  # 恢复后重新开始循环

        # === 读取传感器 ===
        fl, fr = read_front_ir()      # 0 有目标
        sl, sr = read_side_ir()       # 0 有目标
        fg, rg, lg, rgg = read_all_grays()

        # === 目标追踪逻辑 ===
        # 前向两个都检测到 -> 冲撞（带边缘保护）
        if fl == 0 and fr == 0:
            print("前向锁定目标，冲撞！")
            ram_forward_until_safe()
            continue

        # 左前检测到，右前无 -> 左转调整
        if fl == 0 and fr == 1:
            print("左前发现目标，左转调整")
            stop()
            turn_left()
            time.sleep(0.1)
            stop()
            continue

        # 右前检测到，左前无 -> 右转调整
        if fl == 1 and fr == 0:
            print("右前发现目标，右转调整")
            stop()
            turn_right()
            time.sleep(0.1)
            stop()
            continue

        # 侧向红外检测（仅当前面无目标时）
        if fl == 1 and fr == 1:
            if sl == 0:   # 左侧有目标
                print("左侧发现目标，左转引入")
                stop()
                turn_left()
                time.sleep(TURN_SIDE_DUR)
                stop()
                continue
            if sr == 0:   # 右侧有目标
                print("右侧发现目标，右转引入")
                stop()
                turn_right()
                time.sleep(TURN_SIDE_DUR)
                stop()
                continue

        # === 无红外目标，执行灰度巡台 ===
        grayscale_follow(fg, rg, lg, rgg)

        # 主循环节拍
        time.sleep(CYCLE_DELAY)

except KeyboardInterrupt:
    stop()
    print("程序已停止")