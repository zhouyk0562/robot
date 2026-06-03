#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
from enum import Enum, auto

# ---------- 初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ========== 可调参数区 ==========
# 灰度阈值（小于对应值视为检测到边缘）
THRESHOLD_FRONT = 1450
THRESHOLD_REAR  = 2200
THRESHOLD_LEFT  = 1080
THRESHOLD_RIGHT = 1620

# 基础速度
SPEED_FWD_LEFT   = 350
SPEED_FWD_RIGHT  = 350
SPEED_BACK_LEFT  = -350
SPEED_BACK_RIGHT = -350

# 转弯速度（左转：左轮倒转，右轮正转）
TURN_LEFT_SPEED_LEFT   = -500
TURN_LEFT_SPEED_RIGHT  = 500
TURN_RIGHT_SPEED_LEFT  = 500
TURN_RIGHT_SPEED_RIGHT = -500

# 动作持续时间（秒），会被提前终止
BACK_DURATION     = 0.5      # 后退最大时长
TURN_DURATION     = 0.5      # 转弯最大时长
FORWARD_ESCAPE    = 0.3      # 脱离后方边缘时前进时长

# 循环间隔（秒）
LOOP_DELAY = 0.02

# 传感器异常检测
ERROR_CHECK_COUNT = 20       # 连续异常次数阈值
ERROR_RAW_MIN     = 50       # 读数低于此值视为异常
# ================================

# ---------- 状态定义 ----------
class State(Enum):
    FORWARD        = auto()   # 直行
    BACKING        = auto()   # 后退中
    TURNING_LEFT   = auto()   # 左转中
    TURNING_RIGHT  = auto()   # 右转中
    STOPPED        = auto()   # 异常停车

# ---------- 全局变量 ----------
current_state = State.FORWARD
state_start_time = 0.0
error_counter = {ch: 0 for ch in ['front','rear','left','right']}  # 异常计数

# ---------- 辅助函数 ----------
def read_all_grays():
    """读取四个灰度传感器，返回 (前,后,左,右)"""
    adc = up.ADC_Get_All_Channle()
    front = adc[1]
    rear  = adc[0]
    left  = adc[2]
    right = adc[3]
    return front, rear, left, right

def set_motors(left_speed, right_speed):
    """设置左右电机速度（左=通道2，右=通道1）"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop_motors():
    set_motors(0, 0)

# ---------- 状态切换函数 ----------
def switch_to(new_state):
    global current_state, state_start_time
    current_state = new_state
    state_start_time = time.time()

# ---------- 传感器异常检查 ----------
def check_sensors(front, rear, left, right):
    """检查每个传感器是否持续异常（读数极低），返回是否有异常"""
    global error_counter
    readings = {'front': front, 'rear': rear, 'left': left, 'right': right}
    any_error = False
    for ch, val in readings.items():
        if val < ERROR_RAW_MIN:
            error_counter[ch] += 1
            if error_counter[ch] >= ERROR_CHECK_COUNT:
                print(f"传感器异常: {ch} 持续读数 {val} < {ERROR_RAW_MIN}")
                any_error = True
        else:
            error_counter[ch] = 0
    return any_error

# ---------- 边缘检测与优先级决策 ----------
def get_edges(front, rear, left, right):
    """
    返回触发的边缘集合（可以多个），
    以及一个推荐的最高优先级状态。
    优先级：前 > 左 > 右 > 后
    """
    edges = {
        'front': front < THRESHOLD_FRONT,
        'rear':  rear < THRESHOLD_REAR,
        'left':  left < THRESHOLD_LEFT,
        'right': right < THRESHOLD_RIGHT
    }
    # 默认优先级
    if edges['front']:
        return edges, 'front'
    if edges['left']:
        return edges, 'left'
    if edges['right']:
        return edges, 'right'
    if edges['rear']:
        return edges, 'rear'
    return edges, None

# ---------- 主循环 ----------
def main_loop():
    global current_state, state_start_time

    print("巡台程序启动，按 Ctrl+C 退出")
    switch_to(State.FORWARD)

    try:
        while True:
            # 1. 读取传感器
            front, rear, left, right = read_all_grays()
            print(f"前:{front:4d} 后:{rear:4d} 左:{left:4d} 右:{right:4d} 状态:{current_state.name}")

            # 2. 传感器异常检查
            if check_sensors(front, rear, left, right):
                stop_motors()
                switch_to(State.STOPPED)
                print("因传感器异常停车，程序终止")
                break

            # 3. 检测边缘
            edges, top_priority = get_edges(front, rear, left, right)

            # 4. 根据当前状态和边缘信息决定行为
            now = time.time()
            elapsed = now - state_start_time

            if current_state == State.FORWARD:
                if top_priority is None:
                    # 无边缘：继续直行
                    set_motors(SPEED_FWD_LEFT, SPEED_FWD_RIGHT)
                else:
                    # 有边缘触发，切换到对应规避状态
                    if top_priority == 'front':
                        print("前方边缘！后退避让")
                        switch_to(State.BACKING)
                    elif top_priority == 'left':
                        print("左侧边缘！右转")
                        switch_to(State.TURNING_RIGHT)
                    elif top_priority == 'right':
                        print("右侧边缘！左转")
                        switch_to(State.TURNING_LEFT)
                    elif top_priority == 'rear':
                        print("后方边缘！前进脱离")
                        switch_to(State.FORWARD)  # 继续保持前进，但会额外延时
                        # 若已经在直行，增加一个前进时段
                        state_start_time = now  # 重置开始时间，用于后续计时脱离
                        # 其实不需要额外操作，前进状态会自己保持

            elif current_state == State.BACKING:
                # 后退中持续检测：若前方边缘已脱离且后退了至少0.1秒，则提前结束
                if (elapsed > 0.1 and not edges['front']) or elapsed >= BACK_DURATION:
                    # 结束后退，接下来执行一次左转（改变方向）
                    print("后退完成，左转调整方向")
                    switch_to(State.TURNING_LEFT)
                else:
                    # 继续后退
                    set_motors(SPEED_BACK_LEFT, SPEED_BACK_RIGHT)

            elif current_state == State.TURNING_LEFT:
                # 左转中：若右侧边缘（触发左转的原因）已脱离且转弯超过0.1秒，提前结束
                if (elapsed > 0.1 and not edges['right']) or elapsed >= TURN_DURATION:
                    print("左转完成")
                    switch_to(State.FORWARD)
                else:
                    set_motors(TURN_LEFT_SPEED_LEFT, TURN_LEFT_SPEED_RIGHT)

            elif current_state == State.TURNING_RIGHT:
                # 右转中：若左侧边缘已脱离且转弯超过0.1秒，提前结束
                if (elapsed > 0.1 and not edges['left']) or elapsed >= TURN_DURATION:
                    print("右转完成")
                    switch_to(State.FORWARD)
                else:
                    set_motors(TURN_RIGHT_SPEED_LEFT, TURN_RIGHT_SPEED_RIGHT)

            elif current_state == State.STOPPED:
                stop_motors()
                break  # 退出循环

            # 高频循环延时（避免CPU占用100%，但对uptech库可能不需要）
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        stop_motors()
        up.CDS_Close()
        up.ADC_IO_Close()
        print("资源已释放，程序结束")

if __name__ == "__main__":
    main_loop()