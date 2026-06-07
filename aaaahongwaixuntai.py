#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红外边缘巡台程序（纯底部红外，无灰度）
利用两个下置红外传感器（左IO4、右IO6）检测台面悬空：
- 未悬空：直行
- 单侧悬空：后退 → 向反方向大角度转（角度独立可调）
- 双侧悬空：后退 → 按设定方向转大角度（角度独立可调）
所有动作的速度、时间、方向均可独立调节。
"""

import uptech
import time

# ---------- 硬件初始化 ----------
up = uptech.UpTech()
up.CDS_Open()          # 电机控制
up.ADC_IO_Open()       # ADC/IO 扩展板（红外传感器）

# ==================== 可调参数区（所有行为参数） ====================
# 说明：修改下面任何一个数字都会立刻影响对应动作，无需改动代码主体。

# ---- 前进速度（直行时两轮速度） ----
FORWARD_LEFT_SPEED  = 600      # 前进时左轮速度（正转）
FORWARD_RIGHT_SPEED = 600      # 前进时右轮速度（正转）

# ---- 后退速度（脱困后退时两轮速度） ----
BACKWARD_LEFT_SPEED  = -600    # 后退时左轮速度（反转，通常为负值）
BACKWARD_RIGHT_SPEED = -600    # 后退时右轮速度（反转，通常为负值）

# ---- 转向速度（原地旋转时单轮速度绝对值） ----
TURN_SPEED = 700               # 转向时两个车轮的速度大小（左转为 (-TURN_SPEED, TURN_SPEED)）

# ---- 动作持续时间（秒） ----
BACKWARD_TIME = 0.5            # 悬空后先后退的时间（秒），所有悬空情况共用

# 不同悬空情况的独立转向时间（控制转向角度大小）
LEFT_SUSPEND_TURN_TIME   = 0.7 # 左侧悬空 → 向右转的时间（秒）
RIGHT_SUSPEND_TURN_TIME  = 0.7 # 右侧悬空 → 向左转的时间（秒）
DOUBLE_SUSPEND_TURN_TIME = 0.9 # 双侧悬空 → 转向的时间（秒）

# ---- 双侧悬空转向方向 ----
# 'left'  -> 向左转； 'right' -> 向右转
DOUBLE_SUSPEND_TURN_DIRECTION = 'right'

# ---- 其他延迟 ----
STOP_PAUSE    = 0.1            # 动作间短暂停车消抖时间（秒）
CYCLE_DELAY   = 0.02           # 主循环检查间隔（秒），控制传感器读取频率

# ==================== 传感器读取 ====================
def read_edge_ir():
    """
    读取底部两个红外传感器
    IO4 -> 左侧边缘 (1 = 悬空，0 = 在台面上)
    IO6 -> 右侧边缘 (1 = 悬空，0 = 在台面上)
    返回: (左悬空, 右悬空)
    """
    all_input = up.ADC_IO_GetAllInputLevel()
    left_edge  = (all_input >> 4) & 1
    right_edge = (all_input >> 6) & 1
    return left_edge, right_edge

# ==================== 电机控制 ====================
def set_motors(left_speed, right_speed):
    """设置左右电机速度，左电机通道2，右电机通道1"""
    up.CDS_SetSpeed(2, left_speed)
    up.CDS_SetSpeed(1, right_speed)

def stop():
    set_motors(0, 0)

def forward():
    """以设定速度直行"""
    set_motors(FORWARD_LEFT_SPEED, FORWARD_RIGHT_SPEED)

def backward():
    """以设定速度后退（不包含延时，由调用者控制）"""
    set_motors(BACKWARD_LEFT_SPEED, BACKWARD_RIGHT_SPEED)

def turn_left(duration):
    """原地左转并持续指定秒数（duration：转向时间）"""
    set_motors(-TURN_SPEED, TURN_SPEED)
    time.sleep(duration)
    stop()

def turn_right(duration):
    """原地右转并持续指定秒数"""
    set_motors(TURN_SPEED, -TURN_SPEED)
    time.sleep(duration)
    stop()

# ==================== 主巡台逻辑 ====================
def edge_patrol_loop():
    """
    无限循环，根据底部红外信号执行巡台脱困。
    优先级：先悬空判断，再直行。
    """
    while True:
        left, right = read_edge_ir()

        # 任何一侧悬空 -> 进入脱困流程
        if left == 1 or right == 1:
            stop()
            print(f"边缘触发！左悬空={left} 右悬空={right}")

            # 第一步：后退（统一后退时间）
            backward()
            time.sleep(BACKWARD_TIME)
            stop()
            time.sleep(STOP_PAUSE)

            # 第二步：根据悬空状态，使用独立转向时间
            if left == 1 and right == 0:
                print(f"→ 左侧悬空，向右转 {LEFT_SUSPEND_TURN_TIME} 秒")
                turn_right(LEFT_SUSPEND_TURN_TIME)
            elif left == 0 and right == 1:
                print(f"→ 右侧悬空，向左转 {RIGHT_SUSPEND_TURN_TIME} 秒")
                turn_left(RIGHT_SUSPEND_TURN_TIME)
            else:  # 双悬空
                print(f"→ 双侧悬空，向{DOUBLE_SUSPEND_TURN_DIRECTION}转 {DOUBLE_SUSPEND_TURN_TIME} 秒")
                if DOUBLE_SUSPEND_TURN_DIRECTION == 'left':
                    turn_left(DOUBLE_SUSPEND_TURN_TIME)
                else:
                    turn_right(DOUBLE_SUSPEND_TURN_TIME)

            time.sleep(STOP_PAUSE)

        else:
            # 安全，直行
            forward()

        time.sleep(CYCLE_DELAY)

# ==================== 启动入口 ====================
if __name__ == "__main__":
    print("红外边缘巡台程序启动")
    print("所有参数均可独立调节，见代码顶部")
    try:
        edge_patrol_loop()
    except KeyboardInterrupt:
        stop()
        print("程序已停止")