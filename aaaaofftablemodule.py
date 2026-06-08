#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
掉台恢复独立模块
负责人：[你的名字]
功能：掉台检测 + 步进扫描 + 冲台恢复 + 重试机制
版本：v1.1（基于2026中国机器人大赛仿人散打场地优化）
最后修改：2026-06-08
"""

import time

# ==============================
# 【你负责修改的区域】所有参数都在这里
# ==============================
# 掉台判断阈值（四路灰度全部低于此值视为掉台）
THRESHOLD_OFF_TABLE = 1500

# 掉台恢复速度参数
OFF_TABLE_FORWARD_SPEED = 600  # 靠近台面的前进速度
OFF_TABLE_RUSH_SPEED = -1000  # 冲台速度（负值为后退，根据实际情况调整）
OFF_TABLE_TURN_SPEED = 600  # 扫描旋转速度
OFF_TABLE_BACK_SPEED = -600  # 步进后退速度

# 掉台恢复时间参数（基于70cm台下区域优化）
OFF_TABLE_FAST_SCAN_TIME = 0.3  # 快速扫描阶段单次左转时间
OFF_TABLE_PRECISE_SCAN_TIME = 0.15  # 精准对准阶段单次左转时间
OFF_TABLE_STEP_BACK_TIME = 0.3  # 步进后退时间（防止撞到围栏）
OFF_TABLE_APPROACH_TIME = 0.7  # 靠近台面的前进时间
OFF_TABLE_RUSH_TIME = 1.0  # 冲台持续时间（优化为1.6秒，覆盖6cm高度差）
OFF_TABLE_RETRY_DELAY = 0.5  # 冲台失败后重试延迟

# 最大重试次数
MAX_RETRY_COUNT = 3

# ==============================
# 【全局变量，不要修改】
# ==============================
_up = None
_set_motors = None
_stop = None
_forward = None
_backward = None
_turn_left = None
_read_all_grays = None
_read_front_ir = None
_read_edge_ir = None


# ==============================
# 【模块初始化函数，你的队友只需要调用一次】
# ==============================
def init(up_instance, set_motors_func, stop_func, forward_func, backward_func, turn_left_func, read_all_grays_func,
         read_front_ir_func, read_edge_ir_func):
    """
    初始化掉台恢复模块，将主程序的基础函数传递进来
    这样就不会有重复代码，修改主程序的函数这里会自动同步
    """
    global _up, _set_motors, _stop, _forward, _backward, _turn_left
    global _read_all_grays, _read_front_ir, _read_edge_ir

    _up = up_instance
    _set_motors = set_motors_func
    _stop = stop_func
    _forward = forward_func
    _backward = backward_func
    _turn_left = turn_left_func
    _read_all_grays = read_all_grays_func
    _read_front_ir = read_front_ir_func
    _read_edge_ir = read_edge_ir_func

    print("掉台恢复模块初始化完成")


# ==============================
# 【内部函数，你的队友不需要关心】
# ==============================
def _is_off_table():
    """内部掉台判断函数"""
    fg, rg, lg, rgg = _read_all_grays()
    return (fg < THRESHOLD_OFF_TABLE and rg < THRESHOLD_OFF_TABLE and
            lg < THRESHOLD_OFF_TABLE and rgg < THRESHOLD_OFF_TABLE)


def _scan_for_target():
    """两阶段扫描目标：快速扫描+精准对准"""
    print("开始两阶段目标扫描...")

    # 第一阶段：快速扫描，快速覆盖360度
    print("第一阶段：快速扫描")
    for _ in range(12):  # 12×0.3s=3.6s，覆盖360度
        _turn_left()
        time.sleep(OFF_TABLE_FAST_SCAN_TIME)
        _stop()
        time.sleep(0.05)

        fl, fr = _read_front_ir()
        if fl == 0 and fr == 0:
            print("快速扫描发现目标，进入精准对准")
            break

        # 步进后退，防止撞到围栏
        _backward()
        time.sleep(OFF_TABLE_STEP_BACK_TIME)
        _stop()
        time.sleep(0.05)

    # 第二阶段：精准对准，微调角度直到双亮
    print("第二阶段：精准对准")
    for _ in range(10):
        fl, fr = _read_front_ir()
        if fl == 0 and fr == 0:
            print("精准对准完成")
            return True

        _turn_left()
        time.sleep(OFF_TABLE_PRECISE_SCAN_TIME)
        _stop()
        time.sleep(0.05)

    print("扫描超时，未找到目标")
    return False


def _rush_to_table():
    """单次冲台动作"""
    print("开始冲台...")

    # 前进靠近台面
    _forward()
    time.sleep(OFF_TABLE_APPROACH_TIME)
    _stop()
    time.sleep(0.1)

    # 全力后退冲台
    _set_motors(OFF_TABLE_RUSH_SPEED, OFF_TABLE_RUSH_SPEED)
    time.sleep(OFF_TABLE_RUSH_TIME)
    _stop()
    time.sleep(0.2)

    # 检查是否成功上台
    if not _is_off_table():
        print("冲台成功！")
        return True
    else:
        print("冲台失败")
        return False


# ==============================
# 【唯一对外接口，你的队友只需要调用这个】
# ==============================
def off_table_check_and_recover():
    """
    掉台检查和恢复主函数
    返回值：True表示发生了掉台并处理完成，False表示没有掉台
    你的队友只需要在主循环最开头调用这个函数即可
    """
    if not _is_off_table():
        return False

    print("\n===== 检测到掉台，启动恢复程序 =====")
    _stop()
    time.sleep(0.1)

    retry_count = 0
    while retry_count < MAX_RETRY_COUNT:
        print(f"\n第 {retry_count + 1} 次尝试")

        # 扫描目标
        if not _scan_for_target():
            retry_count += 1
            continue

        # 冲台
        if _rush_to_table():
            print("===== 掉台恢复完成 =====\n")
            return True

        retry_count += 1
        time.sleep(OFF_TABLE_RETRY_DELAY)

    print("\n===== 所有重试失败，请手动干预 =====\n")
    return True