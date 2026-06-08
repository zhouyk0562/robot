#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# ---------- 硬件初始化 ----------
up = uptech.UpTech()
up.CDS_Open()
up.ADC_IO_Open()

# ---------- 灰度校正参数（基于你测的台下数据）----------
# 校正系数 [前, 后, 左, 右] ，使台下值都变成约 1000
GRAY_COEFF = [1.2195, 0.625, 1.9231, 1.0]
# 判断阈值：校正后平均值低于此值认为是台下
OFF_TABLE_AVG_THRESHOLD = 1025

print("灰度台上/台下检测程序")
print("校正系数：前=1.2195 后=0.625 左=1.9231 右=1.0")
print(f"判断阈值：校正后平均值 < {OFF_TABLE_AVG_THRESHOLD} → 台下")
print("按 Ctrl+C 退出\n")
print("  前校正   后校正   左校正   右校正   |  平均值  状态")
print("-" * 60)

try:
    while True:
        # 读取原始灰度（adc[1]前, adc[0]后, adc[2]左, adc[3]右）
        adc = up.ADC_Get_All_Channle()
        raw_front = adc[1]
        raw_rear  = adc[0]
        raw_left  = adc[2]
        raw_right = adc[3]

        # 应用校正系数
        adj_front = raw_front * GRAY_COEFF[0]
        adj_rear  = raw_rear  * GRAY_COEFF[1]
        adj_left  = raw_left  * GRAY_COEFF[2]
        adj_right = raw_right * GRAY_COEFF[3]

        # 计算平均值并判断
        avg = (adj_front + adj_rear + adj_left + adj_right) / 4.0
        status = "台上" if avg >= OFF_TABLE_AVG_THRESHOLD else "台下"

        # 输出结果
        print(f"{adj_front:7.1f} {adj_rear:7.1f} {adj_left:7.1f} {adj_right:7.1f}   | {avg:6.1f}   {status}")

        time.sleep(2)   # 每秒更新两次，可根据需要调整

except KeyboardInterrupt:
    print("\n程序已退出")
    up.ADC_IO_Close()