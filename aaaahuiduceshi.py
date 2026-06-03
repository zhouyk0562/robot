#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

# 初始化
up = uptech.UpTech()
up.CDS_Open()        # 如果只测灰度，其实只需要 ADC_IO_Open
up.ADC_IO_Open()     # 打开 ADC 扩展板

print("四路灰度传感器测试开始，按 Ctrl+C 退出\n")
print("时间(s)\t通道0\t通道1\t通道2\t通道3")
print("-" * 50)

try:
    start_time = time.time()
    while True:
        # 读取全部 10 个 ADC 通道
        adc = up.ADC_Get_All_Channle()

        # 提取我们需要的四个通道 (0~3)
        ch0, ch1, ch2, ch3 = adc[0], adc[1], adc[2], adc[3]

        # 计算已经运行的时间
        elapsed = time.time() - start_time

        # 打印结果（对齐美观）
        print(f"{elapsed:6.1f}\t{ch0:5d}\t{ch1:5d}\t{ch2:5d}\t{ch3:5d}")

        time.sleep(1)  # 每秒打印 10 次，可根据需要调整

except KeyboardInterrupt:
    print("\n测试结束，程序已退出。")
    # 可选：关闭 ADC，释放资源
    up.ADC_IO_Close()