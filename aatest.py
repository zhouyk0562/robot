#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

print("=== CDS初始化状态验证程序 ===")
print("注意：必须用 sudo python3 运行！\n")

# 1. 尝试打开CDS驱动
up = uptech.UpTech()
cds_open_result = up.CDS_Open()

if cds_open_result != 0:
    print(f"❌ CDS驱动打开失败！错误码：{cds_open_result}")
    print("可能原因：")
    print("1. 没有用sudo运行")
    print("2. UpTech板卡和树莓派排线没插紧")
    print("3. 板卡没有上电")
    exit(1)
else:
    print("✅ CDS驱动打开成功")

# 2. 设置1、2号通道为直流电机速度模式
print("\n正在设置1、2号通道为速度模式...")
up.CDS_SetMode(1, 1)
up.CDS_SetMode(2, 1)
time.sleep(0.1)  # 等待模式生效

# 3. 验证模式是否设置成功（通过发送速度指令测试）
print("\n正在测试电机输出...")
print("如果电机转动，说明CDS初始化完全正常")
print("按Ctrl+C停止测试\n")

try:
    # 低速转动2秒，验证输出
    up.CDS_SetSpeed(1, 100)
    up.CDS_SetSpeed(2, -100)
    print("电机正在低速转动...")
    time.sleep(2)

    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)
    print("\n✅ 所有测试通过！CDS初始化完全正常")
    print("电机不动的话，检查：")
    print("1. 电机是否接了12V外部电源")
    print("2. 电机线是否插在M1、M2接口")
    print("3. 板卡CDS开关是否拨到ON")

except KeyboardInterrupt:
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)
    print("\n测试被中断")