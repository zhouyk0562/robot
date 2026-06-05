#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time

def main():
    # 初始化硬件
    up = uptech.UpTech()
    up.ADC_IO_Open()          # 开启 ADC 和 IO 接口
    print("ADC_IO 已打开，开始监测 IO 口状态...")
    print("监测接口：io0, io1, io2, io3, io4, io6")
    print("按 Ctrl+C 退出程序\n")

    try:
        while True:
            # 读取全部 8 路 IO 输入（返回一个整数，每 bit 代表一路）
            io_all = up.ADC_IO_GetAllInputLevel()

            # 解析出需要的 IO 口（位运算提取对应 bit）
            io0 = (io_all >> 0) & 1
            io1 = (io_all >> 1) & 1
            io2 = (io_all >> 2) & 1
            io3 = (io_all >> 3) & 1
            io4 = (io_all >> 4) & 1
            io6 = (io_all >> 6) & 1

            # 打印当前状态
            print(f"[{time.strftime('%H:%M:%S')}] "
                  f"io0={io0}, io1={io1}, io2={io2}, io3={io3}, io4={io4}, io6={io6}")

            time.sleep(1)   # 每 0.5 秒刷新一次

    except KeyboardInterrupt:
        print("\n监测已停止，程序退出。")

if __name__ == "__main__":
    main()