# -*- coding: utf-8 -*-
import uptech
import time
from myThread import MyThread

up = uptech.UpTech()
up.LCD_Open(2)
up.ADC_IO_Open()
up.ADC_Led_SetColor(0, 0x2F0000)
up.ADC_Led_SetColor(1, 0x002F00)
up.CDS_Open()
up.CDS_SetMode(1, 1)
up.CDS_SetMode(2, 1)
up.CDS_SetMode(3, 0)
up.CDS_SetMode(4, 0)
up.LCD_PutString(40, 0, 'formal')
up.LCD_Refresh()
up.LCD_SetFont(up.FONT_6X10)
io_data = []


def stracting():
    up.CDS_SetSpeed(1, 999)
    up.CDS_SetSpeed(2, -999)
    if adc_value[1] < 1900:
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, -600)
        time.sleep(0.1)
        # parade_io()


def searching():
    # up.CDS_SetAngle(3,512,512)
    # up.CDS_SetAngle(4,512,512)
    if io_data[4] == 0 and io_data[5] == 0 and io_data[6] == 0:
        up.CDS_SetSpeed(1, 999)
        up.CDS_SetSpeed(2, -999)
        # parade_io()
        if adc_value[1] < 1900:
            up.CDS_SetSpeed(1, -600)
            up.CDS_SetSpeed(2, 600)
            time.sleep(0.1)
        # parade_io()


def brodside():
    up.CDS_SetAngle(3, 492, 512)  # 角度变大舵机向上
    up.CDS_SetAngle(4, 532, 512)  # 角度变小舵机向下
    if io_data[0] == 0 and io_data[1] == 0:
        up.CDS_SetSpeed(1, 550)
        up.CDS_SetSpeed(2, -550)

    elif (io_data[0] == 0 and io_data[1] == 1) or adc_value[1] < 1750:
        up.CDS_SetSpeed(1, 600)
        up.CDS_SetSpeed(2, 600)
        # time.sleep(0.01)
    elif (io_data[0] == 1 and io_data[1] == 0) or adc_value[1] < 1750:
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, -600)
        # time.sleep(0.01)
    elif (io_data[0] == 1 and io_data[1] == 1) or adc_value[1] < 1750:
        # up.CDS_SetSpeed(1,-600)
        # up.CDS_SetSpeed(2,600)
        # time.sleep(0.1)
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, -600)
        time.sleep(0.2)

    # if io_data[4] == 0 or io_data[5] == 0 or io_data[6] == 0:
    if (io_data[4] == 0 and io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
        stracting()
    elif adc_value[3] > 50 or (io_data[5] == 1 and io_data[4] == 0):
        up.CDS_SetSpeed(1, 600)
        up.CDS_SetSpeed(2, 600)
        if (io_data[4] == 0 and io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
            stracting()
    elif adc_value[2] > 50 or (io_data[6] == 1 and io_data[4] == 0):
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, -600)
        if (io_data[4] == 0 or io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
            stracting()


def parade_io():
    # 舵机->贴地
    up.CDS_SetAngle(3, 492, 512)  # 角度变大舵机向上
    up.CDS_SetAngle(4, 532, 512)  # 角度变小舵机向下
    if io_data[0] == 0 and io_data[1] == 0:
        up.CDS_SetSpeed(1, 500)
        up.CDS_SetSpeed(2, -500)

    elif (io_data[0] == 0 and io_data[1] == 1) or adc_value[1] < 1700:
        up.CDS_SetSpeed(1, 500)
        up.CDS_SetSpeed(2, 500)
        # time.sleep(0.01)
    elif (io_data[0] == 1 and io_data[1] == 0) or adc_value[1] < 1700:
        up.CDS_SetSpeed(1, -500)
        up.CDS_SetSpeed(2, -500)
        # time.sleep(0.01)
    elif io_data[0] == 1 and io_data[1] == 1:
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, 600)
        # time.sleep(0.2)
        # up.CDS_SetSpeed(1,-500)
        # up.CDS_SetSpeed(2,-500)
        # time.sleep(0.2)


def steering():
    up.CDS_SetSpeed(1, -750)  # 左
    up.CDS_SetSpeed(2, 750)  # 右
    time.sleep(0.3)
    up.CDS_SetAngle(3, 712, 512)  # 角度变大舵机向上
    up.CDS_SetAngle(4, 312, 512)  # 角度变小舵机向下
    time.sleep(0.5)
    up.CDS_SetAngle(3, 312, 712)  # 角度变大舵机向上
    up.CDS_SetAngle(4, 712, 712)  # 角度变小舵机向下
    time.sleep(0.5)
    # up.CDS_SetAngle(3,512,712)#角度变大舵机向上
    # up.CDS_SetAngle(4,512,712)#角度变小舵机向下


def goup():
    if io_data[2] == 0 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 0)
        up.CDS_SetSpeed(2, 0)
        up.CDS_SetAngle(3, 712, 712)  # 角度变大舵机向上
        up.CDS_SetAngle(4, 312, 712)  # 角度变小舵机向下
        time.sleep(0.5)
        steering()
        # up.CDS_SetSpeed(1, -750)#左
        # up.CDS_SetSpeed(2, 750)#右
        # time.sleep(0.5)
        # if adc_value[6] < 1500:
        # goup()
    elif io_data[2] == 0 and io_data[3] == 1:
        up.CDS_SetSpeed(1, -400)
        up.CDS_SetSpeed(2, -400)
        up.CDS_SetAngle(3, 712, 712)  # 角度变大舵机向上
        up.CDS_SetAngle(4, 312, 712)  # 角度变小舵机向下
    elif io_data[2] == 1 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 400)
        up.CDS_SetSpeed(2, 400)
        up.CDS_SetAngle(3, 712, 712)  # 角度变大舵机向上
        up.CDS_SetAngle(4, 312, 712)  # 角度变小舵机向下
    else:
        up.CDS_SetSpeed(1, -500)
        up.CDS_SetSpeed(2, -500)
        up.CDS_SetAngle(3, 712, 712)  # 角度变大舵机向上
        up.CDS_SetAngle(4, 312, 712)  # 角度变小舵机向下


while True:
    adc_value = up.ADC_Get_All_Channle()
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data.clear()

    for index, value in enumerate(io_array):
        io = (int)(value)
        io_data.insert(0, io)
    if io_data[0] == 0 and io_data[1] == 0:
        break

while True:
    adc_value = up.ADC_Get_All_Channle()
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data.clear()

    for index, value in enumerate(io_array):
        io = (int)(value)
        io_data.insert(0, io)

    # print("adc_value : {}".format(adc_value))
    # print("io_value : {}".format(io_data))
    print(adc_value[0], adc_value[1])  # ad_value[0]台下黑色区域1600以下
    print(adc_value[2], adc_value[3])  # 2->left 3->right
    print(io_data[0], io_data[1], io_data[2], io_data[3])
    print(io_data[4], io_data[5], io_data[6], io_data[7])
    # time.sleep(2)

    # 正方向
    # 速度都为正->右转
    # up.CDS_SetSpeed(1, 500)
    # up.CDS_SetSpeed(2, -500)
    # 舵机->抬起
    # up.CDS_SetAngle(3,482,712)
    # up.CDS_SetAngle(4,542,712)
    if adc_value[1] > 1600:
        # parade_io()
        # time.sleep(0.1)
        # brodside()
        # time.sleep(0.1)
        # searching()
        brodside()

    elif adc_value[1] < 1600:
        goup()
        time.sleep(0.2)
        # if io_data[0] == 1 and io_data[1] == 1:
        # up.CDS_SetSpeed(1,-500)
        # up.CDS_SetSpeed(2,500)
        if adc_value[1] > 1900:
            up.CDS_SetSpeed(1, -500)
            up.CDS_SetSpeed(2, 500)
            # time.sleep(0.1)
        # else:
        # searching()
        # brodside()