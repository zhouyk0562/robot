# -*- coding: utf-8 -*-
import uptech
import time
import cv2
import apriltag
import sys
import numpy as np
import threading

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

tui_con = 0 #推箱子的触发条件（包含近点检测不到二维码的情况）
tag_width = 0
taps1 = 3
k = 0
mid = 0
h0 = 0
h1 = 0
h2 = 0
m1 = 0
m0 = 0
m2 = 0
mx0 = 0
mx1 = 0
mx2 = 0
mid0 = 0
mid1 = 0
mid2 = 0


class ApriltagDetect:
    def __init__(self):
        self.target_id = 0
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))

    def update_frame(self, frame):
        h0 = 0  # shi fou you 0 ma
        h1 = 0  # shi fou you 1 ma
        h2 = 0
        m1 = 0  # zui zhong xin 1 ma zhong xin zuo biao
        m0 = 0  # zui zhong xin 0 ma zhong xin zuo biao
        m2 = 0
        mx0 = 0
        mx1 = 0  # ma zhi zhong xin zuo biao
        mx2 = 0
        mid0 = 0
        mid1 = 0
        mid2 = 0
        global taps1
        global k
        global mid
        global tag_width
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        k = 0

        for tag in tags:
            k = 1
            # print('tagid',tag.tag_id)

            if (tag.tag_id == 1):
                mx1 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
                if (mx1 > m1):
                    m1 = mx1
                    mid1 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
                h1 = 1
            elif (tag.tag_id == 0):
                mx0 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
                if (mx0 > m0):
                    m0 = mx0
                    mid0 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
                h0 = 1
            elif (tag.tag_id == 2):
                mx2 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
                if (mx2 > m2):
                    m2 = mx2
                    mid2 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
                h2 = 1
            # cv2.circle(frame,), 4, (255, 0, 0), 2) # left-top
            # cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            # cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            # cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
        # apriltag_width = abs(tag.corners[0][0] - tag.corners[1][0]) / 2
        # target_x = apriltag_width / 2
        if (h1 == 1):
            mid = mid1
            tag_width = m1
            taps1 = 1
        elif (h0 == 1 and h1 == 0):
            mid = mid0
            tag_width = m0
            taps1 = 0
        elif (h1 == 0 and h0 == 0 and h2 == 1):
            mid = mid2
            tag_width = m2
            taps1 = 2
        # print(k, taps1, mid, tag_width)


def April_start_detect():
    global frame
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    ad = ApriltagDetect()
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        # cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def up_platform_parade():
    global tui_con
    global k
    print(k)
    up.CDS_SetAngle(3, 406, 512)
    up.CDS_SetAngle(4, 621, 512)
    # up.CDS_SetAngle(3, 569, 512)
    # up.CDS_SetAngle(4, 469, 512)
    if io_data[0] == 0 and io_data[1] == 0:
        qian_jin()
        if k == 1:
            ting()
            tag_solve()
        # elif k == 0:
        # if io_data[2] == 0 or io_data[3] == 0:
        # qian_jin()
    elif io_data[0] == 1 and io_data[1] == 0:
        zuo_zhuan()
        time.sleep(0.1)
        tag_find()
    elif io_data[0] == 0 and io_data[1] == 1:
        you_zhuan()
        time.sleep(0.1)
        tag_find()
    elif io_data[0] == 1 and io_data[1] == 1:
        hou_tui()
        time.sleep(0.1)
        you_zhuan()
        time.sleep(0.2)
    if tui_con == 1:
        up.CDS_SetSpeed(1, 300)
        up.CDS_SetSpeed(2, -300)
        if adc_value[1] < 1750:
            ting()


def down_platform_detect():
    if io_data[2] == 0 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 0)
        up.CDS_SetSpeed(2, 0)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
        time.sleep(0.5)
        down_platform_act()
    elif io_data[2] == 0 and io_data[3] == 1:
        up.CDS_SetSpeed(1, -400)
        up.CDS_SetSpeed(2, -400)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
    elif io_data[2] == 1 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 400)
        up.CDS_SetSpeed(2, 400)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
    else:
        up.CDS_SetSpeed(1, -500)
        up.CDS_SetSpeed(2, -500)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)


def down_platform_act():
    up.CDS_SetSpeed(1, -750)
    up.CDS_SetSpeed(2, 750)
    time.sleep(0.3)
    up.CDS_SetAngle(3, 594, 512)
    up.CDS_SetAngle(4, 388, 512)
    time.sleep(0.5)
    up.CDS_SetAngle(3, 348, 712)
    up.CDS_SetAngle(4, 684, 712)
    time.sleep(0.5)


def zuo_zhuan():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, -400)


def you_zhuan():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, 400)


def qian_jin():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, -500)


def hou_tui():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, 500)


def zuo_zhuan_tag():
    up.CDS_SetSpeed(1, -320)
    up.CDS_SetSpeed(2, -320)


def you_zhuan_tag():
    up.CDS_SetSpeed(1, 320)
    up.CDS_SetSpeed(2, 320)


def ting():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)

def tag_find():
    global k
    i = 0
    for i in range(5):
        you_zhuan()
        time.sleep(0.1)
        ting()
        time.sleep(0.1)
        if k == 1:
            ting()
            tag_solve()

def ffs_start_detect():
    global tui_con
    if io_data[2] == 0 or io_data[3] == 0\
            or io_data[4] == 0 or io_data[5] == 0 or io_data[6] == 0:
        tui_con = 1


def tag_solve():
    global c
    global tag_width
    global taps1

    print(k, taps1, mid, tag_width)
    # ting()
    if taps1 == 1 or taps1 == 0:
        if mid < 380 - tag_width / 6:
            you_zhuan_tag()
            time.sleep(0.05)
            ting()
            # time.sleep(0.03)
            print("zuo")
        elif mid > 380 + tag_width / 6:
            zuo_zhuan_tag()
            time.sleep(0.05)
            ting()
            # time.sleep(0.03)
            print("you")
        else:
            ting()
            time.sleep(0.1)
            for c in range(6):
                up.CDS_SetSpeed(1, 550)
                up.CDS_SetSpeed(2, -550)
                time.sleep(0.1)

    elif taps1 == 2:
        you_zhuan()
        time.sleep(0.2)


if __name__ == "__main__":
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
    target1 = threading.Thread(target=April_start_detect)
    target2 = threading.Thread(target=ffs_start_detect)
    target1.start()
    target2.start()

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
        print(adc_value[1])

        for index, value in enumerate(io_array):
            io = (int)(value)
            io_data.insert(0, io)

        if adc_value[1] > 1600:
            up_platform_parade()
        elif adc_value[1] < 1600:
            down_platform_detect()