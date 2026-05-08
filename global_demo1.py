# -*- coding: utf-8 -*-
import uptech
import time
import cv2
import apriltag
import sys
import numpy as np
import threading

flag = 0
shangtai = 0
out = 0
tui_con = 0
zhangaiflag = 0
tag_zhangai = 0

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

io0 = 6
io1 = 6
io2 = 6
io3 = 6
io4 = 6
io5 = 6
io6 = 6

adc1 = 0


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
    up.CDS_SetSpeed(1, -310)
    up.CDS_SetSpeed(2, -310)


def you_zhuan_tag():
    up.CDS_SetSpeed(1, 310)
    up.CDS_SetSpeed(2, 310)


# 320

def zuo_wei_zhuan():
    up.CDS_SetSpeed(1, -250)
    up.CDS_SetSpeed(2, -250)


def you_wei_zhuan():
    up.CDS_SetSpeed(1, 250)
    up.CDS_SetSpeed(2, 250)


def ting():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


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

        #   print(tags)
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
        #            cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
        #            cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
        #            cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
        #            cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
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

        #        cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def adio_start_detect():
    global zhangaiflag
    global tag_zhangai
    global adc1
    global io0
    global io1
    global io2
    global io3
    global io4
    global io5
    global io6
    global tui_con
    global out
    global taps1
    io_data = []

    while True:
        adc_value = up.ADC_Get_All_Channle()
        adc1 = adc_value[1]
        io_all_input = up.ADC_IO_GetAllInputLevel()
        io_array = '{:08b}'.format(io_all_input)
        io_data.clear()
        for index, value in enumerate(io_array):
            io = (int)(value)
            io_data.insert(0, io)
        io0 = io_data[0]
        io1 = io_data[1]
        io2 = io_data[2]
        io3 = io_data[3]
        io4 = io_data[4]
        io5 = io_data[5]
        io6 = io_data[6]
        # print('adio线程的io0 = ',io0)
        if io0 == 0 and io1 == 0:
            out = 1
        elif io0 == 1 and io1 == 0:
            out = 2  # zuo_zhuan
        elif io0 == 0 and io1 == 1:
            out = 3  # you_zhuan
        elif io0 == 1 and io1 == 1:
            out = 4  # hou_tui+you_zhuan

        if (io2 == 0) or (io3 == 0):
            tui_con = 1  # tui_ma_kuai
            # print('adio的tui_con  是 1 撒   ')
        elif io2 == 1 and io3 == 1:
            tui_con = 0
            if io4 == 0:
                tag_zhangai = 1
            elif io5 == 0:
                tag_zhangai = 2
            elif io6 == 0:
                tag_zhangai = 3

        if io4 == 0:
            zhangaiflag = 1     #zuo_wei_zhuan()
        elif io5 == 0:
            zhangaiflag = 2     #man_tui()
        elif io6 == 0:
            zhangaiflag = 3     #you_wei_zhuan()


def tag_find():
    global k
    i = 0
    for i in range(3):
        you_zhuan()
        time.sleep(0.1)
        ting()
        time.sleep(0.1)
        #        if k == 1:
        #            ting()
        tag_solve()


def tag_solve():
    global c
    global tag_width
    global taps1
    global tui_con
    global adc1
    global flag
    #  print('flag= ',flag)
    # print(k, taps1, mid, tag_width)
    # ting()
    while k == 1:
        if taps1 == 1 or taps1 == 0:
            if mid < 350 - tag_width / 6:
                you_zhuan_tag()
                time.sleep(0.03)
                ting()
                # time.sleep(0.01)
                print("zuo")
            elif mid > 350 + tag_width / 6:
                zuo_zhuan_tag()
                time.sleep(0.03)
                ting()
                # time.sleep(0.01)
                print("you")
            else:
                ting()
                time.sleep(0.5)
                print('我对准方块了')

                for c in range(6):
                    up.CDS_SetSpeed(1, 550)
                    up.CDS_SetSpeed(2, -550)
                    time.sleep(0.1)
                flag = 1
        elif taps1 == 2:
            you_zhuan()
            time.sleep(0.1)


def up_platform_parade():
    global tui_con
    global k
    global out
    global shangtai
    # print(k)
    # print('out: ',out)
    up.CDS_SetAngle(3, 445, 512)  # 406贴地  #434略贴地  #445 #470 #569水平抬起
    up.CDS_SetAngle(4, 580, 512)  # 621      #599      #580 #560 #469
    if shangtai == 1:
        print('shangtai: ', shangtai)
        hou_tui()
        time.sleep(0.2)
        shangtai = 0

    if out == 1:
        qian_jin()
        print('前进')
        tag_solve()
        if tag_zhangai == 1:
            zuo_wei_zhuan()
        elif tag_zhangai == 2:
            up.CDS_SetSpeed(1, 350)
            up.CDS_SetSpeed(2, -350)
        elif tag_zhangai == 3:
            you_wei_zhuan()
    elif out == 2:
        zuo_zhuan()
        print('右转')
        time.sleep(0.1)
        # tag_find()
    elif out == 3:
        you_zhuan()
        time.sleep(0.1)
        # tag_find()
    elif out == 4:
        hou_tui()
        time.sleep(0.1)
        you_zhuan()
        time.sleep(0.2)


#    if tui_con == 1:
#        up.CDS_SetSpeed(1, 200)
#        up.CDS_SetSpeed(2, -200)
#        if adc_value[1] < 1750:
#            ting()


#    elif tui_con == 3:
#        you_wei_zhuan()
#        time.sleep(0.1)
#    elif tui_con == 4:
#        zuo_wei_zhuan()
#        time.sleep(0.1)
#
def down_platform_detect():
    global io0
    global io1
    global io2
    global io3
    global io4
    global io5
    global io6
    global shangtai
    if io2 == 0 and io3 == 0:
        up.CDS_SetSpeed(1, 0)
        up.CDS_SetSpeed(2, 0)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
        time.sleep(0.5)
        up.CDS_SetSpeed(1, 500)
        up.CDS_SetSpeed(2, -500)
        time.sleep(0.5)
        down_platform_act()
    elif io2 == 0 and io3 == 1:
        up.CDS_SetSpeed(1, -350)
        up.CDS_SetSpeed(2, -350)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
    elif io2 == 1 and io3 == 0:
        up.CDS_SetSpeed(1, 350)
        up.CDS_SetSpeed(2, 350)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
    else:
        up.CDS_SetSpeed(1, -400)
        up.CDS_SetSpeed(2, -400)
        up.CDS_SetAngle(3, 569, 712)
        up.CDS_SetAngle(4, 469, 712)
    shangtai = 1  # 为了上台后可以转


def down_platform_act():
    up.CDS_SetSpeed(1, -800)
    up.CDS_SetSpeed(2, 800)
    time.sleep(0.3)
    up.CDS_SetAngle(3, 594, 512)
    up.CDS_SetAngle(4, 388, 512)
    time.sleep(0.5)
    up.CDS_SetAngle(3, 348, 712)
    up.CDS_SetAngle(4, 684, 712)
    time.sleep(0.5)

def zhangai_iodetect_act():
    global zhangaiflag
    if zhangaiflag == 1:
        zuo_wei_zhuan()
    elif zhangaiflag == 2:
        up.CDS_SetSpeed(1, 350)
        up.CDS_SetSpeed(2, -350)
    elif zhangaiflag == 3:
        you_wei_zhuan()

def zhangai_act():
    global out
    global shangtai
    # print(k)
    # print('out: ',out)
    up.CDS_SetAngle(3, 569, 512)  # 406贴地  #434略贴地
    up.CDS_SetAngle(4, 469, 512)  # 621      #599
    # up.CDS_SetAngle(3, 569, 512)
    # up.CDS_SetAngle(4, 469, 512)
    if shangtai == 1:
        up.CDS_SetSpeed(1, -500)
        up.CDS_SetSpeed(2, 500)
        time.sleep(0.2)
        shangtai = 0

    if out == 1:
        qian_jin()
        print('前进')
        zhangai_iodetect_act()
    elif out == 2:
        zuo_zhuan()
        print('右转')
        zhangai_iodetect_act()
    elif out == 3:
        you_zhuan()
        zhangai_iodetect_act()
    elif out == 4:
        hou_tui()
        time.sleep(0.1)
        you_zhuan()
        time.sleep(0.1)


if __name__ == "__main__":
    workcon = 0
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

    target1 = threading.Thread(target=April_start_detect)
    target2 = threading.Thread(target=adio_start_detect)
    target1.start()
    target2.start()

    while True:
        if io0 == 0 and io1 == 0:
            break

    while True:
        if work_con != 2:
            while flag == 0:
                if adc1 > 1620:
                    print('adc1 = ', adc1)
                    up_platform_parade()
                elif adc1 < 1620:
                    down_platform_detect()
            while flag == 1:
                if tui_con == 1:
                    print('adio的k: ', k)
                    print('主线程的tui__con: ', tui_con)
                    print('箱子看不到了，但我还要推')
                    up.CDS_SetSpeed(1, 380)
                    up.CDS_SetSpeed(2, -380)
                elif tui_con == 0:
                    ting()
                    time.sleep(0.5)
                    hou_tui()
                    time.sleep(0.5)
                    flag = 0
                    work_con += 1
        elif wor_con == 2:
            while True:
                if adc1 > 1570:
                    zhangai_act()
                elif adc1 < 1570:
                    down_platform_detect()
