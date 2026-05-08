 # -*- coding: utf-8 -*-
import uptech
import time
import cv2
import apriltag
import sys
import numpy as np
import threading

flag = 0
taps1 = 3
k = 0
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
        mid = 0
        global tag_width
        tag_width = 0
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
        print(k, taps1, mid, tag_width)

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
        #cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def zuo_xiang(x):
    up.CDS_SetSpeed(1,200)
    up.CDS_SetSpeed(2,int(-300-x))
    time.sleep(0.01)

def you_xiang(x):
    up.CDS_SetSpeed(1,int(300+x))
    up.CDS_SetSpeed(2,-200)
    time.sleep(0.01)

def qian_jin():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, -500)

def hou_tui():
    up.CDS_SetSpeed(1,-500)
    up.CDS_SetSpeed(2,500)

def zuo_zhuan():
    up.CDS_SetSpeed(1,-500)
    up.CDS_SetSpeed(2,-500)

def you_zhuan():
    up.CDS_SetSpeed(1,500)
    up.CDS_SetSpeed(2,500)

def ting():
    up.CDS_SetSpeed(1,0)
    up.CDS_SetSpeed(2,0)


def up_platform_parade():
    up.CDS_SetAngle(3,492,512)
    up.CDS_SetAngle(4,532,512)
    if io_data[0] == 0 and io_data[1] == 0:
        qian_jin()
        if k == 0 and flag != 0:
            flag_solve()
        elif k == 1:
            taps_solve()
            if flag != 0:
                flag_solve()
    elif io_data[0] == 1 and io_data[1] == 0:
        zuo_zhuan()
    elif io_data[0] == 0 and io_data[1] == 1:
        you_zhuan()
    elif io_data[0] == 1 and io_data[1] == 1:
        hou_tui()
        time.sleep(0.1)
        you_zhuan()
        time.sleep(0.2)

def down_platform_detect():
    if io_data[2] == 0 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 0)
        up.CDS_SetSpeed(2, 0)
        up.CDS_SetAngle(3, 712, 712)
        up.CDS_SetAngle(4, 312, 712)
        time.sleep(0.5)
        down_platform_act()
    elif io_data[2] == 0 and io_data[3] == 1:
        up.CDS_SetSpeed(1, -400)
        up.CDS_SetSpeed(2, -400)
        up.CDS_SetAngle(3, 712, 712)
        up.CDS_SetAngle(4, 312, 712)
    elif io_data[2] == 1 and io_data[3] == 0:
        up.CDS_SetSpeed(1, 400)
        up.CDS_SetSpeed(2, 400)
        up.CDS_SetAngle(3, 712, 712)
        up.CDS_SetAngle(4, 312, 712)
    else:
        up.CDS_SetSpeed(1, -500)
        up.CDS_SetSpeed(2, -500)
        up.CDS_SetAngle(3, 712, 712)
        up.CDS_SetAngle(4, 312, 712)

def down_platform_act():
    up.CDS_SetSpeed(1, -750)
    up.CDS_SetSpeed(2, 750)
    time.sleep(0.3)
    up.CDS_SetAngle(3, 712, 512)
    up.CDS_SetAngle(4, 312, 512)
    time.sleep(0.5)
    up.CDS_SetAngle(3, 312, 712)
    up.CDS_SetAngle(4, 712, 712)
    time.sleep(0.5)


def stract_warn():
    if io_data[0] == 1 or io_data[1] == 1:
        hou_tui()
        time.sleep(0.3)

def stracting():
    up.CDS_SetSpeed(1,650)
    up.CDS_SetSpeed(2,-650)

def up_platform_detect():
    adc_value = up.ADC_Get_All_Channle()
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data.clear()
    global flag
    flag = 0
    for index, value in enumerate(io_array):
        io = (int)(value)
        io_data.insert(0, io)
    while True:
        if (io_data[4] == 0 and io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
            flag = 1
            #stracting()
            #stract_warn()
        elif adc_value[3] > 50 or (io_data[5] == 1 and io_data[4] == 0):
            flag = 2
            #up.CDS_SetSpeed(1, 600)
            #time.sleep(0.05)
            #if (io_data[4] == 0 and io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
                #stracting()
                #stract_warn()
        elif adc_value[2] > 50 or (io_data[6] == 1 and io_data[4] == 0):
            flag = 3
            #up.CDS_SetSpeed(1, -600)
            #up.CDS_SetSpeed(2, -600)
            #time.sleep(0.05)
            #if (io_data[4] == 0 or io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
                #stracting()
                #stract_warn()

def taps_solve():
    global k
    global taps1
    global mid 
    global tag_width
    
    #print(k, taps1, mid, tag_width)
    if taps1 == 0 or taps1 == 1:
        if (mid < 340 - tag_width/3):
            you_xiang(tag_width)
        elif (mid > 340 + tag_width/3):
            zuo_xiang(tag_width )
        else:
            stracting()
            stract_warn()

    elif taps1 == 2:
        pass
        #k = 0

def flag_solve():
    global flag
    if flag == 1:
        stracting()

    elif flag == 2:
        up.CDS_SetSpeed(1, 600)
        up.CDS_SetSpeed(2, 600)
        time.sleep(0.05)
        if (io_data[4] == 0 and io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
            stracting()
    elif flag == 3:
        up.CDS_SetSpeed(1, -600)
        up.CDS_SetSpeed(2, -600)
        time.sleep(0.05)
        if (io_data[4] == 0 or io_data[5] == 0) or (io_data[4] == 0 and io_data[6] == 0) or io_data[4] == 0:
            stracting()

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

    target1 = threading.Thread(target = April_start_detect)
    target2 = threading.Thread(target = up_platform_detect)
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
        #print(adc_value[1])

        for index, value in enumerate(io_array):
            io = (int)(value)
            io_data.insert(0, io)
        #print(io_data[0],io_data[1],io_data[2],io_data[3])
        #print(io_data[4],io_data[5],io_data[6],io_data[7])
        #print(adc_value[3],adc_value[1],adc_value[4])

        #zuo_zhuan()
        #taps_solve()


        if adc_value[1] > 1600:
            up_platform_parade()
        elif adc_value[1] < 1600:
            down_platform_detect()