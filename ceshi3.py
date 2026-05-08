'''import time
import uptech
import apriltag
import cv2
#import atag
import numpy as np

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
global value
global t
global t0
global t1
global t2
global t3
global t4
global t5
global t6
global flag
global tflag
global zhangai
global qi_dong
global distance
global distance_flag
global notflag
io_data = []
def xiao_hou_tui():
    up.CDS_SetSpeed(3, -370)
    up.CDS_SetSpeed(4, 370)

def hou_you_zhuan():
    up.CDS_SetSpeed(3, -400)
    up.CDS_SetSpeed(4, 500)
    print('后右转')

def hou_zuo_zhuan():
    up.CDS_SetSpeed(3, -300)
    up.CDS_SetSpeed(4, 100)
    print('后左转')
def xiao_you_zhuan():
    up.CDS_SetSpeed(3, -390)
    up.CDS_SetSpeed(4, -390)
    print('慢右转')

def xiao_zuo_zhuan():
    up.CDS_SetSpeed(3, 300)
    up.CDS_SetSpeed(4, 300)
    print('慢右转')

def quan_zuo_zhuan():
    up.CDS_SetSpeed(3, -800)
    up.CDS_SetSpeed(4, -800)
    print('全速左转')

def quan_you_zhuan():
    up.CDS_SetSpeed(3, 800)
    up.CDS_SetSpeed(4, 800)
    print('全速右转')

def quan_su():
    up.CDS_SetSpeed(3, 500)
    up.CDS_SetSpeed(4, -500)
    print('全速')

def ting_zhi():
    up.CDS_SetSpeed(3, 9)
    up.CDS_SetSpeed(4, 9)
    print('停止')


def zuo_zhuan():
    up.CDS_SetSpeed(3, -520)
    up.CDS_SetSpeed(4, -520)
    print('左转')


def you_zhuan():
    up.CDS_SetSpeed(3, 520)
    up.CDS_SetSpeed(4, 520)
    print('右转')

def di_su():
     up.CDS_SetSpeed(3, 250)
     up.CDS_SetSpeed(4, -250)
     print('disu前进')

def qian_jin():
    up.CDS_SetSpeed(3, 350)
    up.CDS_SetSpeed(4, -350)
    print('前进')

def hou_tui():
    up.CDS_SetSpeed(3, -600)
    up.CDS_SetSpeed(4, 600)
    print('后退')

def quan_hou_tui():
    up.CDS_SetSpeed(3, -500)
    up.CDS_SetSpeed(4, 500)
    print('全速后退')

def shun_you_zhuan():
    up.CDS_SetSpeed(3, 480)
    up.CDS_SetSpeed(4, -400)
    print('顺右转')

def shun_zuo_zhuan():
    up.CDS_SetSpeed(3, 400)
    up.CDS_SetSpeed(4, -480)
    print('顺左转')

class Atag:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

    def detect(self, gray):
        return self.detector.detect(gray)

    def get_id(self, results):
        result = results[0]
        return result.tag_id

    def get_distance(self, H, t):
            """
            :param H: homography matrix
            :param t: ???
            :return: distance
            """
            ss = 0.5
            src = np.array([[-ss, -ss, 0],
                            [ss, -ss, 0],
                            [ss, ss, 0],
                            [-ss, ss, 0]])
            Kmat = np.array([[700, 0, 0],
                             [0, 700, 0],
                             [0, 0, 1]]) * 1.0
            disCoeffs = np.zeros([4, 1]) * 1.0
            ipoints = np.array([[-1, -1],
                                [1, -1],
                                [1, 1],
                                [-1, 1]])
            for point in ipoints:
                x = point[0]
                y = point[1]
                z = H[2, 0] * x + H[2, 1] * y + H[2, 2]
                point[0] = (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / z * 1.0
                point[1] = (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / z * 1.0
            campoint = ipoints * 1.0
            opoints = np.array([[-1.0, -1.0, 0.0],
                                [1.0, -1.0, 0.0],
                                [1.0, 1.0, 0.0],
                                [-1.0, 1.0, 0.0]])
            opoints = opoints * 0.5
            rate, rvec, tvec = cv2.solvePnP(opoints, campoint, Kmat, disCoeffs)
            point, jac = cv2.projectPoints(src, np.zeros(rvec.shape), tvec, Kmat, disCoeffs)
            points = np.int32(np.reshape(point, [4, 2]))
            distance = np.abs(t / np.linalg.norm(points[0] - points[1]))
            return distance

def April_start_detect():
    m0=0
    m1=0
    m2=0
    mx0 = 0
    mx1 = 0
    mx2 = 0
    mid0=0
    mid1=0
    mid2=0
    atag = Atag()  # 实例化atag
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    while True:
        # ad = ApriltagDetect()
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里
        results_len = len(results)  # 看results里有没有识别到
        # tag_id = results[i].tag_id  # 获取results列表里第i个码的id
        # distance = atag.get_distance(results[i].homography, 4300)  # 获取results列表里第i个码离自己的距离，第二个参数用来校准距离
        if results:
            for result in results:
                distance = atag.get_distance(result.homography, 4300)
                print(result.tag_id, 'distance:', distance)
                if (result.tag_id == 1):
                    mx1 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    #if (mx1 > m1):
                    m1 = mx1
                    mid1 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
                elif (result.tag_id == 0):
                    mx0 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    #if (mx0 > m0):
                    m0 = mx0
                    mid0 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
                elif (result.tag_id == 2):
                    mx2 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    if (mx2 > m2):
                        m2 = mx2
                        mid2 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
        else:
            print("No results")
        print(mid0,mid1,mid2)
    # cap.release()
    # cv2.destroyAllWindows()



def tai_shang():
    if io2 == 1 and io6 == 1 and io4 == 1:#两边都到悬崖了，同时后面没有东西
        # ting_zhi()
        # time.sleep(0.2)
        # xiao_hou_tui()
        you_zhuan()
        time.sleep(0.4)
    if io2 == 1 and io6 == 0 and io4 == 1:#左边到悬崖了后面没东西
        you_zhuan()
        #ting_zhi()
        time.sleep(0.2)
        # tui_ma_kuai
        # print('adio的tui_con  是 1 撒   ')
    if io6 == 1 and io2 == 0 and io4 == 1:#右边到悬崖了后面没东西
        zuo_zhuan()
        #ting_zhi()
        time.sleep(0.2)
    if io6 == 0 and io2==0:
        if distance_flag==1:
            if result.tag_id==1:
                if io2==0 and io6==0:
                    qian_jin()
                    print("识别到tag")
                else:
                    pass
            elif result.tag_id==0:
                    print("错误的tag")
                    if distance < 70 and notflag == 1:
                        quan_you_zhuan()
                        time.sleep(0.6)
                        notflag=1
        else:
            if io0==0 and io1==0 and io2==0 and io6==0:
                quan_su()
    if io0 == 1 and io1 == 0 and io2 == 0 and io6 == 0:
        shun_you_zhuan()
    if io0 == 0 and io1 == 1 and io2 == 0 and io6 == 0:
        shun_zuo_zhuan()
    if io2 == 0 and io6 == 0 and io0 == 1 and io1 == 1:
        qian_jin()
    if io3 == 0 and io5 == 1 and io0 == 1 and io1 == 1:#前面没东西同时左边有东西
        if io6 == 0 and io2 == 0:#不掉下去就左转(后面再改优先级)
            zuo_zhuan()
            time.sleep(0.36)
        # quan_su()
    # if io4 == 0 and io0 == 1 and io1 == 1 and distance_flag==1:#后面有东西，如果我前面没有东西，就转头和它干
    #     quan_you_zhuan()
    #     time.sleep(0.45)
    if io5 == 0 and io3 == 1 and io0 == 1 and io1 == 1:#前面没东西同时右边有东西
        if io6 == 0 and io2 == 0:#不掉下去就右转(后面再改优先级)###
            you_zhuan()
            time.sleep(0.36)
    if io3 == 0 and io5 == 0 and io0 == 1 and io1 == 1:#前面没东西同时两边有东西
        you_zhuan()
        time.sleep(0.36)
        qian_jin()
        time.sleep(0.5)
        #....................
    if io3 == 1 and io5 == 0 and io0 == 0 and io1 == 0:
        you_zhuan()
        time.sleep(0.36)
        qian_jin()
        time.sleep(0.5)
    if io3 == 0 and io5 == 1 and io0 == 0 and io1 == 0:
        zuo_zhuan()
        time.sleep(0.36)
        qian_jin()
        time.sleep(0.5)





def lv_bo():
    if t == 0:
        num1 = io1
        num_1 = io0
    elif t == 1:
        num2 = io1
        num_2 = io0
    elif t == 2:
        num3 = io1
        num_3 = io0
    t = t + 1
    t = t % 3
    t1 = num1 + num2 + num3
    t0 = num_1 + num_2 + num_3
    if 0 == (t1 / 3):
        io1 = 0
    else:
        io1 = 1
    if 0 == (t0 / 3):
        io0 = 0
    else:
        io0 = 1



def jump():
    if 300<adc0<430 and adc>1200: #and :adc>1120:
       up.CDS_SetAngle(1,669,700)#(铲子上台)
       up.CDS_SetAngle(2, 347,700)
       quan_hou_tui()
       time.sleep(1.3)
       print('铲子上台')
    elif adc0>430 and adc>1100:
        up.CDS_SetAngle(1, 540, 512)#（铲子铲人）
        up.CDS_SetAngle(2, 472, 512)
        print('铲子铲人')
    elif 200<adc0<430 and adc<1000:
        up.CDS_SetAngle(1, 209, 512)#(铲子升高)
        up.CDS_SetAngle(2, 864, 512)
        print('铲子升高')
        ##    up.CDS_SetAngle(2, 100,512)
        #  up.CDS_SetAngle(1, 900,512)


if __name__ == "__main__":
        #April_start_detect()
        notflag=1
        qi_dong=1
        zhangai=0
        flag=0
        tflag=0
        num1=0
        t=0
        num2=0
        num3=0
        num_1=0
        num_2=0
        num_3=0
        up = uptech.UpTech()
        up.LCD_Open(2)
        up.ADC_IO_Open()
        up.ADC_Led_SetColor(0, 0x2F0000)
        up.ADC_Led_SetColor(3, 0x002F00)
        up.ADC_Led_SetColor(2, 0x002F00)
        up.CDS_Open()
        up.CDS_SetMode(1, 0)
        up.CDS_SetMode(2, 0)
        up.CDS_SetMode(3, 1)
        up.CDS_SetMode(4, 1)
        up.LCD_PutString(40, 0, 'return:')
        up.LCD_Refresh()
        up.LCD_SetFont(up.FONT_6X10)
        atag = Atag()  # 实例化atag
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        print(cv2.__version__)
        while True:
            while qi_dong:
                io_all_input = up.ADC_IO_GetAllInputLevel()
                io_array = '{:08b}'.format(io_all_input)
                io_data.clear()
                for index, value in enumerate(io_array):
                    io = int(value)
                    io_data.insert(0, io)
                io0 = io_data[0]
                io1 = io_data[1]
                io2 = io_data[2]
                io3 = io_data[3]
                io4 = io_data[4]
                io5 = io_data[5]
                io6 = io_data[6]
                io7 = io_data[7]
                if io3==0:
                    qi_dong=0
                    break
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里
            results_len = len(results)  # 看results里有没有识别到
            # tag_id = results[i].tag_id  # 获取results列表里第i个码的id
            # distance = atag.get_distance(results[i].homography, 4300)
            if results:
                for result in results:
                    distance = atag.get_distance(result.homography, 4100)
                    print(result.tag_id, 'distance:', distance)
                    if result.tag_id ==1 or result.tag_id ==0:
                            distance_flag = 1
            else:
                distance_flag=0
                notflag=1
                print("No results")
            io_all_input = up.ADC_IO_GetAllInputLevel()
            io_array = '{:08b}'.format(io_all_input)
            io_data.clear()
            for index, value in enumerate(io_array):
                io = int(value)
                io_data.insert(0, io)
            io0 = io_data[0]
            io1 = io_data[1]
            io2 = io_data[2]
            io3 = io_data[3]
            io4 = io_data[4]
            io5 = io_data[5]
            io6 = io_data[6]
            io7 = io_data[7]
            if(t<3):
                if t==0:
                    num1=io1
                    num_1=io0
                elif t==1:
                    num2=io1
                    num_2 = io0
                elif t==2:
                    num3=io1
                    num_3 = io0
                t=t+1
                t=t%3
                t1=num1+num2+num3
                t0=num_1+num_2+num_3
            if 0==(t1/3):
                io1=0
            else:
                io1=1
            if 0 == (t0 / 3):
                io0 =0
            else:
                io0 =1
            #pan_duan(t0,t1,t2,t3,t4,t5,t6)
            adc_value = up.ADC_Get_All_Channle()
            adc1 = adc_value[0]
            adc2 = adc_value[3]
            adc0 = adc_value[2]
            adc = adc1 + adc2
            jump()
            if 200<adc0<430 and adc<1000:
                if io0 == 0 or io1 == 0:
                    tflag=0
                    quan_hou_tui()
                    time.sleep(1.5)
                if io0 == 1 and io1 == 1:
                    xiao_you_zhuan()
                    tflag+=1
                if tflag>100 and io0 == 1 and io1 == 1:
                    tflag=0
                    if flag==0:
                        hou_tui()
                        time.sleep(0.4)
                    else:
                       qian_jin()
                       time.sleep(0.4)
                    flag=-flag
                    # hou_tui+you_zhuan
            if 440>adc0>410 and adc<1000:
                if (io2==1 or io6==1) and 350<adc1-adc2<390:
                    you_zhuan()
                    time.sleep(0.5)
                    qian_jin()
                    time.sleep(1)
                if 650<adc1-adc2<700 and (io6==1 or io2==1):
                    zuo_zhuan()
                    time.sleep(0.5)
                    qian_jin()
                    time.sleep(1)
            if adc0>420 and adc>1000:
                if io2 == 1 and io6 == 1 and io4 == 1:  # 两边都到悬崖了，同时后面没有东西
                    you_zhuan()
                    time.sleep(0.4)
                if io2 == 1 and io6 == 0 and io4 == 1:  # 左边到悬崖了后面没东西
                    you_zhuan()
                    # ting_zhi()
                    time.sleep(0.2)
                    # tui_ma_kuai
                    # print('adio的tui_con  是 1 撒   ')
                if io6 == 1 and io2 == 0 and io4 == 1:  # 右边到悬崖了后面没东西
                    zuo_zhuan()
                    # ting_zhi()
                    time.sleep(0.2)
                if io6 == 0 and io2 == 0:
                    if distance_flag == 1:
                        if result.tag_id == 1:
                            if io2 == 0 and io6 == 0:
                                qian_jin()
                                print("识别到tag")
                            else:
                                pass
                        elif result.tag_id == 0:
                            print("错误的tag")
                            if distance < 70 and notflag == 1:
                                quan_you_zhuan()
                                time.sleep(0.4)
                                notflag = 0
                    else:
                        if io0 == 0 and io1 == 0 and io2 == 0 and io6 == 0:
                            quan_su()
                if io0 == 1 and io1 == 0 and io2 == 0 and io6 == 0:
                    shun_you_zhuan()
                if io0 == 0 and io1 == 1 and io2 == 0 and io6 == 0:
                    shun_zuo_zhuan()
                if io2 == 0 and io6 == 0 and io0 == 1 and io1 == 1:
                    qian_jin()
                if io3 == 0 and io5 == 1 and io0 == 1 and io1 == 1:  # 前面没东西同时左边有东西
                    if io6 == 0 and io2 == 0:  # 不掉下去就左转(后面再改优先级)
                        zuo_zhuan()
                        time.sleep(0.36)
                    # quan_su()
                if io5 == 0 and io3 == 1 and io0 == 1 and io1 == 1:  # 前面没东西同时右边有东西
                    if io6 == 0 and io2 == 0:  # 不掉下去就右转(后面再改优先级)###
                        you_zhuan()
                        time.sleep(0.36)
                if io3 == 0 and io5 == 0 and io0 == 1 and io1 == 1:  # 前面没东西同时两边有东西
                    you_zhuan()
                    time.sleep(0.4)
                    qian_jin()
                    time.sleep(0.5)
                    # ....................
                if io3 == 1 and io5 == 0 and io0 == 0 and io1 == 0:
                    you_zhuan()
                    time.sleep(0.36)
                    qian_jin()
                    time.sleep(0.5)
                if io3 == 0 and io5 == 1 and io0 == 0 and io1 == 0:
                    zuo_zhuan()
                    time.sleep(0.36)
                    qian_jin()
                    time.sleep(0.5)
            print("前辉度是", adc0)
            print("左右辉度是", adc1,adc2,adc)
            print(io0,io1,io2,io3,io4,io5)
            print(t, num1, num2 ,num3,'aflag=',tflag)
            time.sleep(0.02)
        # cap.release()
        # cv2.destroyAllWindows()'''
'''import cv2
import numpy as np
import apriltag
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

if __name__ == "__main__":
    # atag = Atag()  # 实例化atag
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    ad = ApriltagDetect()
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()'''

import cv2
import numpy as np

ball_color = 'red'

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              }

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()
    if ret:

        if frame is not None:
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)                     # 高斯模糊
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
            erode_hsv = cv2.erode(hsv, None, iterations=2)                   # 腐蚀 粗的变细
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)

            cv2.imshow('camera', frame)
            cv2.waitKey(1)
        else:
            print("无画面")
    else:
        print("无法读取摄像头！")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
