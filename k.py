import time
import uptech
import apriltag
import cv2
#import atag
import numpy as np
import signal
def signal_handler(handler_signal, handler_frame):
    ting_zhi()
    exit(0)

global adc1
global io0
global io1
global io2
global io3
global io4
global io5
global io6
global io7
global taps1
global value
global flag
global tflag
global zhangai
global qi_dong
global distance
global distance_flag
global notflag
global tui
global index
global jian_ce_flag

global index
io_data = []

you_jian=1
you_bi=2
you_shou=5
zuo_jian=6
zuo_bi=7
zuo_shou=8
zuo_yao=9
you_yao=10

zha_dan_kuai=2
di_fang_kuai=0
middle=290
class Atag:
    def __init__(self):
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)

    def detect(self, gray):
        return self.detector.detect(gray)

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

def init_512():
    up.CDS_SetAngle(1, 420, 350)
    up.CDS_SetAngle(2, 430, 350)
    up.CDS_SetAngle(zuo_shou, 510, 350)
    up.CDS_SetAngle(6, 400, 350)
    up.CDS_SetAngle(7, 500, 350)
    up.CDS_SetAngle(you_shou, 480, 350)
    up.CDS_SetAngle(9, 606, 350)
    up.CDS_SetAngle(10, 361, 350)

def init_gong_ji_middle():
    print("gong-ji-middle")
    up.CDS_SetAngle(1, 560, 500)
    up.CDS_SetAngle(2, 450, 500)
    up.CDS_SetAngle(zuo_shou, 130, 500)
    up.CDS_SetAngle(6, 520, 500)
    up.CDS_SetAngle(7, 520, 500)
    up.CDS_SetAngle(you_shou, 890, 500)
    up.CDS_SetAngle(9, 606, 500)
    up.CDS_SetAngle(10, 361, 500)

def qi_dong_middle():
    print("启动-middle")
    up.CDS_SetAngle(1, 420, 400)
    up.CDS_SetAngle(2, 450, 400)
    up.CDS_SetAngle(zuo_shou, 180, 400)
    up.CDS_SetAngle(6, 450, 400)
    up.CDS_SetAngle(7, 570, 400)
    up.CDS_SetAngle(you_shou, 860, 400)
    up.CDS_SetAngle(9, 549, 400)
    up.CDS_SetAngle(10, 412, 400)
def tui_tag_first():
    global tui_tag_flag
    if tui_tag_flag==1:
        tui_tag_flag=0
        print("第一步推tag")
        up.CDS_SetAngle(zuo_jian, 310, 700)
        up.CDS_SetAngle(you_jian, 310, 700)
        up.CDS_SetAngle(zuo_bi, 380, 700)
        up.CDS_SetAngle(you_bi, 560, 700)
        time.sleep(0.3)
        up.CDS_SetAngle(zuo_shou, 500, 700)
        up.CDS_SetAngle(you_shou, 500, 700)
        up.CDS_SetAngle(zuo_jian, 360, 700)
        up.CDS_SetAngle(you_jian, 360, 700)
    else:
        up.CDS_SetAngle(zuo_bi, 380, 700)
        up.CDS_SetAngle(you_bi, 560, 700)
        up.CDS_SetAngle(zuo_shou, 500, 700)
        up.CDS_SetAngle(you_shou, 500, 700)
        up.CDS_SetAngle(zuo_jian, 360, 700)
        up.CDS_SetAngle(you_jian, 360, 700)

def tui_tag_second():
    global tui_tag_flag
    tui_tag_flag = 1
    up.CDS_SetAngle(zuo_jian, 320, 700)
    up.CDS_SetAngle(you_jian, 320, 700)
    up.CDS_SetAngle(zuo_shou, 360, 700)
    up.CDS_SetAngle(you_shou, 630, 700)
    time.sleep(0.2)
    up.CDS_SetAngle(zuo_bi, 650, 700)
    up.CDS_SetAngle(you_bi, 380, 700)
    time.sleep(0.2)

    up.CDS_SetAngle(you_jian, 280, 700)


def zhi_cua_cua():
    up.CDS_SetAngle(zuo_jian, 150, 700)
    up.CDS_SetAngle(zuo_shou,300,700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_bi,250,700)
    up.CDS_SetAngle(zuo_shou,560,700)
    time.sleep(0.5)
    up.CDS_SetAngle(you_jian,200,700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_bi,700,700)
    '''up.CDS_SetAngle(zuo_jian, 300, 700)
    up.CDS_SetAngle(you_jian, 300, 700)

    up.CDS_SetAngle(zuo_shou, 350, 700)
    up.CDS_SetAngle(you_shou, 650, 700)

    up.CDS_SetAngle(zuo_bi, 400, 700)
    up.CDS_SetAngle(you_bi, 600, 700)

    time.sleep(0.7)
    up.CDS_SetAngle(zuo_jian, 150, 700)
    up.CDS_SetAngle(you_jian, 150, 700)
    up.CDS_SetAngle(zuo_shou, 450, 700)
    up.CDS_SetAngle(you_shou, 600, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_bi, 600, 700)
    up.CDS_SetAngle(you_bi, 400, 700)
    '''

def zuo_cua_cua():
    up.CDS_SetAngle(zuo_jian, 180, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_bi, 750, 700)
    up.CDS_SetAngle(zuo_shou, 450, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_shou,400,700)
    time.sleep(0.4)


def you_cua_cua():
    up.CDS_SetAngle(you_jian, 180, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(you_shou, 510, 700)
    up.CDS_SetAngle(you_bi,200,700)
    time.sleep(0.4)


def qi_chuang():
    print("起床")
    init_512()
    time.sleep(0.5)
    up.CDS_SetAngle(9, 919, 700)
    up.CDS_SetAngle(10, 63, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(you_bi, 200, 700)
    up.CDS_SetAngle(zuo_bi, 800, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_jian, 850, 700)
    up.CDS_SetAngle(you_jian, 910, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_shou, 820, 700)
    up.CDS_SetAngle(you_shou, 180, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_bi, 480, 700)
    up.CDS_SetAngle(you_bi, 470, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_shou, 560, 500)
    up.CDS_SetAngle(you_shou, 450, 500)
    up.CDS_SetAngle(9, 759, 500)
    up.CDS_SetAngle(10, 223, 500)
    up.CDS_SetAngle(zuo_jian, 560, 500)
    up.CDS_SetAngle(you_jian, 600, 500)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_bi, 510, 500)
    up.CDS_SetAngle(you_bi, 440, 500)
    time.sleep(1.5)
    init_512()


#3是右轮，4是左轮
def quan_zuo_zhuan():
    up.CDS_SetSpeed(3, -450)
    up.CDS_SetSpeed(4, -450)
    print('全速左转')

def quan_you_zhuan():
    up.CDS_SetSpeed(3, 480)
    up.CDS_SetSpeed(4, 480)
    print('全速右转')

def quan_su():
    up.CDS_SetSpeed(3, -500)
    up.CDS_SetSpeed(4, 500)
    print('全速')

def ting_zhi():
    up.CDS_SetSpeed(3, -9)
    up.CDS_SetSpeed(4, -9)
    print('停止')

def zuo_zhuan():
    up.CDS_SetSpeed(4, -400)
    up.CDS_SetSpeed(3, -280)
    print('左转')
def you_zhuan():
    up.CDS_SetSpeed(3, 340)
    up.CDS_SetSpeed(4, 400)
    print('右转')
def man_zuo_zhuan():
    up.CDS_SetSpeed(3, -220)
    up.CDS_SetSpeed(4, -310)
    print('man左转')
def man_you_zhuan():
    up.CDS_SetSpeed(3, 210)#240#you lun
    up.CDS_SetSpeed(4, 300)#300
    print('man右转')
def di_su():
    up.CDS_SetSpeed(4, 270)
    up.CDS_SetSpeed(3, -190)
    print('disu前进')

def qian_jin():
    up.CDS_SetSpeed(4, 360)
    up.CDS_SetSpeed(3, -300)#280
    print('前进')

def hou_tui():
    up.CDS_SetSpeed(4, -360)
    up.CDS_SetSpeed(3, 280)#280
    print('后退')

def you_hou_zhuan():
    up.CDS_SetSpeed(3, 330)
    up.CDS_SetSpeed(4, -390)
    print('向右边后退')

def zuo_hou_zhuan():
    up.CDS_SetSpeed(3, 390)
    up.CDS_SetSpeed(4, -330)
    print('向左边后退')

def quan_hou_tui():
    up.CDS_SetSpeed(3, 500)
    up.CDS_SetSpeed(4, -500)
    print('全速后退')

def shun_you_zhuan():
    up.CDS_SetSpeed(3, -210)
    up.CDS_SetSpeed(4, 450)
    print('顺右转')

def shun_zuo_zhuan():
    up.CDS_SetSpeed(3, -390)
    up.CDS_SetSpeed(4, 280)
    print('顺左转')

def io():
    global io0,io1,io2,io3,io4,io5,io6,io7
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

def dao_di():
    global io7,io4
    if io4==0 and io7==1:
        ting_zhi()
        qi_chuang()
    elif io4==1 and io7==1:
        ting_zhi()
        qian_qi_chuang()
    # elif io4==1 and io7==1:
    #     ting_zhi()
    #     qian_qi_chuang()
    # global jian_ce_flag
    # if io4==0:
    #     jian_ce_flag=1#重置标志位
    # if io4==1 and io7==0:
    #     if jian_ce_flag==1:
    #         start_time = time.time()
    #         end_time = start_time + 3
    #         jian_ce_flag =0
    #     if end_time < start_time:
    #         qi_chuang()

def qian_qi_chuang():
    print("前面起床")
    up.CDS_SetAngle(you_jian, 820, 700)
    up.CDS_SetAngle(zuo_jian, 820, 700)
    time.sleep(1)
    up.CDS_SetAngle(1, 420, 700)
    up.CDS_SetAngle(2, 430, 700)
    up.CDS_SetAngle(zuo_shou, 510, 700)
    up.CDS_SetAngle(6, 400, 700)
    up.CDS_SetAngle(7, 500, 700)
    up.CDS_SetAngle(you_shou, 480, 700)
    up.CDS_SetAngle(9, 606, 700)
    up.CDS_SetAngle(10, 361, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(you_bi, 140, 700)
    up.CDS_SetAngle(zuo_bi, 820, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_jian, 200, 700)
    up.CDS_SetAngle(you_jian, 130, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_shou, 180, 700)
    up.CDS_SetAngle(you_shou, 820, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_bi, 500, 700)
    up.CDS_SetAngle(you_bi, 480, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(9, 452, 700)
    up.CDS_SetAngle(10, 509, 700)
    up.CDS_SetAngle(zuo_jian, 60, 700)
    up.CDS_SetAngle(you_jian, 40, 700)
    up.CDS_SetAngle(zuo_shou, 510, 700)
    up.CDS_SetAngle(you_shou, 510, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(9, 452, 700)
    up.CDS_SetAngle(10, 509, 700)
    up.CDS_SetAngle(zuo_bi, 550, 700)
    up.CDS_SetAngle(you_bi, 430, 700)
    up.CDS_SetAngle(zuo_jian, 300, 700)
    up.CDS_SetAngle(you_jian, 300, 700)
    time.sleep(1.5)
    init_512()

if __name__ == "__main__":
        tui_tag_flag=1
        distance=0 #距离
        tui_flag=1#标志位
        tui_flag_1=1
        distance_flag=0 #标志位
        tui=0#标志位
        qi_dong=1 #标志位
        flag=0#滤波标志
        tflag=0#滤波标志
        mid = 0#横向距离
        up = uptech.UpTech()
        up.LCD_Open(2)
        up.ADC_IO_Open()
        # up.ADC_Led_SetColor(0, 0x2F0000)
        # up.ADC_Led_SetColor(3, 0x002F00)
        # up.ADC_Led_SetColor(2, 0x002F00)
        up.CDS_Open()
        up.CDS_SetMode(1, 0)
        up.CDS_SetMode(2, 0)
        up.CDS_SetMode(5, 0)
        up.CDS_SetMode(6, 0)
        up.CDS_SetMode(7, 0)
        up.CDS_SetMode(8, 0)
        up.CDS_SetMode(9, 0)
        up.CDS_SetMode(10, 0)
        up.CDS_SetMode(3, 1)
        up.CDS_SetMode(4, 1)
        time.sleep(1)
        up.LCD_PutString(40, 0, '222')
        up.LCD_Refresh()
        up.LCD_SetFont(up.FONT_6X10)
        atag = Atag()  # 实例化atag
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        signal.signal(signal.SIGINT, signal_handler)
        print(cv2.__version__)
        init_gong_ji_middle()
        while True:
            io()
            print(io4,io7)
            while qi_dong==1:
                qi_dong_middle()
                io()
                if io3==0:
                    qi_dong = 0
            if qi_dong == 0:
                qi_dong_middle()
                up.CDS_SetSpeed(4, 470)
                up.CDS_SetSpeed(3, -390)
                print("开始前进")
                time.sleep(1.7)
                up.CDS_SetSpeed(3, -400)
                up.CDS_SetSpeed(4, 590)
                print("开始右转")
                time.sleep(1.7)
                qi_dong=2
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里
            results_len = len(results)  # 看results里有没有识别到
            print('检测到tag个数为', results_len)
            index=0
            distance,mid=0,0
            if results:
                for i in range(1, results_len):
                    if results[i].tag_id == di_fang_kuai:
                        if results[index].tag_id != di_fang_kuai:
                            index=i
                        if atag.get_distance(results[index].homography, 4300) > atag.get_distance(results[i].homography, 4300):
                            index = i
                if results[index].tag_id == di_fang_kuai:
                    distance_flag = 1
                    distance = int(atag.get_distance(results[index].homography, 4300))
                    mid = tuple(results[index].corners[0].astype(int))[0] / 2 + tuple(results[index].corners[2].astype(int))[0] / 2
                    print('id为',results[index].tag_id,'distance:', distance)
                    print('横向距离为', mid-middle)
                else:
                    distance_flag = 0
                    print('只检测到了炸弹块')
            else:
                tui_flag_1 = 1
                tui_flag = 1
                distance_flag = 0  #用于是否检测到能量块的标志位
                print("一个tag块都没有检测到")
            print('index      ', index)
            print('distance_flag      ', distance_flag)
            io()
            dao_di()
            adc_value = up.ADC_Get_All_Channle()
            adc0 = adc_value[0]
            print('io0~io7',io0,io1,io2,io3,io4,io5,io6,io7,'adc',adc0)
            print('tui_flag==',tui_flag)
            if io3==0 and distance_flag==0:
                ting_zhi()
                zhi_cua_cua()
            elif io0==0 and distance_flag==0:
                ting_zhi()
                zuo_cua_cua()
            elif io1==0 and distance_flag==0:
                ting_zhi()
                you_cua_cua()
            if distance_flag == 1:
                if (io2==1 or io6==1) and distance>50:
                    if io2==1:
                        hou_tui()
                        time.sleep(1.5)
                        quan_you_zhuan()
                        time.sleep(0.53)
                    elif io6==1:
                        hou_tui()
                        time.sleep(1.5)
                        quan_zuo_zhuan()
                        time.sleep(0.53)
                if distance < 70:
                    if distance < 36:#39
                        if tui_flag ==1:
                            tui_flag = 0
                            ting_zhi()
                            # time.sleep(0.2)
                            tui_tag_second()
                            time.sleep(0.8)
                            print("推完tag后退")
                            hou_tui()
                            time.sleep(2)
                            zuo_zhuan()
                            time.sleep(0.63)
                        else:
                            qian_jin()
                    else:
                        if tui_flag_1==1:
                            ting_zhi()
                            tui_tag_first()
                            time.sleep(0.5)
                            tui_flag_1=0
                        if mid - middle > 70 and (io2==0 and io6==0):
                            shun_zuo_zhuan()
                        elif mid - middle < -70 and (io2==0 and io6==0):
                            shun_you_zhuan()
                        elif mid - middle > 70 and (io2==1 or io6==1):
                            man_zuo_zhuan()
                        elif mid - middle < -70 and (io2==1 or io6==1):
                            man_you_zhuan()
                        else:
                            qian_jin()
                elif mid - middle > 70:
                        shun_zuo_zhuan()
                elif mid - middle < -70:
                        shun_you_zhuan()
                else:
                    qian_jin()
            elif io6 == 1:
                init_gong_ji_middle()
                if io2 == 1:
                    ting_zhi()
                    time.sleep(0.7)
                    hou_tui()
                    time.sleep(1.5)
                    quan_zuo_zhuan()
                    time.sleep(0.53)
                else:
                    ting_zhi()
                    time.sleep(0.7)
                    hou_tui()
                    time.sleep(1.5)
                    quan_zuo_zhuan()
                    time.sleep(0.53)
            elif io2 == 1 and io6 == 0:
                init_gong_ji_middle()
                ting_zhi()
                time.sleep(0.7)
                hou_tui()
                time.sleep(1.5)
                quan_you_zhuan()
                time.sleep(0.73)
            elif io2==0 and io6 == 0:
                init_gong_ji_middle()
                qian_jin()

