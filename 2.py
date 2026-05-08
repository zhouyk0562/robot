# a='sdadafsds'
# d={}
# for i in a:
#     if i in d.keys():
#         d[i]+=1
#     else:
#         d[i]=1
# print(d)
# print(len(a))
# print(list(d.values()))
import time
import uptech
import apriltag
import cv2
#import atag
import numpy as np
import ceshi

global xun_xian
global xun_xian_flag
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
global io7
global tui_con
global out
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
global first
global first_2
global jian_ce_flag
io_data = []

you_jian=1
you_bi=2
you_shou=5
zuo_jian=6
zuo_bi=7
zuo_shou=8
zuo_yao=9
you_yao=10

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
    up.CDS_SetAngle(1, 420, 700)
    up.CDS_SetAngle(2, 430, 700)
    up.CDS_SetAngle(zuo_shou, 510, 700)
    up.CDS_SetAngle(6, 500, 700)
    up.CDS_SetAngle(7, 650, 700)
    up.CDS_SetAngle(you_shou, 480, 700)
    up.CDS_SetAngle(9, 515, 700)
    up.CDS_SetAngle(10, 669, 700)

def init_gong_ji_middle():
    up.CDS_SetAngle(1, 420, 700)
    up.CDS_SetAngle(2, 450, 700)
    up.CDS_SetAngle(zuo_shou, 180, 700)
    up.CDS_SetAngle(6, 450, 700)
    up.CDS_SetAngle(7, 570, 700)
    up.CDS_SetAngle(you_shou, 860, 700)
    up.CDS_SetAngle(9, 515, 700)
    up.CDS_SetAngle(10, 669, 700)

def tui_tag():
    print("推tag")
    init_gong_ji_middle()
    time.sleep(1)
    up.CDS_SetAngle(zuo_jian, 380, 700)
    up.CDS_SetAngle(you_jian, 310, 700)
    up.CDS_SetAngle(zuo_bi, 490, 700)
    up.CDS_SetAngle(you_bi, 560, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_shou, 500, 700)
    up.CDS_SetAngle(you_shou, 500, 700)
    up.CDS_SetAngle(zuo_jian, 430, 700)
    up.CDS_SetAngle(you_jian, 360, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_jian, 290, 700)
    up.CDS_SetAngle(you_jian, 220, 700)


def gong_ji():
    init_gong_ji_middle()
    time.sleep(0.4)
    up.CDS_SetAngle(9, 515, 700)
    up.CDS_SetAngle(10, 669, 700)

    up.CDS_SetAngle(zuo_jian, 340, 700)
    up.CDS_SetAngle(you_jian, 290, 700)

    up.CDS_SetAngle(zuo_shou, 300, 700)
    up.CDS_SetAngle(you_shou, 700, 700)

    up.CDS_SetAngle(zuo_bi, 430, 700)
    up.CDS_SetAngle(you_bi, 600, 700)

    time.sleep(0.5)

    up.CDS_SetAngle(zuo_jian, 310, 700)
    up.CDS_SetAngle(you_jian, 260, 700)

    up.CDS_SetAngle(zuo_shou, 400, 700)
    up.CDS_SetAngle(you_shou, 600, 700)

    up.CDS_SetAngle(zuo_bi, 430, 700)
    up.CDS_SetAngle(you_bi, 600, 700)



def qi_chuang():
    print("起床")
    init_512()
    time.sleep(1)
    up.CDS_SetAngle(9, 821, 700)
    up.CDS_SetAngle(10, 362, 700)
    time.sleep(0.4)
    up.CDS_SetAngle(zuo_shou, 180, 700)
    up.CDS_SetAngle(you_shou, 820, 700)
    up.CDS_SetAngle(you_bi, 200, 700)
    up.CDS_SetAngle(zuo_bi, 800, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_jian, 930, 700)
    up.CDS_SetAngle(you_jian, 910, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_shou, 820, 700)
    up.CDS_SetAngle(you_shou, 180, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_bi, 590, 700)
    up.CDS_SetAngle(you_bi, 450, 700)
    time.sleep(0.5)
    up.CDS_SetAngle(zuo_shou, 560, 700)
    up.CDS_SetAngle(you_shou, 450, 700)
    up.CDS_SetAngle(9, 700, 700)
    up.CDS_SetAngle(10, 482, 700)
    up.CDS_SetAngle(zuo_jian, 650, 700)
    up.CDS_SetAngle(you_jian, 600, 700)
    time.sleep(1)
    # up.CDS_SetAngle(9, 515, 700)
    # up.CDS_SetAngle(10, 669, 700)
    # time.sleep(0.4)
    init_512()


def xiao_you_zhuan():
    up.CDS_SetSpeed(3, 320)
    up.CDS_SetSpeed(4, 320)
    print('慢右转')

def xiao_zuo_zhuan():
    up.CDS_SetSpeed(3, -300)
    up.CDS_SetSpeed(4, -300)
    print('慢右转')
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
def man_zuo_zhuan():
    up.CDS_SetSpeed(4, -320)
    up.CDS_SetSpeed(3, -320)
    print("zuo")
def man_you_zhuan():
    up.CDS_SetSpeed(4, 480)#380
    up.CDS_SetSpeed(3, 280)#200
    print("you")
def zuo_zhuan():
    up.CDS_SetSpeed(4, -600)
    up.CDS_SetSpeed(3, -280)
    print('左转')
def you_zhuan():
    up.CDS_SetSpeed(3, 600)
    up.CDS_SetSpeed(4, 280)
    print('右转')
def di_su():
     up.CDS_SetSpeed(3, -290)
     up.CDS_SetSpeed(4, 290)
     print('disu前进')

def qian_jin():
    up.CDS_SetSpeed(4, 300)
    up.CDS_SetSpeed(3, -320)
    print('前进')

def hou_tui():
    up.CDS_SetSpeed(3, 300)
    up.CDS_SetSpeed(4, -300)
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
    up.CDS_SetSpeed(3, -150)
    up.CDS_SetSpeed(4, 320)
    print('顺右转')

def shun_zuo_zhuan():
    up.CDS_SetSpeed(3, -350)
    up.CDS_SetSpeed(4, 150)
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

def lv_bo():
    global io0,io1,t,t0,t1,num1,num2,num3,num_1,num_2,num_3
    if t < 3:
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


def tai_shang():
    global end_time
    global xun_xian_flag
    global xun_xian
    global tui
    global distance2, distance1, distance0
    global tag0, tag1, tag2
    global notflag
    print("xunxian", xun_xian)
    if (io0 == 0 or io1 == 0) and tag0 != 1:
        tui = 1
        if mid1 > 600:
            if io0 == 0 and io1 == 1:
                shun_zuo_zhuan()
            elif io0 == 1 and io1 == 0:
                shun_you_zhuan()
            else:
                qian_jin()
        else:
            di_su()
    if tui == 1 and io0 == 1 and io1 == 1:
        hou_tui()
        time.sleep(0.5)
        quan_you_zhuan()
        time.sleep(0.43)
        tui = 0
    elif (io0 == 0 or io1 == 0) and tag0 == 1:
        if tag1 == 1:
            if mid1 - 290 > 50:
                shun_zuo_zhuan()
            elif mid1 - 290 < -50:
                shun_you_zhuan()
            else:
                qian_jin()
        else:
            quan_you_zhuan()
    elif io0 == 1 and io1 == 1:
        # while xun_xian < 2:
        #     print('first  2',first,first_2)
        #     io()
        #     print(io0,io1,io2,io3,io4,io5,io6)
        #     if io4 == 0:
        #         xun_xian_flag=1
        #     if io4==1:
        #         if xun_xian_flag==1:
        #             xun_xian = xun_xian + 1
        #             xun_xian_flag=0
        #     if io6 == 1 or io2==1:
        #         print("first",first)
        #         if io3==0:
        #             quan_you_zhuan()
        #             # first_2=1
        #         else:
        #             man_you_zhuan()
        #     else:
        #         if first_2==1:
        #            first=0
        #         up.CDS_SetSpeed(4,  400)
        #         up.CDS_SetSpeed(3, -385)
        if xun_xian == 1:
            start_time = time.time()
            end_time = start_time + 23  # 21
            xun_xian = 2
        while time.time() < end_time:
            xun_xian = 3
            io()
            print(io0, io1, io2, io3, io4, io5, io6)
            if io6 == 1 or io2 == 1:
                print("first", first)
                if io3 == 0:
                    up.CDS_SetSpeed(4, 300)
                    up.CDS_SetSpeed(3, 500)
                    print("第一次")
                    # first_2=1
                else:
                    man_you_zhuan()
            else:
                up.CDS_SetSpeed(4, 400)
                up.CDS_SetSpeed(3, -398)
        if xun_xian == 3:
            up.CDS_SetSpeed(4, 600)
            up.CDS_SetSpeed(3, 100)
            time.sleep(0.6)
            qian_jin()
            time.sleep(0.4)
            xun_xian = xun_xian + 1
        if distance_flag == 1:
            if tag1 == 1:
                if tag2 == 1:
                    if distance1 + abs(mid1 - 290) <= distance2 + abs(mid2 - 290):
                        print("1 码离得近")
                        if mid1 - 290 > 50:
                            shun_zuo_zhuan()
                        elif mid1 - 290 < -50:
                            shun_you_zhuan()
                        else:
                            qian_jin()
                    else:
                        print("2码离得近")
                        if mid2 - 290 > 50:
                            shun_zuo_zhuan()
                        elif mid2 - 290 < -50:
                            shun_you_zhuan()
                        else:
                            qian_jin()
                elif mid1 - 290 > 50:
                    shun_zuo_zhuan()
                elif mid1 - 290 < -50:
                    shun_you_zhuan()
                else:
                    qian_jin()
            elif tag2 == 1:
                if mid2 - 290 > 50:
                    shun_zuo_zhuan()
                elif mid2 - 290 < -50:
                    shun_you_zhuan()
                else:
                    qian_jin()
            elif tag0 == 1:
                print("错误的tag")
                if distance < 50:
                    up.CDS_SetSpeed(3, 630)
                    up.CDS_SetSpeed(4, 630)
                else:
                    qian_jin()
        else:
            if io6 == 1:
                if io2 == 1:
                    hou_tui()
                    time.sleep(0.4)
                    quan_you_zhuan()
                    time.sleep(0.6)
                else:
                    hou_tui()
                    time.sleep(0.4)
                    quan_zuo_zhuan()
                    time.sleep(0.8)
            elif io2 == 1 and io6 == 0:
                hou_tui()
                time.sleep(0.4)
                quan_you_zhuan()
                time.sleep(0.7)
            else:
                qian_jin()
        '''elif io2 == 0 and io6 == 0:
            if io5 == 0:
                you_zhuan()
                time.sleep(0.34)
            elif io3 == 0:
                zuo_zhuan()
                time.sleep(0.34)
            elif io3 == 1 and io5 == 1:
                  qian_jin()'''

# if __name__ == "__main__":
#         distance_flag=1
#         first_2=0
#         first=1
#         xun_xian_flag=0
#         xun_xian=1
#         tui=0
#         siwang=1
#         distance0=0
#         distance1=0
#         distance2=0
#         tag1=0
#         tag2=0
#         tag0=0
#         #April_start_detect()
#         notflag=1
#         qi_dong=1
#         zhangai=0
#         flag=0
#         tflag=0
#         num1=0
#         t=0
#         num2=0
#         num3=0
#         num_1=0
#         num_2=0
#         num_3=0
#         mid0 = 0
#         mid1 = 0
#         mid2 = 0
#         jian_ce_flag = 1
#         up = uptech.UpTech()
#         up.LCD_Open(2)
#         up.ADC_IO_Open()
#         # up.ADC_Led_SetColor(0, 0x2F0000)
#         # up.ADC_Led_SetColor(3, 0x002F00)
#         # up.ADC_Led_SetColor(2, 0x002F00)
#         up.CDS_Open()
#         up.CDS_SetMode(1, 0)
#         up.CDS_SetMode(2, 0)
#         up.CDS_SetMode(5, 0)
#         up.CDS_SetMode(6, 0)
#         up.CDS_SetMode(7, 0)
#         up.CDS_SetMode(8, 0)
#         up.CDS_SetMode(9, 0)
#         up.CDS_SetMode(10, 0)
#         up.CDS_SetMode(3, 1)
#         up.CDS_SetMode(4, 1)
#         up.LCD_PutString(40, 0, '222')
#         up.LCD_Refresh()
#         up.LCD_SetFont(up.FONT_6X10)
#         atag = Atag()  # 实例化atag
#         cap = cv2.VideoCapture(0)
#         cap.set(3, 640)
#         cap.set(4, 480)
#         print(cv2.__version__)
#         while True:
#             while qi_dong==1:
#                 init_512()
#                 io()
#                 if io3==0:
#                     qi_dong=0
#                     break
#             if qi_dong==0:
#                 init_gong_ji_middle()
#                 up.CDS_SetSpeed(4, 430)
#                 up.CDS_SetSpeed(3, -360)
#                 print("开始前进")
#                 time.sleep(2.5)
#                 up.CDS_SetSpeed(3, -200)
#                 up.CDS_SetSpeed(4, 450)
#                 print("开始右转")
#                 time.sleep(1)
#                 qi_dong=2
#             ret, frame = cap.read()
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里
#             results_len = len(results)  # 看results里有没有识别到
#             # tag_id = results[i].tag_id  # 获取results列表里第i个码的id
#             # distance = atag.get_distance(results[i].homography, 4300)
#             if results:
#                 tag0 = 0
#                 tag1 = 0
#                 tag2 = 0
#                 mid0 = 0
#                 mid1 = 0
#                 mid2 = 0
#                 for result in results:
#                     distance = atag.get_distance(result.homography, 4100)
#                     print(result.tag_id, 'distance:', distance)
#                     if result.tag_id == 1 or result.tag_id == 0 or result.tag_id == 2:
#                         distance_flag = 1
#                     if (result.tag_id == 1):
#                         tag1 = 1
#                         distance1 = distance
#                         mid1 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
#                     elif (result.tag_id == 0):
#                         tag0 = 1
#                         distance0 = distance
#                         mid0 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
#                     elif (result.tag_id == 2):
#                         tag2 = 1
#                         distance2 = distance
#                         mid2 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
#                     print("三个距离为", mid0, mid1, mid2)
#             else:
#                 distance_flag = 0
#                 notflag = 1
#                 print("No results")
#             io()
#             lv_bo()
#             dao_di()
#             adc_value = up.ADC_Get_All_Channle()
#             adc0 = adc_value[0]
#             print(adc0,io4,io7)
#             if io6 == 1:
#                 init_gong_ji_middle()
#                 if io2 == 1:
#                     ting_zhi()
#                     time.sleep(0.7)
#                     hou_tui()
#                     time.sleep(0.8)
#                     quan_you_zhuan()
#                     time.sleep(0.3)
#                 else:
#                     ting_zhi()
#                     time.sleep(0.7)
#                     hou_tui()
#                     time.sleep(0.8)
#                     quan_zuo_zhuan()
#                     time.sleep(0.3)
#             elif io2 == 1 and io6 == 0:
#                 init_gong_ji_middle()
#                 ting_zhi()
#                 time.sleep(0.7)
#                 hou_tui()
#                 time.sleep(0.8)
#                 quan_you_zhuan()
#                 time.sleep(0.3)
#             elif io2==0 and io6 == 0:
#                 init_gong_ji_middle()
#                 if distance_flag == 1:
#                     if tag1==1:
#                         if mid1 - 290 > 50:
#                             shun_zuo_zhuan()
#                         elif mid1 - 290 < -50:
#                             shun_you_zhuan()
#                         else:
#                             qian_jin()
#                         if distance1 < 40 and -50 < mid1 - 290 < 50:
#                             qian_jin()
#                             tui_tag()
#                             init_gong_ji_middle()
#                 else:
#                     qian_jin()
#             '''if (io0 == 0 or io1 == 0) and tag1 != 1:
#                 gong_ji()
#                 tui = 1
#                 if adc0 > 600:
#                     if io0 == 0 and io1 == 1:
#                         shun_zuo_zhuan()
#                     elif io0 == 1 and io1 == 0:
#                         shun_you_zhuan()
#                     else:
#                         qian_jin()
#                 else:
#                     di_su()
#             else:
#                 init_gong_ji_middle()
#                 if io6 == 1:
#                     if io2 == 1:
#                         quan_you_zhuan()
#                         time.sleep(0.3)
#                     else:
#                         quan_zuo_zhuan()
#                         time.sleep(0.2)
#                 elif io2 == 1 and io6 == 0:
#                     quan_you_zhuan()
#                     time.sleep(0.3)
#                 else:
#                     ting_zhi()'''


if __name__ == "__main__":
    atag = Atag()  # 实例化atag
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    ad = ceshi.ApriltagDetect()
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()