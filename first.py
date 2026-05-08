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
global adc0
global adc1
global adc2
global adc3
global ADC
global xun_xian
global xun_xian_flag
global zhangaiflag
global tag_zhangai
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

io_data = []
zha_dan_kuai=2
zhong_li_kuai=0
di_fang_kuai=1
middle=290
aadc=800        #减速
aaa=690#ADC
bbb=462#adc
#小细节修改测试
def xiao_you_zhuan():
    up.CDS_SetSpeed(3, 320)
    up.CDS_SetSpeed(4, 320)
    print('慢右转')

def xiao_zuo_zhuan():
    up.CDS_SetSpeed(3, -300)
    up.CDS_SetSpeed(4, -300)
    print('慢右转')

def quan_zuo_zhuan():
    up.CDS_SetSpeed(3, -460)
    up.CDS_SetSpeed(4, -460)
    print('全速左转')

def quan_you_zhuan():
    up.CDS_SetSpeed(3, 480)
    up.CDS_SetSpeed(4, 480)
    print('全速右转')

def quan_su():
    up.CDS_SetSpeed(3, -625)
    up.CDS_SetSpeed(4, 640)
    print('全速')

def ting_zhi():
    up.CDS_SetSpeed(3, -9)
    up.CDS_SetSpeed(4, -9)
    print('停止')
def man_zuo_zhuan():
    up.CDS_SetSpeed(4, -370)
    up.CDS_SetSpeed(3, -390)
    print("zuo")
def man_you_zhuan():
    up.CDS_SetSpeed(4, 360)#380
    up.CDS_SetSpeed(3, 410)#200
    print("you")
def zuo_zhuan():
    up.CDS_SetSpeed(4, -380)
    up.CDS_SetSpeed(3, -370)
    print('左转')
def you_zhuan():
    up.CDS_SetSpeed(3, 380)
    up.CDS_SetSpeed(4, 400)
    print('右转')
def di_su():
     up.CDS_SetSpeed(3, -340)
     up.CDS_SetSpeed(4, 320)
     print('disu前进')

def qian_jin():
    up.CDS_SetSpeed(3, -430)
    up.CDS_SetSpeed(4, 443)
    print('前进')

def hou_tui():
    up.CDS_SetSpeed(3, 410)#右轮
    up.CDS_SetSpeed(4, -420)
    print('后退')


def quan_hou_tui():
    up.CDS_SetSpeed(3, 540)
    up.CDS_SetSpeed(4, -560)
    print('全速后退')

def shun_you_zhuan():
    up.CDS_SetSpeed(3, -400)
    up.CDS_SetSpeed(4, 510)
    print('顺右转')

def shun_zuo_zhuan():
    up.CDS_SetSpeed(3, -520)
    up.CDS_SetSpeed(4, 380)
    print('顺左转')

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

def April_start_detect():
    m0 = 0
    m1 = 0
    m2 = 0
    mx0 = 0
    mx1 = 0
    mx2 = 0
    mid0 = 0
    mid1 = 0
    mid2 = 0
    atag = Atag()  # 实例化atag
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    while True:
        # ad = ApriltagDetect()
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里
        #results_len = len(results)  # 看results里有没有识别到
        # tag_id = results[i].tag_id  # 获取results列表里第i个码的id
        # distance = atag.get_distance(results[i].homography, 4300)  # 获取results列表里第i个码离自己的距离，第二个参数用来校准距离
        if results:
            for result in results:
                distance = atag.get_distance(result.homography, 4300)
                print('distance:', distance)
                if result.tag_id == 1:
                    mx1 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    #if (mx1 > m1):
                    m1 = mx1
                    mid1 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
                elif result.tag_id == 0:
                    mx0 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    #if (mx0 > m0):
                    m0 = mx0
                    mid0 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
                elif result.tag_id == 2:
                    mx2 = abs(tuple(result.corners[0].astype(int))[0] - tuple(result.corners[2].astype(int))[0])
                    if mx2 > m2:
                        m2 = mx2
                        mid2 = tuple(result.corners[0].astype(int))[0] / 2 + tuple(result.corners[2].astype(int))[0] / 2
        else:
            print("No results")
        print(mid0,mid1,mid2)
    # cap.release()
    # cv2.destroyAllWindows()


def shang_tai():
    global zha_dan_flag
    global tui
    global distance_flag
    global distance, mid
    if zha_dan_flag == 1:
        if io7 == 1:
            if io2 == 1:
                ting_zhi()
                time.sleep(0.3)
                hou_tui()
                time.sleep(0.5)
                quan_you_zhuan()
                time.sleep(0.5)
            else:
                ting_zhi()
                time.sleep(0.3)
                hou_tui()
                time.sleep(0.5)
                quan_zuo_zhuan()
                time.sleep(0.6)
        elif io2 == 1 and io7 == 0:
            ting_zhi()
            time.sleep(0.3)
            hou_tui()
            time.sleep(0.5)
            quan_you_zhuan()
            time.sleep(0.7)
        elif 100>mid - middle > 0:
            you_zhuan()
        elif -100<mid - middle < 0:
            zuo_zhuan()
        elif io3 == 0:
            quan_zuo_zhuan()
            time.sleep(0.45)
        elif io6 == 0:
            quan_you_zhuan()
            time.sleep(0.45)
        else:
            if ADC > aadc :
                quan_su()
            else:
                qian_jin()  # 以上是只看见炸弹快的操作

    elif distance_flag == 1:
            if io0 == 0 or io1==0:
                while True:
                    ret, frame = cap.read()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里（识别的底层逻辑就是检测tag码块上的灰度）
                    results_len = len(results)  # 看results里有没有识别到
                    print('检测到tag个数为', results_len)
                    index = 0  # 定义一个索引为0，便于在存储的灰度results列表中冒泡排序筛选出距离最短的Tag码
                    distance, mid = 0, 0  # 每次程序执行一遍，Tag码与机器人的距离和横向距离都清零
                    zha_dan_flag = 0
                    if results:
                        distance_flag = 1  # 这是个标志位
                        for i in range(1, results_len):  # 循环从第二个（results[1]）索引开始，进行冒泡排序。（因为前方index是从零开始的）所以排序没有遗漏
                            if results[i].tag_id == di_fang_kuai or results[
                                i].tag_id == zhong_li_kuai:  # 如果tag码块id是敌方或中立才进行距离比较，炸弹不管
                                if results[
                                    index].tag_id == zha_dan_kuai:  # 这一步确保index=0的那个id不是炸弹，若是炸弹，则将索引就改成i（i现在肯定不是炸弹）
                                    index = i
                                    if results[i].tag_id == zhong_li_kuai:
                                        if results[index].tag_id == di_fang_kuai:
                                            index = i
                                        elif atag.get_distance(results[index].homography, 4300) > atag.get_distance(
                                                results[i].homography, 4300):  # 冒泡排序
                                            index = i
                                    elif results[index].tag_id == di_fang_kuai:
                                        if atag.get_distance(results[index].homography, 4300) > atag.get_distance(
                                                results[i].homography, 4300):  # 冒泡排序
                                            index = i
                        if results[index].tag_id == di_fang_kuai or results[
                            index].tag_id == zhong_li_kuai:  # 冒泡后如果最近的id是中立或敌方
                            zha_dan_flag = 0
                        else:  # 冒泡后如果的id是炸弹块(侧面证明了没有检测到敌方和中立)
                            zha_dan_flag = 1
                            print('只检测到了炸弹块')
                        distance = int(atag.get_distance(results[index].homography, 4300))
                        mid = tuple(results[index].corners[0].astype(int))[0] / 2 + \
                              tuple(results[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
                        print('id为', results[index].tag_id, 'distance:', distance)
                        print('横向距离为', mid - middle)
                    else:
                        zha_dan_flag = 0
                        tui_flag = 0
                        distance_flag = 0
                        print("一个tag块都没有检测到")
                    io()
                    if (io0==1 and io1==1) or distance_flag==0 or zha_dan_flag==1:
                        break
                    if io0 == 0 and io1 == 0:
                        if ADC < aadc or io2 == 1 or io7 == 1:
                            di_su()
                        else:
                            qian_jin()
                    elif mid - middle > 80:
                        man_zuo_zhuan()
                    elif mid - middle < -80:
                        man_you_zhuan()
                    else:
                        if ADC < aadc or io2==1 or io7==1:
                            di_su()
                        else:
                            qian_jin()


            elif io0 == 0 and io1 ==0:
                if ADC < aadc or io2==1 or io7==1:
                    di_su()
                else:
                    qian_jin()
            elif distance<30:
                if mid - middle > 80:
                    man_zuo_zhuan()
                elif mid - middle < -80:
                    man_you_zhuan()
                else:
                    if ADC < aadc or(io2==1 or io7==1):
                        di_su()
                    else:
                        qian_jin()
            elif io7 == 1:
                if io2 == 1:
                    ting_zhi()
                    time.sleep(0.3)
                    hou_tui()
                    time.sleep(0.5)
                    quan_you_zhuan()
                    time.sleep(0.5)
                else:
                    ting_zhi()
                    time.sleep(0.3)
                    hou_tui()
                    time.sleep(0.5)
                    quan_zuo_zhuan()
                    time.sleep(0.6)
            elif io2 == 1 and io7 == 0:
                ting_zhi()
                time.sleep(0.3)
                hou_tui()
                time.sleep(0.5)
                quan_you_zhuan()
                time.sleep(0.7)
            elif ADC > aadc:
                if mid - middle > 60:
                    shun_zuo_zhuan()
                elif mid - middle < -60:
                    shun_you_zhuan()
                else:
                    quan_su()
            else:
                print('/////////////////////////////////////')
                if mid - middle > 80:
                    man_zuo_zhuan()
                elif mid - middle < -80:
                    man_you_zhuan()
                else:
                    di_su()


    elif io7 == 1:
        if io2 == 1:
            ting_zhi()
            time.sleep(0.3)
            hou_tui()
            time.sleep(0.5)
            quan_you_zhuan()
            time.sleep(0.5)
        else:
            ting_zhi()
            time.sleep(0.3)
            hou_tui()
            time.sleep(0.5)
            quan_zuo_zhuan()
            time.sleep(0.6)
    elif io2 == 1 and io7 == 0:
        ting_zhi()
        time.sleep(0.3)
        hou_tui()
        time.sleep(0.5)
        quan_you_zhuan()
        time.sleep(0.7)
    elif io4==0:
        up.CDS_SetSpeed(3, -650)
        up.CDS_SetSpeed(4, 490)
        print("快速顺左转")
    elif io5==0:
        up.CDS_SetSpeed(3, -490)
        up.CDS_SetSpeed(4, 660)
        print("快速顺右转")
    elif io0 == 0 or io1 == 0:  # 后面整合一起
        if ADC > aadc:
            if io0 == 0 and io1 == 1:
                up.CDS_SetSpeed(3, -650)
                up.CDS_SetSpeed(4, 490)
                print("快速顺左转")
            elif io0 == 1 and io1 == 0:
                up.CDS_SetSpeed(3, -490)
                up.CDS_SetSpeed(4, 660)
                print("快速顺右转")
            else:
                up.CDS_SetSpeed(3, -655)
                up.CDS_SetSpeed(4, 650)
                print('超级全速')
        else:
            qian_jin()
    elif io3 == 0:
        quan_zuo_zhuan()
        time.sleep(0.45)
    elif io6 == 0:
        quan_you_zhuan()
        time.sleep(0.45)
    else:
        if ADC > aadc:
            quan_su()
        else:
            qian_jin()

def tai_shang():
    global tui
    global distance_flag
    global distance,mid
    if zha_dan_flag==1:
        if mid - middle > 100:
            you_zhuan()
        elif mid - middle < -100:
            zuo_zhuan()
        elif io3==0:
            quan_zuo_zhuan()
            time.sleep(0.45)
        elif io6==0:
            quan_you_zhuan()
            time.sleep(0.45)
        else:
            if io7 == 1:
                if io2 == 1:
                    hou_tui()
                    time.sleep(0.4)
                    quan_you_zhuan()
                    time.sleep(0.5)
                else:
                    hou_tui()
                    time.sleep(0.4)
                    quan_zuo_zhuan()
                    time.sleep(0.6)
            elif io2 == 1 and io7 == 0:
                hou_tui()
                time.sleep(0.4)
                quan_you_zhuan()
                time.sleep(0.7)
            else:
                if adc0 > 600:
                    quan_su()
                else:
                    qian_jin()#以上是只看见炸弹快的操作
    else:
        if distance_flag == 1:
            if adc0>500:
                if mid - middle > 80:
                    shun_zuo_zhuan()
                elif mid - middle < -80:
                    shun_you_zhuan()
                else:
                    quan_su()
            else:
                if mid - middle > 80:
                    man_zuo_zhuan()
                elif mid - middle < -80:
                    man_you_zhuan()
                else:
                    di_su()
        elif io0==0 or io1==0:#后面整合一起
            if adc0 >600:
                if io0 == 0 and io1 == 1:
                    up.CDS_SetSpeed(3, -390)
                    up.CDS_SetSpeed(4, 560)
                    print("快速顺左转")
                elif io0 == 1 and io1 == 0:
                    up.CDS_SetSpeed(3, -550)
                    up.CDS_SetSpeed(4, 390)
                    print("快速顺右转")
                else:
                    up.CDS_SetSpeed(3, -655)
                    up.CDS_SetSpeed(4, 650)
                    print('超级全速')
            else:
                    qian_jin()
        elif io3==0:
            quan_zuo_zhuan()
            time.sleep(0.45)
        elif io6==0:
            quan_you_zhuan()
            time.sleep(0.45)
        else:
            if io7 == 1:
                if io2 == 1:
                    hou_tui()
                    time.sleep(0.4)
                    quan_you_zhuan()
                    time.sleep(0.5)
                else:
                    hou_tui()
                    time.sleep(0.4)
                    quan_zuo_zhuan()
                    time.sleep(0.6)
            elif io2 == 1 and io7 == 0:
                hou_tui()
                time.sleep(0.4)
                quan_you_zhuan()
                time.sleep(0.6)
            else:
                if adc0 > 650:
                    quan_su()
                else:
                    qian_jin()

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


def jump():
    global chan_zi
    # if 630 < ADC < 670 and 580 < adc < 670 and io2 == 0 and io7 == 0:  # and :adc>1120:
    #     up.CDS_SetAngle(1, 209, 512)  # (铲子升高)
    #     up.CDS_SetAngle(2, 854, 512)
    if ADC<650 and 750<adc<920 and io2==0 and io7==0 and io0==0 and io1==0: #and :adc>1120:
       chan_zi = 1
       up.CDS_SetAngle(1,672,700)#(铲子上台)
       up.CDS_SetAngle(2, 400,700)
       quan_hou_tui()
       print(io0,io1,io2,io3,io4,io5,io6)
       time.sleep(0.8)
       print('铲子上台')
       # quan_zuo_zhuan()
       # time.sleep(0.5)
    elif ADC>aaa:
        up.CDS_SetAngle(1, 529, 512)#（铲子铲人）
        up.CDS_SetAngle(2, 540, 512)
        chan_zi = 1
        print('铲子铲人')
    elif ADC<aaa and adc<bbb:
        if (io0 == 0 or io1 == 0) and distance_flag == 1:
            pass
        else:
            if chan_zi==1:
                ting_zhi()
                up.CDS_SetAngle(1, 209, 512)#(铲子升高)
                up.CDS_SetAngle(2, 854, 512)
                time.sleep(1)
                chan_zi = 0
            else:
                up.CDS_SetAngle(1, 209, 512)  # (铲子升高)
                up.CDS_SetAngle(2, 854, 512)
            print('铲子升高')
            ##    up.CDS_SetAngle(2, 100,512)
            #  up.CDS_SetAngle(1, 900,512)
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

if __name__ == "__main__":
        distance_flag=0
        hou_tui_flag=1
        chan_zi=1
        shang_tai_flag=1
        signal.signal(signal.SIGINT, signal_handler)
        zha_dan_flag = 0
        tui=0
        #April_start_detect()
        qi_dong=1
        flag=1
        tflag=0
        num1=0
        t=0
        num2=0
        num3=0
        num_1=0
        num_2=0
        num_3=0
        mid=0
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
        up.LCD_PutString(40, 0, 'ghtdrhv')
        up.LCD_Refresh()
        up.LCD_SetFont(up.FONT_6X10)
        atag = Atag()  # 实例化atag
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        print(cv2.__version__)
        while True:
            # adc_value = up.ADC_Get_All_Channle()
            # adc0 = adc_value[2]
            # adc1 = adc_value[0]
            # adc2 = adc_value[3]
            # adc3 = adc_value[4]
            # adc = adc1 + adc2
            # ADC = adc0 + adc3
            # print("前后灰度是", adc0, adc3, '加起来是', ADC)
            # print("左右灰度是", adc1, adc2, '加起来是', adc)

            # quan_hou_tui()
            # adc_value = up.ADC_Get_All_Channle()
            # if adc_value[2] <adc0:
            #     adc0 = adc_value[2]
            # if adc_value[0] <adc1:
            #     adc1 = adc_value[0]
            # if adc_value[4] <adc2:
            #     adc2 = adc_value[4]
            # if adc_value[3] <adc3:
            #     adc3 = adc_value[3]
            # print("前后灰度是min", adc0, adc3, '加起来是', adc0+adc3)
            # print("左右灰度是min", adc1, adc2, '加起来是', adc1+adc2)
            #
            # if adc_value[2] >adc0q:
            #     adc0q = adc_value[2]
            # if adc_value[0] >adc1q:
            #     adc1q = adc_value[0]
            # if adc_value[4] >adc2q:
            #     adc2q = adc_value[4]
            # if adc_value[3] >adc3q:
            #     adc3q = adc_value[3]
            # print("前后灰度是max", adc0q, adc3q, '加起来是', adc0q+adc3q)
            # print("左右灰度是max", adc1q, adc2q, '加起来是', adc1q+adc2q)
            while qi_dong==1:
                io()
                if io3==0:
                    qi_dong=0
                    break
            if qi_dong==0:
                quan_hou_tui()
                time.sleep(1.1)
                up.CDS_SetAngle(1, 672, 700)  # (铲子上台)
                up.CDS_SetAngle(2, 400, 700)
                quan_hou_tui()
                time.sleep(0.9)
                qi_dong=2
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = atag.detect(gray)  # 对灰度图进行检测，结果放results列表里（识别的底层逻辑就是检测tag码块上的灰度）
            results_len = len(results)  # 看results里有没有识别到
            print('检测到tag个数为', results_len)
            index = 0           #定义一个索引为0，便于在存储的灰度results列表中冒泡排序筛选出距离最短的Tag码
            distance, mid = 0, 0  #每次程序执行一遍，Tag码与机器人的距离和横向距离都清零
            zha_dan_flag=0
            if results:
                distance_flag = 1                                                               #这是个标志位
                for i in range(1, results_len):  #循环从第二个（results[1]）索引开始，进行冒泡排序。（因为前方index是从零开始的）所以排序没有遗漏
                    if results[i].tag_id == di_fang_kuai or results[i].tag_id == zhong_li_kuai:#如果tag码块id是敌方或中立才进行距离比较，炸弹不管
                        if results[index].tag_id == zha_dan_kuai:#这一步确保index=0的那个id不是炸弹，若是炸弹，则将索引就改成i（i现在肯定不是炸弹）
                            index = i
                            if results[i].tag_id == zhong_li_kuai:
                                if results[index].tag_id == di_fang_kuai:
                                    index=i
                                elif atag.get_distance(results[index].homography, 4300) > atag.get_distance(results[i].homography,4300):#冒泡排序
                                    index = i
                            elif results[index].tag_id == di_fang_kuai:
                                if atag.get_distance(results[index].homography, 4300) > atag.get_distance(results[i].homography, 4300):  # 冒泡排序
                                    index = i
                if results[index].tag_id == di_fang_kuai or results[index].tag_id == zhong_li_kuai:#冒泡后如果最近的id是中立或敌方
                    zha_dan_flag = 0
                else:                                                                               #冒泡后如果的id是炸弹块(侧面证明了没有检测到敌方和中立)
                    zha_dan_flag=1
                    print('只检测到了炸弹块')
                distance = int(atag.get_distance(results[index].homography, 4300))
                mid = tuple(results[index].corners[0].astype(int))[0] / 2 + \
                      tuple(results[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
                print('id为', results[index].tag_id, 'distance:', distance)
                print('横向距离为', mid - middle)
            else:
                zha_dan_flag = 0
                tui_flag = 0
                distance_flag = 0
                print("一个tag块都没有检测到")
            io()
            lv_bo()
            adc_value = up.ADC_Get_All_Channle()
            adc0 = adc_value[2]
            adc1 = adc_value[0]
            adc2 = adc_value[3]
            adc3 = adc_value[4]
            adc = adc1 + adc2
            ADC = adc0 + adc3
            print("前后灰度是", adc0,adc3,'加起来是',ADC)
            print("左右灰度是", adc1, adc2,'加起来是',adc)
            jump()

            if ADC<aaa and adc<bbb:
                if hou_tui_flag==1:
                    hou_tui()
                    time.sleep(0.6)
                    hou_tui_flag=0
                if io0 == 0 or io1 == 0:
                    tflag=0
                    quan_hou_tui()
                    time.sleep(1.3)
                elif io4==0 or io5==0:
                    if io4==0:
                        up.CDS_SetSpeed(4, -390)
                        up.CDS_SetSpeed(3, -410)
                        print("zuo")
                    elif io5==0:
                        man_you_zhuan()
                if io0 == 1 and io1 == 1:#如果io0和io1没有亮过（即在台下一直转圈，没有调整好机器人的位置）
                    up.CDS_SetSpeed(4, 390)  # 380
                    up.CDS_SetSpeed(3, 420)  # 200
                    print("you")
                    tflag+=1        #不断计数累加的tflag
                if tflag>100 and io0 == 1 and io1 == 1:#如果计数到100且还没有摆正位置（证明被卡住了 ）
                    tflag=0                          #先清零计数
                    if flag==1:                      #flag=1时后退脱离被卡着的状况
                        qian_jin()
                        time.sleep(0.4)
                    else:                            #flag=-1时后退脱离被卡着的状况（有时光后退摆脱不了被卡着，所以前进后退轮流执行）
                       hou_tui()
                       time.sleep(0.4)
                    flag=-flag
                    # hou_tui+you_zhuan
            # elif ADC>640:#执行台上代码
            else:
                hou_tui_flag=1
                shang_tai()
            print("前灰度是", adc0)
            print("左右灰度是", adc1,adc2,adc)
            print('红外。',io0,io1,io2,io3,io4,io5,io6,io7)
            print(t, num1, num2 ,num3,'aflag=',tflag)
            # time.sleep(0.02)
        # cap.release()
        # cv2.destroyAllWindows()
        # import cv2
        # import pandas as pd'''


'''def color_detection(frame):
    # 定义颜色范围（在HSV颜色空间中）
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    lower_blue = np.array([110, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])

    # 将帧转换为HSV颜色空间
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 根据颜色范围创建掩膜
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # 对掩膜进行形态学操作，以去除噪声
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    # 在原始帧中找到颜色区域并绘制方框
    contours, _ = cv2.findContours(red_mask + blue_mask + green_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        color = ""
        if cv2.contourArea(contour) > 500:  # 设置最小区域面积以排除噪声
            if np.any(red_mask[y:y + h, x:x + w]):
                color = "红色"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            elif np.any(blue_mask[y:y + h, x:x + w]):
                color = "蓝色"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            elif np.any(green_mask[y:y + h, x:x + w]):
                color = "绿色"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        if (color == "绿色"):
            print("1")
    return frame


# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    # 读取摄像头帧
    ret, frame = cap.read()

    # 进行颜色识别
    result = color_detection(frame)

    # 显示结果帧
    cv2.imshow("Color Detection", result)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头和关闭窗口
cap.release()'''

