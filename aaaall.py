import threading
import uptech
import time
import apriltag
import numpy as np
import cv2
import signal

ADC1 = 500  # 285  470
ADC3 = 273  # 308  # 285   300
ADC2 = 450
flag = 0
out = 0
io0 = 6
io1 = 6
io2 = 6
io3 = 6
io4 = 6
io5 = 6
io6 = 6
adc1 = 0
adc = 0
adc_1 = 0
adc_2 = 0
adc_value = [0]
io_data = [0]
zhuan = 0
k = 0
taps1 = 0
mid = 0
tag_width = 0
tags = []
distance = 0
di_fang_kuai = 1
zhong_li_kuai = 0
zha_dan_kuai = 2
index = 0


# tui =0

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
        global taps1, index
        global k
        global mid
        global tag_width
        global tags
        global distance
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        k = 0
        index = 0
        if tags:
            k = 1  # 这是个标志位
            for i in range(1, len(tags)):
                # 循环从第二个（results[1]）索引开始，进行冒泡排序。（因为前方index是从零开始的）所以排序没有遗漏
                if tags[i].tag_id == di_fang_kuai or tags[i].tag_id == zhong_li_kuai:
                    # 如果tag码块id是敌方或中立才进行距离比较，炸弹不管
                    if tags[index].tag_id == zha_dan_kuai:
                        # 这一步确保index=0的那个id不是炸弹，若是炸弹，则将索引就改成i（i现在肯定不是炸弹）
                        index = i
                        if tags[i].tag_id == zhong_li_kuai:
                            if tags[index].tag_id == di_fang_kuai:
                                index = i
                            elif (self.get_distance(tags[index].homography, 4300) >
                                  self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                        elif tags[index].tag_id == di_fang_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
            if tags[index].tag_id == di_fang_kuai or tags[index].tag_id == zhong_li_kuai:  # 冒泡后如果最近的id是中立或敌方
                taps1 = 1
            else:  # 冒泡后如果的id是炸弹块(侧面证明了没有检测到敌方和中立)
                taps1 = 0
            distance = int(self.get_distance(tags[index].homography, 4300))
            mid = tuple(tags[index].corners[0].astype(int))[0] / 2 + \
                  tuple(tags[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
            tag_width = abs(tuple(tags[index].corners[0].astype(int))[0] - tuple(tags[index].corners[2].astype(int))[0])
            # print(taps1, tags[index].tag_id)
        else:
            k = 0

        # for tag in tags:
        #     k = 1
        #     distance = self.get_distance(tag.homography, 4300)
        #     if tag.tag_id == 1:
        #         mx1 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
        #         if mx1 > m1:
        #             m1 = mx1
        #             mid1 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
        #         h1 = 1
        #     if tag.tag_id == 0:
        #         mx0 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
        #         if mx0 > m0:
        #             m0 = mx0
        #             mid0 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
        #         h0 = 1
        #     elif tag.tag_id == 2:
        #         mx2 = abs(tuple(tag.corners[0].astype(int))[0] - tuple(tag.corners[2].astype(int))[0])
        #         if mx2 > m2:
        #             m2 = mx2
        #             mid2 = tuple(tag.corners[0].astype(int))[0] / 2 + tuple(tag.corners[2].astype(int))[0] / 2
        #         h2 = 1
        # # cv2.circle(frame,), 4, (255, 0, 0), 2) # left-top
        # # cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
        # # cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
        # # cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
        # # apriltag_width = abs(tag.corners[0][0] - tag.corners[1][0]) / 2
        # # target_x = apriltag_width / 2
        # if h1 == 1:
        #     mid = mid1
        #     tag_width = m1
        #     taps1 = 1
        # if h0 == 1 and h1 == 0:
        #     mid = mid0
        #     tag_width = m0
        #     taps1 = 0
        # elif h1 == 0 and h0 == 0 and h2 == 1:
        #     mid = mid2
        #     tag_width = m2
        #     taps1 = 2
        # if not tags:
        #     k = 0

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


def qian_jin():
    up.CDS_SetSpeed(2, -800)
    up.CDS_SetSpeed(1, 800)


def qian_jin_kuai():
    up.CDS_SetSpeed(2, -500)
    up.CDS_SetSpeed(1, 500)


def kuai_jin():
    up.CDS_SetSpeed(2, -900)
    up.CDS_SetSpeed(1, 900)


def wei_jin():
    up.CDS_SetSpeed(2, -250)
    up.CDS_SetSpeed(1, 250)


def hou_tui():
    up.CDS_SetSpeed(2, 500)
    up.CDS_SetSpeed(1, -500)


def wei_tui():
    up.CDS_SetSpeed(2, 300)
    up.CDS_SetSpeed(1, -300)


def kuai_tui():
    up.CDS_SetSpeed(2, 800)
    up.CDS_SetSpeed(1, -800)


def zuo_zhuan():
    up.CDS_SetSpeed(2, -400)
    up.CDS_SetSpeed(1, -400)


def zuo_zhuan_1():
    up.CDS_SetSpeed(2, -420)
    up.CDS_SetSpeed(1, 350)


def you_zhuan_1():
    up.CDS_SetSpeed(2, -350)
    up.CDS_SetSpeed(1, 420)


def zuo_kuai_zhuan():
    up.CDS_SetSpeed(2, -800)
    up.CDS_SetSpeed(1, -800)


def you_zhuan():
    up.CDS_SetSpeed(2, 400)
    up.CDS_SetSpeed(1, 400)


def you_kuai_zhuan():
    up.CDS_SetSpeed(2, 800)
    up.CDS_SetSpeed(1, 800)


def zuo_zhuan_tag():
    up.CDS_SetSpeed(1, -360)
    up.CDS_SetSpeed(2, -360)


def you_zhuan_tag():
    up.CDS_SetSpeed(1, 360)
    up.CDS_SetSpeed(2, 360)


def ting():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


def tiaozheng_1():
    up.CDS_SetAngle(3, 334, 900)
    up.CDS_SetAngle(4, 690, 900)
    time.sleep(0.5)
    while out == 1:
        print('在台下对准')
        ting()
        time.sleep(0.3)
        # kuai_tui()
        # time.sleep(0.8)
        print("正在上台")
        kuai_tui()
        time.sleep(0.9)
        if adc1 > ADC1:
            hou_tui()
            up.CDS_SetAngle(3, 890, 800)
            up.CDS_SetAngle(4, 134, 800)
            time.sleep(0.7)
            up.CDS_SetAngle(3, 750, 712)
            up.CDS_SetAngle(4, 274, 712)
            time.sleep(0.5)
            print("上台完成")
            ting()
            time.sleep(0.3)
            zuo_kuai_zhuan()
            time.sleep(0.7)
    if out == 4 or out == 2:
        zuo_kuai_zhuan()
        # time.sleep(0.1)
    elif out == 3:
        you_kuai_zhuan()
        # time.sleep(0.1)


def tag_solve_1():
    global tag_width
    global taps1
    global tui
    print(k, taps1)
    print("juli:", distance)
    print(mid - 350 + tag_width / 6)
    if out == 4 and distance > 80:
        if mid < 350 - tag_width / 6:
            # zuo_zhuan()
            # time.sleep(0.03)
            # qian_jin_kuai()
            # time.sleep(0.015)
            zuo_zhuan_1()
            time.sleep(0.03)
            print("zuo1")
        elif mid > 350 + tag_width / 6:
            # you_zhuan()
            # time.sleep(0.03)
            # qian_jin_kuai()
            # time.sleep(0.015)
            you_zhuan_1()
            time.sleep(0.03)
            print("you1")
        else:
            qian_jin_kuai()
            # time.sleep(1)
            print("摄像头对准目标1")
    elif 45 < distance < 80:
        if mid < 300 - tag_width / 6:
            zuo_zhuan_tag()
            time.sleep(0.01)
            print("zuo2")
        elif mid > 380 + tag_width / 6:
            you_zhuan_tag()
            time.sleep(0.01)
            print("you2")
        else:
            qian_jin_kuai()
            print("摄像头对准目标2")
    while out == 1 and distance < 45:
        # tui = 1
        if adc > ADC2:
            qian_jin_kuai()
            print("内侧对准二维码1")
        elif adc < ADC2:
            wei_jin()
            print("红外对准二维码2")
        if zhuan != 0:
            ting()
    # elif (out != 4) and (zhuan == 5):
    #     wei_jin()
    #     print("红外对准二维码2")
    # if out == 4 and tui == 1:
    #     print("前方无障碍")
    #     kuai_tui()
    #     time.sleep(0.2)
    #     tui = 0


def tag_solve_2():
    if distance < 150:
        print("炸弹块")
        hou_tui()
        time.sleep(0.15)
        zuo_kuai_zhuan()
        time.sleep(0.3)
        print("左转")


# def car_solve():
#     if out != 4 and adc > ADC2:
#         print("有车1")
#         kuai_jin()
#     elif out != 4 and adc < ADC2:
#         print("有车2")
#         qian_jin()
# def taishang_0():
#     # up.CDS_SetAngle(3, 742, 512)
#     # up.CDS_SetAngle(4, 282, 512)
#     # if zhuan == 0:
#     #     qian_jin()
#     if zhuan == 3:
#         kuai_tui()
#         time.sleep(0.5)
#         zuo_kuai_zhuan()
#         time.sleep(0.5)
#     elif zhuan == 4:
#         kuai_tui()
#         time.sleep(0.5)
#         you_kuai_zhuan()
#         time.sleep(0.5)
#     elif zhuan == 5:
#         print("到边缘2")
#         kuai_tui()
#         time.sleep(0.5)
#         you_zhuan()
#         time.sleep(0.5)
#
#
# def taishang_1():
#     tag_solve_1()
#     if out == 4:
#         taishang_0()
#
#
# def taishang_2():
#     taishang_0()
#     tag_solve_2()
#     if zhuan == 0:
#         qian_jin()


# def taishang_3():
#     taishang_0()
#     car_solve()


# def taishang():
#     up.CDS_SetAngle(3, 747, 512)
#     up.CDS_SetAngle(4, 277, 512)
#     if k == 1:
#         if taps1 == 1:
#             taishang_1()
#             # if zhuan == 1:
#             #     kuai_tui()
#             #     time.sleep(0.3)
#             #     ting()
#             #     zuo_kuai_zhuan()
#             #     time.sleep(0.3)
#             #     print("车左转", k, zhuan)
#             # elif zhuan == 2:
#             #     kuai_tui()
#             #     time.sleep(0.3)
#             #     ting()
#             #     you_kuai_zhuan()
#             #     time.sleep(0.3)
#             #     print("车右转", k, zhuan)
#         elif taps1 == 0:
#             taishang_2()
#     elif k == 0:
#         taishang_0()
#         if zhuan == 0 and adc > ADC2:
#             qian_jin()
#             if out != 4:
#                 print("cheqianjin")
#                 qian_jin()//////
#             # elif zhuan == 1:
#             #     kuai_tui()
#             #     time.sleep(0.3)
#             #     ting()
#             #     zuo_kuai_zhuan()
#             #     time.sleep(0.3)
#             #     print("车左转", k)
#             # elif zhuan == 2:
#             #     kuai_tui()
#             #     time.sleep(0.3)
#             #     ting()
#             #     you_kuai_zhuan()
#             #     time.sleep(0.3)
#             #     print("车右转", k)
#         elif zhuan == 0 and adc < ADC2:
#             up.CDS_SetSpeed(2, -300)
#             up.CDS_SetSpeed(1, 300)


def taishang():
    global k
    up.CDS_SetAngle(3, 702, 512)
    up.CDS_SetAngle(4, 322, 512)
    # if adc > ADC2:
    #     qian_jin()
        # if zhuan == 1:
        #     zuo_kuai_zhuan()
        #     print("车左转")
        # elif zhuan == 2:
        #     you_kuai_zhuan()
        #     print("车右转")
    # else:
    if zhuan == 0:
        qian_jin()
        # if out != 4:
        #     print("cheqianjin")
        #     kuai_jin()
    elif zhuan == 3:
        # kuai_tui()
        # time.sleep(0.1)
        ting()
        # kuai_tui()
        # time.sleep(0.05)
        zuo_kuai_zhuan()
        time.sleep(0.05)
        # tag_solve()
    elif zhuan == 4:
        ting()
        # kuai_tui()
        # time.sleep(0.05)
        you_kuai_zhuan()
        time.sleep(0.05)
        # tag_solve()
    elif zhuan == 5:
        print("到边缘2")
        kuai_tui()
        time.sleep(0.3)
        # ting()
        # kuai_tui()
        # time.sleep(0.05)
        you_kuai_zhuan()
        # time.sleep(0.1)


def adio_start_detect():
    global adc1
    global adc
    global out
    global adc_value
    global io6
    global io5
    global io4
    global io3
    global io2
    global io1
    global io0
    global io_data
    global flag
    global zhuan
    global adc_1
    global adc_2
    while True:
        adc_value = up.ADC_Get_All_Channle()
        adc = (adc_value[1] + adc_value[0]) / 2
        adc1 = (adc_value[2] + adc_value[3]) / 2
        adc_1 = adc_value[1]
        adc_2 = adc_value[3]
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

        if adc < ADC3:
            flag = 0
        elif adc > ADC3:
            flag = 1

        if io0 == 0 and io1 == 0:
            out = 1
        elif io0 == 1 and io1 == 0:
            out = 2
        elif io0 == 0 and io1 == 1:
            out = 3
        elif io0 == 1 and io1 == 1:
            out = 4

        if io2 == 0 and io3 == 0:
            zhuan = 0  # 前进
        elif io2 == 1 and io3 == 0:
            zhuan = 3  # 左转
        elif io2 == 0 and io3 == 1:
            zhuan = 4  # 右转
        elif io2 == 1 and io3 == 1:
            zhuan = 5  # 后退

        if io5 == 0:
            zhuan = 1  # zuozhuan
        elif io4 == 0:
            zhuan = 2  # youzhuan


def signal_handler(handler_signal, handler_frame):
    ting()
    exit(0)


if __name__ == "__main__":
    # 读取数据
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
    signal.signal(signal.SIGINT, signal_handler)
    target2 = threading.Thread(target=adio_start_detect)
    target2.start()

    while True:
        taishang()
        # up.CDS_SetAngle(3, 702, 512)
        # up.CDS_SetAngle(4, 322, 512)
        # print("adc:", adc)
        # print("后灰度:", adc_1)
        # print("左灰度:", adc_2)
        # print("adc1:", adc1)
        # time.sleep(0.1)
        qian_jin()
