import time
from uptech import UpTech


class Servo(object):
    up = UpTech()

    def __init__(self):
        up = self.up
        up.ADC_IO_Open()
        up.CDS_Open()
        """
        0为舵机，1为电机
        调试舵机不需要定义01

        6号越大越往前转
        5号越往后越大
        如果要使机器人往后站立，减6加5（必须加减相同角度）

        4号越往前越小
        10号越往后越大
        8大 上
        3大 下
        9大 上
        7大 下
        """

    def attack_straight(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(4, 1023, 500)
        up.CDS_SetAngle(10, 100, 500)
        up.CDS_SetAngle(8, 600, 500)
        up.CDS_SetAngle(7, 700, 500)
        up.CDS_SetAngle(9, 660, 500)
        up.CDS_SetAngle(3, 400, 500)

    def attack_left(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(4, 420, 500)
        """
        up.CDS_SetAngle(8, 1023, 500)
        up.CDS_SetAngle(7, 1023, 500)
        time.sleep(0.5)
        """
        up.CDS_SetAngle(8,750,900)
        up.CDS_SetAngle(7, 690, 950)
        up.CDS_SetAngle(10, 0, 800)
        up.CDS_SetAngle(9, 500, 650)
        up.CDS_SetAngle(3,500,800)

    def attack_right(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(10, 300, 800)
        up.CDS_SetAngle(3, 560, 970)
        time.sleep(0.05)
        up.CDS_SetAngle(9, 810, 970)
        up.CDS_SetAngle(4,130,800)
        up.CDS_SetAngle(7, 650, 500)
        up.CDS_SetAngle(8, 450, 500)

    def stand_up(self):
        up = self.up
        up.CDS_SetAngle(5, 623, 500)
        up.CDS_SetAngle(6, 400, 500)
        # up.CDS_SetSpeed(8,100)

    def hands_up(self):
        up = self.up
        up.CDS_SetAngle(4, 450, 500)
        up.CDS_SetAngle(10, 300, 500)
        up.CDS_SetAngle(8, 990, 500)
        up.CDS_SetAngle(7, 1023, 500)
        up.CDS_SetAngle(3, 900, 500)
        up.CDS_SetAngle(9, 1023, 500)

    def hands_up_left(self):
        up = self.up
        up.CDS_SetAngle(4, 450, 500)
        up.CDS_SetAngle(10, 300, 500)
        up.CDS_SetAngle(8, 990, 500)
        up.CDS_SetAngle(7, 1023, 500)
        #up.CDS_SetAngle(3, 900, 500)
        #up.CDS_SetAngle(9, 1023, 500)

        up.CDS_SetAngle(3, 400, 500)
        up.CDS_SetAngle(9, 500, 500)

    def hands_up_right(self):
        up = self.up
        up.CDS_SetAngle(4, 450, 500)
        up.CDS_SetAngle(10, 300, 500)
        up.CDS_SetAngle(3, 950, 500)
        up.CDS_SetAngle(9, 1023, 500)
        up.CDS_SetAngle(7, 650, 500)
        up.CDS_SetAngle(8, 450, 500)

    def intial_action(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(4,200,500)
        up.CDS_SetAngle(10,200,500)
        self.hands_up()

    def pull_down_left(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(4,130,500)
        up.CDS_SetAngle(8,300,500)
        up.CDS_SetAngle(7,1023,500)

    def pull_down_right(self):
        up = self.up
        #self.stand_up()
        #time.sleep(0.5)
        up.CDS_SetAngle(10,0,500)
        up.CDS_SetAngle(9,100,500)
        up.CDS_SetAngle(3,1000,500)

    def wave_left(self):
        up = self.up
        up.CDS_SetAngle(4,130,900)
        up.CDS_SetAngle(10, 0, 500)
        up.CDS_SetAngle(9,500,500)
        up.CDS_SetAngle(8, 780, 900)
        up.CDS_SetAngle(7, 690, 950)
        time.sleep(0.5)
        up.CDS_SetAngle(10, 350, 600)

        up.CDS_SetAngle(4,490,950)

    def wave_right(self):
        up = self.up
        up.CDS_SetAngle(10,0,600)
        up.CDS_SetAngle(4, 130, 500)
        up.CDS_SetAngle(8, 500, 500)
        up.CDS_SetAngle(3, 530, 500)
        up.CDS_SetAngle(9, 800, 500)
        time.sleep(0.5)
        up.CDS_SetAngle(10,410,1000)
        up.CDS_SetAngle(4,520, 1000)

    def test(self):
        up = self.up
        up.CDS_SetAngle(3,800,500)
        up.CDS_SetAngle(4, 130, 500)
        #up.CDS_SetAngle(8,1023,500)
        #up.CDS_SetAngle(7, 1023, 300)

if __name__ == '__main__':
    servo = Servo()
    #servo.stand_up()
    #servo.test()
    #servo.hands_up()
    #servo.hands_up_left()
    servo.hands_up_right()
    time.sleep(1.5)

    #servo.attack_right()
    #servo.attack_left()

    #servo.wave_left()
    servo.wave_right()

    #servo.intial_action()

    #servo.pull_down_left()
    #servo.pull_down_right()

    print("Testing...")
    # servo.attack()
