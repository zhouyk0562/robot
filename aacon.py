# -*- coding: utf-8 -*-
import uptech
import time
import sys
import tty
import termios


class KeyboardControl:
    def __init__(self):
        """初始化电机和传感器 Initialize motors and sensors"""
        self.board = uptech.UpTech()
        self.board.CDS_Open()  # 开启电机驱动 Enable motor driver
        self.board.ADC_IO_Open()  # 开启ADC输入 Enable ADC input

        # 运动参数配置 Motion parameters
        self.base_speed = 350  # 常规移动速度(PWM值) Normal moving speed (PWM value)
        self.turn_speed = 420  # 转向速度 Turning speed
        self.speed_step = 50  # 每次加减速幅度 Speed adjustment step
        self.speed_offset = 10  # 左右电机速度差 Speed difference between left and right motors

        # 当前运动状态 Current movement status
        self.left_speed = 0
        self.right_speed = 0

    def get_key(self):
        """非阻塞式获取键盘输入 Non-blocking keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def set_speed(self, left, right):
        """更新电机速度并记录当前值，左电机速度比右电机绝对值大50"""
        # 根据方向调整速度差
        if left > 0:
            adjusted_left = abs(left) + self.speed_offset
        elif left < 0:
            adjusted_left = -(abs(left) + self.speed_offset)
        else:
            adjusted_left = 0

        if right > 0:
            adjusted_right = abs(right)
        elif right < 0:
            adjusted_right = -abs(right)
        else:
            adjusted_right = 0

        # 限幅处理 (确保速度值在-1000到1000之间)
        adjusted_left = max(min(adjusted_left, 1000), -1000)
        adjusted_right = max(min(adjusted_right, 1000), -1000)

        self.left_speed = adjusted_left
        self.right_speed = adjusted_right

        # 注意第二个电机可能需要反向 Note: second motor may need reverse (adjust according to wiring)
        self.board.CDS_SetSpeed(1, adjusted_left)
        self.board.CDS_SetSpeed(2, -adjusted_right)

    def emergency_stop(self):
        """立即停止所有电机 Immediately stop all motors"""
        self.set_speed(0, 0)
        print("! EMERGENCY STOP !")

    def smooth_stop(self):
        """减速停止 Deceleration stop"""
        print("Stopping smoothly...")
        current_left = self.left_speed
        current_right = self.right_speed

        # 计算减速步数 Calculate deceleration steps
        steps = max(abs(current_left), abs(current_right)) // self.speed_step

        for i in range(steps, 0, -1):
            new_left = int(current_left * i / steps)
            new_right = int(current_right * i / steps)
            self.set_speed(new_left, new_right)
            time.sleep(0.05)

        self.emergency_stop()

    def speed_up(self):
        """逐步增加速度 Gradually increase speed"""
        new_left = self.left_speed + self.speed_step if self.left_speed >= 0 else self.left_speed - self.speed_step
        new_right = self.right_speed + self.speed_step if self.right_speed >= 0 else self.right_speed - self.speed_step

        # 速度限幅 Speed limit
        new_left = max(min(new_left, 1000), -1000)
        new_right = max(min(new_right, 1000), -1000)

        self.set_speed(new_left, new_right)
        print(f"Current speed: Left={new_left}, Right={new_right}")

    def speed_down(self):
        """逐步降低速度 Gradually decrease speed"""
        new_left = self.left_speed - self.speed_step if self.left_speed > 0 else self.left_speed + self.speed_step
        new_right = self.right_speed - self.speed_step if self.right_speed > 0 else self.right_speed + self.speed_step

        # 当速度接近0时直接停止 Stop directly when speed approaches 0
        if abs(new_left) < self.speed_step:
            new_left = 0
        if abs(new_right) < self.speed_step:
            new_right = 0

        self.set_speed(new_left, new_right)
        print(f"Current speed: Left={new_left}, Right={new_right}")

    def run(self):
        """主控制循环 Main control loop"""
        print("Keyboard Control Instructions:")
        print("W: Forward  S: Backward")
        print("A: Turn Left  D: Turn Right")
        print("Q: Strafe Left  E: Strafe Right")
        print("Z: Stop  X: Exit")
        print("+: Speed Up  -: Speed Down")

        try:
            while True:
                key = self.get_key().lower()

                if key == 'w':  # 前进 Forward
                    self.set_speed(self.base_speed, self.base_speed)
                    print("Moving FORWARD")
                elif key == 's':  # 后退 Backward
                    self.set_speed(-self.base_speed, -self.base_speed)
                    print("Moving BACKWARD")
                elif key == 'a':  # 左转 Turn Left
                    self.set_speed(-self.turn_speed, self.turn_speed)
                    print("Turning LEFT")
                elif key == 'd':  # 右转 Turn Right
                    self.set_speed(self.turn_speed, -self.turn_speed)
                    print("Turning RIGHT")
                elif key == 'q':  # 左平移 Strafe Left
                    self.set_speed(-self.base_speed, self.base_speed)
                    print("Strafing LEFT")
                elif key == 'e':  # 右平移 Strafe Right
                    self.set_speed(self.base_speed, -self.base_speed)
                    print("Strafing RIGHT")
                elif key == 'z':  # 停止 Stop
                    self.smooth_stop()
                elif key == '+':  # 增加速度 Speed Up
                    self.speed_up()
                elif key == '-':  # 减少速度 Speed Down
                    self.speed_down()
                elif key == 'x':  # 退出 Exit
                    print("Exiting program...")
                    break

                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        finally:
            self.emergency_stop()
            self.board.CDS_Close()
            self.board.ADC_IO_Close()


if __name__ == "__main__":
    controller = KeyboardControl()
    controller.run()