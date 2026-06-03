# -*- coding: utf-8 -*-
import uptech
import time


class MotorGrayscaleTester:
    def __init__(self):
        """初始化UpTech板卡和必要参数"""
        self.up = uptech.UpTech()
        self.up.CDS_Open()  # 开启电机控制
        self.up.ADC_IO_Open()  # 开启ADC输入
        self.test_speed = 300  # 默认测试速度

    def set_motors(self, left_speed, right_speed):
        """设置左右电机速度"""
        self.up.CDS_SetSpeed(1, left_speed)
        self.up.CDS_SetSpeed(2, right_speed)  # 注意右电机可能需要反向

    def stop_motors(self):
        """停止所有电机"""
        self.set_motors(0, 0)

    def get_grayscale(self):
        """获取前后灰度传感器值（返回：后, 前）"""
        adc_values = self.up.ADC_Get_All_Channle()
        return adc_values[0], adc_values[1]  # 根据实际接线调整通道号

    def test_movement(self):
        """电机运动测试序列"""
        try:
            print("\n=== 电机基础测试 ===")

            # 前进测试
            print("\n[1] 前进测试 2秒...")
            self.set_motors(self.test_speed, self.test_speed)
            time.sleep(0.5)

            # 左转测试
            print("[2] 左转测试 1.5秒...")
            self.set_motors(self.test_speed, -self.test_speed)
            time.sleep(0.5)

            # # 右转测试
            print("[3] 右转测试 1.5秒...")
            self.set_motors(self.test_speed, -self.test_speed // 2)
            time.sleep(0.5)

            #后退测试
            print("[4] 后退测试 2秒...")
            self.set_motors(-self.test_speed, -self.test_speed)
            time.sleep(0.5)

            #停止
            self.stop_motors()
            print("\n测试完成！")

        except KeyboardInterrupt:
            self.stop_motors()
            print("\n测试被中断")

    def test_grayscale(self, duration=30):
        """灰度传感器持续读取测试"""
        try:
            print("\n=== 灰度传感器测试 ===")
            print("将每秒输出传感器值，持续{}秒".format(duration))
            print("按Ctrl+C可提前结束\n")

            start_time = time.time()
            while time.time() - start_time < duration:
                rear, front = self.get_grayscale()
                print("前传感器: {:4d} | 后传感器: {:4d} | 差值: {:4d}".format(
                    front, rear, front - rear))
                time.sleep(1)

            print("\n测试完成！")

        except KeyboardInterrupt:
            print("\n测试被中断")

    def interactive_test(self):
        """交互式测试菜单"""
        while True:
            print("\n" + "=" * 40)
            print("UpTech 电机与灰度传感器测试程序")
            print("=" * 40)
            print("1. 电机运动测试")
            print("2. 灰度传感器测试")
            print("3. 设置测试速度 (当前: {})".format(self.test_speed))
            print("0. 退出程序")

            choice = input("请选择测试项目: ")

            if choice == "1":
                self.test_movement()
            elif choice == "2":
                duration = int(input("输入测试时长(秒): ") or "10")
                self.test_grayscale(duration)
            elif choice == "3":
                self.test_speed = int(input("输入测试速度(0-1000): ") or "300")
            elif choice == "0":
                print("程序退出")
                break
            else:
                print("无效输入，请重新选择")


if __name__ == "__main__":
    tester = MotorGrayscaleTester()
    try:
        tester.interactive_test()
    finally:
        tester.stop_motors()  # 确保程序退出时电机停止