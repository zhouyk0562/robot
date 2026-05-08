#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from uptech import UpTech  # 导入UpTech控制库


class ServoController:
    def __init__(self):
        """初始化舵机控制器"""
        # 创建UpTech板卡实例
        self.board = UpTech()

        # 定义可控制的舵机ID范围
        self.servo_ids = range(3, 11)  # ID 3-10

        # 舵机运动速度设置（0-100）
        self.servo_speed = 70

        # 舵机角度范围（0-1023）
        self.min_angle = 0
        self.max_angle = 1023

        # 初始化舵机系统
        self.initialize_servos()

    def initialize_servos(self):
        """初始化所有舵机"""
        print("正在初始化舵机系统...")

        # 打开CDS（舵机）接口
        if self.board.CDS_Open() == 0:
            print("舵机接口打开成功")
        else:
            print("舵机接口打开失败")
            return False

        # 将所有舵机设置为舵机模式（非电机模式）
        for servo_id in self.servo_ids:
            self.board.CDS_SetMode(servo_id, self.board.CDS_MODE_SERVO)
            print(f"舵机 {servo_id} 已设置为舵机模式")

        return True

    def set_servo_position(self, servo_id, position):
        """
        设置指定舵机的位置
        :param servo_id: 舵机ID (3-10)
        :param position: 目标位置 (0-1023)
        """
        if servo_id not in self.servo_ids:
            print(f"错误：无效的舵机ID {servo_id}，必须在3-10之间")
            return

        if position < self.min_angle or position > self.max_angle:
            print(f"错误：位置 {position} 超出范围，必须在{self.min_angle}-{self.max_angle}之间")
            return

        # 设置舵机位置
        self.board.CDS_SetAngle(servo_id, position, self.servo_speed)
        print(f"舵机 {servo_id} 已设置为位置 {position}")

        # 获取并显示当前实际位置
        current_pos = self.board.CDS_GetCurPos(servo_id)
        print(f"舵机 {servo_id} 当前实际位置: {current_pos}")

    def interactive_control(self):
        """交互式舵机控制"""
        print("\n舵机实时控制程序 (位置范围: 0-1023)")
        print("可用命令:")
        print("  set <ID> <位置> - 设置舵机位置 (如: set 5 512)")
        print("  get <ID>        - 获取舵机当前位置")
        print("  speed <值>      - 设置舵机运动速度 (0-100)")
        print("  list            - 显示所有舵机当前位置")
        print("  quit            - 退出程序")
        print("  help            - 显示帮助信息")

        while True:
            try:
                # 获取用户输入
                cmd = input("\n请输入命令: ").strip().lower()

                if cmd == 'quit' or cmd == 'exit':
                    break

                elif cmd == 'help':
                    print("\n命令说明:")
                    print("set <ID> <位置>: 设置指定ID舵机的位置 (0-1023)")
                    print("get <ID>: 获取指定ID舵机的当前位置")
                    print("speed <值>: 设置所有舵机的运动速度 (0-100)")
                    print("list: 显示所有舵机的当前位置")
                    print("quit: 退出程序")

                elif cmd.startswith('set '):
                    # 设置舵机位置命令
                    parts = cmd.split()
                    if len(parts) == 3:
                        try:
                            servo_id = int(parts[1])
                            position = int(parts[2])
                            self.set_servo_position(servo_id, position)
                        except ValueError:
                            print("错误：参数必须是数字")
                    else:
                        print("用法: set <ID> <位置>")

                elif cmd.startswith('get '):
                    # 获取舵机当前位置命令
                    parts = cmd.split()
                    if len(parts) == 2:
                        try:
                            servo_id = int(parts[1])
                            if servo_id in self.servo_ids:
                                pos = self.board.CDS_GetCurPos(servo_id)
                                print(f"舵机 {servo_id} 当前位置: {pos}")
                            else:
                                print("错误：无效的舵机ID")
                        except ValueError:
                            print("错误：ID必须是数字")
                    else:
                        print("用法: get <ID>")

                elif cmd.startswith('speed '):
                    # 设置速度命令
                    parts = cmd.split()
                    if len(parts) == 2:
                        try:
                            speed = int(parts[1])
                            if 0 <= speed <= 100:
                                self.servo_speed = speed
                                print(f"已设置舵机运动速度为 {speed}")
                            else:
                                print("错误：速度必须在0-100之间")
                        except ValueError:
                            print("错误：速度必须是数字")
                    else:
                        print("用法: speed <值>")

                elif cmd == 'list':
                    # 列出所有舵机当前位置
                    print("\n舵机当前位置:")
                    for servo_id in self.servo_ids:
                        pos = self.board.CDS_GetCurPos(servo_id)
                        print(f"  舵机 {servo_id}: {pos}")

                else:
                    print("未知命令，请输入help查看可用命令")

            except KeyboardInterrupt:
                print("\n接收到中断信号，正在退出...")
                break
            except Exception as e:
                print(f"发生错误: {str(e)}")

    def cleanup(self):
        """清理资源"""
        print("\n正在关闭舵机系统...")
        self.board.CDS_Close()
        self.board.stop()
        print("舵机系统已关闭")


if __name__ == "__main__":
    # 创建舵机控制器实例
    controller = ServoController()

    try:
        # 启动交互式控制
        controller.interactive_control()
    finally:
        # 确保程序退出前清理资源
        controller.cleanup()