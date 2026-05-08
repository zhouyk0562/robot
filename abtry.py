# -*- coding: utf-8 -*-
import uptech
import time


class GrayscaleMonitor:
    def __init__(self):
        self.up = uptech.UpTech()
        self.up.CDS_Open()
        self.up.ADC_IO_Open()
        self.sampling_interval = 0.2  # 采样间隔(秒)

    def get_grayscale(self):
        """获取前后灰度传感器值"""
        adc_values = self.up.ADC_Get_All_Channle()
        return adc_values[0], adc_values[1]  # 前,后

    def realtime_monitor(self, duration=60):
        """
        实时监测灰度传感器值
        :param duration: 监测持续时间(秒)，默认为60秒
        """
        print(f"开始实时灰度监测，将持续{duration}秒...")
        print("按Ctrl+C可提前终止")
        print("----------------------------")

        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                front, rear = self.get_grayscale()
                timestamp = time.strftime("%H:%M:%S", time.localtime())

                print(f"[{timestamp}] 前传感器: {front:4d} | 后传感器: {rear:4d} | 差值: {front - reear:4d}")
                time.sleep(self.sampling_interval)

        except KeyboardInterrupt:
            print("\n监测提前终止")
        finally:
            print("----------------------------")
            print("灰度监测结束")


if __name__ == "__main__":
    monitor = GrayscaleMonitor()
    monitor.realtime_monitor(duration=60)  # 可以修改监测时长