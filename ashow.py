import cv2
import time


def check_camera_availability(device_index=0, max_attempts=3):
    """
    检测摄像头是否可用
    :param device_index: 摄像头设备索引(通常0是默认摄像头)
    :param max_attempts: 最大尝试次数
    :return: (bool) 摄像头是否可用
    """
    print(f"正在检测摄像头/dev/video{device_index}...")

    cap = None
    for attempt in range(1, max_attempts + 1):
        try:
            cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
            if cap.isOpened():
                # 尝试读取一帧测试
                ret, _ = cap.read()
                if ret:
                    print(f"摄像头/dev/video{device_index}检测成功 (尝试 {attempt}/{max_attempts})")
                    return True
                else:
                    print(f"摄像头/dev/video{device_index}可以打开但无法读取帧 (尝试 {attempt}/{max_attempts})")
            else:
                print(f"无法打开摄像头/dev/video{device_index} (尝试 {attempt}/{max_attempts})")
        except Exception as e:
            print(f"检测摄像头时发生异常: {str(e)}")

        time.sleep(1)  # 每次尝试间隔1秒

    return False


def list_available_cameras(max_to_check=5):
    """列出所有可用的摄像头设备"""
    print("\n扫描可用的摄像头设备...")
    available_cams = []

    for i in range(max_to_check):
        if check_camera_availability(i, max_attempts=1):
            available_cams.append(i)

    if available_cams:
        print(f"\n找到 {len(available_cams)} 个可用摄像头:")
        for cam in available_cams:
            print(f"  /dev/video{cam}")
    else:
        print("\n未找到任何可用摄像头")

    return available_cams


if __name__ == "__main__":
    print("=== 摄像头检测程序 ===")

    # 1. 检测默认摄像头(通常为0)
    default_cam_available = check_camera_availability()

    if not default_cam_available:
        print("\n默认摄像头不可用，开始扫描其他摄像头...")
        list_available_cameras()
    else:
        # 获取摄像头信息
        cap = cv2.VideoCapture(0)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()

        print("\n默认摄像头信息:")
        print(f"  分辨率: {width}x{height}")
        print(f"  帧率: {fps:.2f} FPS")

    print("\n检测完成")