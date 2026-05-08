# -*- coding: utf-8 -*-
import threading
import time


def test_thread():
    while True:
        print("子线程运行!\n")
        time.sleep(1)


if __name__ == '__main__':
    t = threading.Thread(target=test_thread)
    t.daemon = True
    t.start()
    print('主线程运行..')
    time.sleep(0.6)
    print('主线程运行...')
    time.sleep(0.6)
    print('主线程运行...')
    time.sleep(0.6)
    print('主线程运行...')
    time.sleep(0.6)
    print('主线程运行完毕')