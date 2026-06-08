#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
if __name__ == '__main__':
    up=uptech.UpTech()
    up.CDS_Open()
    while True:
        up.CDS_SetSpeed(1, -1000)
        up.CDS_SetSpeed(2, -1000)
        time.sleep(1.5)
        up.CDS_SetSpeed(1, 0)
        up.CDS_SetSpeed(2, 0)
        break  # 退出循环