#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import uptech
import time
if __name__ == '__main__':
    up=uptech.UpTech()
    up.CDS_Open()
    while True:
        # up.CDS_SetSpeed(1,-300)
        # time.sleep(0.5)
        up.CDS_SetSpeed(1,500)
        up.CDS_SetSpeed(2,0)
        time.sleep(3)