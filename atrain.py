# -*- coding: utf-8 -*-
import uptech
import time


def zuo_zhuan():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, -400)


def you_zhuan():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, 400)


def qian_jin():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, -500)


def hou_tui():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, 500)


def ting():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


if __name__ == "__main__":
    up = uptech.UpTech()
    up.CDS_Open()
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)

    try:
        while True:
            print("\nTest Menu:")
            print("1. Forward")
            print("2. Backward")
            print("3. Turn Left")
            print("4. Turn Right")
            print("5. Stop")
            print("6. Exit")

            choice = input("Enter your choice (1-6): ")

            if choice == '1':
                print("Moving forward...")
                qian_jin()
            elif choice == '2':
                print("Moving backward...")
                hou_tui()
            elif choice == '3':
                print("Turning left...")
                zuo_zhuan()
            elif choice == '4':
                print("Turning right...")
                you_zhuan()
            elif choice == '5':
                print("Stopping...")
                ting()
            elif choice == '6':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")

            time.sleep(0.1)  # Small delay to prevent rapid switching

    except KeyboardInterrupt:
        print("\nProgram stopped by user")

    finally:
        ting()  # Ensure the robot stops
        up.CDS_Close()
        print("Motors stopped and resources released")