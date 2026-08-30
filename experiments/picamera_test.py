# simple_picamera2_test.py

from picamera2 import Picamera2
import time

# 初始化 Pi Camera
picam2 = Picamera2()

# 開啟相機
picam2.start()

# 等待相機自動調整曝光
time.sleep(2)

# 拍照並存檔
picam2.capture_file("test.jpg")

print("拍照完成，已存檔為 test.jpg")

# 停止相機
picam2.stop()
