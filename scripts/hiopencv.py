# 打开摄像头并灰度化显示
import cv2
import time
capture = cv2.VideoCapture(0)

while(True):
    starttime = time.time()
    # 获取一帧
    ret, frame = capture.read()
    # 将这帧转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('frame', gray)
    endtime = time.time()
    print(1/(endtime - starttime))
    if cv2.waitKey(1) == ord('q'): 
        break