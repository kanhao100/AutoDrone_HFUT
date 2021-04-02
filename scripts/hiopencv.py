# 打开摄像头并灰度化显示
import cv2
import time
capture = cv2.VideoCapture(0)
horizontal_resolution = 640
vertical_resolution = 480
capture.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_resolution)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_resolution)
while(True):
    starttime = time.time()
    # 获取一帧
    ret, frame = capture.read()
    # 将这帧转换为灰度图
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    time_print = time.strftime("%Y-%m-%d-%H-%M-%S",time.localtime(int(time.time())))
    cv2.putText(frame, 'time:{}'.format(time_print), (0,15), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 255, 0), 1, lineType=cv2.LINE_AA)
    cv2.putText(frame, '{}'.format('got target 测试'), (0,30), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
    cv2.imshow('frame', frame)
    endtime = time.time()
    if endtime != starttime:
        print('FPS:{}'.format(1/(endtime - starttime)))

    if cv2.waitKey(1) == ord('q'): 
        break