import cv2
import time
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
outfile = cv2.VideoWriter('output'+'{}'.format(time.time())+'.avi', fourcc, 30., (320, 240))
while(capture.isOpened()):
    ret, frame = capture.read()
    if ret:
        outfile.write(frame)  # 写入文件
        #cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    else:
        break