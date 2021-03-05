import cv2
import numpy as np
import time

capture = cv2.VideoCapture(0)
width, height = capture.get(3), capture.get(4)
#print(width,height)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, width / 2)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height / 2)
capture.set(cv2.CAP_PROP_FPS, 60)


#frame =cv2.imread("./3.jpg")

#capture.set(cv2.CAP_PROP_CONTRAST,50)


'''
#ret, frame = capture.read()

#kernel = np.ones((5, 5), np.uint8)
#kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#灰度化
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#高斯滤波
gaussian_smoothing = cv2.GaussianBlur(gray,(5,5),0)
#canny边缘检测算法
canny_dection = cv2.Canny(gaussian_smoothing,35,120)
#实验性内容
#erosion = cv2.erode(canny_dection, kernel)
#opening = cv2.morphologyEx(canny_dection, cv2.MORPH_OPEN, kernel)
#canny_dection = cv2.Canny(gray,75,200)
ret, thresh = cv2.threshold(gaussian_smoothing, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
frame_new=frame.copy()
frame_new_gray=cv2.cvtColor(frame_new,cv2.COLOR_BGR2GRAY)

#cv2.drawContours(canny_dection_new, contours, -1, (255, 255, 255), 1)
circle1 = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=50, minRadius=30, maxRadius=120)

frame_circles=frame.copy()
if circle1 is None:
    pass
else:
    circles = circle1[0, :, :]  # 提取为二维
    circles = np.uint16(np.around(circles))  # 四舍五入，取整
    for i in circles[:]:
        cv2.circle(frame_circles, (i[0], i[1]), i[2], (255, 0, 0), 2)  # 画圆
        cv2.circle(frame_circles, (i[0], i[1]), 2, (255, 0, 0), 2)  # 画圆心

#检测H字符
H=cv2.imread(r"./H.jpg")
H_gray = cv2.cvtColor(H,cv2.COLOR_BGR2GRAY)
H_gaussian_smoothing = cv2.GaussianBlur(H_gray,(5,5),0)
H_ret, H_thresh = cv2.threshold(H_gaussian_smoothing,0,255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
H_contours, H_hierarchy = cv2.findContours(H_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for i in range(len(contours)):
    if(cv2.matchShapes(H_contours[0],contours[i:i+1][0],cv2.CONTOURS_MATCH_I1,0.0) < 0.2):
        taget = i
        print('the tageted contours number is %d' %(i))
        print("It's H ")
        break
    if(i == len(contours)):
        print("NO H")

    #print(contours[i:i+1])

canny_dection_new=canny_dection.copy()
cv2.drawContours(frame_new, contours, i, (0, 0, 255), 2)



#cv2.imshow('frame', frame)
#cv2.imshow('gray', gray)
#cv2.imshow('gaussian_smoothing', gaussian_smoothing)
cv2.imshow('canny_dection', canny_dection)
cv2.imshow('thresh',thresh)
cv2.imshow('frame_new',frame_new)
#cv2.imshow('frame_circles',frame_circles)

cv2.waitKey(0)
cv2.destroyAllWindows()
'''

while(True):
    starttime = time.time()
    ret, frame = capture.read()

    #kernel = np.ones((5, 5), np.uint8)
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    #灰度化
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #高斯滤波
    gaussian_smoothing = cv2.GaussianBlur(gray,(5,5),0)
    #canny边缘检测算法
    canny_dection = cv2.Canny(gaussian_smoothing,35,120)
    #实验性内容
    #erosion = cv2.erode(canny_dection, kernel)
    #opening = cv2.morphologyEx(canny_dection, cv2.MORPH_OPEN, kernel)
    #canny_dection = cv2.Canny(gray,75,200)
    ret, thresh = cv2.threshold(gaussian_smoothing, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    frame_new=frame.copy()
    frame_new_gray=cv2.cvtColor(frame_new,cv2.COLOR_BGR2GRAY)

    #cv2.drawContours(canny_dection_new, contours, -1, (255, 255, 255), 1)
    circle1 = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=50, minRadius=30, maxRadius=120)

    frame_circles=frame.copy()
    if circle1 is None:
        pass
    else:
        circles = circle1[0, :, :]  # 提取为二维
        circles = np.uint16(np.around(circles))  # 四舍五入，取整
        for i in circles[:]:
            cv2.circle(frame_circles, (i[0], i[1]), i[2], (255, 0, 0), 2)  # 画圆
            cv2.circle(frame_circles, (i[0], i[1]), 2, (255, 0, 0), 2)  # 画圆心

    #检测H字符
    H=cv2.imread(r"./H.jpg")
    H_gray = cv2.cvtColor(H,cv2.COLOR_BGR2GRAY)
    H_gaussian_smoothing = cv2.GaussianBlur(H_gray,(5,5),0)
    H_ret, H_thresh = cv2.threshold(H_gaussian_smoothing,0,255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    H_contours, H_hierarchy = cv2.findContours(H_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    contours_number = []
    #contours_number.clear()
    for i in range(len(contours)):
        contours_number.append(cv2.matchShapes(H_contours[0],contours[i:i+1][0],cv2.CONTOURS_MATCH_I1,0.0))
    try:
        print(min(contours_number))
    except:
        print("玄学异常")
    try:
        if(min(contours_number) < 0.8):
            temp = contours_number.index(min(contours_number))
            print('It\'s H, the tageted contours number is %d' %(temp))
            print(len(contours_number))
            print(contours_number[temp])
            try:
                cv2.drawContours(frame_new, contours, temp, (0, 0, 255), 2)
                #cv2.putText(frame_new, 'got H', (10, 500), cv2.FONT_HERSHEY_SIMPLEX,4, (0, 0, 255), 2, lineType=cv2.LINE_AA)
            except:
                pass
            contours_number=[]
        else:
            print("NO H")
            #if(i == len(contours)-1):
            #    print("NO H")
    except:
        print("玄学异常")
        #print(contours[i:i+1])
    canny_dection_new=canny_dection.copy()
    endtime = time.time()
    print('FPS:',1/(endtime - starttime))
    #if cv2.waitKey(30) == 27:
    #    break
