import argparse
import math as m
import numpy as np
import cv2
import sys
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

#######################################
# Global variables
#######################################
# 是否开启录像保存
enable_capture_save = True

# 开启获取数据集模式(踩点模式)，即另外保存一个仅将高度打在屏幕上的录像
# 可以单独开启
enable_capture_simple = True

# 输入检测的圆的实际直径,单位m
d_true = 0.5

#精确降落部分#
# 相机fov参数设置
horizontal_fov = 73.3 * m.pi/180
vertical_fov = 58.3 * m.pi/180
# 分辨率
horizontal_resolution = 640
vertical_resolution = 480

current_time_us = 0
#######################################
# User input
#######################################
# Set up option parsing to get connection string
parser = argparse.ArgumentParser(
    description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#######################################
# Functions
#######################################
capture = cv2.VideoCapture(0)
#width, height = capture.get(3), capture.get(4)
# 设置摄像头的分辨率
capture.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_resolution)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_resolution)
#capture.set(cv2.CAP_PROP_FPS, 60)
#frame =cv2.imread("./3.jpg")
# capture.set(cv2.CAP_PROP_CONTRAST,50)
if enable_capture_save:
    time_print = time.strftime(
        "%Y-%m-%d-%H-%M-%S", time.localtime(int(time.time())))
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    outfile = cv2.VideoWriter(
        'output'+'{}'.format(time_print)+'.avi', fourcc, 30., (horizontal_resolution, vertical_resolution))
if enable_capture_simple:
    outfile_simple = cv2.VideoWriter(
        's_output'+'{}'.format(time_print)+'.avi', fourcc, 30., (horizontal_resolution, vertical_resolution))

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


@vehicle.on_message('RANGEFINDER')
def listener(self, name, message):
    global rangefinder_dis_land
    rangefinder_dis_land = message.distance - 0.12


@vehicle.on_message('LANDING_TARGET')
def listener(self, name, message):
    global got_landing_massage
    print(message)
    if message is not None:
        got_landing_massage = True
        print("got a land message")


'''
@vehicle.on_message('*')
def listener(self, name, message):
    print ('message: {}'.format(message))
'''


def send_land_message(x, y):
    global current_time_us
    msg = vehicle.message_factory.landing_target_encode(
        # time target data was processed, as close to sensor capture as possible
        current_time_us,
        0,                                  # target num, not used
        # frame, not used   #疑问：为什么给的注释是not used，未来的测试点   #经查阅Ardupilot的源码,此参数确实没用
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        angle_x,                       # X-axis angular offset, in radians
        angle_y,                       # Y-axis angular offset, in radians
        rangefinder_dis_land,                           # distance, in meters
        0,                                  # Target x-axis size, in radians
        0,                                  # Target y-axis size, in radians
        0,                                  # x	float	X Position of the landing target on MAV_FRAME
        0,                                  # y	float	Y Position of the landing target on MAV_FRAME
        0,                                  # z	float	Z Position of the landing target on MAV_FRAME
        # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        (1, 0, 0, 0),
        3,              # type of landing target: 2 = Fiducial marker
        1,              # position_valid boolean
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


while(True):
    starttime = time.time()
    ret, frame = capture.read()
    # if (vehicle.mode.name == "LAND"):
    if True:
        current_time_us = int(round(time.time() * 1000000))
        # 灰度化
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 高斯滤波
        #gaussian_smoothing = cv2.GaussianBlur(gray,(5,5),0)
        # canny边缘检测算法
        #canny_dection = cv2.Canny(gaussian_smoothing,35,120)
        #ret, thresh = cv2.threshold(gaussian_smoothing, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        #contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(canny_dection_new, contours, -1, (255, 255, 255), 1)
        if (rangefinder_dis_land+0.12) >= 1.6:
            circle1 = cv2.HoughCircles(
                gray, cv2.HOUGH_GRADIENT, 1, 400, param1=50, param2=75, minRadius=20, maxRadius=100)
        else:
            circle1 = cv2.HoughCircles(
                gray, cv2.HOUGH_GRADIENT, 1, 400, param1=100, param2=75, minRadius=60, maxRadius=320)
        # 需要调参
        #method: 检测方法，有HOUGH_GRADIENT以HOUGH_GRADIENT_ALT两种方法选择及
        #dp: 累加器分辨率与图像分辨率的反比，如果dp=1，累加器的分辨率与输入图像相同。如果dp=2，蓄能器有宽度和高度的一半。对于HOUGH_梯度_ALT，建议值为dp=1.5，
        #minDist: 检测到的圆中心之间的最小距离。如果参数是太小，除了一个真实的圆外，可能会错误地检测到多个相邻圆。如果是的话太大，可能会漏掉一些圆圈
        #param1: 在HOUGH_GRADIENT和HOUGH_GRADIENT_ALT两种模式时，它是传递给Canny边缘检测器的两个阈值中较高的一个（较低的阈值小两倍），注意HOUGH_GRADIENT_ALT模式使用Scharr算法，所以阈值通常较高
        #param2: 该值越小，可以检测到更多根本不存在的圆，该值越大，能通过检测的圆就更加接近完美的圆形
        #minRadius: 检测圆形的最小半径
        #maxRadius: 检测圆形的最大半径

        frame_circles = frame.copy()
        '''
        try:
            rangefinder_dis_land = round(vehicle.rangefinder,3) - 0.12
            rangefinder_dis_flag = True
            #print("rangefinder_dis: %s" %rangefinder_dis)
        except:
            rangefinder_dis_flag = False
            print("Can't read the value of rangefinder")
        '''
        if circle1 is None:
            pass
        else:
            circles = circle1[0, :, :]  # 提取为二维
            circles = np.uint16(np.around(circles))  # 四舍五入，取整
            for i in circles[:]:
                if enable_capture_save:
                    cv2.circle(frame_circles,
                               (i[0], i[1]), i[2], (255, 0, 0), 2)  # 画圆
                    cv2.circle(frame_circles,
                               (i[0], i[1]), 2, (255, 0, 0), 2)  # 画圆心
                x = i[0]
                y = i[1]
                r = i[2]
                angle_x = (x-horizontal_resolution/2) * \
                    horizontal_fov/horizontal_resolution
                angle_y = (y-vertical_resolution/2) * \
                    vertical_fov/vertical_resolution
                r_x = (d_true*horizontal_resolution) / \
                    (2*(rangefinder_dis_land+0.12)*horizontal_fov)
                r_y = (d_true*vertical_resolution) / \
                    (2*(rangefinder_dis_land+0.12)*vertical_fov)
            if x is not None:
                if y is not None:
                    try:
                        send_land_message(x, y)
                    except:
                        print("error to send a land message")
                        pass
                    if enable_capture_save:
                        cv2.putText(frame_circles, '{}'.format('got a land target'), (0, 45),
                                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                        cv2.putText(frame_circles, 'x:{},y:{},r:{}'.format(
                            x, y, r), (0, 60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                        cv2.putText(frame_circles, 'angle_x:{} , angle_y:{}'.format(round(angle_x, 6), round(
                            angle_y, 6)), (0, 75), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                        cv2.putText(frame_circles, 'r_x:{},r_y:{}'.format(
                            r_x, r_y), (0, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                    endtime = time.time()
                    if endtime != starttime:
                        print("send a land message, FPS:{}".format(1/(endtime - starttime)))
                    else:
                        print("send a land message, FPS:error")
                else:
                    print("LAND MESSAGE IS FALSE!")
                    pass
            else:
                print("LAND MESSAGE IS FALSE!")
                pass

        #cv2.imshow('frame', frame)
        #cv2.imshow('gray', gray)
        #cv2.imshow('gaussian_smoothing', gaussian_smoothing)
        #cv2.imshow('canny_dection', canny_dection)
        # cv2.imshow('thresh',thresh)
        # cv2.imshow('frame_new',frame_new)
        # cv2.imshow('frame_circles',frame_circles)

    if cv2.waitKey(1) & 0xFF == 27:
        break

    if enable_capture_save:
        if ret is not None:
            # if vehicle.mode.name == "LAND":
            # 画面添加高度信息
            cv2.putText(frame_circles, 'distance:{}m'.format(round(rangefinder_dis_land+0.12, 2)),
                        (0, 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, lineType=cv2.LINE_AA)
            # 添加FPS信息
            endtime = time.time()
            if endtime != starttime:
                cv2.putText(frame_circles, 'FPS:{}'.format(1/(endtime - starttime)), (0, 30),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
            # 写入文件
            # 写入绘制了圆和圆心的图像
            outfile.write(frame_circles)

    # 踩点模式
    if enable_capture_simple:
        cv2.putText(frame, '{}'.format(round(rangefinder_dis_land+0.12, 2)), (0, 15),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1, lineType=cv2.LINE_AA)
        #endtime = time.time()
        # if endtime != starttime:
        #    cv2.putText(frame, 'FPS:{}'.format(1/(endtime - starttime)), (0,30), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
        outfile_simple.write(frame)
        #cv2.imshow('frame', frame)
