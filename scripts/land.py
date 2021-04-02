## Additional installation for SITL:
##      pip3 install dronekit-sitl -UI
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
import sys
import cv2
import numpy as np
import math as m

#######################################
# Global variables
#######################################
#是否开启录像保存
enable_capture_save = True

#精确降落部分#
#相机fov参数设置
horizontal_fov = 73.3 * m.pi/180
vertical_fov = 58.3 * m.pi/180
#分辨率
horizontal_resolution = 640
vertical_resolution = 480

current_time_us = 0
#######################################
# User input
#######################################
# Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#######################################
# Functions
#######################################
capture = cv2.VideoCapture(0)
#width, height = capture.get(3), capture.get(4)
#设置摄像头的分辨率
capture.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_resolution)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_resolution)
#capture.set(cv2.CAP_PROP_FPS, 60)
#frame =cv2.imread("./3.jpg")
#capture.set(cv2.CAP_PROP_CONTRAST,50)
if enable_capture_save:
    time_print = time.strftime("%Y-%m-%d-%H-%M-%S",time.localtime(int(time.time())))
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    outfile = cv2.VideoWriter('output'+'{}'.format(time_print)+'.avi', fourcc, 30., (640, 480))

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
def listener(self,name,message):
    global got_landing_massage
    if message is not None:
        got_landing_massage = True
        print("got a land message")

def send_land_message(x, y):
    global current_time_us
    msg = vehicle.message_factory.landing_target_encode(
        current_time_us,                       # time target data was processed, as close to sensor capture as possible
        0,                                  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used   #疑问：为什么给的注释是not used，未来的测试点   #经查阅Ardupilot的源码,此参数确实没用
        angle_x,                       # X-axis angular offset, in radians
        angle_y,                       # Y-axis angular offset, in radians
        rangefinder_dis_land,                           # distance, in meters
        0,                                  # Target x-axis size, in radians
        0,                                  # Target y-axis size, in radians
        0,                                  # x	float	X Position of the landing target on MAV_FRAME
        0,                                  # y	float	Y Position of the landing target on MAV_FRAME
        0,                                  # z	float	Z Position of the landing target on MAV_FRAME
        (1,0,0,0),      # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        3,              # type of landing target: 2 = Fiducial marker
        1,              # position_valid boolean
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

while(True):
    starttime = time.time()
    ret, frame = capture.read()
    if (vehicle.mode.name == "LAND"):
        current_time_us = int(round(time.time() * 1000000))
        #灰度化
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #高斯滤波
        #gaussian_smoothing = cv2.GaussianBlur(gray,(5,5),0)
        #canny边缘检测算法
        #canny_dection = cv2.Canny(gaussian_smoothing,35,120)
        #ret, thresh = cv2.threshold(gaussian_smoothing, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        #contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(canny_dection_new, contours, -1, (255, 255, 255), 1)

        circle1 = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=50, minRadius=30, maxRadius=120)
        #需要调参
        #frame_circles=frame.copy()
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
                    cv2.circle(frame_circles, (i[0], i[1]), i[2], (255, 0, 0), 2)  # 画圆
                    cv2.circle(frame_circles, (i[0], i[1]), 2, (255, 0, 0), 2)  # 画圆心
                x = i[0]
                y = i[1]
                angle_x = (x-horizontal_resolution/2)*horizontal_fov/horizontal_resolution
                angle_y = (y-vertical_resolution/2)*vertical_fov/vertical_resolution
            if x is not None:
                if y is not None:
                    send_land_message(x,y)
                    if enable_capture_save:
                        cv2.putText(frame_circles, '{}'.format('got a land target'), (0,30), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                        cv2.putText(frame_circles, 'x:{} , y:{}'.format(x,y), (0,45), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                        cv2.putText(frame_circles, 'angle_x:{} , angle_y:{}'.format(round(angle_x,6),round(angle_y,6)), (0,60), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
                    print("send a land message")
            else:
                print("LAND MESSAGE IS FALSE!")
                pass

        #cv2.imshow('frame', frame)
        #cv2.imshow('gray', gray)
        #cv2.imshow('gaussian_smoothing', gaussian_smoothing)
        #cv2.imshow('canny_dection', canny_dection)
        #cv2.imshow('thresh',thresh)
        #cv2.imshow('frame_new',frame_new)
        #cv2.imshow('frame_circles',frame_circles)

    if cv2.waitKey(1) & 0xFF == 27:
            break
    
    if enable_capture_save:
        if ret is not None:
            #画面添加高度信息
            cv2.putText(frame_circles, 'dis:{}m'.format(round(rangefinder_dis_land+0.12,2)), (0,15), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 255, 0), 1, lineType=cv2.LINE_AA)
            #添加FPS信息
            if endtime != starttime:
                cv2.putText(frame_circles, 'FPS:{}'.format(1/(endtime - starttime)), (0,75), cv2.FONT_HERSHEY_COMPLEX,0.5, (0, 0, 255), 1, lineType=cv2.LINE_AA)
            #写入文件
            #写入原图像
            #outfile.write(frame)
            #写入绘制了圆和圆心的图像
            outfile.write(frame_circles)
            #cv2.imshow('frame', frame)


