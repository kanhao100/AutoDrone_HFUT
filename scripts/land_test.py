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
import threading
#######################################
# Global variables
#######################################
#精确降落
horizontal_fov = 85 * m.pi/180
vertical_fov = 50 * m.pi/180
horizontal_resolution = 320
vertical_resolution = 240
capture = cv2.VideoCapture(0)
width, height = capture.get(3), capture.get(4)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
#capture.set(cv2.CAP_PROP_FPS, 60)
#frame =cv2.imread("./3.jpg")
#capture.set(cv2.CAP_PROP_CONTRAST,50)

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
def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while True:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)
# Listen to ATTITUDE data: https://mavlink.io/en/messages/common.html#ATTITUDE
def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        progress("INFO: Received ATTITUDE msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad),))

connection_string_default = '/dev/ttyACM0'
connection_baudrate_default = 921600
connection_string = args.connect
connection_baudrate = args.baudrate
sitl = None
if not connection_baudrate:
    connection_baudrate = connection_baudrate_default
if not connection_string:
    connection_string = connection_string_default
# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)
mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

def send_land_message(x, y):
    global current_time_us
    msg = vehicle.message_factory.landing_target_encode(
        current_time_us,                       # time target data was processed, as close to sensor capture as possible
        0,                                  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        (x-horizontal_resolution/2)*horizontal_fov/horizontal_resolution,                       # X-axis angular offset, in radians
        (y-vertical_resolution/2)*vertical_fov/vertical_resolution,                       # Y-axis angular offset, in radians
        0,                           # distance, in meters
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
    if (vehicle.mode.name == "LAND"):
        current_time_us = int(round(time.time() * 1000000))
        ret, frame = capture.read()
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
        #frame_circles=frame.copy()
        if circle1 is None:
            pass
        else:
            circles = circle1[0, :, :]  # 提取为二维
            circles = np.uint16(np.around(circles))  # 四舍五入，取整
            for i in circles[:]:
                #cv2.circle(frame_circles, (i[0], i[1]), i[2], (255, 0, 0), 2)  # 画圆
                #cv2.circle(frame_circles, (i[0], i[1]), 2, (255, 0, 0), 2)  # 画圆心
                x = i[0]
                y = i[1]
            if x is not None:
                send_land_message(x,y)
        #cv2.imshow('frame', frame)
        #cv2.imshow('gray', gray)
        #cv2.imshow('gaussian_smoothing', gaussian_smoothing)
        #cv2.imshow('canny_dection', canny_dection)
        #cv2.imshow('thresh',thresh)
        #cv2.imshow('frame_new',frame_new)
        #cv2.imshow('frame_circles',frame_circles)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    else:
        pass