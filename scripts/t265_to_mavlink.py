#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# This script assume more_t265_devices.py
# Install required packages: 
#   pip3 install pyrealsense2
#   pip3 install transformations
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial
# Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")
sys.path.append("/home/hfut/.local/lib/python3.6/site-packages/")
# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
# Import the libraries
import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading
import signal
import time
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

start_time = time.time()
# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()
# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyACM0'
connection_baudrate_default = 921600
connection_timeout_sec_default = 5


# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation_default = 0

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
enable_msg_vision_position_delta = False
vision_position_delta_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
enable_msg_vision_speed_estimate = True
vision_speed_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#STATUSTEXT
enable_update_tracking_confidence_to_gcs = True
update_tracking_confidence_to_gcs_hz_default = 1.0

# Monitor user's online input via keyboard, can only be used when runs from terminal
enable_user_keyboard_input = True

# Default global position for EKF home/ origin
enable_auto_set_ekf_home = True
home_lat = 151269321    # Somewhere random
home_lon = 16624301     # Somewhere random
home_alt = 163000       # Somewhere random

# TODO: Taken care of by ArduPilot, so can be removed (once the handling on AP side is confirmed stable)
# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0  # In meters (m)
body_offset_y = 0  # In meters (m)
body_offset_z = 0  # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')

# lock for thread synchronization
lock = threading.Lock()
mavlink_thread_should_exit = False

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1


#######################################
# Global variables
#######################################
# FCU connection variables

# Camera-related variables
pipe = None
pose_sensor = None
pipe_2 = None
pose_sensor_2 = None
linear_accel_cov = 0.01
angular_vel_cov  = 0.01

# Data variables
data = None
prev_data = None
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = None
current_confidence_level = None
current_time_us = 0
all_tracker_confidnece = None

# Increment everytime pose_jumping or relocalization happens
# See here: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#are-there-any-t265-specific-options
# For AP, a non-zero "reset_counter" would mean that we could be sure that the user's setup was using mavlink2
reset_counter = 1

#######################################
# Parsing user' inputs
#######################################
parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--vision_position_estimate_msg_hz', type=float,
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--vision_position_delta_msg_hz', type=float,
                    help="Update frequency for VISION_POSITION_DELTA message. If not specified, a default value will be used.")
parser.add_argument('--vision_speed_estimate_msg_hz', type=float,
                    help="Update frequency for VISION_SPEED_DELTA message. If not specified, a default value will be used.")
parser.add_argument('--scale_calib_enable', default=False, action='store_true',
                    help="Scale calibration. Only run while NOT in flight")
parser.add_argument('--camera_orientation', type=int,
                    help="Configuration for camera orientation. Currently supported: forward, usb port to the right - 0; downward, usb port to the right - 1, 2: forward tilted down 45deg")
parser.add_argument('--debug_enable',type=int,
                    help="Enable debug messages on terminal")
args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_position_estimate_msg_hz = args.vision_position_estimate_msg_hz
vision_position_delta_msg_hz = args.vision_position_delta_msg_hz
vision_speed_estimate_msg_hz = args.vision_speed_estimate_msg_hz
scale_calib_enable = args.scale_calib_enable
camera_orientation = args.camera_orientation
debug_enable = args.debug_enable

# Using default values if no specified inputs
if not connection_string:
    connection_string = connection_string_default
    progress("INFO: Using default connection_string %s" % connection_string)
else:
    progress("INFO: Using connection_string %s" % connection_string)

if not connection_baudrate:
    connection_baudrate = connection_baudrate_default
    progress("INFO: Using default connection_baudrate %s" % connection_baudrate)
else:
    progress("INFO: Using connection_baudrate %s" % connection_baudrate)

if not vision_position_estimate_msg_hz:
    vision_position_estimate_msg_hz = vision_position_estimate_msg_hz_default
    progress("INFO: Using default vision_position_estimate_msg_hz %s" % vision_position_estimate_msg_hz)
else:
    progress("INFO: Using vision_position_estimate_msg_hz %s" % vision_position_estimate_msg_hz)
    
if not vision_position_delta_msg_hz:
    vision_position_delta_msg_hz = vision_position_delta_msg_hz_default
    progress("INFO: Using default vision_position_delta_msg_hz %s" % vision_position_delta_msg_hz)
else:
    progress("INFO: Using vision_position_delta_msg_hz %s" % vision_position_delta_msg_hz)

if not vision_speed_estimate_msg_hz:
    vision_speed_estimate_msg_hz = vision_speed_estimate_msg_hz_default
    progress("INFO: Using default vision_speed_estimate_msg_hz %s" % vision_speed_estimate_msg_hz)
else:
    progress("INFO: Using vision_speed_estimate_msg_hz %s" % vision_speed_estimate_msg_hz)

if body_offset_enabled == 1:
    progress("INFO: Using camera position offset: Enabled, x y z is %s %s %s" % (body_offset_x, body_offset_y, body_offset_z))
else:
    progress("INFO: Using camera position offset: Disabled")

if compass_enabled == 1:
    progress("INFO: Using compass: Enabled. Heading will be aligned to north.")
else:
    progress("INFO: Using compass: Disabled")

if scale_calib_enable == True:
    progress("\nINFO: SCALE CALIBRATION PROCESS. DO NOT RUN DURING FLIGHT.\nINFO: TYPE IN NEW SCALE IN FLOATING POINT FORMAT\n")
else:
    if scale_factor == 1.0:
        progress("INFO: Using default scale factor %s" % scale_factor)
    else:
        progress("INFO: Using scale factor %s" % scale_factor)

if not camera_orientation:
    camera_orientation = camera_orientation_default
    progress("INFO: Using default camera orientation %s" % camera_orientation)
else:
    progress("INFO: Using camera orientation %s" % camera_orientation)

if camera_orientation == 0:     # Forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:   # Downfacing, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
elif camera_orientation == 2:   # 45degree forward
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref))
else:                           # Default is facing forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

if not debug_enable:
    debug_enable = 0
else:
    debug_enable = 1
    np.set_printoptions(precision=4, suppress=True) # Format output on terminal 
    progress("INFO: Debug messages enabled.")

#######################################
# Functions - MAVLink
#######################################
def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
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

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_estimate_message():
    global current_time_us, H_aeroRef_aeroBody, reset_counter
    with lock:
        if H_aeroRef_aeroBody is not None:
            # Setup angle data
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(data.tracker_confidence))
            cov_twist   = angular_vel_cov  * pow(10, 1 - int(data.tracker_confidence))
            covariance  = np.array([cov_pose, 0, 0, 0, 0, 0,
                                       cov_pose, 0, 0, 0, 0,
                                          cov_pose, 0, 0, 0,
                                            cov_twist, 0, 0,
                                               cov_twist, 0,
                                                  cov_twist])

            try:
                rangefinder_dis = 0.12 - round(conn.messages['RANGEFINDER'].distance,3)
                rangefinder_dis_flag = True
                #progress("rangefinder_dis: %s" %rangefinder_dis)
            except:
                rangefinder_dis_flag = False
                #progress("Can't read the value of rangefinder!!maybe rangefinder is bad or mavlink is bad")
            # Send the message
            if rangefinder_dis_flag:
                conn.mav.vision_position_estimate_send(
                    current_time_us,            # us Timestamp (UNIX time or time since system boot)
                    H_aeroRef_aeroBody[0][3],   # Global X position
                    H_aeroRef_aeroBody[1][3],   # Global Y position
                    #H_aeroRef_aeroBody[2][3],   # Global Z position
                    rangefinder_dis,
                    rpy_rad[0],	                # Roll angle
                    rpy_rad[1],	                # Pitch angle
                    rpy_rad[2],	                # Yaw angle
                    covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                    reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
                )
            else:
                conn.mav.vision_position_estimate_send(
                    current_time_us,            # us Timestamp (UNIX time or time since system boot)
                    H_aeroRef_aeroBody[0][3],   # Global X position
                    H_aeroRef_aeroBody[1][3],   # Global Y position
                    H_aeroRef_aeroBody[2][3],   # Global Z position
                    #rangefinder_dis,
                    rpy_rad[0],	                # Roll angle
                    rpy_rad[1],	                # Pitch angle
                    rpy_rad[2],	                # Yaw angle
                    covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                    reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
                )

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
def send_vision_position_delta_message():
    global current_time_us, current_confidence_level, H_aeroRef_aeroBody
    with lock:
        if H_aeroRef_aeroBody is not None:
            # Calculate the deltas in position, attitude and time from the previous to current orientation
            H_aeroRef_PrevAeroBody      = send_vision_position_delta_message.H_aeroRef_PrevAeroBody
            H_PrevAeroBody_CurrAeroBody = (np.linalg.inv(H_aeroRef_PrevAeroBody)).dot(H_aeroRef_aeroBody)

            delta_time_us    = current_time_us - send_vision_position_delta_message.prev_time_us
            delta_position_m = [H_PrevAeroBody_CurrAeroBody[0][3], H_PrevAeroBody_CurrAeroBody[1][3], H_PrevAeroBody_CurrAeroBody[2][3]]
            delta_angle_rad  = np.array( tf.euler_from_matrix(H_PrevAeroBody_CurrAeroBody, 'sxyz'))

            # Send the message
            conn.mav.vision_position_delta_send(
                current_time_us,    # us: Timestamp (UNIX time or time since system boot)
                delta_time_us,	    # us: Time since last reported camera frame
                delta_angle_rad,    # float[3] in radian: Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientation
                delta_position_m,   # float[3] in m: Change in position from previous to current frame rotated into body frame (0=forward, 1=right, 2=down)
                current_confidence_level # Normalized confidence value from 0 to 100. 
            )

            # Save static variables
            send_vision_position_delta_message.H_aeroRef_PrevAeroBody = H_aeroRef_aeroBody
            send_vision_position_delta_message.prev_time_us = current_time_us

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
def send_vision_speed_estimate_message():
    global current_time_us, V_aeroRef_aeroBody, reset_counter
    with lock:
        if V_aeroRef_aeroBody is not None:

            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(data.tracker_confidence))
            covariance  = np.array([cov_pose,   0,          0,
                                    0,          cov_pose,   0,
                                    0,          0,          cov_pose])
            
            # Send the message
            conn.mav.vision_speed_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                V_aeroRef_aeroBody[0][3],   # Global X speed
                V_aeroRef_aeroBody[1][3],   # Global Y speed
                V_aeroRef_aeroBody[2][3],   # Global Z speed
                covariance,                 # covariance
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )

# Update the changes of confidence level on GCS and terminal
def update_tracking_confidence_to_gcs():
    if data is not None and update_tracking_confidence_to_gcs.prev_confidence_level != data.tracker_confidence:
        confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
        send_msg_to_gcs(confidence_status_string)
        update_tracking_confidence_to_gcs.prev_confidence_level = data.tracker_confidence

# https://mavlink.io/en/messages/common.html#STATUSTEXT
def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'T265: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
    conn.mav.set_gps_global_origin_send(
        1,
        home_lat, 
        home_lon,
        home_alt
    )

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    conn.mav.set_home_position_send(
        1,
        home_lat, 
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )
    print("set ekf origin successfully!")


# Request a timesync update from the flight controller, for future work.
# TODO: Inspect the usage of timesync_update 
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    conn.mav.timesync_send(
        tc,     # tc1
        ts      # ts1
    )

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Received first ATTITUDE message with heading yaw %.2f degrees" % m.degrees(heading_north_yaw))

#######################################
# Functions - T265
#######################################
def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255:
        reset_counter = 1
    reset_counter += 1

# List of notification events: https://github.com/IntelRealSense/librealsense/blob/development/include/librealsense2/h/rs_types.h
# List of notification API: https://github.com/IntelRealSense/librealsense/blob/development/common/notifications.cpp
def realsense_notification_callback(notif):
    progress("INFO: T265 event: " + notif)
    if notif.get_category() is rs.notification_category.pose_relocalization:
        increment_reset_counter()
        send_msg_to_gcs('Relocalization detected')

def realsense_connect():
    global pipe, pose_sensor, pipe_2, pose_sensor_2

    send_msg_to_gcs('Connecting to camera_1...')
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()
    # Build config object before requesting data
    cfg = rs.config()
    cfg.enable_device('2322110082')
    # Enable the stream we are interested in
    cfg.enable_stream(rs.stream.pose) # Positional data
    # Configure callback for relocalization event
    device = cfg.resolve(pipe).get_device()
    pose_sensor = device.first_pose_sensor()
    pose_sensor.set_notifications_callback(realsense_notification_callback)
    # Start streaming with requested config
    pipe.start(cfg)
    send_msg_to_gcs('Camera_1 connected.')

    send_msg_to_gcs('Connecting to camera_2...')
    pipe_2 = rs.pipeline()
    cfg_2 = rs.config()
    cfg_2.enable_device('2322110308')
    cfg_2.enable_stream(rs.stream.pose)
    device_2 = cfg_2.resolve(pipe_2).get_device()
    pose_sensor_2 = device_2.first_pose_sensor()
    pose_sensor_2.set_notifications_callback(realsense_notification_callback)
    pipe_2.start(cfg_2)
    send_msg_to_gcs('Camera_2 connected.')

def fusion(sensor_1, sensor_2):

    if all_tracker_confidnece is not 0 or None:
        fusion_result = (data.tracker_confidence * sensor_1 + data_2.tracker_confidence * sensor_2) / all_tracker_confidnece
        return fusion_result
    else:
        progress("ERROR:Fusion error OR loss all T265 track ")
        progress("running time:{}".format(time.time() - start_time))

#######################################
# Functions - WCS84 TO XY axis (used in waypoint)
#######################################
def LatLon2XY(latitude, longitude):
    a = 6378137.0
    # b = 6356752.3142
    # c = 6399593.6258
    # alpha = 1 / 298.257223563
    e2 = 0.0066943799013
    # epep = 0.00673949674227
    #将经纬度转换为弧度
    latitude2Rad = (math.pi / 180.0) * latitude
    beltNo = int((longitude + 1.5) / 3.0) #计算3度带投影度带号
    L = beltNo * 3 #计算中央经线
    l0 = longitude - L #经差
    tsin = math.sin(latitude2Rad)
    tcos = math.cos(latitude2Rad)
    t = math.tan(latitude2Rad)
    m = (math.pi / 180.0) * l0 * tcos
    et2 = e2 * pow(tcos, 2)
    et3 = e2 * pow(tsin, 2)
    X = 111132.9558 * latitude - 16038.6496 * math.sin(2 * latitude2Rad) + 16.8607 * math.sin(
        4 * latitude2Rad) - 0.0220 * math.sin(6 * latitude2Rad)
    N = a / math.sqrt(1 - et3)

    x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (
    61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0)
    y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (
    5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0)

    return y, x, L #X,Y轴反了！反了！

def XY2LatLon(X, Y, L0): # L0中央经线

    iPI = 0.0174532925199433
    a = 6378137.0
    f= 0.00335281006247
    ZoneWide = 3 #按3度带进行投影

    ProjNo = int(X / 1000000)
    L0 = L0 * iPI
    X0 = ProjNo * 1000000 + 500000
    Y0 = 0
    xval = X - X0
    yval = Y - Y0

    e2 = 2 * f - f * f #第一偏心率平方
    e1 = (1.0 - math.sqrt(1 - e2)) / (1.0 + math.sqrt(1 - e2))
    ee = e2 / (1 - e2) #第二偏心率平方

    M = yval
    u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256))

    fai = u \
          + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * math.sin(2 * u) \
          + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * math.sin(4 * u) \
          + (151 * e1 * e1 * e1 / 96) * math.sin(6 * u)\
          + (1097 * e1 * e1 * e1 * e1 / 512) * math.sin(8 * u)
    C = ee * math.cos(fai) * math.cos(fai)
    T = math.tan(fai) * math.tan(fai)
    NN = a / math.sqrt(1.0 - e2 * math.sin(fai) * math.sin(fai))
    R = a * (1 - e2) / math.sqrt(
        (1 - e2 * math.sin(fai) * math.sin(fai)) * (1 - e2 * math.sin(fai) * math.sin(fai)) * (1 - e2 * math.sin(fai) * math.sin(fai)))
    D = xval / NN

    #计算经纬度（弧度单位的经纬度）
    longitude1 = L0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (
    5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / math.cos(fai)
    latitude1 = fai - (NN * math.tan(fai) / R) * (
    D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (
    61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720)

    #换换为deg
    longitude = longitude1 / iPI
    latitude = latitude1 / iPI

    return latitude, longitude

def targetXY2LB(X, Y):
    HomeXY = LatLon2XY(home_lat,home_lon)
    targetXY = (HomeXY[0] + X, HomeXY[1] + Y)
    return XY2LatLon(targetXY[0], targetXY[1], targetXY[2])

def relativeLatLon2XY(L1, B1, L2, B2):
    return(math.sqrt((LatLon2XY(L1,B1)[0]-LatLon2XY(L2,B2)[0])**2+(LatLon2XY(L1,B1)[1]-LatLon2XY(L2,B2)[1])**2))

#######################################
# Functions - Miscellaneous
#######################################

# Monitor user input from the terminal and perform action accordingly
def user_input_monitor():
    global scale_factor
    while True:
        # Special case: updating scale
        if scale_calib_enable == True:
            scale_factor = float(input("INFO: Type in new scale as float number\n"))
            progress("INFO: New scale is %s" % scale_factor)

        if enable_auto_set_ekf_home:
            send_msg_to_gcs('Set EKF home with default GPS location')
            set_default_global_origin()
            set_default_home_position()
            time.sleep(5) # Wait a short while for FCU to start working

        # Add new action here according to the key pressed.
        # Enter: Set EKF home when user press enter
        try:
            c = input()
            if c == "":
                send_msg_to_gcs('Set EKF home with default GPS location')
                set_default_global_origin()
                set_default_home_position()
            else:
                progress("Got keyboard input %s" % c)
        except: pass


#######################################
# Main code starts here
#######################################
try:
    progress("INFO: pyrealsense2 version: %s" % str(rs.__version__))
except Exception:
    # fail silently
    pass

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
    retries=20,
)

mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
}

mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

# connecting and configuring the camera is a little hit-and-miss.
# Start a timer and rely on a restart of the script to get it working.
# Configuring the camera appears to block all threads, so we can't do
# this internally.
# send_msg_to_gcs('Setting timer...')
signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
realsense_connected = 0
while realsense_connected == 0:
    try:
        realsense_connect()
        realsense_connected = 1
    except:
        print("ERROR: T265's serial_number is not setted correctly OR not plug all T265")
        time.sleep(1)
signal.setitimer(signal.ITIMER_REAL, 0)  # cancel alarm


# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

if enable_msg_vision_position_estimate:
    sched.add_job(send_vision_position_estimate_message, 'interval', seconds = 1/vision_position_estimate_msg_hz, max_instances=2)

if enable_msg_vision_position_delta:
    sched.add_job(send_vision_position_delta_message, 'interval', seconds = 1/vision_position_delta_msg_hz, max_instances=2)
    send_vision_position_delta_message.H_aeroRef_PrevAeroBody = tf.quaternion_matrix([1,0,0,0]) 
    send_vision_position_delta_message.prev_time_us = int(round(time.time() * 1000000))

if enable_msg_vision_speed_estimate:
    sched.add_job(send_vision_speed_estimate_message, 'interval', seconds = 1/vision_speed_estimate_msg_hz, max_instances=2)

if enable_update_tracking_confidence_to_gcs:
    sched.add_job(update_tracking_confidence_to_gcs, 'interval', seconds = 1/update_tracking_confidence_to_gcs_hz_default)
    update_tracking_confidence_to_gcs.prev_confidence_level = -1

# A separate thread to monitor user input
if enable_user_keyboard_input:
    user_keyboard_input_thread = threading.Thread(target=user_input_monitor)
    user_keyboard_input_thread.daemon = True
    user_keyboard_input_thread.start()
    progress("INFO: Press Enter to set EKF home at default location")

sched.start()

# gracefully terminate the script if an interrupt signal (e.g. ctrl-c)
# is received.  This is considered to be abnormal termination.
main_loop_should_quit = False
def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
signal.signal(signal.SIGINT, sigint_handler)

# gracefully terminate the script if a terminate signal is received
# (e.g. kill -TERM).  
def sigterm_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
    global exit_code
    exit_code = 0

signal.signal(signal.SIGTERM, sigterm_handler)

if compass_enabled == 1:
    time.sleep(1) # Wait a short while for yaw to be correctly initiated

send_msg_to_gcs('Sending vision messages to FCU')

try:
    while not main_loop_should_quit:
        for i in range(30):
            try: 
                frames = pipe.wait_for_frames()
                pose = frames.get_pose_frame()
                frames_2 = pipe_2.wait_for_frames()
                pose_2 = frames_2.get_pose_frame()
                break
            except: 
                time.sleep(0.5)
                continue
        # Process data
        if pose:
            with lock:
                # Store the timestamp for MAVLink messages
                current_time_us = int(round(time.time() * 1000000))
                # Pose data consists of translation and rotation
                data = pose.get_pose_data()
                data_2 = pose_2.get_pose_data()
                # need to do:
                # 位置修正
                # Confidence level value from T265: 0-3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High  
                all_tracker_confidnece = data.tracker_confidence + data_2.tracker_confidence
                current_confidence_level = float(all_tracker_confidnece / 2 * 100 / 3)  

                # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
                if data.tracker_confidence <= 0 and time.time() - start_time >= 1.5:
                    print("[严重问题]前置双目摄像头失效,旋转数据失效")
                    print("两个相机的置信度依次为：{}，{}".format(data.tracker_confidence,data_2.tracker_confidence))            
                else:
                    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
                '''
                H_T265Ref_T265body = tf.quaternion_matrix([ fusion(data.rotation.w, data_2.rotation.w), 
                                                        fusion(data.rotation.x, data_2.rotation.x),
                                                        fusion(data.rotation.y, data_2.rotation.y), 
                                                        fusion(data.rotation.z, data_2.rotation.z)]) 
                '''

                H_T265Ref_T265body[0][3] = fusion(data.translation.x, -data_2.translation.x) * scale_factor
                H_T265Ref_T265body[1][3] = fusion(data.translation.y, data_2.translation.y) * scale_factor
                H_T265Ref_T265body[2][3] = fusion(data.translation.z, -data_2.translation.z) * scale_factor

                # Transform to aeronautic coordinates (body AND reference frame!)
                H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody))

                # Calculate GLOBAL XYZ speed (speed from T265 is already GLOBAL)
                V_aeroRef_aeroBody = tf.quaternion_matrix([1,0,0,0])
                V_aeroRef_aeroBody[0][3] = fusion(data.velocity.x, -data_2.velocity.x)
                V_aeroRef_aeroBody[1][3] = fusion(data.velocity.y, data_2.velocity.y)
                V_aeroRef_aeroBody[2][3] = fusion(data.velocity.z, -data_2.velocity.z)
                V_aeroRef_aeroBody = H_aeroRef_T265Ref.dot(V_aeroRef_aeroBody)

                # Check for pose jump and increment reset_counter
                if prev_data != None:
                    delta_translation = [   fusion(data.translation.x, -data_2.translation.x) - fusion(prev_data.translation.x, -prev_data_2.translation.x), 
                                            fusion(data.translation.y, data_2.translation.y) - fusion(prev_data.translation.y, prev_data_2.translation.y), 
                                            fusion(data.translation.z, -data_2.translation.z) - fusion(prev_data.translation.z, -prev_data_2.translation.z)]
                    delta_velocity = [  fusion(data.velocity.x, -data_2.velocity.x) - fusion(prev_data.velocity.x, -prev_data_2.velocity.x), 
                                        fusion(data.velocity.y, data_2.velocity.y) - fusion(prev_data.velocity.y, prev_data_2.velocity.y), 
                                        fusion(data.velocity.z, -data_2.velocity.z) - fusion(prev_data.velocity.z, -prev_data_2.velocity.z)]
                    position_displacement = np.linalg.norm(delta_translation)
                    speed_delta = np.linalg.norm(delta_velocity)

                    # Pose jump is indicated when position changes abruptly. The behavior is not well documented yet (as of librealsense 2.34.0)
                    jump_threshold = 0.1 # in meters, from trials and errors, should be relative to how frequent is the position data obtained (200Hz for the T265)
                    jump_speed_threshold = 20.0 # in m/s from trials and errors, should be relative to how frequent is the velocity data obtained (200Hz for the T265)
                    if (position_displacement > jump_threshold) or (speed_delta > jump_speed_threshold):
                        send_msg_to_gcs('VISO jump detected')
                        if position_displacement > jump_threshold:
                            progress("Position jumped by: %s" % position_displacement)
                        elif speed_delta > jump_speed_threshold:
                            progress("Speed jumped by: %s" % speed_delta)
                        increment_reset_counter()
                    
                prev_data = data
                prev_data_2 = data_2

                # Take offsets from body's center of gravity (or IMU) to camera's origin into account
                if body_offset_enabled == 1:
                    H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                    H_body_camera[0][3] = body_offset_x
                    H_body_camera[1][3] = body_offset_y
                    H_body_camera[2][3] = body_offset_z
                    H_camera_body = np.linalg.inv(H_body_camera)
                    H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

                # Realign heading to face north using initial compass data
                if compass_enabled == 1:
                    H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz'))

                # Show debug messages here
                if debug_enable == 1:
                    os.system('clear') # This helps in displaying the messages to be more readable
                    progress("DEBUG: Raw RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_T265Ref_T265body, 'sxyz')) * 180 / m.pi))
                    progress("DEBUG: NED RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
                    progress("DEBUG: Raw pos xyz : {}".format( np.array( [data.translation.x, data.translation.y, data.translation.z])))
                    progress("DEBUG: Raw pos xyz : {}".format( np.array( [data_2.translation.x, data_2.translation.y, data_2.translation.z])))
                    progress("DEBUG: two device tracker:{}".format(np.array([data.tracker_confidence, data_2.tracker_confidence])))
                    progress("DEBUG: NED pos xyz : {}".format( np.array( tf.translation_from_matrix(H_aeroRef_aeroBody))))
                    progress("DEBUG: velocity{}".format(np.array([V_aeroRef_aeroBody[0][3], V_aeroRef_aeroBody[1][3], V_aeroRef_aeroBody[1][3]])))
except Exception as e:
    progress(e)

except:
    send_msg_to_gcs('ERROR IN SCRIPT')  
    progress("Unexpected error: %s" % sys.exc_info()[0])

finally:
    progress('Closing the script...')
    # start a timer in case stopping everything nicely doesn't work.
    signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
    pipe.stop()
    pipe_2.stop()
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    progress("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit(exit_code)
