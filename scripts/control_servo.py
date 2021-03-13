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

import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

@vehicle.on_message('RANGEFINDER')
def listener(self, name, message):
    rangefinder_dis_land = message.distance - 0.12
    #rangefinder_dis_land = round(message.distance,3) - 0.12
    print 'distance: %s' % message.distance

#明天需要修改
@vehicle.on_message('RC_CHANNELS')
def listener(self, name, message):
    rangefinder_dis_land = message.distance - 0.12
    #rangefinder_dis_land = round(message.distance,3) - 0.12
    print 'distance: %s' % message.distance


def send_land_message():
    msg = vehicle.message_factory.landing_target_encode(
        0,                       # time target data was processed, as close to sensor capture as possible
        0,                                  # target num, not used
        0, 
        0,                       # X-axis angular offset, in radians
        0,                       # Y-axis angular offset, in radians
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

def do_set_servo(servo_number,pwm)：
    msg = vehicle.message_factory.mav_cmd_do_set_servo_encode(
        servo_number,
        pwm,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send.mavlink(msg)
    vehicle.flush()


print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

time.sleep(10)
#print (" Ch8: %s" % vehicle.channels['8'])
#vehicle.channels.overrides['8'] = 1130
#print (" Ch8: %s" % vehicle.channels['8'])
do_set_servo(8,1130)

while(True):
    #rangefinder_dis_land = round(vehicle.rangefinder,3) - 0.12
    time.sleep(0.5)
    print(rangefinder_dis_land)
    send_land_message()