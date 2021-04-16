from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK10"] = "1"
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
time.sleep(3)
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

@vehicle.on_message('RANGEFINDER')
def listener(self, name, message):
    global rangefinder_dis_land
    rangefinder_dis_land = message.distance - 0.12
    #rangefinder_dis_land = round(message.distance,3) - 0.12
    print ('distance: %s' % message.distance)

'''
#明天需要修�?
@vehicle.on_message('RC_CHANNELS')
def listener(self, name, message):
    global chan8
    chan8 = message.chan8_raw
    #rangefinder_dis_land = round(message.distance,3) - 0.12
    print ('chan8: %s' % message.chan8_raw)
'''

@vehicle.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):
    global rc_channel_value, rc_control_channel, curr_channels_values
    
    # TO-DO: find a less hard-coded solution
    curr_channels_values = [message.chan1_raw, message.chan2_raw, message.chan3_raw, message.chan4_raw, message.chan5_raw, message.chan6_raw, message.chan7_raw, message.chan8_raw]

    #rc_channel_value = curr_channels_values[rc_control_channel]

    print(curr_channels_values)
    # # Print out the values to debug
    # print('%s attribute is: %s' % (name, message)) # Print all info from the messages
    # os.system('clear') # This helps in displaying the messages to be more readable
    # for channel in range(8):
    #     print("Number of RC channels: ", message.chancount, ". Individual RC channel value:")
    #     print(" CH", channel, curr_channels_values[channel])

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

def do_set_servo(servo_number,pwm):
    msg = vehicle.message_factory.command_long_encode(
        0,0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        servo_number,
        pwm,
        0,
        0,
        0,
        0,
        0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print("succeess set servo")

#vehicle.mode = VehicleMode ("MANUAL")
time.sleep(5)
print (" Ch8: %s" % vehicle.channels['8'])
vehicle.channels.overrides['8'] = 1130
time.sleep(2)
print (" Ch8: %s" % vehicle.channels['8'])

vehicle.mode = VehicleMode("GUIDED")
#vehicle.armed = True
time.sleep(3)
vehicle.simple_takeoff(0.5)
# while 
vehicle.mode = VehicleMode("AUTO")
# vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)
do_set_servo(7,983)
vehicle.mode = VehicleMode("LAND")

print("success!")

while(True):
    vehicle.channels.overrides={'8':1130}
    #rangefinder_dis_land = round(vehicle.rangefinder,3) - 0.12
    time.sleep(0.5)
    print(rangefinder_dis_land)
    #send_land_message()



