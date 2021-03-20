# 测试项目为起飞0.5m,悬停5秒然后降落
# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

######################################################
##  Sending control commands to AP via MAVLink      ##
##  Based on set_attitude_target.py: https://github.com/dronekit/dronekit-python/blob/master/examples/set_attitude_target/set_attitude_target.py
######################################################

## Additional installation for SITL:
##      pip3 install dronekit-sitl -UI

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
import sys
import control_function

#######################################
# Parameters
#######################################

rc_control_channel = 6     # Channel to check value, start at 0 == chan1_raw
rc_control_thres = 2000    # Values to check

#######################################
# Global variables
#######################################

rc_channel_value = 0
vehicle = None

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

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

@vehicle.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):
    global rc_channel_value, rc_control_channel
    
    # TO-DO: find a less hard-coded solution
    curr_channels_values = [message.chan1_raw, message.chan2_raw, message.chan3_raw, message.chan4_raw, message.chan5_raw, message.chan6_raw, message.chan7_raw, message.chan8_raw]

    rc_channel_value = curr_channels_values[rc_control_channel]

    # # Print out the values to debug
    # print('%s attribute is: %s' % (name, message)) # Print all info from the messages
    # os.system('clear') # This helps in displaying the messages to be more readable
    # for channel in range(8):
    #     print("Number of RC channels: ", message.chancount, ". Individual RC channel value:")
    #     print(" CH", channel, curr_channels_values[channel])

#######################################
# Main program starts here
#######################################

try:
    # If using SITL: Take off in GUIDED_NOGPS mode.
    while(True):
        if (rc_channel_value > rc_control_thres):
            arm_and_takeoff_nogps(0.5)
            print("Hold position for 3 seconds")
            set_attitude(duration = 5)
            break
        else:
            print("Checking rc channel:", rc_control_channel, ", current value:", rc_channel_value, ", threshold to start: ", rc_control_thres)
            time.sleep(1)

    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)
    vehicle.armed = False
    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    print("Completed")

except KeyboardInterrupt:
    vehicle.close()
    print("Vehicle object closed.")
    sys.exit()
