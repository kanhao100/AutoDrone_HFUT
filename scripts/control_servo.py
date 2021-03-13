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

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


vehicle.channels.overrides['8'] = 1130

when True:
    rangefinder_dis_land = round(Vehicle.rangefinder,3) - 0.12
    time.sleep(1)
    print(rangefinder_dis_land)