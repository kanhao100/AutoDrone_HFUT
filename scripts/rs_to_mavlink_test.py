#!/usr/bin/env python3

# Install required packages: 
#   pip3 install MAVProxy

import os
import threading

connection_in_port = "/dev/ttyACM0"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14550"      # T265
connection_out_p02 = "127.0.0.1:14560"      # 
connection_out_p03 = "127.0.0.1:14570"      # Control (GUIDED)

def mavproxy_create_connection():
    os.system("mavproxy.py" + \
            " --master="   + connection_in_port + \
            " --baudrate=" + connection_in_baud + \
            " --out udp:"  + connection_out_p01 + \
            " --out udp:"  + connection_out_p02 + \
            " --out udp:"  + connection_out_p03)

def run_t265():
    os.system("python3 t265_to_mavlink.py --connect=" + connection_out_p01)

def run_landing():
    os.system("python3 land.py --connect=" + connection_out_p02)

def run_control():
    os.system("python3 mavlink_control.py --connect=" + connection_out_p03)

def run_control_servo():
    os.system("python3 control_servo.py --connect+" + connection_out_p03)

thread1 = threading.Thread(target=mavproxy_create_connection)
thread1.start()

thread2 = threading.Thread(target=run_t265)
thread2.start()

#thread3 = threading.Thread(target=run_landing)
#thread3.start()

thread4 = threading.Thread(target=run_control_servo)
thread4.start()

# thread4 = threading.Thread(target=run_control)
# thread4.start()