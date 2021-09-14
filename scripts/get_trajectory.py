import sys
sys.path.append("/usr/local/lib/")
import os
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil, mavwp
import threading
import math as m 
import time
import argparse

enable_test_in_windows = False


if enable_test_in_windows: 
    connection_string = 'COM22' #填写COM口，使用设备管理器查看即可
else: 
    connection_string = '/dev/ttyACM0'
connection_baudrate = 115200
connection_timeout_sec = 5
heading_north_yaw = None

parser = argparse.ArgumentParser(
    description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
if args.connect is not None:
    connection_string = args.connect

def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

# lock for thread synchronization
lock = threading.Lock()
mavlink_thread_should_exit = False

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

def att_msg_callback(value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Received first ATTITUDE message with heading yaw %.2f degrees" % m.degrees(heading_north_yaw))

print("INFO: Starting Vehicle communications")
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

while True: 
    mavtlog = conn.recv_match(type=['ATTITUDE'])
    print (mavtlog.get_type())
    print (mavtlog.get_type())
    time.sleep(1)