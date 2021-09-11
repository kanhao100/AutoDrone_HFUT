#   pip3 install MAVProxy
import os
import threading

### thread2 ###
enable_t265_old = False #只使用一个双目相机
enable_t265_low_rate = False

### thread3 ###
enable_land = False

### thread4 ###
enable_control = False #危险操作,无开发人员在场请勿修改
enable_servo = False #可能存在危险,拆桨测试
enable_auto_arm = False

### thread5 ###
enable_get_tarjectory = True

connection_in_port = "/dev/ttyACM0"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14550"      # T265
connection_out_p02 = "127.0.0.1:14560"      # 
connection_out_p03 = "127.0.0.1:14570"      # Control (GUIDED)
connection_out_p04 = "127.0.0.1:14580" 
# connection_out_listen = '192.168.137.1:14580'

def mavproxy_create_connection():
    os.system("mavproxy.py" + \
            " --master="   + connection_in_port + \
            " --baudrate=" + connection_in_baud + \
            " --out udp:"  + connection_out_p01 + \
            " --out udp:"  + connection_out_p02 + \
            " --out udp:"  + connection_out_p03 + \
            " --out udp:"  + connection_out_p04)

def run_t265():
    os.system("python3 t265_to_mavlink.py --connect=" + connection_out_p01)

def run_t265_low_rate():
    os.system("python3 t265_to_mavlink_lowrate.py --connect=" + connection_out_p01)

def run_t265_old():
    os.system("python3 old_t265_to_mavlink.py --connect=" + connection_out_p01)

def run_landing():
    os.system("python3 land.py --connect=" + connection_out_p02)

def run_control():
    os.system("python3 mavlink_control_test4.py --connect=" + connection_out_p03)

def run_control_servo():
    os.system("python3 control_servo.py --connect=" + connection_out_p03)

def auto_arm():
    os.system("python3 auto_arm.py --connect=" + connection_out_p03)

def get_trajectory(): 
    os.system("python3 get_trajectory.py --connect=" + connection_out_p04)

thread1 = threading.Thread(target=mavproxy_create_connection)
thread1.start()

if not enable_t265_old:
    if enable_t265_low_rate:
        thread2 = threading.Thread(target=run_t265_low_rate)
        thread2.start()
    else:
        thread2 = threading.Thread(target=run_t265)
        thread2.start()
else:
    thread2 = threading.Thread(target=run_t265_old)
    thread2.start()

if enable_land:
    thread3 = threading.Thread(target=run_landing)
    thread3.start()

if enable_control:
    thread4 = threading.Thread(target=run_control)
    thread4.start()

if enable_servo:
    thread4 = threading.Thread(target=run_control_servo)
    thread4.start()

if enable_auto_arm:
    thread4 = threading.Thread(target=auto_arm)
    thread4.start()

if enable_get_tarjectory: 
    thread5 = threading.Thread(target=get_trajectory)
    thread5.start()