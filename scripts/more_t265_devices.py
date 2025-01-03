#!/usr/bin/env python3
# -*- coding: utf-8 -*-
__author__ = "Ivo Marvan"
__email__ = "ivo@marvan.cz"
__description__ = '''
    Experiments with multiple T265 cameras.
    hfut_autoUAV修改，在t265_to_mavlink.py中import
'''
import pyrealsense2 as rs
import datetime
import time
import logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(levelname)s] %(asctime)s.%(msecs)03d: (%(threadName)-9s) %(message)s',
    datefmt="%H:%M:%S"
)

def get_devices_serial_numbers(device_suffix:str='T265') -> [str]:
    '''
    Return list of serial numbers conected devices.
    Eventualy only fit given suffix (like T265, D415, ...)
    (based on https://github.com/IntelRealSense/librealsense/issues/2332)
    '''
    ret_list = []
    ctx = rs.context()
    for d in ctx.devices:
        if device_suffix and not d.get_info(rs.camera_info.name).endswith(device_suffix):
            continue
        ret_list.append(d.get_info(rs.camera_info.serial_number))
    return ret_list


class T265CameraSource:

    def __init__(self, serial_number:str):
        self.__serial_number = serial_number
        self.__pipeline = None
        self.__config = None
        self.__start_pipeline()


    def __del__(self):
        if not self.__pipeline is None:
            self.__pipeline.stop()
            
    def get_serial_number(self):
        return self.__serial_number

    def __start_pipeline(self):
        # Configure depth and color streams
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_device(self.__serial_number)
        self.__config.enable_stream(rs.stream.pose)
        self.__pipeline.start(self.__config)
        # logging.debug('T265 ({}) camera is ready.'.format(self.__serial_number))
    
    def __restart_pipeline(self):
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_device(self.__serial_number)
        self.__config.enable_stream(rs.stream.pose)
        self.__pipeline.start(self.__config)
        # logging.debug('Tracking is lossing more than 5s, T265 ({}) camera was resarted.'.format(self.__serial_number))


    def get(self) -> rs.pose:
        try:
            frames = self.__pipeline.wait_for_frames()
            #return frames.get_pose_frame()
            data = frames.get_pose_frame()
            return data.get_pose_data()
        except:
            self.__restart_pipeline()
            return 'restared'

    def get_xyz(self) -> (float, float, float):
        data = self.get()
        return round(data.translation.x,3), round(data.translation.y,3), round(data.translation.z,3           ),
    def get_rotation(self) -> (float, float, float):
        data = self.get()
        return data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z

if __name__ == "__main__":
    number_of_experiments = 99999
    serial_numbers =  get_devices_serial_numbers()
    sources = [T265CameraSource(serial_number) for serial_number in serial_numbers]

    print('Serial experiment', '-' * 50)
    print('serial_numbers', serial_numbers)
    '''
    for camera_index, source in enumerate(sources):
        for experiment_index in range(number_of_experiments):
            print(experiment_index, camera_index,  source.get_serial_number(), source.get_xyz(), datetime.datetime.now())
    '''
    print('Parallel experiment', '-' * 50)
    for experiment_index in range(number_of_experiments):
        time.sleep(1)
        print()
        for camera_index, source in enumerate(sources):
            print(experiment_index, camera_index, source.get_serial_number(), source.get_rotation(), datetime.datetime.now())




