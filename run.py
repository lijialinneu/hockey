# !/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Motion detection module
Author: lijialin 1040591521@qq.com
Date: July 2017

Algorithm:
    Image thresholding segmentation
    Contour extraction
    Hough circle

Solution:
    Step1. Init camera and serial
    Step2. Init puck object and robot object
    Step3. Start solution

The core program in the Solution.py
"""


import serial
from Solution import *
from ReadVideoSingleton import *


if __name__ == "__main__":

    
    # Step1. init video and serial
    # ReadVideoSingleton is a singleton class used to capture frames
    # Arguments 0 is camera index
    # get_capture() return the cv2.VideoCapture object
    cap = ReadVideoSingleton(0).get_capture()
    start_time = clock() # start to clock
    serial = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

    # Step2. init puck and robot
    # We can change arguments here
    # puck's arguments
    puck_position = [0, 0]
    '''
    puck_th_hsv_low = np.array([50, 120, 46])
    puck_th_hsv_high = np.array([60, 255, 255])
    '''
    puck_th_hsv_low = np.array([0, 20, 120])
    puck_th_hsv_high = np.array([31, 126, 255])
    
    # puck_th_area = [150, 200]
    puck_th_area = [100, 200]
    puck_th_roundness = 8
    puck_delta = 10
    puck = TableObjects(puck_position,
                        puck_th_hsv_low,
                        puck_th_hsv_high,
                        puck_th_area,
                        puck_th_roundness,
                        puck_delta)

    # robot's arguments
    robot_position = [0, 0]
    robot_th_hsv_low = np.array([110, 120, 46])
    robot_th_hsv_high = np.array([124, 255, 255])
    robot_th_area = [200, 600]
    robot_th_roundness = 20
    robot_delta = 10
    robot = TableObjects(robot_position,
                         robot_th_hsv_low,
                         robot_th_hsv_high,
                         robot_th_area,
                         robot_th_roundness,
                         robot_delta)

    # Step3. start solution
    # move_th is the threshold of motion distance
    # True : display trace window
    move_th = 0
    solution = Solution(cap, puck, robot,
                        serial, start_time, move_th, True)
    solution.solution_core()

    # finish
    print('finish')
