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
from multiprocessing import Process, Queue


if __name__ == "__main__":

    
    # Step1. init video and serial
    # ReadVideoSingleton is a singleton class used to capture frames
    # Arguments 0 is camera index
    # get_capture() return the cv2.VideoCapture object
    cap = ReadVideoSingleton(0).get_capture()
    start_time = clock() # start to clock

    serial = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
    # serial = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)
    # serial = None


    # Step2. init puck and robot
    # puck's arguments
    puck_position = [0, 0]

    # ping-pong
    # puck_th_hsv_low = np.array([1, 112, 144])
    # puck_th_hsv_high = np.array([28, 255, 255])

    # green-puck
    # puck_th_hsv_low = np.array([30, 110, 39])
    # puck_th_hsv_high = np.array([74, 199, 143])

    # red-puck
    puck_th_hsv_low = np.array([0, 196, 62])
    puck_th_hsv_high = np.array([10, 255, 188])

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

    # blue-robot
    # robot_th_hsv_low = np.array([100, 176, 0])
    # robot_th_hsv_high = np.array([139, 255, 111])

    # yellow-robot
    robot_th_hsv_low = np.array([20, 186, 126])
    robot_th_hsv_high = np.array([255, 255, 255])


    robot_th_area = [200, 600]
    robot_th_roundness = 20
    robot_delta = 10
    robot = TableObjects(robot_position,
                         robot_th_hsv_low,
                         robot_th_hsv_high,
                         robot_th_area,
                         robot_th_roundness,
                         robot_delta)

    # Step3. create solution
    # True : display trace window
    solution = Solution(cap, puck, robot, serial, start_time, True)


    # Step4. start process
    queue = Queue()
    image_process = Process(target=solution.solution_core, args=(queue,))
    message_process = Process(target=solution.send_message, args=(queue,))
    image_process.start()
    message_process.start()
    image_process.join()
    message_process.terminate()
        
    # finish
    print('finish')
