# !/usr/bin/env python
# -*- coding: utf-8 -*-


"""
The core solution rogram
Author: lijialin 1040591521@qq.com
Date: July 2017

Variables:
    self.cap:          cv2.VideoCapture object
    self.puck:         puck 
    self.robot:        robot
    self.serial:       serial(arduino)
    self.start_time:   the start clock of run.py 
    self.test_flag:    if True then show the trace window 

Functions:
    solution_core():   the core program of the solution
    detect_puck():     detect puck
    detect_robot():    detect robot
    create_message():  create messages sent to arduino
    high_and_low():    return high eight bit and low eight bit
    release_windows(): release the trace windows

"""


import numpy as np
import cv
import time
from TableObjects import *
from time import clock, ctime


class Solution:


    def __init__(self, cap, puck, robot, serial, time, flag=True):        
        self.cap = cap
        self.puck = puck
        self.robot = robot
        self.serial = serial
        self.start_time = time
        self.test_flag = flag
        #self.count = 0
        #print(ctime())

    def prepare(self):
        while self.cap.isOpened():            
            ret, frame = self.cap.read()            
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # print(hue_image.shape[0], ' ', hue_image.shape[1]) # 240 320

            time = clock() * 1000 - self.start_time * 1000 
            time_h, time_l = self.high_and_low(long(time))

            cv2.circle(frame, (320, 20), 2, (0, 255, 0), 2)
            cv2.circle(frame, (160, 120), 2, (0, 255, 0), 2)
            cv2.circle(frame, (320, 220), 2, (0, 255, 0), 2)
            cv2.circle(frame, (200, 150), 2, (255, 0, 0), 2)
            cv2.circle(frame, (310, 10), 2, (255, 0, 0), 2)
            cv2.circle(frame, (298, 45), 2, (255, 0, 0), 2)
            cv2.circle(frame, (298, 199), 2, (255, 0, 0), 2)

            pos_r = self.detect_robot(hue_image, frame)
            r_x = 2 * self.robot.lastX
            r_y = 2 * self.robot.lastY

            print(r_x, r_y)
        
            '''
            puck_x_h, puck_x_l = self.high_and_low(320)
            puck_y_h, puck_y_l = self.high_and_low(300)
            area_h, area_l = self.high_and_low(150)
            robot_x_h, robot_x_l = self.high_and_low(r_x)
            robot_y_h, robot_y_l = self.high_and_low(r_y)
            '''

            '''
            print('mm', time_h, time_l, puck_x_h, puck_x_l,
                  puck_y_h, puck_y_l, area_h, area_l,
                  robot_x_h, robot_x_l, robot_y_h, robot_y_l)
            
            
            message = 'mm' + chr(time_h) + chr(time_l) \
                           + chr(puck_x_h) + chr(puck_x_l) \
                           + chr(puck_y_h) + chr(puck_y_l) \
                           + chr(area_h) + chr(area_l) \
                           + chr(robot_x_h) + chr(robot_x_l) \
                           + chr(robot_y_h) + chr(robot_y_l)
            self.serial.write(message) # send data
            self.serial.flushInput() # flush input
            '''
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  
        self.release_window()

    
    def send_message(self, queue):
        while True:
            message = queue.get(True)   
            if message:
                #self.count += 1
                #if self.count == 200:
                    #break
                #print('get message = ', message)
                self.serial.write(message) # send data
                self.serial.flushInput() # flush input
        #print(ctime())
    

    def solution_core(self, queue):
        while self.cap.isOpened():            
            ret, frame = self.cap.read()            
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            pos_p = self.detect_puck(hue_image, frame)
            pos_r = self.detect_robot(hue_image, frame)

            if pos_p :
                #print('put message')
                queue.put(self.create_message())
                    
            if self.test_flag:
                cv2.imshow('frame', frame) # show trace windows
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()


    def detect_puck(self, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.puck.th_hsv_low, self.puck.th_hsv_high)
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=6, maxRadius=8)
        if circles is not None:
            x, y, r = circles[0][0]
            center = (x, y)
            self.puck.area = 3.14 * r * r
            self.puck.update_position(x, y)

            if self.test_flag:                
                cv2.circle(frame, (x, y), 1, (0, 255, 0), 2)
            return x, y, r
        return None


    def detect_robot(self, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.robot.th_hsv_low, self.robot.th_hsv_high)
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=12, maxRadius=15)
        if circles is not None:
            x, y, r = circles[0][0]
            center = (x, y)                
            self.robot.update_position(x, y)

            if self.test_flag:                
                cv2.circle(frame, (x, y), 1, (0, 0, 255), 2)
            return x, y, r
        return None
    

    def create_message(self):
        """
        Serial packet:
            Sync start: 2 bytes: mm
            TimeStamp: 2 bytes
            Pos_X:     2 bytes (0-640)
            Pos_Y:     2 bytes (0-480)
            Object size:     2 bytes (0-400)
            Robot Pos_X:     2 bytes (0-640)
            Robot Pos_Y:     2 bytes (0-480)
        """
        message = ''

        # millisecond
        time = clock() * 1000 - self.start_time * 1000 
        time_h, time_l = self.high_and_low(long(time))

        # coordinate x and y
        p_x = 2 * self.puck.lastX
        p_y = 2 * self.puck.lastY
        r_x = 2 * self.robot.lastX
        r_y = 2 * self.robot.lastY

        if p_x in range(0, 640) and p_y in range(0, 480) :
            
            puck_x_h, puck_x_l = self.high_and_low(p_x)
            puck_y_h, puck_y_l = self.high_and_low(p_y)
            area_h, area_l = self.high_and_low(int(self.puck.area))
            robot_x_h, robot_x_l = self.high_and_low(r_x)
            robot_y_h, robot_y_l = self.high_and_low(r_y)

            '''
            print('mm', time_h, time_l, puck_x_h, puck_x_l,
                  puck_y_h, puck_y_l, area_h, area_l,
                  robot_x_h, robot_x_l, robot_y_h, robot_y_l)
            '''
            
            # TODO use join
            message = 'mm' + chr(time_h) + chr(time_l) \
                           + chr(puck_x_h) + chr(puck_x_l) \
                           + chr(puck_y_h) + chr(puck_y_l) \
                           + chr(area_h) + chr(area_l) \
                           + chr(robot_x_h) + chr(robot_x_l) \
                           + chr(robot_y_h) + chr(robot_y_l)
        return message


    # not used
    """
    def meet_condition(self, my_object, contour):
        area = my_object.cal_area(contour)
        # print('area = ', area)
        if area in range(my_object.th_area[0], my_object.th_area[1]):
            my_object.cal_perimeter(contour)
            my_object.cal_roundness()
            return my_object.meet_roundness()
        return False    


    def if_move(self, my_object, move):
        return move and \
               (my_object.deltaX > self.move_th or \
                my_object.deltaY > self.move_th)
    """

    def high_and_low(self, x):
        return (x >> 8) & 255, x & 255


    def release_window(self):
        self.cap.release()
        cv2.destroyAllWindows()

