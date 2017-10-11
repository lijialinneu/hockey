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


    file = 'data.txt'

    def __init__(self, cap, puck, robot, serial, time, flag=True):        
        self.cap = cap
        self.puck = puck
        self.robot = robot
        self.serial = serial
        self.start_time = time
        self.test_flag = flag

    
    def send_message(self, queue):
        while True:
            message = queue.get(True)   
            if message:
                # print('get message = ', message)
                self.serial.write(message) # send data
                self.serial.flushInput() # flush input
    

    def solution_core(self, queue):
        while self.cap.isOpened():            
            ret, frame = self.cap.read()            
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)            
            pos_p = self.detect_puck(hue_image, frame)
            pos_r = self.detect_robot(hue_image, frame)

            if pos_p and pos_r:
                #print('put message')
                queue.put(self.create_message())
                    
            if self.test_flag:
                cv2.circle(frame, (160, 120), 2, (0, 0, 255), 2)
                cv2.imshow('frame', frame) # show trace windows
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()


    def detect_puck(self, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.puck.th_hsv_low, self.puck.th_hsv_high)
        # cv2.imshow('puck_th', th_img)
        # ping-pong
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=6, maxRadius=8)

        # green-puck
        '''
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=5, maxRadius=15)
        '''
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
        # cv2.imshow('robot_th', th_img)
        # dst = cv2.dilate(th_img, cv2.getStructuringElement(0, (3,3)))
        # blue robot
        '''
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                     param1=15, param2=7, minRadius=12, maxRadius=15)
        '''

        # blue-paper-robot
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                   param1=15, param2=7, minRadius=16, maxRadius=19)

        '''
        contours, hierarchy = cv2.findContours(
                th_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours :
            (x, y, w, h) = cv2.boundingRect(contours[0])
            center_x = x + w / 2
            center_y = y + h / 2
            self.puck.update_position(center_x, center_y)
            if self.test_flag:                
                cv2.circle(frame, (center_x, center_y), 1, (0, 255, 0), 2)
            return x, y, w, h
        '''
        # yellow-robot
        '''
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                  param1=15, param2=7, minRadius=5, maxRadius=10)        
        '''
        
        if circles is not None:
            x, y, r = circles[0][0]
            # print('r = %d ' % r)
            center = (x, y)                
            self.robot.update_position(x, y)
            if self.test_flag:                
                cv2.circle(frame, (x, y), 1, (0, 255, 0), 2)
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
        p_x = self.puck.lastX << 1
        p_y = self.puck.lastY << 1
        r_x = self.robot.lastX << 1
        r_y = self.robot.lastY << 1

        if p_x in range(0, 640) and p_y in range(0, 480) :

            # save to data.txt
            # p_x p_y r_x r_y
            data_list = ['']
            data_list.append(str(p_x) + ' ')
            data_list.append(str(p_y) + ' ')
            data_list.append(str(r_x) + ' ')
            data_list.append(str(r_y) + ' ')
            data_list.append(str(time) + ' ')
            data = ''.join(data_list)
            self.write_file(data)
            

            # send to arduino
            puck_x_h, puck_x_l = self.high_and_low(p_x)
            puck_y_h, puck_y_l = self.high_and_low(p_y)
            area_h, area_l = self.high_and_low(int(self.puck.area))
            robot_x_h, robot_x_l = self.high_and_low(r_x)
            robot_y_h, robot_y_l = self.high_and_low(r_y)
           
            list = ['mm']
            list.append(chr(time_h))
            list.append(chr(time_l))
            list.append(chr(puck_x_h))
            list.append(chr(puck_x_l))
            list.append(chr(puck_y_h))
            list.append(chr(puck_y_l))
            list.append(chr(area_h))
            list.append(chr(area_l))
            list.append(chr(robot_x_h))
            list.append(chr(robot_x_l))
            list.append(chr(robot_y_h))
            list.append(chr(robot_y_l))

            message = ''.join(list)
        return message


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


    def write_file(self, data):
        with open(self.file, 'a') as f:
            f.write(data + '\n')
        f.close()
