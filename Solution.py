import numpy as np
import cv

from TableObjects import *
from SerialSingleton import *
from time import clock

class Solution:

    def __init__(self, cap, puck, robot, serial, time, flag=True):        
        self.cap = cap
        self.puck = puck
        self.robot = robot
        self.serial = serial
        self.start_time = time
        self.test_flag = flag

    def solution_core(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            result_p = self.detect(self.puck, hue_image, frame)
            result_r = self.detect(self.robot, hue_image, frame)
            # TODO write serial
            message = self.create_message()
            self.serial.send_message(message)
            if self.test_flag:
                if result_p:
                    x, y, w, h = result_p
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if result_r:
                    x, y, w, h = result_r
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()

    def detect(self, my_object, hue_image, frame):
        th_img = cv2.inRange(hue_image, my_object.th_hsv_low, my_object.th_hsv_high)
        dilated = cv2.dilate(th_img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
        contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours and self.meet_condition(my_object, contours[0]):
            (x, y, w, h) = cv2.boundingRect(contours[0])
            my_object.update_position(x, y)
            return x, y, w, h
        return None

    def meet_condition(self, my_object, contour):
        area = my_object.cal_area(contour)
        if area in range(my_object.th_area[0], my_object.th_area[1]):
            my_object.cal_perimeter(contour)
            my_object.cal_roundness()
            return my_object.meet_roundness()
        return False    

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
        time = clock() * 1000 - self.start_time * 1000 # change to millisecond
        time_h, time_l = self.high_and_low(time)
        puck_x_h, puck_x_l = self.high_and_low(self.puck.lastX)
        puck_y_h, puck_y_l = self.high_and_low(self.puck.lastY)
        
        robot_x_h, robot_x_l = self.high_and_low(self.robot.lastX)
        robot_y_h, robot_y_l = self.high_and_low(self.robot.lastY)
        
        message = '\x6d\x6d'
        return "asdf hahaha lalala hehehe"

    def high_and_low(self, x):
        return (x >> 8) & 255, x & 255

    def release_window(self):
        self.cap.release()
        cv2.destroyAllWindows()

