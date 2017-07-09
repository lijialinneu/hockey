import numpy as np
import cv

from TableObjects import *
from SerialSingleton import *

class Solution:

    def __init__(self, cap, puck, robot, flag=True):
        self.test_flag = flag
        self.cap = cap
        self.puck = puck
        self.robot = robot

    def detect(self, my_object, hue_image, frame):
        th_img = cv2.inRange(hue_image, my_object.th_hsv_low, my_object.th_hsv_high)
        dilated = cv2.dilate(th_img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
        contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours and self.meet_condition(my_object, contours[0]):
            (x, y, w, h) = cv2.boundingRect(contours[0])
            my_object.update_position(x, y)
            if self.test_flag:
                if my_object is self.puck:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                elif my_object is self.robot:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return frame

    def solution_core(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            self.detect(self.puck, hue_image, frame)
            self.detect(self.robot, hue_image, frame)
            if self.test_flag:
                cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()

    def release_window(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def meet_condition(self, my_object, contour):
        area = my_object.cal_area(contour)
        print('area = ', area)
        if area in range(my_object.th_area[0], my_object.th_area[1]):
            my_object.cal_perimeter(contour)
            my_object.cal_roundness()
            return my_object.meet_roundness()
        return False    
