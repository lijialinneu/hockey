import numpy as np
import serial
import cv

from TableObjects import *


class Solution:

    def __init__(self, cap, puck, robot, flag=True):
        self.test_flag = flag
        self.cap = cap
        self.puck = puck
        self.robot = robot

    def detect(self, my_object, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.puck.th_hsv_low, self.puck.th_hsv_high)
        dilated = cv2.dilate(th_img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                   param1=15, param2=7, minRadius=10, maxRadius=20)
        if circles is not None:
            x, y, radius = circles[0][0]
            center = (x, y)
            if self.meet_condition_circle(my_object, x, y, radius):
                my_object.update_position(x, y)
                my_object.make_serial()

                if self.test_flag:
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    end_x = my_object.lastX + my_object.deltaX
                    end_y = my_object.lastY + my_object.deltaY
                    cv2.line(frame, (my_object.lastX, my_object.lastY), (end_x, end_y), (0, 255, 0), 2)
        else:
            contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours and self.meet_condition_rect(my_object, contours):
                (x, y, w, h) = cv2.boundingRect(contours[0])
                my_object.update_position(x, y)
                if self.test_flag:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return frame

    def solution_core(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            dst_puck = self.detect(self.puck, hue_image, frame)
            dst_robot = self.detect(self.robot, hue_image, frame)

            if self.test_flag:
                cv2.imshow('puck', dst_puck)
                cv2.imshow('robot', dst_robot)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()

    def release_window(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def serial_test(self):
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
        # ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        # while 1:
        #     x = raw_input("Enter x and y value")
        #     print(x)
        #     ser.write(x)
        pass

    def meet_condition_circle(self, my_object, x, y, r):
        area = 3.14 * r * r
        return area in range(my_object.th_area[0], my_object.th_area[1]) and \
               np.abs(x - my_object.lastX) > my_object.th_delta and \
               np.abs(y - my_object.lastY) > my_object.th_delta

    def meet_condition_rect(self, my_object, contours):
        area = cv2.contourArea(contours[0])
        if area in range(my_object.th_area[0], my_object.th_area[1]):
            perimeter = cv2.arcLength(contours[0], True)
            roundness = (perimeter * perimeter) / (6.28 * area)
            return roundness < 8
        return False    
