import numpy as np
import cv
from TableObjects import *
from time import clock


class Solution:


    def __init__(self, cap, puck, robot, serial,
                 time, move, origin, flag=True):        
        self.cap = cap
        self.puck = puck
        self.robot = robot
        self.serial = serial
        self.start_time = time
        self.move_th = move
        self.origin = origin
        self.test_flag = flag


    def solution_core(self):
        while self.cap.isOpened():            
            ret, frame = self.cap.read()
            # cv2.circle(frame, (160, 120), 1, (0,255,0), 2)
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            pos_p = self.detect_puck(hue_image, frame)
            pos_r = self.detect_robot(hue_image, frame)
            
            if self.if_move(self.puck, pos_p) or \
               self.if_move(self.robot, pos_r):                        
                # send data
                message = self.create_message()
                self.serial.write(message)
            
            if self.test_flag:
                if pos_p:
                    x, y, w, h = pos_p
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if pos_r:
                    x, y, r = pos_r
                    cv2.circle(frame, (x, y), r, (0, 0, 255), 2)
                cv2.imshow('frame', frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        self.release_window()


    def detect_puck(self, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.puck.th_hsv_low, self.puck.th_hsv_high)
        contours, hierarchy = cv2.findContours(
                th_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours and self.meet_condition(self.puck, contours[0]):
            (x, y, w, h) = cv2.boundingRect(contours[0])
            self.puck.update_position(x, y)
            return x, y, w, h
        return None


    def detect_robot(self, hue_image, frame):
        th_img = cv2.inRange(hue_image, self.robot.th_hsv_low, self.robot.th_hsv_high)
        circles = cv2.HoughCircles(th_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=10, maxRadius=20)
        if circles is not None:
            x, y, r = circles[0][0]
            center = (x, y)                
            self.robot.update_position(x, y)
            return x, y, r
        return None


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

        # change to millisecond
        time = clock() * 1000 - self.start_time * 1000 
        time_h, time_l = self.high_and_low(long(time))

        # change coordinate x and y
        p_x = 2 * (self.puck.lastX - self.origin[0])
        p_y = 2 * (self.puck.lastY - self.origin[1])
        r_x = 2 * (self.robot.lastX - self.origin[0])
        r_y = 2 * (self.robot.lastY - self.origin[1])

        if p_x in range(0, 640) and p_y in range(0, 480) :

            print(p_x, ' ',p_y, '     ', r_x, ' ', r_y)

            puck_x_h, puck_x_l = self.high_and_low(p_x)
            puck_y_h, puck_y_l = self.high_and_low(p_y)
            area_h, area_l = self.high_and_low(int(self.puck.area))
            robot_x_h, robot_x_l = self.high_and_low(r_x)
            robot_y_h, robot_y_l = self.high_and_low(r_y)
            
            
            message = 'mm' + chr(time_h) + chr(time_l) \
                           + chr(puck_x_h) + chr(puck_x_l) \
                           + chr(puck_y_h) + chr(puck_y_l) \
                           + chr(area_h) + chr(area_l) \
                           + chr(robot_x_h) + chr(robot_x_l) \
                           + chr(robot_y_h) + chr(robot_y_l)
            '''

            message = 'mm' + \
                      self.my_chr(int(time)) + \
                      self.my_chr(int(p_x)) + self.my_chr(int(p_y)) + \
                      self.my_chr(int(self.puck.area)) + \
                      self.my_chr(int(r_x)) + self.my_chr(int(r_y))
            '''
            
        print('message = ', message)
        return message


    '''
    def my_chr(self, x):
        if x not in range(0, 256):
            return chr(255)
        else:
            return chr(x)
    '''

    def high_and_low(self, x):
        return (x >> 8) & 255, x & 255


    def release_window(self):
        self.cap.release()
        cv2.destroyAllWindows()

