# !/usr/bin/env python
# -*- coding: utf-8 -*-


"""
TableObjects class
Author: lijialin 1040591521@qq.com
Date: July 2017

Variables:
    self.lastX:        object's last x
    self.lastY:        object's last y 
    self.deltaX:       object's delta x
    self.deltaY:       object's delta y
    self.area:         object's area
    self.perimeter:    object's perimeter
    self.roundness:    object's roundness
    self.th_hsv_low:   object's hsv low threshold
    self.th_hsv_high:  object's hsv low threshold
    self.th_area:      threshold of object's area
    self.th_roundness: threshold of object's roundness

Functions:
    cal_area():        calculate the area of a contour
    cal_area_circle(): calculate the area of a circle
    cal_perimeter():   calculate the perimeter of a contour
    cal_roundness():   calculate roundness of a contour
    meet_area():       return whether area satisfacts area threshold
    meet_roundness():  return whether roundness satisfacts threshold
    update_position(): update the position of table objects

"""


import cv2
import sys


class TableObjects:


    def __init__(self, position, th_hsv_low, th_hsv_high, th_area, th_roundness, th_delta):
        self.lastX, self.lastY = position
        self.deltaX = 0
        self.deltaY = 0
        self.area = 0
        self.perimeter = 0
        self.roundness = 0
        self.th_hsv_low = th_hsv_low
        self.th_hsv_high = th_hsv_high
        self.th_area = th_area
        self.th_roundness = th_roundness


    def cal_area(self, contour):
        self.area = cv2.contourArea(contour)
        return self.area


    def cal_area_circle(self, radius):
        self.area = 3.14 * radius * radius
        return min(self.area, 1000)


    def cal_perimeter(self, contour):
        self.perimeter = cv2.arcLength(contour, True)
        return self.perimeter


    def cal_roundness(self):
        self.roundness = (self.perimeter * self.perimeter) / (6.28 * self.area)
        return self.roundness


    def meet_area(self):
        return self.area in range(self.th_area[0], self.th_area[1])


    def meet_roundness(self):
        return self.roundness < self.th_roundness


    def update_position(self, x, y):
        x = int(x)
        y = int(y)
        self.deltaX = x - self.lastX
        self.deltaY = y - self.lastY
        self.lastX = x
        self.lastY = y

