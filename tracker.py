# !/usr/bin/env python
# -*- coding: utf-8 -*-



"""
Adjust parameters module
Author: lijialin 1040591521@qq.com
Date: July 2017

This program is used to help us to adjust
parameters of hsv low threshold and hsv
high threshold
"""



import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
cap.set(5, 60)

global th_image

global hsv_low
hsv_low = np.array([0, 0, 0])
global hsv_high
hsv_high = np.array([0, 0, 0])


def h_low(value):
    hsv_low[0] = value

def h_high(value):
    hsv_high[0] = value

def s_low(value):
    hsv_low[1] = value

def s_high(value):
    hsv_high[1] = value

def v_low(value):
    hsv_low[2] = value

def v_high(value):
    hsv_high[2] = value
    

cv2.namedWindow('image')
cv2.createTrackbar('H low','image', 0,255, h_low)
cv2.createTrackbar('H high','image',0,255, h_high)
cv2.createTrackbar('S low','image', 0,255, s_low)
cv2.createTrackbar('S high','image',0,255, s_high)
cv2.createTrackbar('V low','image', 0,255, v_low)
cv2.createTrackbar('V high','image',0,255, v_high)


while cap.isOpened():
    ret, frame = cap.read()
    hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    th_image = cv2.inRange(hue_image, hsv_low, hsv_high)
    cv2.imshow('image', th_image)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print('finish')
