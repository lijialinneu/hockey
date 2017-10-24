# !/usr/bin/env python
# -*- coding: utf-8 -*-


"""
ReadVideoSingleton class
Author: lijialin 1040591521@qq.com
Date: July 2017

This class is used to start video capture

Functions:
    get_capture(): return cv2.VideoCapture object

"""

import cv2
import cv
import threading

class ReadVideoSingleton:


    __instance = None
    lock = threading.Lock()


    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(3, 320) # set width 320px
        self.cap.set(4, 240) # set height 240px
        # self.cap.set(5, 90)  # set fps 60


    def __new__(cls, index):
        try:
            lock.acquire()
            if not cls.__instance:
                cls.__instance = super(ReadVideoSingleton, cls).__new__(cls)
        finally:
            lock.release()
        return cls.__instance


    def get_capture(self):
        return self.cap
