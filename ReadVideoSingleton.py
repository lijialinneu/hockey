import cv2
import cv


class ReadVideoSingleton:


    __instance = None


    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(3, 320)
        self.cap.set(4, 240)
        self.cap.set(5, 1)


    def __new__(cls, index):
        if not cls.__instance:
            cls.__instance = super(ReadVideoSingleton, cls).__new__(cls)
        return cls.__instance


    def get_capture(self):
        return self.cap
