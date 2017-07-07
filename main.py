import cv
import cv2
import numpy as np
import serial


class Solution:

    lastX, lastY, deltaX, deltaY = [0, 0, 0, 0]

    def __init__(self, low = np.array([0, 123, 100]),
                 high = np.array([5, 255, 255]), delta = 5, flag = True):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 360)
        self.cap.set(4, 240)
        self.low_range = low
        self.high_range = high
        self.threshold_delta = delta
        self.test_flag = flag

    
    def serial_test(self):
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
        while 1:
            x = raw_input("Enter x and y value")
            print(x)
            ser.write(x)
        pass


    def send_data(self):
        pass


    def cal_v(self):
        pass


    def update_position(self, x, y):
        x = int(x)
        y = int(y)
        self.deltaX = x - self.lastX
        self.deltaY = y - self.lastY
        self.lastX = x
        self.lastY = y


    def meet_condition_circle(self, x, y, r):
        return np.abs(x - self.lastX) > self.threshold_delta and \
               np.abs(y - self.lastY) > self.threshold_delta


    def meet_condition_rect(self, contours):
        area = cv2.contourArea(contours[0])
        perimeter = cv2.arcLength(contours[0], True)
        roundness = (perimeter * perimeter) / (6.28 * area)
        return roundness < 8
        
    
    def solution_core(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()   
            hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            threshold_img = cv2.inRange(hue_image, self.low_range, self.high_range)

            dilated = cv2.dilate(threshold_img,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
            circles = cv2.HoughCircles(threshold_img, cv.CV_HOUGH_GRADIENT, 1, 100,
                                       param1=15, param2=7, minRadius=10, maxRadius=20)
            if circles is not None:
                x, y, radius = circles[0][0]
                center = (x, y)

                if self.meet_condition_circle(x, y, radius):     
                    self.update_position(x, y)
                    self.send_data()

                    # test draw circle and line
                    if self.test_flag:
                        cv2.circle(frame, center, radius, (0, 255, 0), 2)
                        endX = self.lastX + self.deltaX
                        endY = self.lastY + self.deltaY
                        cv2.line(frame, (self.lastX, self.lastY),(endX, endY), (0, 255, 0), 2)

            else:
                contours, hierarchy = cv2.findContours(dilated,
                                                       cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if contours and self.meet_condition_rect(contours):
                    (x, y, w, h) = cv2.boundingRect(contours[0])
                    self.update_position(x, y)

                    # test draw rectangle
                    if self.test_flag:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if self.test_flag:
                cv2.imshow('video', frame)
        
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_window()


    def release_window(self):
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    solution = Solution()
    solution.solution_core()
