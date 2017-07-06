import cv
import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(3, 360)
cap.set(4, 240)

low_range = np.array([0, 123, 100])
high_range = np.array([5, 255, 255])
lastX, lastY, deltaX, deltaY = [0, 0, 0, 0]

def send_data():
    pass

while cap.isOpened():
    ret, frame = cap.read()
    
    hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    th = cv2.inRange(hue_image, low_range, high_range)
    dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    circles = cv2.HoughCircles(th, cv.CV_HOUGH_GRADIENT, 1, 100, \
                                param1=15, param2=7, minRadius=10, maxRadius=20)
    if circles is not None:
        x, y, radius = circles[0][0]
        center = (x, y)

        # update position
        deltaX = int(x) - lastX
        deltaY = int(y) - lastY
        lastX = int(x)
        lastY = int(y)

        if np.abs(deltaX) > 5 and np.abs(deltaY) > 5:
            send_data()
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            cv2.line(frame, (lastX, lastY), (lastX + deltaX, lastY + deltaY),(0, 255, 0), 2)
    
    else:
        contour, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contour:
            (x, y, w, h) = cv2.boundingRect(contour[0])

            deltaX = int(x) - lastX
            deltaY = int(y) - lastY
            lastX = int(x)
            lastY = int(y)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    cv2.imshow('video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()





