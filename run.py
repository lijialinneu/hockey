from Solution import *
from ReadVideoSingleton import *
import serial
"""
Hockey main program
Solution: contour and threshold
"""


if __name__ == "__main__":

    '''
    test Solution on 20170707
    '''
    # Step1. init video and serial
    cap = ReadVideoSingleton(0).get_capture()
    start_time = clock()
    serial = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

    # Step2. init puck and robot
    puck_position = [0, 0]
    puck_th_hsv_low = np.array([50, 120, 46])
    puck_th_hsv_high = np.array([60, 255, 255])
    puck_th_area = [150, 200]
    puck_th_roundness = 8
    puck_delta = 10
    puck = TableObjects(puck_position, puck_th_hsv_low, puck_th_hsv_high,
                        puck_th_area, puck_th_roundness, puck_delta)
    robot_position = [0, 0]
    robot_th_hsv_low = np.array([0, 120, 100])
    robot_th_hsv_high = np.array([5, 255, 255])
    robot_th_area = [100, 150]
    robot_th_roundness = 8
    robot_delta = 10
    robot = TableObjects(robot_position, robot_th_hsv_low, robot_th_hsv_high,
                         robot_th_area, robot_th_roundness, robot_delta)

    # Step3. start solution
    solution = Solution(cap, puck, robot, serial, start_time, True)
    solution.solution_core()

    print('finish')
