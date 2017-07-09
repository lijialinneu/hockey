from Solution import *
from ReadVideoSingleton import *
from TableObjects import *

"""
Hockey main program
Solution: contour and threshold
"""


if __name__ == "__main__":

    '''
    test Solution on 20170707
    '''
    cap = ReadVideoSingleton(0).get_capture()


    # arguments for puck green
    puck_position = [0, 0]
    puck_th_hsv_low = np.array([50, 120, 46])
    puck_th_hsv_high = np.array([60, 255, 255])
    puck_th_area = [150, 200]
    puck_th_roundness = 8
    puck_delta = 10
    puck = TableObjects(puck_position, puck_th_hsv_low, puck_th_hsv_high,
                        puck_th_area, puck_th_roundness, puck_delta)

    # argument for robot red
    robot_position = [0, 0]
    robot_th_hsv_low = np.array([0, 120, 100])
    robot_th_hsv_high = np.array([5, 255, 255])
    robot_th_area = [100, 150]
    robot_th_roundness = 8
    robot_delta = 10
    robot = TableObjects(robot_position, robot_th_hsv_low, robot_th_hsv_high,
                         robot_th_area, robot_th_roundness, robot_delta)

    solution = Solution(cap, puck, robot)
    solution.solution_core()

    print('finish')
