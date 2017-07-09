import serial


class SerialSingleton:

    __instance = None

    def __init__(self, time, puck, robot):
        self.serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
        self.start_time = time
        self.puck = puck
        self.robot = robot

    def __new__(cls,):
        if not cls.__instance:
            cls.__instance = super(SerialSingleton, cls).__new__(cls)
        return cls.__instance
    
    def write_serial(self, time):
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

