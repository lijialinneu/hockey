import serial


class SerialSingleton:

    __instance = None

    def __init__(self):
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(SerialSingleton, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def get_serial(self):
        return self.serial
    
    def send_message(self, message):
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
        print(message)

