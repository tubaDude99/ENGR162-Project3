# sensors.py
import grovepi
import mpu9250_i2c as imu

class GroveUltrasonic:
    def __init__(self, PORT):
        self.PORT = PORT
    def read(self):
        return grovepi.ultrasonicRead(self.PORT)
class BrickUltrasonic:
    def __init__(self, BP, PORT):
        self.BP = BP
        self.PORT = PORT
class GroveIMU:
    def read():
        return imu.read_imu()
    def readAccel():
        output = imu.read_imu()
        return output[0], output[1], output[2]
    def readAngVel():
        output = imu.read_imu()
        return output[3], output[4], output[5]