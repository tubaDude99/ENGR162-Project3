# califDrive.py
import time

class Motor:
    def __init__(self, BP, port, direction):
        self.BP = BP
        self.port = port
        self.direction = direction
    def setPower(self, power):
        self.BP.set_motor_power(self.port, power * self.direction)
    def getPosition(self):
        return self.BP.get_motor_encoder(self.port)
    def resetEncoder(self):
        self.BP.offset_motor_encoder(self.port, self.getPosition())

class CalifDrive:
    def __init__(self, BP, leftMotor, rightMotor):
        self.BP = BP
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        
        self.leftMotor.resetEncoder()
        self.rightMotor.resetEncoder()
    def drive(self, power):
        self.leftMotor.setPower(power)
        self.rightMotor.setPower(power)
    def driveDiff(self, leftPower, rightPower):
        self.leftMotor.setPower(leftPower)
        self.rightMotor.setPower(rightPower)
    def turnSpeed(self, power):
        self.leftMotor.setPower(-power)
        self.rightMotor.setPower(power)
    def turnAngle(self, angle, power=100):
        pass