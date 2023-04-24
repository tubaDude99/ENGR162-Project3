# califDrive.py
import math

class Motor:
    def __init__(self, BP, port, direction = 1, maxPower = 100):
        self.BP = BP
        self.port = port
        self.direction = direction
        self.maxPower = maxPower
        
        self.BP.set_motor_limits(self.port, self.maxPower)
    def setPower(self, power):
        if power > self.maxPower:
            power = self.maxPower
        elif power < -self.maxPower:
            power = -self.maxPower
        self.BP.set_motor_power(self.port, power * self.direction)
    def setPowerLimit(self, maxPower):
        self.maxPower = maxPower
        self.BP.set_motor_limits(self.port, maxPower)
    def setPosition(self, position):
        self.BP.set_motor_position(self.port, position*self.direction)
    def setSpeed(self, speed):
        self.BP.set_motor_dps(self.port, speed*self.direction)
    def getPosition(self):
        return self.direction * self.BP.get_motor_encoder(self.port)
    def resetEncoder(self):
        self.BP.offset_motor_encoder(self.port, self.getPosition())

class CalifDrive:
    def __init__(self, BP, leftMotor, rightMotor, turnRatio=2, wheelDia=.069):
        self.BP = BP
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.turnRatio = turnRatio;
        self.wheelDia = wheelDia;
        
        self.leftMotor.resetEncoder()
        self.rightMotor.resetEncoder()
    def drive(self, power):
        self.leftMotor.setPower(power)
        self.rightMotor.setPower(power)
    def driveDiff(self, leftPower, rightPower):
        self.leftMotor.setPower(leftPower)
        self.rightMotor.setPower(rightPower)
    def driveDistance(self, distance):
        # Takes a distance in centimeters and sets the robot to drive that distance, then returns the expected time
        leftPos = self.leftMotor.getPosition()
        rightPos = self.rightMotor.getPosition()
        distance /= 100
        angleDelta = 360 * distance / (self.wheelDia * math.pi)
        self.leftMotor.setPosition(leftPos + angleDelta)
        self.rightMotor.setPosition(rightPos + angleDelta)
        return abs(angleDelta*.0027) + .4
    def driveSpeed(self, speed):
        self.leftMotor.setSpeed(speed)
        self.rightMotor.setSpeed(speed)
    def driveDiffSpeed(self, lSpeed, rSpeed):
        self.leftMotor.setSpeed(lSpeed)
        self.rightMotor.setSpeed(rSpeed)
    def turnSpeed(self, power):
        self.leftMotor.setPower(-power)
        self.rightMotor.setPower(power)
    def turnAngle(self, angle):
        angleDelta = angle * self.turnRatio
        leftPos = self.leftMotor.getPosition()
        rightPos = self.rightMotor.getPosition()
        self.leftMotor.setPosition(leftPos+angleDelta)
        self.rightMotor.setPosition(rightPos-angleDelta)
        return abs(angleDelta*.0029) + .4