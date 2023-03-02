# gears.py

import time
import math
import califDrive
import brickpi3
import grovepi
from vector import Vector2

def avg(x,y):
    return (x + y)/2
def angleVector(theta):
    return Vector2(math.cos(theta), -math.sin(theta))

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, 1)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

wheelBase = .13
wheelRad = .065 / 2
delay = .02

pos = Vector2()
posList = []
theta = 0
prevTheta = 0
prevL = leftMotor.getPosition()
prevR = rightMotor.getPosition()

prevTime = time.time() - delay
try:
    while 1:
        dTime = time.time() - prevTime
        prevTime = time.time()
        
        L = leftMotor.getPosition()
        R = rightMotor.getPosition()
        dL = wheelRad * (L - prevL) * math.pi / 180
        dR = wheelRad * (R - prevR) * math.pi / 180
        
        dTheta = math.atan((dL - dR)/wheelBase) / 2
        theta += dTheta * dTime
        dx = avg(dL,dR)
        pos = pos.add(angleVector(avg(theta,prevTheta)).multiply(dx))
        posList.append(pos)
        
        prevTheta = theta
        prevL = L
        prevR = R
        
        time.sleep(delay)
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()