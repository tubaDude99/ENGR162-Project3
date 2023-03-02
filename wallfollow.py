# wallfollow.py

import time
import math
import califDrive
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, 1)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

FRONT_LEFT_ULTRASONIC_PORT = 3
REAR_LEFT_ULTRASONIC_PORT = 2
FRONT_ULTRASONIC_PORT = BP.PORT_3
sensorWidth = 15

wheelBase = .13
wheelRad = .065 / 2
delay = .02
p = 1

distance = 0
prevTime = time.time() - delay
try:
    while 1:
        dTime = time.time() - prevTime
        prevTime = time.time()
        
        frontDistance = grovepi.ultrasonicRead(FRONT_LEFT_ULTRASONIC_PORT)
        rearDistance = grovepi.ultrasonicRead(REAR_LEFT_ULTRASONIC_PORT)
        delta = rearDistance - frontDistance
        
        if abs(delta) < 12:
            heading = -180*(1/math.pi)*math.atan(delta/sensorWidth)
            print(heading)
        
        time.sleep(delay)
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()