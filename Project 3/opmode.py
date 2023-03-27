# opmode.py
# Template for creating a new opmode

import time
import califDrive
import brickpi3
import grovepi
import sensors

LEFT_ULTRASONIC_PORT = 1
RIGHT_ULTRASONIC_PORT = 2
FRONT_ULTRASONIC_PORT = 3

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, -1)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

leftUltrasonic = sensors.GroveUltrasonic(LEFT_ULTRASONIC_PORT)
rightUltrasonic = sensors.GroveUltrasonic(RIGHT_ULTRASONIC_PORT)
frontUltrasonic = sensors.BrickUltrasonic(BP, FRONT_ULTRASONIC_PORT)
imu = sensors.GroveIMU()

try:
    while 1:
        pass
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()