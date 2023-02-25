# wallfollow.py

import time
import califDrive
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, 1)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

try:
    while 1:
        pass
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()