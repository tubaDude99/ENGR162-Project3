# turntest.py

#import califDrive
import brickpi3
import time
import califDrive

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, -1, 20)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1, 20)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

#BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
#BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
print(leftMotor.getPosition())
print(rightMotor.getPosition())
print()

#BP.set_motor_limits(BP.PORT_A, 20)
#BP.set_motor_limits(BP.PORT_D, 20)

startAngle = 0
targetAngle = 360

R = 2.118
targetAngle *= R
#BP.set_motor_dps(PORT, target) - sets a target speed for the motor in degrees/s
try:
    BP.set_motor_position(BP.PORT_A, targetAngle)
    BP.set_motor_position(BP.PORT_D, -targetAngle)
    time.sleep(10)
    print(BP.get_motor_encoder(BP.PORT_A))
    print(BP.get_motor_encoder(BP.PORT_D))
    print()
    BP.set_motor_position(BP.PORT_A, 0)
    BP.set_motor_position(BP.PORT_D, 0)
    time.sleep(1.7*targetAngle/90)
    print(BP.get_motor_encoder(BP.PORT_A))
    print(BP.get_motor_encoder(BP.PORT_D))
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()
