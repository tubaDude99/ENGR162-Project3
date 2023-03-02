# turntest.py

#import califDrive
import brickpi3

BP = brickpi3.BrickPi3()
#leftMotor = califDrive.Motor(BP, BP.PORT_A, -1)
#rightMotor = califDrive.Motor(BP, BP.PORT_D, -1)
#califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
print(BP.get_motor_encoder(BP.PORT_A))
print(BP.get_motor_encoder(BP.PORT_D))

startAngle = 0
targetAngle = 360

R = 2
targetAngle *= R

try:
    BP.set_motor_position(BP.PORT_A, targetAngle)
    BP.set_motor_position(BP.PORT_D, -targetAngle)
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()
