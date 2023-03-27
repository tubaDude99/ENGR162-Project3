import brickpi3
BP = brickpi3.BrickPi3()

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
print(BP.get_motor_encoder(BP.PORT_A))
print(BP.get_motor_encoder(BP.PORT_D))

BP.set_motor_position(BP.PORT_A, 180)
