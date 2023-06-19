# calibration.py

import time
import grovepi
import brickpi3

BP = brickpi3.BrickPi3()
#BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

try:
    while 1:
        input()
        for i in range(5):
            print(grovepi.ultrasonicRead(4))
            #try:
                #print(BP.get_sensor(BP.PORT_1))
            #except Exception as e:
                #print(e)
            time.sleep(.2)
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()