# balance.py
from MPU9250 import MPU9250
import sys
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import WindowFilterDyn
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

mpu9250 = MPU9250()
BP = brickpi3.BrickPi3()

width=1
depth=100
dly=0.01
adv = True

accelx=genWindow(width,0)#Can play with width to adjust system
accely=genWindow(width,0)
accelz=genWindow(width,0)
gyrox=genWindow(width,0)#Can play with width to adjust system
gyroy=genWindow(width,0)
gyroz=genWindow(width,0)
magx=genWindow(width,0)#Can play with width to adjust system
magy=genWindow(width,0)
magz=genWindow(width,0)
flter=[[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[5,2],[5,2],[5,2]]

biases=AvgCali(mpu9250,depth,dly)
state=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0,0,0,0,0,0,0,0,0]]#Estimated error (p) and measurement state (x) 
out=[0,0,0,0,0,0,0,0,0]
std=FindSTD(biases,mpu9250,dly)
pick = 2 #1 uses window filter, anything else uses Kalman
count = 3 #Number of standard deviations used for filtering

t0=time.time()

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))

aOffset = 0
dOffset = 0

p = .1

delay = .02
try:
    while True:
        state=KalmanFilter(mpu9250,state,flter,dly)
        out[0]=InvGaussFilter(adv,state[1][0], biases[0],std[0],count)
        out[1]=InvGaussFilter(adv,state[1][1], biases[1],std[1],count)
        out[2]=InvGaussFilter(adv,state[1][2], biases[2],std[2],count)
        out[3]=InvGaussFilter(adv,state[1][3], biases[3],std[3],count)
        out[4]=InvGaussFilter(adv,state[1][4], biases[4],std[4],count)
        out[5]=InvGaussFilter(adv,state[1][5], biases[5],std[5],count)
        out[6]=InvGaussFilter(adv,state[1][6], biases[6],std[6],count)
        out[7]=InvGaussFilter(adv,state[1][7], biases[7],std[7],count)
        out[8]=InvGaussFilter(adv,state[1][8], biases[8],std[8],count)
        aOffset += out[0] * p
        dOffset += out[1] * p
        BP.set_motor_position(BP.PORT_A, aOffset)
        BP.set_motor_position(BP.PORT_D, dOffset)
        time.sleep(delay)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()