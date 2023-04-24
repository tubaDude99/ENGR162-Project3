# hazardTest.py

import time
import math
import grovepi
from MPU9250 import MPU9250

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

def avg(x,y):
    return (x + y)/2

mpu = MPU9250()

width=1
depth=100
deltaLy=0.01
adv = True

accelx=genWindow(width,0)
accely=genWindow(width,0)
accelz=genWindow(width,0)
gyrox=genWindow(width,0)
gyroy=genWindow(width,0)
gyroz=genWindow(width,0)
magx=genWindow(width,0)
magy=genWindow(width,0)
magz=genWindow(width,0)
flter=[[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[5,2],[5,2],[5,2]]

biases=AvgCali(mpu,depth,deltaLy)
state=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0,0,0,0,0,0,0,0,0]]
out=[0,0,0,0,0,0,0,0,0]
std=FindSTD(biases,mpu,deltaLy)
pick = 2
count = 3

logging = False
if logging:
    dataFile = open("imu_data.csv", 'w')

try:
    t0 = time.time()
    while 1:
        t = time.time() - t0
        state=KalmanFilter(mpu,state,flter,deltaLy)
        out[0]=InvGaussFilter(adv,state[1][0], biases[0],std[0],count)
        out[1]=InvGaussFilter(adv,state[1][1], biases[1],std[1],count)
        out[2]=InvGaussFilter(adv,state[1][2], biases[2],std[2],count)
        out[3]=InvGaussFilter(adv,state[1][3], biases[3],std[3],count)
        out[4]=InvGaussFilter(adv,state[1][4], biases[4],std[4],count)
        out[5]=InvGaussFilter(adv,state[1][5], biases[5],std[5],count)
        out[6]=InvGaussFilter(adv,state[1][6], biases[6],std[6],count)
        out[7]=InvGaussFilter(adv,state[1][7], biases[7],std[7],count)
        out[8]=InvGaussFilter(adv,state[1][8], biases[8],std[8],count)
        print(out[0],'\t',out[1],'\t',out[2])
        if logging:
            dataFile.write(','.join([str(t),str(out[6]),str(out[7]),str(out[8])]))
        time.sleep(.1)
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    if logging:
        dataFile.close()