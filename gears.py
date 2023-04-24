# gears.py

from MPU9250 import MPU9250
import time
import math
import brickpi3
import grovepi
import califDrive
from vector import Vector2
import pathMap

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

def avg(x,y):
    return (x + y)/2
def angleVector(theta):
    return Vector2(math.cos(theta), -math.sin(theta))

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

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, -1, 30)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1, 30)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

FRONT_LEFT_ULTRASONIC_PORT = 4
REAR_LEFT_ULTRASONIC_PORT = 2
FRONT_ULTRASONIC_PORT = BP.PORT_1
sensorWidth = 15.9
BP.set_sensor_type(FRONT_ULTRASONIC_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)

wheelBase = .151
wheelDia = .069
delay = .1
maxDPS = 300

pos = Vector2()
posList = []
theta = 0
prevW = 0
prevTheta = 0
x = 0

xsize = 10
ysize = xsize
history = [[-1 for i in range(xsize)] for j in range(ysize)]
visitCounter = 1
headingGlobal = 0 # N=-1, E=0, S=1, W=2/-2

turn = 0
forward = 0

prevTime = time.time() - delay
try:
    try:
        while 1:
            dTime = time.time() - prevTime
            prevTime = time.time()
            
            '''L = leftMotor.getPosition()
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
            prevR = R'''
            
            pos = pos.add(angleVector(-1 * headingGlobal * math.pi / 2).multiply(forward))
            xGrid = math.floor((pos.x+20)/40)
            yGrid = math.floor((pos.y+20)/40)
            history[xGrid][yGrid] = visitCounter

            print("\n\n MOVEMENT ", visitCounter)
            
            visitCounter = visitCounter + 1
            leftDist = grovepi.ultrasonicRead(FRONT_LEFT_ULTRASONIC_PORT)
            frontDist = BP.get_sensor(FRONT_ULTRASONIC_PORT)
            
            turn = 0
            forward = 0
            
            if xGrid == 0 and yGrid == 0:
                if history[0][1] >= history[1][0]:
                    headingTarget = 0 #turn to face east (towards 1,0)
                    turn = (90*(headingTarget-headingGlobal))
                    forward = 40
                else:
                    headingTarget = 1 #turn to face south (towards 0,1)
                    turn = (90*(headingTarget-headingGlobal))
                    forward = 40
            elif leftDist < 30 and frontDist > 30: #just left wall
                print("LEFT, NO FRONT")
                forward = 40
            elif leftDist > 30 and frontDist < 30: #if no left wall and wall in front (T intersection)
                print("NO LEFT, FRONT")
                if headingGlobal == 0: #facing east
                    if(history[xGrid][yGrid-1] < history[xGrid][yGrid+1]): #if above is more recent
                        headingTarget = 1
                        turn = (90*(headingTarget-headingGlobal))
                    else:
                        headingTarget = -1
                        turn = (90*(headingTarget-headingGlobal))
                        forward = 40
                elif(headingGlobal == 2 or headingGlobal == -2): #facing west
                    if history[xGrid][yGrid+1] < history[xGrid][yGrid-1]: #if below is more recent
                        headingTarget = -1
                        turn = (90*(headingTarget-headingGlobal))
                    else:
                        headingTarget = 1
                        turn = (90*(headingTarget-headingGlobal))
                        forward = 40
                elif(headingGlobal == -1): #facing north
                    if(history[xGrid+1][yGrid] < history[xGrid-1][yGrid]): #if west is more recent
                        headingTarget = 0
                        turn = (90*(headingTarget-headingGlobal))
                    else:
                        headingTarget = -2
                        turn = (90*(headingTarget-headingGlobal))
                        forward = 40
                elif(headingGlobal == 1): #facing south
                    if(history[xGrid-1][yGrid] < history[xGrid+1][yGrid]): #if east is more recent
                        headingTarget = 2
                        turn = (90*(headingTarget-headingGlobal))
                    else:
                        headingTarget = 0
                        turn = (90*(headingTarget-headingGlobal))
                        forward = 40  
            elif(leftDist < 30 and frontDist < 30): #left and front wall
                print("LEFT, FRONT")
                headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting 2 to -2 (west to west)
                headingTarget = headingGlobal + 1
                turn = (90*(headingTarget-headingGlobal))
            elif(leftDist > 30 and frontDist > 30): #no walls
                print("NO LEFT, NO FRONT")
                headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting -2 to 2 (west to west)
                headingTarget = headingGlobal - 1
                turn = (90*(headingTarget-headingGlobal))
                forward = 40

            print(pos)
            print("xGrid",xGrid)
            print("yGrid", yGrid)
            print("headingGlobal", headingGlobal)
            print("headingTarget", headingTarget)
            #print("turn", turn)
            #print("forward", forward)
            if turn != 0:
                turn = ((turn+180)%360)-180
                if(turn == 90):
                    print("\nRIGHT TURN")
                elif(turn == -90):
                    print("\nLEFT TURN")
                
                waitTime = califDrive.turnAngle(turn)
                time0 = time.time()
                while time.time() < time0 + waitTime:
                    time.sleep(.02)
                #time.sleep(waitTime)
                headingGlobal = headingTarget
            if forward != 0:
                print("STRAIGHT")
                waitTime = califDrive.driveDistance(forward)
                time.sleep(waitTime)

            headingGlobal = ((headingGlobal+2)%4)-2
            
            time.sleep(70*delay)
    except brickpi3.SensorError as error:
        print(error)
except KeyboardInterrupt:
    print("Program forcibly terminated")
    BP.set_motor_power(BP.PORT_B, -40)
    time.sleep(0.5)
except Exception as e:
    print(e)
finally:
    BP.reset_all()
