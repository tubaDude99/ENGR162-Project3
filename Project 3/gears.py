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

VERBOSE = True
CARGO = True

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
if CARGO:
    BP.set_motor_power(BP.PORT_B, 0)

FRONT_LEFT_ULTRASONIC_PORT = 4
REAR_LEFT_ULTRASONIC_PORT = 2
FRONT_ULTRASONIC_PORT = BP.PORT_1
sensorWidth = 15.9
BP.set_sensor_type(FRONT_ULTRASONIC_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
try:
    print(BP.get_sensor(FRONT_ULTRASONIC_PORT))
except Exception as e:
    print(e)

wheelBase = .151
wheelDia = .069
delay = .01
maxDPS = 300

pos = Vector2()
posList = []
theta = 0
prevW = 0
prevTheta = 0
x = 0

threshold = 30
xsize = 10
ysize = xsize
history = [[-1 for i in range(xsize)] for j in range(ysize)]
visitCounter = 1
headingGlobal = 0 # N=-1, E=0, S=1, W=2/-2

def turnRobot(turn):
    global state
    global theta
    global prevTheta
    global prevW
    
    turn = ((turn+180)%360)-180
    if VERBOSE:
        if(turn == 90):
            print("\nRIGHT TURN")
        elif(turn == -90):
            print("\nLEFT TURN")
        print("Start Heading:", theta)
    
    waitTime = califDrive.turnAngle(turn)
    time0 = time.time()
    prevTime = time0 - delay
    while time.time() < time0 + waitTime:
        dTime = time.time() - prevTime
        prevTime = time.time()
        
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
        
        theta = prevTheta + avg(-out[5], prevW) * dTime
        
        prevW = -out[5]
        prevTheta = theta
        
        time.sleep(delay)

turn = 0
forward = 0

#prevTime = time.time() - delay

try:
    input("Press ENTER to start")
    while 1:
        leftDist = grovepi.ultrasonicRead(FRONT_LEFT_ULTRASONIC_PORT)
        frontDist = 100
        try:
            frontDist = BP.get_sensor(FRONT_ULTRASONIC_PORT)
        except Exception as e:
            print(e)
    
        pos = pos.add(angleVector(-headingGlobal * math.pi / 2).multiply(forward))
        posList.append(pos)
        x = 0
        
        xGrid = math.floor((pos.x+20)/40)
        yGrid = math.floor((pos.y+20)/40)
        history[xGrid][yGrid] = visitCounter
        
        if VERBOSE:
            print("\n\nMOVEMENT", visitCounter)
        visitCounter = visitCounter + 1
        
        turn = 0
        forward = 0
        
        if xGrid == 0 and yGrid == 0:
            if VERBOSE:
                print("STARTING SQUARE")
            if history[0][1] >= history[1][0]:
                headingTarget = 0 #turn to face east (towards 1,0)
                turn = (90*(headingTarget-headingGlobal))%360
                forward = 40
            else:
                headingTarget = 1 #turn to face south (towards 0,1)
                turn = (90*(headingTarget-headingGlobal))%360
                forward = 40
        elif leftDist < threshold and frontDist > threshold: #just left wall
            if VERBOSE:
                print("LEFT, NO FRONT")
            forward = 40
        elif leftDist > threshold and frontDist < threshold: #if no left wall and wall in front (T intersection)
            if VERBOSE:
                print("NO lEFT, FRONT")
            if headingGlobal == 0: #facing east
                if history[xGrid][yGrid-1] < history[xGrid][yGrid+1]: #if north is more recent
                    headingTarget = 1
                    turn = 90
                else:
                    headingTarget = -1
                    turn = -90
                    forward = 40
            elif headingGlobal == 2 or headingGlobal == -2 : #facing west
                if history[xGrid][yGrid+1] < history[xGrid][yGrid-1]: #if south is more recent
                    headingTarget = -1
                    turn = 90
                else:
                    headingTarget = 1
                    turn = -90
                    forward = 40
            elif headingGlobal == -1: #facing north
                if history[xGrid+1][yGrid] < history[xGrid-1][yGrid]: #if west is more recent
                    headingTarget = 0
                    turn = 90
                else:
                    headingTarget = 2
                    turn = -90
                    forward = 40
            elif headingGlobal == 1: #facing south
                if history[xGrid-1][yGrid] < history[xGrid+1][yGrid]: #if east is more recent
                    headingTarget = 2
                    turn = 90
                else:
                    headingTarget = 0
                    turn = -90
                    forward = 40
        elif leftDist < threshold and frontDist < threshold: #left and front wall
            if VERBOSE:
                print("LEFT, FRONT")
            headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting 2 to -2 (west to west)
            headingTarget = headingGlobal + 1
            turn = 90
        elif leftDist > threshold and frontDist > threshold: #no walls
            headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting -2 to 2 (west to west)
            headingTarget = headingGlobal - 1
            turn = -90
            forward = 40
        
        if VERBOSE:
            print(pos)
            print("xGrid",xGrid)
            print("yGrid", yGrid)
            print("headingGlobal", headingGlobal)
            print("headingTarget", headingTarget)
            print("turn", turn)
            print("forward", forward)
        
        if turn != 0:
            '''turn = ((turn+180)%360)-180
            if VERBOSE:
                if(turn == 90):
                    print("\nRIGHT TURN")
                elif(turn == -90):
                    print("\nLEFT TURN")
                print("Start Heading:", theta)
            
            waitTime = califDrive.turnAngle(turn)
            time0 = time.time()
            prevTime = time0 - delay
            while time.time() < time0 + waitTime:
                dTime = time.time() - prevTime
                prevTime = time.time()
                
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
                
                theta = prevTheta + avg(-out[5], prevW) * dTime
                
                prevW = -out[5]
                prevTheta = theta
                
                time.sleep(delay)'''
            #time.sleep(waitTime)
            headingGlobal = headingTarget
            headingGlobal = ((headingGlobal+2)%4)-2
            
            if VERBOSE:
                print("End Heading:", theta)
        if forward != 0:
            if VERBOSE:
                print("STRAIGHT")
            waitTime = califDrive.driveDistance(forward)
            prevL = leftMotor.getPosition()
            prevR = rightMotor.getPosition()
            time0 = time.time()
            while time.time() < time0 + waitTime:
                L = leftMotor.getPosition()
                R = rightMotor.getPosition()
                deltaL = wheelDia * (L - prevL) * math.pi / 360
                deltaR = wheelDia * (R - prevR) * math.pi / 360
                dx = avg(deltaL,deltaR)
                x += dx
            #time.sleep(waitTime)
            if VERBOSE:
                print("X Distance:", x)
        
        time.sleep(3)
except KeyboardInterrupt:
    print("Program forcibly terminated")
    if CARGO:
        print("Depositing cargo")
        BP.set_motor_power(BP.PORT_B, -40)
        time.sleep(0.3)
except Exception as e:
    print(e)
finally:
    BP.reset_all()
