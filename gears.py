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

R = 2.1

VERBOSE = True
CARGO = False

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

print("\nEstablishing IMU baseline")
baseline = [0,0,0,0,0,0,0,0,0]
for i in range(30):
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
    if i >= 15:
        baseline[0]+=out[0]/15
        baseline[1]+=out[1]/15
        baseline[2]+=out[2]/15
        baseline[3]+=out[3]/15
        baseline[4]+=out[4]/15
        baseline[5]+=out[5]/15
        baseline[6]+=out[6]/15
        baseline[7]+=out[7]/15
        baseline[8]+=out[8]/15
    time.sleep(.1)
print("Baseline Established:")
print("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" %(baseline[0],baseline[1],baseline[2],baseline[3],baseline[4],baseline[5],baseline[6],baseline[7],baseline[8]))

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, -1, 30)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1, 30)
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor, R)
if CARGO:
    BP.set_motor_power(BP.PORT_B, 0)

FRONT_LEFT_ULTRASONIC_PORT = 4
REAR_LEFT_ULTRASONIC_PORT = 3
INFRARED_SENSOR_PORT = 0
FRONT_ULTRASONIC_PORT = BP.PORT_1
sensorWidth = 15.9
grovepi.pinMode(INFRARED_SENSOR_PORT,"INPUT")
BP.set_sensor_type(FRONT_ULTRASONIC_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
try:
    print(BP.get_sensor(FRONT_ULTRASONIC_PORT))
except Exception as e:
    print(e)

wheelBase = .151
wheelDia = .069
delay = .1
maxDPS = 300

pos = Vector2()
posList = []
prevW = 0
prevTheta = 0
x = 0

thetaTarget = 0
thetaActual=  0

hazardList = []

threshold = 30
xsize = 20
ysize = xsize
history = [[-1 for i in range(xsize)] for j in range(ysize)]
visitCounter = 1
headingGlobal = 0 # N=-1, E=0, S=1, W=2/-2

def hazardScan():
    global hazardList
    global state
    magThreshold = 10
    irThreshold = 120
    
    state=KalmanFilter(mpu,state,flter,deltaLy)
    magz=InvGaussFilter(adv,state[1][8], biases[8],std[8],count)
    if abs(magz - baseline[8]) > magThreshold:
        if VERBOSE:
            print("Magnetic Hazard Detected")
        hazardPos = pos.add(angleVector(-headingGlobal * math.pi / 2).multiply(forward))
        hazardList.append([hazardPos.x, hazardPos.y, abs(magz - baseline[8]), "magnet"])
        return 1
    irReading = 0
    try:
        irReading = grovepi.analogRead(INFRARED_SENSOR_PORT)
    except Exception as e:
        print(e)
    if irReading > irThreshold:
        if VERBOSE:
            print("Infrared Hazard Detected")
        hazardPos = pos.add(angleVector(-headingGlobal * math.pi / 2).multiply(forward))
        hazardList.append([hazardPos.x, hazardPos.y, irReading, "fire"])
        return 2
    return 0
        
def turnRobot(turn):
    global state
    global thetaActual
    global prevTheta
    global prevW
    
    turn = ((turn+180)%360)-180
    if VERBOSE:
        if(turn > 0):
            print("\nTURN RIGHT", turn)
        elif(turn < 0):
            print("\nTURN LEFT", turn)
        print("Start Heading:", thetaActual)
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
        
        thetaActual = prevTheta + avg(-(out[5]-baseline[5]), prevW) * dTime
        
        prevW = -(out[5])
        prevTheta = thetaActual
        
        time.sleep(delay)
    if VERBOSE:
        print("End Heading:", thetaActual)

turn = 0
forward = 0

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
        headingGlobal = ((headingGlobal+2)%4)-2
        
        if xGrid == 0 and yGrid == 0:
            if VERBOSE:
                print("STARTING SQUARE")
            if history[0][1] < history[1][0]:
                headingTarget = 1 #turn to face south (towards 0,1)
                turn = 90*(headingTarget-headingGlobal)
                forward = 40
            else:
                headingTarget = 0 #turn to face east (towards 1,0)
                turn = 90*(headingTarget-headingGlobal)
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
                    turn = 90*(headingTarget-headingGlobal)
                else:
                    headingTarget = -1
                    turn = 90*(headingTarget-headingGlobal)
                    forward = 40
            elif headingGlobal == 2 or headingGlobal == -2 : #facing west
                if history[xGrid][yGrid+1] < history[xGrid][yGrid-1]: #if south is more recent
                    headingTarget = -1
                    turn = 90*(headingTarget-headingGlobal)
                else:
                    headingTarget = 1
                    turn = 90*(headingTarget-headingGlobal)
                    forward = 40
            elif headingGlobal == -1: #facing north
                if history[xGrid+1][yGrid] < history[xGrid-1][yGrid]: #if west is more recent
                    headingTarget = 0
                    turn = 90*(headingTarget-headingGlobal)
                else:
                    headingTarget = 2
                    turn = 90*(headingTarget-headingGlobal)
                    forward = 40
            elif headingGlobal == 1: #facing south
                if history[xGrid-1][yGrid] < history[xGrid+1][yGrid]: #if east is more recent
                    headingTarget = 2
                    turn = 90*(headingTarget-headingGlobal)
                else:
                    headingTarget = 0
                    turn = 90*(headingTarget-headingGlobal)
                    forward = 40
        elif leftDist < threshold and frontDist < threshold: #left and front wall
            if VERBOSE:
                print("LEFT, FRONT")
            headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting 2 to -2 (west to west)
            headingTarget = headingGlobal + 1
            turn = 90*(headingTarget-headingGlobal)
        elif leftDist > threshold and frontDist > threshold: #no walls
            if VERBOSE:
                print("NO WALLS")
            headingGlobal = ((headingGlobal+2)%4)-2 #normalize global heading by setting 2 to -2 (west to west)
            headingTarget = headingGlobal - 1
            turn = 90*(headingTarget-headingGlobal)
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
            thetaTarget += turn
            turnRobot(thetaTarget - thetaActual)
            headingGlobal = headingTarget
            headingGlobal = ((headingGlobal+2)%4)-2
        
        hazard = 0
        if abs(xGrid) > 1 or abs(yGrid) > 1:
            hazard = hazardScan()
        if hazard:
            if turn == 90:
                turn = 180
            thetaTarget += turn
            turnRobot(thetaTarget - thetaActual)
            headingGlobal += round(turn/90)
            headingGlobal = ((headingGlobal+2)%4)-2
            forward = 0
            
        if forward != 0:
            if VERBOSE:
                print("STRAIGHT")
            '''waitTime = califDrive.driveDistance(forward)
            prevL = leftMotor.getPosition()
            prevR = rightMotor.getPosition()
            time0 = time.time()
            while time.time() < time0 + waitTime:
                L = leftMotor.getPosition()
                R = rightMotor.getPosition()
                deltaL = wheelDia * (L - prevL) * math.pi / 360
                deltaR = wheelDia * (R - prevR) * math.pi / 360
                dx = avg(deltaL,deltaR)
                x += dx'''
            angleDelta = 360 * forward / (100 * wheelDia * math.pi)
            rDelta = angleDelta
            lDelta = angleDelta
            lStart = leftMotor.getPosition()
            rStart = rightMotor.getPosition()
            speed = 0
            direction = round(abs(forward) / forward)
            maxSpeed = maxDPS * direction
            prevL = lStart
            prevR = rStart
            
            time0 = time.time()
            prevTime = time0 - delay
            while abs(rDelta) > 40 and abs(lDelta) > 40:
                dTime = time.time() - prevTime
                prevTime = time.time()
                if speed < maxDPS:
                    speed += 20
                califDrive.driveSpeed(speed * direction)
                #Calculating position with encoder data
                L = leftMotor.getPosition()
                R = rightMotor.getPosition()
                deltaL = wheelDia * (L - prevL) * math.pi / 360
                deltaR = wheelDia * (R - prevR) * math.pi / 360
                dx = avg(deltaL,deltaR)
                x += dx
                
                '''state=KalmanFilter(mpu,state,flter,deltaLy)
                w = InvGaussFilter(adv,state[1][5], biases[5],std[5],count)
                
                thetaActual = prevTheta + avg(-(out[5]-baseline[5]), prevW) * dTime
                
                prevW = -(w-baseline[5])
                prevTheta = thetaActual'''
                
                prevL = L
                prevR = R
                lDelta = lStart + angleDelta - L
                rDelta = rStart + angleDelta - R
                time.sleep(delay)
            leftMotor.setPosition(lStart + angleDelta)
            rightMotor.setPosition(rStart + angleDelta)
            waitTime = .48
            
            time0 = time.time()
            while time.time() < time0 + waitTime:
                L = leftMotor.getPosition()
                R = rightMotor.getPosition()
                deltaL = wheelDia * (L - prevL) * math.pi / 360
                deltaR = wheelDia * (R - prevR) * math.pi / 360
                dx = avg(deltaL,deltaR)
                x += dx
                
                prevL = L
                prevR = R
                time.sleep(delay)
            
            if VERBOSE:
                print("X Distance:", x)
        
        time.sleep(6)
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
