# turntest2.py

from MPU9250 import MPU9250
import time
import math
import brickpi3
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
califDrive = califDrive.CalifDrive(BP, leftMotor, rightMotor, 1.92)

wheelBase = .151
wheelDia = .069
delay = .01
maxDPS = 300

pos = Vector2()
posList = []
theta = 0
prevTheta = 0
prevL = leftMotor.getPosition()
prevR = rightMotor.getPosition()
x = 0

pos2 = Vector2()
posList2 = []
theta2 = 0
prevTheta2 = 0
prevAcc = 0
prevVel = 0
prevW = 0
x2 = 0

p = .1

with open("path.csv", 'r') as pathFile:
    pathLines = pathFile.readlines()
path = []
for line in pathLines:
    line = line.split(',')
    path.append([line[0], float(line[1])])

i = 0
t0=time.time()
prevTime = time.time() - delay - deltaLy
try:
    for command in path:
        if command[0] == "forward":
            lStart = leftMotor.getPosition()
            rStart = rightMotor.getPosition()
            distance = command[1] / 100
            angleDelta = 360 * distance / (wheelDia * math.pi)
            rDelta = angleDelta
            lDelta = angleDelta
            speed = 0
            direction = int(abs(distance) / distance)
            maxSpeed = maxDPS * (abs(distance) / distance)
            while abs(rDelta) > 30 and abs(lDelta) > 30:
                if speed < maxDPS:
                    speed += 5
                califDrive.driveSpeed(speed * direction)
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
                
                #Calculating position with encoder data
                L = leftMotor.getPosition()
                R = rightMotor.getPosition()
                deltaL = wheelDia * (L - prevL) * math.pi / 360
                deltaR = wheelDia * (R - prevR) * math.pi / 360
                
                deltaTheta = math.atan((deltaL - deltaR)/wheelBase) / 2
                theta += deltaTheta
                dx = avg(deltaL,deltaR)
                x += dx
                pos = pos.add(angleVector(avg(theta,prevTheta)).multiply(dx))
                posList.append(pos)
                
                prevTheta = theta
                prevL = L
                prevR = R
                
                #Calculating position with imu data
                vel = prevVel + avg(out[1], prevAcc) * dTime
                dx = avg(vel, prevVel) * dTime
                
                x2 += dx
                theta2 = prevTheta2 + avg(-out[5], prevW) * dTime
                pos2 = pos2.add(angleVector(avg(theta2,prevTheta2)*math.pi/180).multiply(dx))
                
                posList2.append(pos2)
                
                prevTheta2 = theta2
                prevAcc= out[1]
                prevVel = vel
                prevW = -out[5]
                
                lPos = leftMotor.getPosition()
                rPos = rightMotor.getPosition()
                lDelta = lStart + angleDelta - lPos
                rDelta = rStart + angleDelta - rPos
                time.sleep(delay)
            leftMotor.setPosition(lStart + angleDelta)
            rightMotor.setPosition(rStart + angleDelta)
            waitTime = .48
            
            #waitTime = califDrive.driveDistance(command[1])
        elif command[0] == "turn":
            waitTime = califDrive.turnAngle(command[1])
        else:
            print("Shit yourself *dies*")
        start = time.time()
        print("Executing command:",command)
        while time.time() < start + waitTime:
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
            
            #Calculating position with encoder data
            L = leftMotor.getPosition()
            R = rightMotor.getPosition()
            deltaL = wheelDia * (L - prevL) * math.pi / 360
            deltaR = wheelDia * (R - prevR) * math.pi / 360
            
            deltaTheta = math.atan((deltaL - deltaR)/wheelBase) / 2
            theta += deltaTheta
            dx = avg(deltaL,deltaR)
            x += dx
            pos = pos.add(angleVector(avg(theta,prevTheta)).multiply(dx))
            posList.append(pos)
            
            prevTheta = theta
            prevL = L
            prevR = R
            
            #Calculating position with imu data
            vel = prevVel + avg(out[1], prevAcc) * dTime
            dx = avg(vel, prevVel) * dTime
            
            x2 += dx
            theta2 = prevTheta2 + avg(-out[5], prevW) * dTime
            pos2 = pos2.add(angleVector(avg(theta2,prevTheta2)*math.pi/180).multiply(dx))
            
            posList2.append(pos2)
            
            prevTheta2 = theta2
            prevAcc= out[1]
            prevVel = vel
            prevW = -out[5]
            
            #pos = pos.add(angleVector(avg(theta2,prevTheta2)*math.pi/180).multiply(dx))
            #posList.append(pos)
            
            time.sleep(delay)
        i += 1
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()
with open("Path_Encoder_Data.csv", 'w') as dataFile:
    dataFile.write("x,y")
    for pose in posList:
        dataFile.write("\n"+str(pose.x)+","+str(pose.y))
with open("Path_IMU_Data.csv", 'w') as dataFile:
    dataFile.write("x,y")
    for pose in posList2:
        dataFile.write("\n"+str(pose.x)+","+str(pose.y))
print("\nEncoder Data:")
print("Pos:", pos)
print("Heading:", theta)
print("\nIMU Data:")
print("Pos:", pos2)
print("Heading:", theta2)
pMap = pathMap.PathMap(posList, 16, 10, .1)
pathMap.DisplayMap(pMap)