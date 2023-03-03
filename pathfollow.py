# pathfollow.py

import time
import califDrive
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()
leftMotor = califDrive.Motor(BP, BP.PORT_A, -1, 30)
rightMotor = califDrive.Motor(BP, BP.PORT_D, -1, 30)
drive = califDrive.CalifDrive(BP, leftMotor, rightMotor)

gridWidth = .2
logging = False

with open("path.csv", 'r') as pathFile:
    pathLines = pathFile.readlines()
path = []
for line in pathLines:
    line = line.split(',')
    path.append([line[0], float(line[1])])
i = 1
zeroTime = time.time()
try:
    for command in path:
        if logging:
            outFile = open("path_test_data" + str(i) + ".csv",'w')
            outFile.write("time,leftPos,rightPos\n")
        if command[0] == "forward":
            waitTime = drive.driveDistance(command[1])
        elif command[0] == "turn":
            waitTime = drive.turnAngle(command[1])
        else:
            print("Shit yourself")
        start = time.time()
        print("Executing command:",command)
        while time.time() < start + waitTime:
            if logging:
                outFile.write("%.4f,%i,%i\n" %(time.time() - zeroTime,leftMotor.getPosition(),rightMotor.getPosition()))
            time.sleep(.02)
        if logging:
            outFile.close()
        i += 1
except KeyboardInterrupt:
    print("Program forcibly terminated")
except Exception as e:
    print(e)
finally:
    BP.reset_all()
    if logging:
        outFile.close()
