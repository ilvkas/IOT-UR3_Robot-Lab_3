import urx
from Gripper import *
import urllib.request
import time
from threading import Thread
import sys

#set robot ip adress
#this is the left robot
r1="158.39.162.177"
#this is the right robot
r2="158.39.162.151"

cam101='http://158.39.162.190/CmdChannel?sINT_1_2'
cam102='http://158.39.162.190/CmdChannel?sINT_1_3'
cam11='http://158.39.162.190/CmdChannel?TRIG'
cam12='http://158.39.162.190/CmdChannel?gRES'
cam201='http://158.39.162.199/CmdChannel?sINT_1_2'
cam202='http://158.39.162.199/CmdChannel?sINT_1_3'
cam21='http://158.39.162.199/CmdChannel?TRIG'
cam22='http://158.39.162.199/CmdChannel?gRES'

#robot velocity and acceleration
v = 0.8
a = 0.5
#these x and y values are the distance from the robot base to the middle of the camera
x = 0
y = 0
lasty = y
lastx = x
objectLocated = 0
Count = 0
objectCount = 0
CylinderConveyorCount = 0
CylinderSensorCount = 0
CubeConveyorCount = 0
CubeSensorCount = 0
CubeHomeCount = 0
CylinderHomeCount = 0
CylinderConveyorCount = 0
CylinderConveyorPlaced = 0
CubeConveyorCount = 0
CubeConveyorPlaced = 0


#positions x, y, z, rx, ry, rz
clearCamera_r1 = -0.25, 0.0, 0.20, 0, 3.14, 0
clearCamera_r2 = 0.25, 0.0, 0.20, 0, 3.14, 0

#We have to change all the coordonates
placeObjectHome_r1 = 0.3, -0.15, 0.15, 0, 3.14, 0
placeObjectHomeDown_r1 = 0.3, -0.15, 0.005, 0, 3.14, 0

placeObjectConveyor_r1 = 0.05, 0.4, 0.20, 0, 3.14, 0
placeObjectConveyorDown_r1 = 0.05, 0.4, 0.010, 0, 3.14, 0

pickObjectConveyor_r1 = 0.05, 0.4, 0.20, 0, 3.14, 0
pickObjectConveyorDown_r1 = 0.05, 0.4, 0.010, 0, 3.14, 0

transitionHomePos_r1 = 0.0, -0.3, 0.20, 0, 3.14, 0 
transitionConvPos_r1 = -0.25, -0.22, 0.20, 0, 3.14, 0

HomePosition1_r1 = 0.35, -0.13, 0.15, 0, 3.14, 0
HomePositionDown1_r1 = 0.35, -0.13, 0.03, 0, 3.14, 0
HomePosition2_r1 = 0.25, -0.13, 0.15, 0, 3.14, 0
HomePositionDown2_r1 = 0.25, -0.13, 0.03, 0, 3.14, 0
HomePosition3_r1 = 0.35, -0.20, 0.15, 0, 3.14, 0
HomePositionDown3_r1 = 0.35, -0.20, 0.03, 0, 3.14, 0
HomePosition4_r1 = 0.25, -0.20, 0.15, 0, 3.14, 0
HomePositionDown4_r1 = 0.25, -0.20, 0.03, 0, 3.14, 0
HomePosition5_r1 = 0.35, -0.27, 0.15, 0, 3.14, 0
HomePositionDown5_r1 = 0.35, -0.27, 0.03, 0, 3.14, 0
HomePosition6_r1 = 0.25, -0.27, 0.15, 0, 3.14, 0
HomePositionDown6_r1 = 0.25, -0.27, 0.03, 0, 3.14, 0

ConveyorPosition1_r1 = 0.091, 0.334, 0.15, 0, 3.14, 0
ConveyorPositionDown1_r1 = 0.091, 0.334, 0, 0, 3.14, 0
ConveyorPosition2_r1 = -0.009, 0.334, 0.15, 0, 3.14, 0
ConveyorPositionDown2_r1 = -0.009, 0.334, 0, 0, 3.14, 0
ConveyorPosition3_r1 = 0.091, 0.434, 0.15, 0, 3.14, 0
ConveyorPositionDown3_r1 = 0.091, 0.434, 0, 0, 3.14, 0
ConveyorPosition4_r1 = -0.009, 0.434, 0.15, 0, 3.14, 0
ConveyorPositionDown4_r1 = -0.009, 0.434, 0, 0, 3.14, 0
ConveyorPosition5_r1 = -0.109, 0.334, 0.15, 0, 3.14, 0
ConveyorPositionDown5_r1 = -0.109, 0.334, 0, 0, 3.14, 0
ConveyorPosition6_r1 = -0.109, 0.434, 0.15, 0, 3.14, 0
ConveyorPositionDown6_r1 = -0.109, 0.434, 0, 0, 3.14, 0

ConveyorPickPosition1_r1 = 0.088, 0.334, 0.15, 0, 3.14, 0
ConveyorPickPositionDown1_r1 = 0.088, 0.334, 0, 0, 3.14, 0
ConveyorPickPosition2_r1 = -0.012, 0.334, 0.15, 0, 3.14, 0
ConveyorPickPositionDown2_r1 = -0.012, 0.334, 0, 0, 3.14, 0
ConveyorPickPosition3_r1 = 0.088, 0.434, 0.15, 0, 3.14, 0
ConveyorPickPositionDown3_r1 = 0.088, 0.434, 0, 0, 3.14, 0
ConveyorPickPosition4_r1 = -0.012, 0.434, 0.15, 0, 3.14, 0
ConveyorPickPositionDown4_r1 = -0.012, 0.434, 0, 0, 3.14, 0
ConveyorPickPosition5_r1 = -0.112, 0.334, 0.15, 0, 3.14, 0
ConveyorPickPositionDown5_r1 = -0.112, 0.334, 0, 0, 3.14, 0
ConveyorPickPosition6_r1 = -0.112, 0.434, 0.15, 0, 3.14, 0
ConveyorPickPositionDown6_r1 = -0.112, 0.434, 0, 0, 3.14, 0


placeObjectHome_r2 = 0.25, -0.15, 0.20, 0, 3.14, 0
placeObjectHomeDown_r2 = 0.25, -0.15, 0.010, 0, 3.14, 0

placeObjectConveyor_r2 = 0.05, 0.4, 0.20, 0, 3.14, 0
placeObjectConveyorDown_r2 = 0.05, 0.4, 0.010, 0, 3.14, 0

pickObjectConveyor_r2 = 0.05, 0.4, 0.20, 0, 3.14, 0
pickObjectConveyorDown_r2 = 0.05, 0.4, 0.010, 0, 3.14, 0

transitionHomePos_r2 = 0.0, -0.3, 0.22, 0, 3.14, 0 
transitionConvPos_r2 = -0.25, -0.22, 0.22, 0, 3.14, 0
transitionConvPos2_r2 = -0.25, 0.21, 0.22, 0, 3.14, 0

HomePosition1_r2 = -0.26, -0.11, 0.22, 0, 3.14, 0
HomePositionDown1_r2 = -0.26, -0.11, 0.025, 0, 3.14, 0
HomePosition2_r2 = -0.26, -0.185, 0.22, 0, 3.14, 0
HomePositionDown2_r2 = -0.26, -0.185, 0.025, 0, 3.14, 0
HomePosition3_r2 = -0.26, -0.26, 0.22, 0, 3.14, 0
HomePositionDown3_r2 = -0.26, -0.26, 0.025, 0, 3.14, 0
HomePosition4_r2 = -0.26, -0.11, 0.22, 0, 3.14, 0
HomePositionDown4_r2 = -0.26, -0.11, 0.1, 0, 3.14, 0
HomePosition5_r2 = -0.26, -0.185, 0.22, 0, 3.14, 0
HomePositionDown5_r2 = -0.26, -0.185, 0.1, 0, 3.14, 0
HomePosition6_r2 = -0.26, -0.26, 0.22, 0, 3.14, 0
HomePositionDown6_r2 = -0.26, -0.26, 0.1, 0, 3.14, 0

ConveyorPosition1_r2 = 0.006, 0.246, 0.15, 0, 3.14, 0
ConveyorPositionDown1_r2 = 0.006, 0.246, 0, 0, 3.14, 0
ConveyorPosition2_r2 = -0.094, 0.246, 0.15, 0, 3.14, 0
ConveyorPositionDown2_r2 = -0.094, 0.246, 0, 0, 3.14, 0
ConveyorPosition3_r2 = 0.006, 0.346, 0.15, 0, 3.14, 0
ConveyorPositionDown3_r2 = 0.006, 0.346, 0, 0, 3.14, 0
ConveyorPosition4_r2 = -0.094, 0.346, 0.15, 0, 3.14, 0
ConveyorPositionDown4_r2 = -0.094, 0.346, 0, 0, 3.14, 0
ConveyorPosition5_r2 = -0.194, 0.246, 0.15, 0, 3.14, 0
ConveyorPositionDown5_r2 = -0.194, 0.246, 0, 0, 3.14, 0
ConveyorPosition6_r2 = -0.194, 0.346, 0.15, 0, 3.14, 0
ConveyorPositionDown6_r2 = -0.194, 0.346, 0, 0, 3.14, 0

ConveyorPickPosition1_r2 = 0.01, 0.246, 0.15, 0, 3.14, 0
ConveyorPickPositionDown1_r2 = 0.01, 0.246, 0, 0, 3.14, 0
ConveyorPickPosition2_r2 = -0.09, 0.246, 0.15, 0, 3.14, 0
ConveyorPickPositionDown2_r2 = -0.09, 0.246, 0, 0, 3.14, 0
ConveyorPickPosition3_r2 = 0.01, 0.346, 0.15, 0, 3.14, 0
ConveyorPickPositionDown3_r2 = 0.01, 0.346, 0, 0, 3.14, 0
ConveyorPickPosition4_r2 = -0.09, 0.346, 0.15, 0, 3.14, 0
ConveyorPickPositionDown4_r2 = -0.09, 0.346, 0, 0, 3.14, 0
ConveyorPickPosition5_r2 = -0.19, 0.246, 0.15, 0, 3.14, 0
ConveyorPickPositionDown5_r2 = -0.19, 0.246, 0, 0, 3.14, 0
ConveyorPickPosition6_r2 = -0.19, 0.346, 0.15, 0, 3.14, 0
ConveyorPickPositionDown6_r2 = -0.19, 0.346, 0, 0, 3.14, 0

#connects to robot
def robConnect():
    global rob, rob2
    while True:
        try:
            rob = urx.Robot(r1, use_rt=True, urFirm=5.1)
            rob2 = urx.Robot(r2, use_rt=True, urFirm=5.1)
        except:
            pass
        else:
            break
        time.sleep(3)
    time.sleep(1)


#function for moving robot using moveJ
def move(robot, location, moveWait):
    robot.movex("movej", location, acc=a, vel=v, wait=moveWait, relative=False, threshold=None)
    if moveWait == False:
        time.sleep(0.1)


#Uses camera to locate objects
def locateObject_r1(object, camera0, camera1, camera2):
    global x_r1, y_r1, result
    x_r1 = 0
    y_r1 = 0
    page = urllib.request.urlopen(camera0)
    time.sleep(1)
    page = urllib.request.urlopen(camera1)
    time.sleep(1)
    page = urllib.request.urlopen(camera2)
    #reads output from camera
    coords = page.read().decode('utf-8')
    #splits output
    x1 = coords.split(",")
    objectLocated = int(x1[2])
    if objectLocated == 1:
        if int(x1[1])==object:
            y_r1 = x1[4]
            x_r1 = x1[3]
            y_r1 = float(y_r1)
            x_r1 = float(x_r1)
            x_r1 = x_r1*0.95
            x_r1 = x_r1  /1000
            y_r1 = y_r1  /1000
            time.sleep(1)
            print("R1: " + str(x_r1), str(y_r1))
            result=1
        else:
            print("R1 No object " ,object, " detected")
            result=0
    else:
        print("R1 no object detected")
        result=0
    return(result)

def locateObject_r2(object, camera0, camera1, camera2):
    global x_r2, y_r2, result
    x_r2 = 0
    y_r2 = 0
    page = urllib.request.urlopen(camera0)
    time.sleep(1)
    page = urllib.request.urlopen(camera1)
    time.sleep(1)
    page = urllib.request.urlopen(camera2)
    #reads output from camera
    coords = page.read().decode('utf-8')
    #splits output
    x1 = coords.split(",")
    objectLocated = int(x1[2])
    if objectLocated == 1:
        if int(x1[1])==object:
            y_r2 = x1[4]
            x_r2 = x1[3]
            y_r2 = float(y_r2)
            x_r2 = float(x_r2)
            x_r2 = x_r2*0.95
            x_r2 = x_r2  /1000
            y_r2 = y_r2 /1000
            time.sleep(1)
            print("R2: " + str(x_r2), str(y_r2))
            result=1
        else:
            print("R2 No object " +object+ " detected")
            result=0
    else:
        print("R2 no object detected")
        result=0
    return(result)


#Transitions for Conveyor (T2 and T5)
def startConveyor():
    #start coveyor
    rob2.set_digital_out(5, 1)
    #allow digital out 5 to stay active for 0.1s
    time.sleep(0.1)
    #set digital out back to 0
    rob2.set_digital_out(5, 0)
    #conveyor started

def stopConveyor():
    #stop conveyor
    rob2.set_digital_out(7, 1)
    #allow digital out 7 to stay active for 0.1s
    time.sleep(0.1)
    #set digital out back to 0
    rob2.set_digital_out(7, 0)
    #conveyor stopped

def reverseConveyor():
    #start coveyor in reverse direction
    rob2.set_digital_out(6, 1)
    #allow digital out 6 to stay active for 0.1s
    time.sleep(0.1)
    #set digital out back to 0
    rob2.set_digital_out(6, 0)
    #conveyor started in reverse direction

def setConveyorSpeed(voltage):
    #sets analog out to voltage instead of current
    rob2.send_program("set_analog_outputdomain(1, 1)")
    #sets analog out 1 to desired voltage. 0.012 is the slowest speed.
    rob2.set_analog_out(1, voltage)


#Transition cylinder to conveyor (T1)
def CylinderToConveyor():
    if locateObject_r1(3,cam102,cam11,cam12) == 1:
        global x_r1, y_r1, placeObjectConveyor_r1, placeObjectConveyorDown_r1, CylinderConveyorCount, CylinderConveyorPlaced

        if CylinderConveyorCount == 1:
            placeObjectConveyor_r1 = ConveyorPosition1_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown1_r1
        elif CylinderConveyorCount == 2:
            placeObjectConveyor_r1 = ConveyorPosition2_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown2_r1
        elif CylinderConveyorCount == 3:
            placeObjectConveyor_r1 = ConveyorPosition3_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown3_r1
        elif CylinderConveyorCount == 4:
            placeObjectConveyor_r1 = ConveyorPosition4_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown4_r1
        elif CylinderConveyorCount == 5:
            placeObjectConveyor_r1 = ConveyorPosition5_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown5_r1
        elif CylinderConveyorCount == 6:
            placeObjectConveyor_r1 = ConveyorPosition6_r1
            placeObjectConveyorDown_r1 = ConveyorPositionDown6_r1

        overPickPos = x_r1, y_r1, 0.1, 0.0, 3.14, 0.0
        pickPos = x_r1, y_r1, 0.02, 0.0, 3.14, 0.0
        print(pickPos)
        rob.send_program(rq_open())
        time.sleep(0.1)
        move(rob, transitionHomePos_r1, True)
        move(rob, overPickPos, True)
        move(rob, pickPos, True)
        #closes gripper
        rob.send_program(rq_close())
        #sleep to allow gripper to close fully before program resumes
        time.sleep(0.6)
        move(rob, overPickPos, True)
        move(rob, transitionConvPos_r1, True)
        move(rob, placeObjectConveyor_r1, True)
        move(rob, placeObjectConveyorDown_r1, True)
        rob.send_program(rq_open())
        time.sleep(0.6)
        move(rob, placeObjectConveyor_r1, True)
        move(rob, clearCamera_r1, True)
        time.sleep(0.2)
        CylinderConveyorPlaced += 1
    else:
        print("No cylinder to put on conveyor")


#Transition cylinder on conveyor to home (T3)
def CylinderConveyorToHome():
    global pickObjectConveyor_r2, pickObjectConveyorDown_r2, placeObjectHome_r2, placeObjectHomeDown_r2, objectCount, CylinderHomeCount, CylinderConveyorCount

    if CylinderConveyorCount == 1:
        pickObjectConveyor_r2 = ConveyorPickPosition1_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown1_r2
    elif CylinderConveyorCount == 2:
        pickObjectConveyor_r2 = ConveyorPickPosition2_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown2_r2
    elif CylinderConveyorCount == 3:
        pickObjectConveyor_r2 = ConveyorPickPosition3_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown3_r2
    elif CylinderConveyorCount == 4:
        pickObjectConveyor_r2 = ConveyorPickPosition4_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown4_r2
    elif CylinderConveyorCount == 5:
        pickObjectConveyor_r2 = ConveyorPickPosition5_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown5_r2
    elif CylinderConveyorCount == 6:
        pickObjectConveyor_r2 = ConveyorPickPosition6_r2
        pickObjectConveyorDown_r2 = ConveyorPickPositionDown6_r2

    if CylinderHomeCount%6 == 0 and CylinderHomeCount != 0 :
        CylinderHomeCount = 4

    if CylinderHomeCount == 0:
        placeObjectHome_r2 = HomePosition1_r2
        placeObjectHomeDown_r2 = HomePositionDown1_r2
    elif CylinderHomeCount == 1:
        placeObjectHome_r2 = HomePosition2_r2
        placeObjectHomeDown_r2 = HomePositionDown2_r2
    elif CylinderHomeCount == 2:
        placeObjectHome_r2 = HomePosition3_r2
        placeObjectHomeDown_r2 = HomePositionDown3_r2
    elif CylinderHomeCount == 3:
        placeObjectHome_r2 = HomePosition4_r2
        placeObjectHomeDown_r2 = HomePositionDown4_r2
    elif CylinderHomeCount == 4:
        placeObjectHome_r2 = HomePosition5_r2
        placeObjectHomeDown_r2 = HomePositionDown5_r2
    elif CylinderHomeCount == 5:
        placeObjectHome_r2 = HomePosition6_r2
        placeObjectHomeDown_r2 = HomePositionDown6_r2

    rob2.send_program(rq_open())
    time.sleep(0.1)
    move(rob2, transitionHomePos_r2, True)
    move(rob2, transitionConvPos_r2, True)
    move(rob2, transitionConvPos2_r2, True)
    move(rob2, pickObjectConveyor_r2, True)
    move(rob2, pickObjectConveyorDown_r2, True)
    #closes gripper
    rob2.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob2, pickObjectConveyor_r2, True)
    move(rob2, transitionConvPos2_r2, True)
    move(rob2, placeObjectHome_r2, True)
    move(rob2, placeObjectHomeDown_r2, True)
    rob2.send_program(rq_open())
    time.sleep(0.6)
    move(rob2, placeObjectHome_r2, True)
    move(rob2, transitionHomePos_r2, True)
    move(rob2, clearCamera_r2, True)
    time.sleep(0.2)
    objectCount += 1
    CylinderHomeCount += 1


#Transition cube to conveyor (T4)
def CubeToConveyor():
    if locateObject_r2(2,cam201,cam21,cam22) == 1:
        global x_r2, y_r2, placeObjectConveyor_r2, placeObjectConveyorDown_r2, CubeConveyorCount, CubeConveyorPlaced

        if CubeConveyorCount == 1:
            placeObjectConveyor_r2 = ConveyorPosition1_r2
            placeObjectConveyorDown_r2 = ConveyorPositionDown1_r2
        elif CubeConveyorCount == 2:
            placeObjectConveyor_r2 = ConveyorPosition2_r1
            placeObjectConveyorDown_r2 = ConveyorPositionDown2_r2
        elif CubeConveyorCount == 3:
            placeObjectConveyor_r2 = ConveyorPosition3_r2
            placeObjectConveyorDown_r2 = ConveyorPositionDown3_r2
        elif CubeConveyorCount == 4:
            placeObjectConveyor_r2 = ConveyorPosition4_r2
            placeObjectConveyorDown_r2 = ConveyorPositionDown4_r2
        elif CubeConveyorCount == 5:
            placeObjectConveyor_r2 = ConveyorPosition5_r2
            placeObjectConveyorDown_r2 = ConveyorPositionDown5_r2
        elif CubeConveyorCount == 6:
            placeObjectConveyor_r2 = ConveyorPosition6_r2
            placeObjectConveyorDown_r2 = ConveyorPositionDown6_r2

        overPickPos = x_r2, y_r2, 0.22, 0.0, 3.14, 0.0
        pickPos = x_r2, y_r2, 0.02, 0.0, 3.14, 0.0
        print(pickPos)
        rob2.send_program(rq_open())
        time.sleep(0.1)
        move(rob2, transitionHomePos_r2, True)
        move(rob2, overPickPos, True)
        move(rob2, pickPos, True)
        #closes gripper
        rob2.send_program(rq_close())
        #sleep to allow gripper to close fully before program resumes
        time.sleep(0.6)
        move(rob2, overPickPos, True)
        move(rob2, transitionConvPos_r2, True)
        move(rob2, transitionConvPos2_r2, True)
        move(rob2, placeObjectConveyor_r2, True)
        move(rob2, placeObjectConveyorDown_r2, True)
        rob2.send_program(rq_open())
        time.sleep(0.6)
        move(rob2, placeObjectConveyor_r2, True)
        move(rob2, transitionConvPos2_r2, True)
        move(rob2, transitionConvPos_r2, True)
        move(rob2, transitionHomePos_r2, True)
        move(rob2, clearCamera_r2, True)
        time.sleep(0.2)
        CubeConveyorPlaced += 1
    else:
        print("No cube to put on conveyor")


#Transition cube on conveyor to home (T6)
def CubeConveyorToHome():
    global pickObjectConveyor_r1, pickObjectConveyorDown_r1, placeObjectHome_r1, placeObjectHomeDown_r1, objectCount, CubeHomeCount, CubeConveyorCount

    if CubeConveyorCount == 1:
        pickObjectConveyor_r1 = ConveyorPickPosition1_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown1_r1
    elif CubeConveyorCount == 2:
        pickObjectConveyor_r1 = ConveyorPickPosition2_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown2_r1
    elif CubeConveyorCount == 3:
        pickObjectConveyor_r1 = ConveyorPickPosition3_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown3_r1
    elif CubeConveyorCount == 4:
        pickObjectConveyor_r1 = ConveyorPickPosition4_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown4_r1
    elif CubeConveyorCount == 5:
        pickObjectConveyor_r1 = ConveyorPickPosition5_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown5_r1
    elif CubeConveyorCount == 6:
        pickObjectConveyor_r1 = ConveyorPickPosition6_r1
        pickObjectConveyorDown_r1 = ConveyorPickPositionDown6_r1

    if CubeHomeCount%6 == 0:
        CubeHomeCount = 0

    if CubeHomeCount == 0:
        placeObjectHome_r1 = HomePosition1_r1
        placeObjectHomeDown_r1 = HomePositionDown1_r1
    elif CubeHomeCount == 1:
        placeObjectHome_r1 = HomePosition2_r1
        placeObjectHomeDown_r1 = HomePositionDown2_r1
    elif CubeHomeCount == 2:
        placeObjectHome_r1 = HomePosition3_r1
        placeObjectHomeDown_r1 = HomePositionDown3_r1
    elif CubeHomeCount == 3:
        placeObjectHome_r1 = HomePosition4_r1
        placeObjectHomeDown_r1 = HomePositionDown4_r1
    elif CubeHomeCount == 4:
        placeObjectHome_r1 = HomePosition5_r1
        placeObjectHomeDown_r1 = HomePositionDown5_r1
    elif CubeHomeCount == 5:
        placeObjectHome_r1 = HomePosition6_r1
        placeObjectHomeDown_r1 = HomePositionDown6_r1

    rob.send_program(rq_open())
    time.sleep(0.1)
    move(rob, pickObjectConveyor_r1, True)
    move(rob, pickObjectConveyorDown_r1, True)
    #closes gripper
    rob.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob, pickObjectConveyor_r1, True)
    move(rob, transitionConvPos_r1, True)
    move(rob, placeObjectHome_r1, True)
    move(rob, placeObjectHomeDown_r1, True)
    rob.send_program(rq_open())
    time.sleep(0.6)
    move(rob, placeObjectHome_r1, True)
    move(rob, transitionHomePos_r1, True)
    move(rob, clearCamera_r1, True)
    time.sleep(0.2)
    objectCount += 1
    CubeHomeCount += 1


#Transition cube to home (T7)
def CubeToHome():
    if locateObject_r1(2,cam101,cam11,cam12) == 1:
        global x_r1, y_r1, placeObjectHome_r1, placeObjectHomeDown_r1, objectCount, CubeHomeCount

        if CubeHomeCount%6 == 0:
            CubeHomeCount = 0

        if CubeHomeCount == 0:
            placeObjectHome_r1 = HomePosition1_r1
            placeObjectHomeDown_r1 = HomePositionDown1_r1
        elif CubeHomeCount == 1:
            placeObjectHome_r1 = HomePosition2_r1
            placeObjectHomeDown_r1 = HomePositionDown2_r1
        elif CubeHomeCount == 2:
            placeObjectHome_r1 = HomePosition3_r1
            placeObjectHomeDown_r1 = HomePositionDown3_r1
        elif CubeHomeCount == 3:
            placeObjectHome_r1 = HomePosition4_r1
            placeObjectHomeDown_r1 = HomePositionDown4_r1
        elif CubeHomeCount == 4:
            placeObjectHome_r1 = HomePosition5_r1
            placeObjectHomeDown_r1 = HomePositionDown5_r1
        elif CubeHomeCount == 5:
            placeObjectHome_r1 = HomePosition6_r1
            placeObjectHomeDown_r1 = HomePositionDown6_r1

        overPickPos = x_r1, y_r1, 0.1, 0.0, 3.14, 0.0
        pickPos = x_r1, y_r1, 0.02, 0.0, 3.14, 0.0
        print(pickPos)
        rob.send_program(rq_open())
        time.sleep(0.1)
        move(rob, transitionHomePos_r1, True)
        move(rob, overPickPos, True)
        move(rob, pickPos, True)
        #closes gripper
        rob.send_program(rq_close())
        #sleep to allow gripper to close fully before program resumes
        time.sleep(0.6)
        move(rob, overPickPos, True)
        move(rob, transitionHomePos_r1, True)
        move(rob, placeObjectHome_r1, True)
        move(rob, placeObjectHomeDown_r1, True)
        rob.send_program(rq_open())
        time.sleep(0.6)
        move(rob, placeObjectHome_r1, True)
        move(rob, transitionHomePos_r1, True)
        move(rob, clearCamera_r1, True)
        time.sleep(0.2)
        objectCount += 1
        CubeHomeCount += 1
    else:
        print("No cube to place at home")


#Transition cylinder to home (T8)
def CylinderToHome():
    if locateObject_r2(3,cam202,cam21,cam22) == 1:
        global x_r2, y_r2, placeObjectHome_r2, placeObjectHomeDown_r2, objectCount, CylinderHomeCount

        if CylinderHomeCount%6 == 0 and CylinderHomeCount != 0:
            CylinderHomeCount = 4

        if CylinderHomeCount == 0:
            placeObjectHome_r2 = HomePosition1_r2
            placeObjectHomeDown_r2 = HomePositionDown1_r2
        elif CylinderHomeCount == 1:
            placeObjectHome_r2 = HomePosition2_r2
            placeObjectHomeDown_r2 = HomePositionDown2_r2
        elif CylinderHomeCount == 2:
            placeObjectHome_r2 = HomePosition3_r2
            placeObjectHomeDown_r2 = HomePositionDown3_r2
        elif CylinderHomeCount == 3:
            placeObjectHome_r2 = HomePosition4_r2
            placeObjectHomeDown_r2 = HomePositionDown4_r2
        elif CylinderHomeCount == 4:
            placeObjectHome_r2 = HomePosition5_r2
            placeObjectHomeDown_r2 = HomePositionDown5_r2
        elif CylinderHomeCount == 5:
            placeObjectHome_r2 = HomePosition6_r2
            placeObjectHomeDown_r2 = HomePositionDown6_r2

        overPickPos = x_r2, y_r2, 0.22, 0.0, 3.14, 0.0
        pickPos = x_r2, y_r2, 0.02, 0.0, 3.14, 0.0
        print(pickPos)
        rob2.send_program(rq_open())
        time.sleep(0.1)
        move(rob2, transitionHomePos_r2, True)
        move(rob2, overPickPos, True)
        move(rob2, pickPos, True)
        #closes gripper
        rob2.send_program(rq_close())
        #sleep to allow gripper to close fully before program resumes
        time.sleep(0.6)
        move(rob2, overPickPos, True)
        move(rob2, transitionHomePos_r2, True)
        move(rob2, placeObjectHome_r2, True)
        move(rob2, placeObjectHomeDown_r2, True)
        rob2.send_program(rq_open())
        time.sleep(0.6)
        move(rob2, placeObjectHome_r2, True)
        move(rob2, transitionHomePos_r2, True)
        move(rob2, clearCamera_r2, True)
        time.sleep(0.2)
        objectCount += 1
        CylinderHomeCount += 1
    else:
        print("No cylinder to place at home")



robConnect()
#activates gripper. only needed once per power cycle
rob.send_program(rq_activate())
time.sleep(2.5)
#sets speed of gripper to max
rob.send_program(rq_set_speed(250))
time.sleep(0.1)
#sets force of gripper to a low value
rob.send_program(rq_set_force(20))
time.sleep(0.1)
#sets robot tcp, the distance from robot flange to gripper tips. 
rob.set_tcp((0,0,0.16,0,0,0))

#activates gripper. only needed once per power cycle
rob2.send_program(rq_activate())
time.sleep(2.5)
#sets speed of gripper to max
rob2.send_program(rq_set_speed(250))
time.sleep(0.1)
#sets force of gripper to a low value
rob2.send_program(rq_set_force(20))
time.sleep(0.1)
#sets robot tcp, the distance from robot flange to gripper tips. 
rob2.set_tcp((0,0,0.16,0,0,0))

setConveyorSpeed(0.3)

Thread_1 = Thread(target = move(rob, clearCamera_r1, True))
Thread_2 = Thread(target = move(rob2, clearCamera_r2, True))
Thread_1.start()
Thread_2.start()
Thread_1.join()
Thread_2.join()


while True : #Change this line if you want a precise number of Objects placed
    if locateObject_r1(3,cam102,cam11,cam12) == 1:
        while True :
            if locateObject_r1(3,cam102,cam11,cam12) == 0:
                break
            else:
                if locateObject_r2(3,cam202,cam21,cam22) == 1:
                    CylinderConveyorCount += 1
                    Thread_1 = Thread(target=CylinderToConveyor)
                    Thread_2 = Thread(target=CylinderToHome)
                    Thread_1.start()
                    Thread_2.start()
                    Thread_1.join()
                    Thread_2.join()
                elif locateObject_r2(3,cam202,cam21,cam22) == 0:
                    CylinderConveyorCount += 1
                    CylinderToConveyor()
        CylinderConveyorCount = 0
        startConveyor()
        time.sleep(7.5)
        stopConveyor()
        while CylinderConveyorCount < CylinderConveyorPlaced:
            if locateObject_r1(2,cam101,cam11,cam12) == 1:
                CylinderConveyorCount += 1
                Thread_1 = Thread(target=CylinderConveyorToHome)
                Thread_2 = Thread(target=CubeToHome)
                Thread_1.start()
                Thread_2.start()
                Thread_1.join()
                Thread_2.join()
            elif locateObject_r1(2,cam101,cam11,cam12) == 0:
                CylinderConveyorCount += 1
                CylinderConveyorToHome()
        CylinderConveyorCount = 0
        CylinderConveyorPlaced = 0
    

    elif locateObject_r2(2,cam201,cam21,cam22) == 1:
        while True :
            if locateObject_r2(2,cam201,cam21,cam22) == 0:
                break
            else:
                if locateObject_r1(2,cam101,cam11,cam12) == 1:
                    CubeConveyorCount += 1
                    Thread_1 = Thread(target=CubeToConveyor)
                    Thread_2 = Thread(target=CubeToHome)
                    Thread_1.start()
                    Thread_2.start()
                    Thread_1.join()
                    Thread_2.join()
                elif locateObject_r1(2,cam101,cam11,cam12) == 0:
                    CubeConveyorCount += 1
                    CubeToConveyor()
        CubeConveyorCount = 0
        reverseConveyor()
        time.sleep(7.5)
        stopConveyor()
        while CubeConveyorCount < CubeConveyorPlaced:
            if locateObject_r2(3,cam202,cam21,cam22) == 1:
                CubeConveyorCount += 1
                Thread_1 = Thread(target=CubeConveyorToHome)
                Thread_2 = Thread(target=CylinderToHome)
                Thread_1.start()
                Thread_2.start()
                Thread_1.join()
                Thread_2.join()
            elif locateObject_r2(3,cam202,cam21,cam22) == 0:
                CubeConveyorCount += 1
                CubeConveyorToHome()
        CubeConveyorCount = 0
        CubeConveyorPlaced = 0

    
    elif locateObject_r1(2,cam101,cam11,cam12) == 1 or locateObject_r2(3,cam202,cam21,cam22) == 1:
        if locateObject_r1(2,cam101,cam11,cam12) == 1 and locateObject_r2(3,cam202,cam21,cam22) == 1:
            Thread_1 = Thread(target=CubeToHome)
            Thread_2 = Thread(target=CylinderToHome)
            Thread_1.start()
            Thread_2.start()
            Thread_1.join()
            Thread_2.join()
        elif locateObject_r1(2,cam101,cam11,cam12) == 1:
            CubeToHome()
        elif locateObject_r2(3,cam202,cam21,cam22) == 1:
            CylinderToHome()
        else:
            break

    else:
        break
        
print("Program Complete")
sys.exit
