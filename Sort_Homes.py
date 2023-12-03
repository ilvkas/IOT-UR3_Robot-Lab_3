import urx
from Gripper import *
import urllib.request
import time
from threading import Thread
import sys
import requests

#set robot ip adress
#this is the left robot
r1="158.39.162.177"
#this is the right robot
r2="158.39.162.151"

cam11='http://158.39.162.190/CmdChannel?TRIG'
cam12='http://158.39.162.190/CmdChannel?gRES'
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
objectCount = 0
CylinderConveyorCount = 0
CylinderSensorCount = 0
CubeConveyorCount = 0
CubeSensorCount = 0


#positions x, y, z, rx, ry, rz
clearCamera = 0.25, -0.22, 0.20, 0, 3.14, 0

#We have to change all the coordinates
placeObjectConveyor_r1 = -0.3, 1.05, 0.15, 0, 3.14, 0
placeObjectConveyorDown_r1 = -0.3, 1.05, 0.005, 0, 3.14, 0
placeObjectHome_r1 = 0.3, -0.15, 0.15, 0, 3.14, 0
placeObjectHomeDown_r1 = 0.3, -0.15, 0.005, 0, 3.14, 0
pickObjectConveyor_r1 = -0.3, 1.05, 0.15, 0, 3.14, 0
pickObjectConveyorDown_r1 = -0.3, 1.05, 0.15, 0, 3.14, 0
transitionPos_r1 = 0.0, -0.3, 0.20, 0, 3.14, 0 

placeObjectConveyor_r2 = 0.3, -0.25, 0.15, 0, 3.14, 0
placeObjectConveyorDown_r2 = 0.3, -0.25, 0.005, 0, 3.14, 0
placeObjectHome_r2 = 0.25, -0.15, 0.20, 0, 3.14, 0
placeObjectHomeDown_r2 = 0.25, -0.15, 0.010, 0, 3.14, 0
pickObjectConveyor_r2 = -0.3, 1.05, 0.15, 0, 3.14, 0
pickObjectConveyorDown_r2 = -0.3, 1.05, 0.15, 0, 3.14, 0

gamma = 0

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
def locateObject(object, camera1, camera2):
    global x, y, objectLocated
    x = 0
    y = 0
    page = urllib.request.urlopen(camera1)
    time.sleep(2)
    page = urllib.request.urlopen(camera2)
    #reads output from camera
    coords = page.read().decode('utf-8')
    #splits output
    x1 = coords.split(",")
    objectLocated = int(x1[2])
    if objectLocated == 1:
        if int(x1[1])==object:
            y = x1[4]
            x = x1[3]
            y = float(y)
            x = float(x)
            x = (x - 0) /1000
            y = (y - 0) /1000
            time.sleep(3)
            print(x, y)
        else:
            print("Object number ",object," not detected")

def modify_object_coordinates(Object, index, gamma=None, omega=None):
    if (gamma is None and omega is None) or (gamma is not None and omega is not None):
        raise ValueError("Either gamma or omega should be provided, but not both")
    adjustment_value = gamma if gamma is not None else omega
    if index < 0 or index >= len(Object):
        raise ValueError("Index out of range")
    Object = list(Object)
    Object[index] -= adjustment_value
    Object = tuple(Object)
    gamma = 0
    omega = 0
    return Object

#Transition cube to home (T7)
def CubeToHome():
    locateObject(2,cam11,cam12)
    global x, y, placeObjectHome_r1, placeObjectHomeDown_r1, objectCount, gamma
    overPickPos = x, y, 0.1, 0.0, 3.14, 0.0
    pickPos = x, y, 0.005, 0.0, 3.14, 0.0
    print(pickPos)
    rob.send_program(rq_open())
    time.sleep(0.1)
    move(rob, transitionPos_r1, True)
    move(rob, overPickPos, True)
    move(rob, pickPos, True)
    #closes gripper
    rob.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob, overPickPos, True)
    move(rob, placeObjectHome_r1, True)
    move(rob, placeObjectHomeDown_r1, True)
    rob.send_program(rq_open())
    time.sleep(0.6)
    move(rob, placeObjectHome_r1, True)
    gamma += 0.07
    #update value
    placeObjectHome_r1 = modify_object_coordinates(placeObjectHome_r1, 1, gamma)
    placeObjectHomeDown_r1 = modify_object_coordinates(placeObjectHomeDown_r1, 1, gamma)
    #placeObjectHome_r1 = 0.3, -0.15-gamma, 0.15, 0, 3.14, 0
    #placeObjectHomeDown_r1 = 0.3, -0.15-gamma, 0.005, 0, 3.14, 0
    time.sleep(0.2)
    objectCount += 1


#Transition cylinder to home (T8)
def CylinderToHome():
    locateObject(3,cam21,cam22)
    global x, y, placeObjectHome_r2, placeObjectHomeDown_r2, objectCount, gamma
    overPickPos = x, y, 0.1, 0.0, 3.14, 0.0
    pickPos = x, y, 0.005, 0.0, 3.14, 0.0
    print(pickPos)
    rob2.send_program(rq_open())
    time.sleep(0.1)
    move(rob2, overPickPos, True)
    move(rob2, pickPos, True)
    #closes gripper
    rob2.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob2, overPickPos, True)
    move(rob2, placeObjectHome_r2, True)
    move(rob2, placeObjectHomeDown_r2, True)
    rob2.send_program(rq_open())
    time.sleep(0.6)
    move(rob2, placeObjectHome_r2, True)
    gamma += 0.071
    #update value
    placeObjectHome_r2 = modify_object_coordinates(placeObjectHome_r2, 1, gamma)
    placeObjectHomeDown_r2 = modify_object_coordinates(placeObjectHomeDown_r2, 1, gamma)
    gamma = 0
    time.sleep(0.2)
    objectCount += 1



robConnect()
#activates gripper. only needed once per power cycle
rob.send_program(rq_activate())
time.sleep(2.5)
#sets speed of gripper to max
rob.send_program(rq_set_speed(250))
time.sleep(0.1)
#sets force of gripper to a low value
rob.send_program(rq_set_force(10))
time.sleep(0.1)
#sets robot tcp, the distance from robot flange to gripper tips. 
rob.set_tcp((0,0,0.16,0,0,0))

rob2.send_program(rq_activate())
time.sleep(2.5)
#sets speed of gripper to max
rob2.send_program(rq_set_speed(250))
time.sleep(0.1)
#sets force of gripper to a low value
rob2.send_program(rq_set_force(10))
time.sleep(0.1)
#sets robot tcp, the distance from robot flange to gripper tips. 
rob2.set_tcp((0,0,0.16,0,0,0))

move(rob, clearCamera, True)
move(rob2, clearCamera, True)

#CubeToHome()
objectCount = 0
while objectCount < 4:
    CubeToHome()