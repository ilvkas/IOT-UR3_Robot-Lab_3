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
CubeHomeCount = 0
CylinderHomeCount = 0


#positions x, y, z, rx, ry, rz
clearCamera1 = 0.25, -0.22, 0.20, 0, 3.14, 0
clearCamera2 = 0.25, -0.22, 0.20, 0, 3.14, 0

#We have to change all the coordonates
placeObjectConveyor_r1 = 0.05, 0.4, 0.20, 0, 3.14, 0
placeObjectConveyorDown_r1 = 0.05, 0.4, 0.010, 0, 3.14, 0
placeObjectHome_r1 = -0.3, -0.3, 0.15, 0, 3.14, 0
placeObjectHomeDown_r1 = -0.3, -0.3, 0.005, 0, 3.14, 0
pickObjectConveyor_r1 = -0.3, 1.05, 0.15, 0, 3.14, 0
pickObjectConveyorDown_r1 = -0.3, 1.05, 0.15, 0, 3.14, 0
transitionHomePos_r1 = 0.0, -0.3, 0.20, 0, 3.14, 0 
transitionConvPos_r1 = -0.25, -0.22, 0.20, 0, 3.14, 0

placeObjectConveyor_r2 = 0.3, -0.25, 0.15, 0, 3.14, 0
placeObjectConveyorDown_r2 = 0.3, -0.25, 0.005, 0, 3.14, 0
placeObjectHome_r2 = 0.3, -0.25, 0.20, 0, 3.14, 0
placeObjectHomeDown_r2 = 0.3, -0.25, 0.010, 0, 3.14, 0
pickObjectConveyor_r2 = -0.3, 1.05, 0.15, 0, 3.14, 0
pickObjectConveyorDown_r2 = -0.3, 1.05, 0.15, 0, 3.14, 0

gamma = 0
omega = 0

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


#change port[n] to change sensor. 1 is closest to the door, 4 is furthest away from the door
def sensors(sensor):
    r = requests.post('http://158.39.162.193', json={"code":"request","cid":1,"adr":"/getdatamulti","data":{"datatosend":["/iolinkmaster/port[",sensor,"]/iolinkdevice/pdin"]}})
    res = r.json()
    res1 = res['data']
    data = str(res1)
    if data[53] == "2":
        d = data[68]+data[69]
        p = int(d,16)
    else:
        p = ("out of range")
    return(p)


#Uses camera to locate objects
def locateObject(object, camera1, camera2):
    global x, y, objectLocated, result
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
            result=1
        else:
            print("Object number ",object," not detected")
            result=0
    else:
        print("No object detected")
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
    if locateObject(3,cam11,cam12) == 1:
        global x, y, placeObjectConveyor_r1, placeObjectConveyorDown_r1
        overPickPos = x, y, 0.1, 0.0, 3.14, 0.0
        pickPos = x, y, 0.005, 0.0, 3.14, 0.0
        print(pickPos)
        rob.send_program(rq_open())
        time.sleep(0.1)
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
        time.sleep(0.2)
    else:
        print("No cylinder to put on conveyor")


#Transition cylinder on conveyor to home (T3)
def CylinderConveyorToHome():
    global pickObjectConveyor_r2, pickObjectConveyorDown_r2, placeObjectHome_r2, placeObjectHomeDown_r2, objectCount, CylinderHomeCount, gamma, omega
    if CylinderHomeCount == 0:
        placeObjectHome_r2 = ()
        placeObjectHomeDown_r2 = ()
    elif CylinderHomeCount % 2 == 0:
        placeObjectHome_r2 = ()
        placeObjectHomeDown_r2 = ()
    elif CylinderHomeCount % 2 == 1:
        placeObjectHome_r2 = ()
        placeObjectHomeDown_r2 = ()
    elif CylinderHomeCount % 6 == 0:
        placeObjectHome_r2 = ()
        placeObjectHomeDown_r2 = ()
        CylinderHomeCount = 0
    rob2.send_program(rq_open())
    time.sleep(0.1)
    move(rob2, pickObjectConveyor_r2, True)
    move(rob2, pickObjectConveyorDown_r2, True)
    #closes gripper
    rob2.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob2, pickObjectConveyor_r2, True)
    move(rob2, placeObjectHome_r2, True)
    move(rob2, placeObjectHomeDown_r2, True)
    rob2.send_program(rq_open())
    time.sleep(0.6)
    move(rob2, placeObjectHome_r2, True)
    time.sleep(0.2)
    objectCount += 1
    CylinderHomeCount += 1


#Transition cube to conveyor (T4)
def CubeToConveyor():
    if locateObject(2,cam21,cam22) == 1:
        global x, y, placeObjectConveyor_r2, placeObjectConveyorDown_r2
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
        move(rob2, placeObjectConveyor_r2, True)
        move(rob2, placeObjectConveyorDown_r2, True)
        rob2.send_program(rq_open())
        time.sleep(0.6)
        move(rob2, placeObjectConveyor_r2, True)
        time.sleep(0.2)
    else:
        print("No cube to put on conveyor")


#Transition cube on conveyor to home (T6)
def CubeConveyorToHome():
    global pickObjectConveyor_r1, pickObjectConveyorDown_r1, placeObjectHome_r1, placeObjectHomeDown_r1, objectCount, CubeHomeCount, gamma, omega
    if CubeHomeCount == 0:
        placeObjectHome_r1 = ()
        placeObjectHomeDown_r1 = ()
    elif CubeHomeCount % 2 == 0:
        placeObjectHome_r1 = ()
        placeObjectHomeDown_r1 = ()
    elif CubeHomeCount % 2 == 1:
        placeObjectHome_r1 = ()
        placeObjectHomeDown_r1 = ()
    elif CubeHomeCount % 6 == 0:
        placeObjectHome_r1 = ()
        placeObjectHomeDown_r1 = ()
        CubeHomeCount = 0
    rob.send_program(rq_open())
    time.sleep(0.1)
    move(rob, pickObjectConveyor_r1, True)
    move(rob, pickObjectConveyorDown_r1, True)
    #closes gripper
    rob.send_program(rq_close())
    #sleep to allow gripper to close fully before program resumes
    time.sleep(0.6)
    move(rob, pickObjectConveyor_r1, True)
    move(rob, placeObjectHome_r1, True)
    move(rob, placeObjectHomeDown_r1, True)
    rob.send_program(rq_open())
    time.sleep(0.6)
    move(rob, placeObjectHome_r1, True)
    time.sleep(0.2)
    objectCount += 1
    CubeHomeCount += 1


#Transition cube to home (T7)
def CubeToHome():
    if locateObject(2,cam11,cam12) == 1:
        global x, y, placeObjectHome_r1, placeObjectHomeDown_r1, objectCount, CubeHomeCount, gamma, omega
        if CubeHomeCount == 0:
            placeObjectHome_r1 = ()
            placeObjectHomeDown_r1 = ()
        elif CubeHomeCount % 2 == 0:
            placeObjectHome_r1 = ()
            placeObjectHomeDown_r1 = ()
        elif CubeHomeCount % 2 == 1:
            placeObjectHome_r1 = ()
            placeObjectHomeDown_r1 = ()
        elif CubeHomeCount % 6 == 0:
            placeObjectHome_r1 = ()
            placeObjectHomeDown_r1 = ()
            CubeHomeCount = 0
        overPickPos = x, y, 0.1, 0.0, 3.14, 0.0
        pickPos = x, y, 0.005, 0.0, 3.14, 0.0
        print(pickPos)
        rob.send_program(rq_open())
        time.sleep(0.1)
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
        time.sleep(0.2)
        objectCount += 1
        CubeHomeCount += 1
    else:
        print("No cube to place at home")

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

#Transition cylinder to home (T8)
def CylinderToHome():
    if locateObject(3,cam21,cam22) == 1:
        global x, y, placeObjectHome_r2, placeObjectHomeDown_r2, objectCount, CylinderHomeCount, gamma, omega
        if CylinderHomeCount == 0:
            placeObjectHome_r2 = ()
            placeObjectHomeDown_r2 = ()
        elif CylinderHomeCount % 2 == 0:
            placeObjectHome_r2 = ()
            placeObjectHomeDown_r2 = ()
        elif CylinderHomeCount % 2 == 1:
            placeObjectHome_r2 = ()
            placeObjectHomeDown_r2 = ()
        elif CylinderHomeCount % 6 == 0:
            placeObjectHome_r2 = ()
            placeObjectHomeDown_r2 = ()
            CylinderHomeCount = 0
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
        CylinderHomeCount
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
rob.send_program(rq_set_force(10))
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
rob2.send_program(rq_set_force(10))
time.sleep(0.1)
#sets robot tcp, the distance from robot flange to gripper tips. 
rob2.set_tcp((0,0,0.16,0,0,0))

move(rob, clearCamera1, True)
move(rob2, clearCamera2, True)
setConveyorSpeed(0.03)

while locateObject(3,cam11,cam12) == 1:
    CylinderConveyorCount += 1
    startConveyor()
    while locateObject(3,cam21,cam22) == 1:
        Thread(target=CylinderToConveyor()).start()
        Thread(target=CylinderToHome()).start()
    CylinderToConveyor()