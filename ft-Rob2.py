import urx
from Gripper import *
import urllib.request
import time
from threading import Thread
import sys
import requests

#this is the right robot
r2="158.39.162.151"

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



#positions x, y, z, rx, ry, rz
clearCamera = 0.25, -0.22, 0.20, 0, 3.14, 0


#connects to robot
def robConnect():
    global rob2
    while True:
        try:
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


robConnect()
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

move(rob2, clearCamera, False)


