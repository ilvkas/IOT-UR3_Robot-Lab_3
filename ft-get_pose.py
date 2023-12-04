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


robConnect()
''' #activates gripper. only needed once per power cycle
rob.send_program(rq_activate())
time.sleep(2.5)
#sets speed of gripper to max
rob.send_program(rq_set_speed(250))
time.sleep(0.1)
#sets force of gripper to a low value
rob.send_program(rq_set_force(10))
time.sleep(0.1) '''
#sets robot tcp, the distance from robot flange to gripper tips. 
rob.set_tcp((0,0,0.16,0,0,0))

trans = rob.get_pose()  # get current transformation matrix (tool to base)
with open("output.txt", "a") as f:
  print(trans, file=f)
print("done")