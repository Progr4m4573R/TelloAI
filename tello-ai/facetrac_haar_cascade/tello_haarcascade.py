#Source: https://www.youtube.com/watch?v=P2wl3N2JW9c
from utils import *
import cv2
import time

#import scripts from different directories: https://www.codegrepper.com/code-examples/python/python+how+to+include+script+from+different+directory
import sys
sys.path.insert(1,'/home/thinkpad/Desktop/Tello_Drone_Control/tello-ai/')
#import keyboard module
import keyboardControl as kbc
#initialise keyboard control
kbc.init()

#Configurations------------------------------ 
tello_fly = False
keyboard_control = True
facetracking = True

#set frame size for processing-----------------
w,h = 360,240
pid = [0.5,0.5,0]#change these values to make the movement smoother Proportional Intergral Derivitate

#safe distance
safe_distance = [6200,6800] #smaller values mean drone is closer about 2 metres equivalent

#Actual error
pErrorLR = 0
pErrorUD = 0
#initialise tello---------------------------
tello = initialisetello()

while True:

    ##initial step
    if tello_fly == True:
        tello.takeoff()
        tello_fly = False
    
    ##Step 1 get frame from drone cam
    img = telloGetFrame(tello,w,h)

    #step 2 find face in frame with viola jones haar cascade method
    img,info = findface(img)
    #print("Centre:", info[0], "Face Area:", info[1])

    #step 3 track the face with PID
    #error in x ais, y axis and front back
    pErrorLR, pErrorUD = trackface(tello,info,w,h,pid,pErrorLR,pErrorUD,safe_distance,facetracking)
    cv2.imshow("TelloHAAR",img)
    
    #step 4 optional------control drone with keyboard, set to true to enable keyboard control
    vals = kbc.action(keyboard_control)
    tello.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    time.sleep(0.05)
    #print("lr,fb,ud,yv commands from keyboard: ",vals)
    
    #step 5 optional------control drone with ble devices i.e phones and watches
    print("Battery at: ",tello.get_battery(),"%")
    if cv2.waitKey(1) & kbc.getKey("c"):
        print("Communication Terminated....")
        break
        

