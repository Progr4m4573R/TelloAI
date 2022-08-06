#Source: https://www.youtube.com/watch?v=P2wl3N2JW9c
from djitellopy import Tello
import numpy as np
import cv2


def initialisetello():
    tello = Tello()

    tello.connect()
    #set velocities to zero
    tello.for_back_velocity = 0
    tello.left_right_velocity = 0
    tello.up_down_velocity = 0
    tello.yaw_velocity = 0 
    tello.speed = 0

    print("Tello battery at ",tello.get_battery(),"%")
    tello.streamoff()
    tello.streamon()
    return tello

def telloGetFrame(tello, w, h):
    tello_cam = tello.get_frame_read()
    tello_cam = tello_cam.frame
    img = cv2.resize(tello_cam,(w,h))
    return img
    #viola-jones method to detect faces
    #https://pyimagesearch.com/2021/04/12/opencv-haar-cascades/

def findface(img):
    facecascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    faces = facecascade.detectMultiScale(image=imgGray, scaleFactor=1.2, minNeighbors=8)#scale and minimum factor
    
    facecoords = []
    facecoordsArea = []

    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
        cx = x + w//2 
        cy = y + h //2
        area = w*h

        #cv2.circle(img,(cx,cy),5,(0,255,0),cv2.FILLED)

        facecoordsArea.append(area)
        facecoords.append([cx,cy])

    if len(facecoords) != 0:
        i = facecoordsArea.index(max(facecoordsArea))
        return img, [facecoords[i],facecoordsArea[i]]
    else:
        return img,[[0,0],0]

def trackface(tello, info,w=360,h=240,pid= [0.5,0.5,0],pErrorLR=0,pErrorUD=0,safe_distance = [6200,6800],face_tracking=True):
    if face_tracking == True:

        #---------------------------------PID control------------------------------------------------

        x,y = info[0]
        area = info[1]
        speedFB = 0
        #PID controller for left and right----------------------------------------------------------

        #subtract object detected distance from screen centre to find the difference for correction
        errorLR  = x - w//2
        speedLR =  pid[0] *errorLR + pid[1] * (errorLR-pErrorLR)
        speedLR = int(np.clip(speedLR,-100,100))#constraints the values for yaw between -100 and 100 where 0 is do nothing
        
        #PID for up and down--------------------------------------------------------------------
        errorUD = y - h//2
        speedUD = pid[0] *errorUD + pid[1] * (errorUD-pErrorUD)
        speedUD = int(np.clip(speedUD,-100,100))
        
        if(speedUD>0):
            print("Drone moving down...")
        elif(speedUD<0):
            print("Drone moving up...")
        #PID for forwards and backwards---------------------------------------------------------

        if area > safe_distance[0] and area < safe_distance[1]:
            speedFB = 0
        #move backwards
        elif area > safe_distance[1]:
            speedFB = -40
        #move forwards
        elif area < safe_distance[0] and area != 0:
            speedFB = 40

        #---------------------------------PID control------------------------------------------------

        #print("Left, Right PID correction",speedLR)
        #print("Up, Down PID correction", speedUD)
        #print("Forward, Back PID correction", speedFB)

        #check if face detected in frame
        if x != 0:
            #tello.yaw_velocity = speedLR
            tello.up_down_velocity = speedUD
            tello.left_right_velocity = speedLR
            tello.for_back_velocity = speedFB
        else:
            tello.for_back_velocity = 0
            tello.left_right_velocity = 0
            tello.up_down_velocity = 0
            tello.yaw_velocity = 0
            errorLR = 0
            errorUD = 0
            speedFB = 0
        
        tello.send_rc_control(tello.left_right_velocity,
        tello.for_back_velocity,
        tello.up_down_velocity,
        tello.yaw_velocity)

        return [errorLR,errorUD]
    else:
        pass
