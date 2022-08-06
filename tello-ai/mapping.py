#get the speed of the drone and time taken to plot x an y in cartesian coordinates


from djitellopy import Tello
import numpy as np
import time
import cv2

from face_tracking import *
tello = initialisetello()
####---------------default parameters for configuration--------------####

fSpeed = 117/10 #forward speed cm/s
aSpeed = 360/10 # angular speed degrees/s
interval = 0.25 # how often to check tello speed/ lower is more accurate

#units of tello actual movement represented in map for every unit of movement
dInterval = fSpeed * interval
aInterval = aSpeed * interval
###--------------------------################-----------------------######
#coords to represent Tello coordinates
tello_x, tello_y = 500,500
a = 0


def drawPoints(img):
    img,info = findfacehaar(img)
    _,_,speedLR,speedFB = trackface(tello,info)
    d = 0
    if speedLR < 0:
        d = dInterval
        a = -180
        print("moving left")
    elif speedLR > 0:
        d = -dInterval
        a = 180
        print("moving right")
    if speedFB > 0:
        d = dInterval
        a = 270
        print("moving foward")
    elif speedFB < 0:
        d = -dInterval
        a = -90
        print("moving backward")
    
    cv2.circle(img,(tello_x,tello_y),20,(0,0,255),cv2.FILLED)#BGR



def mapping():
    img = np.zeros((1000,1000,3),np.uint8)#3 representscolored image
    drawPoints(img)
    cv2.imshow("Tello map", img)




def main():
    mapping()
if __name__ == '__main__':

    while True:
        main()
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            print("Exit mapping...")
            break
