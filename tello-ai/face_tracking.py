#Source: https://www.youtube.com/watch?v=P2wl3N2JW9c
from djitellopy import Tello
import numpy as np
import cv2
#additions for yoloface
from yoloface import face_analysis
#additions for detection with svm
from logging import exception
import face_recognition
import os 
import threading
import socket



face=face_analysis()    #  Auto Download a large weight files from Google Drive.
                            #  only first time.
                            #  Automatically  create folder .yoloface on cwd.

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


def findfacehaar(img):
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

def findfaceyolo(img):
    _,box,conf=face.face_detection(frame_arr=img,frame_status=True,model='tiny')
    output_img=face.show_output(img,box,frame_status=True)
    
    facecoords = []
    facecoordsArea = []
    
    for (x,y,w,h) in box:
        cx = x + w//2 
        cy = y + h //2
        area = w*h

        facecoordsArea.append(area)
        facecoords.append([cx,cy])

    if len(facecoords) != 0:
        i = facecoordsArea.index(max(facecoordsArea))
        return output_img, [facecoords[i],facecoordsArea[i]]
    else:
        return output_img,[[0,0],0]

def findfaceSVM(frame):

    #load a target image relative to code location
    #cwd = os.getcwd()
    default_folder = "/home/thinkpad/Desktop/msc_face_recognition_project/examples/default_image/"
    target_folder = "/home/thinkpad/Desktop/msc_face_recognition_project/examples/target_image/"
    try:
        tmp = os.listdir(target_folder)

        target_image = face_recognition.load_image_file(target_folder+tmp[0])
        target_face_encoding = face_recognition.face_encodings(target_image)[0]

        #get name from Target folder and strip file extension
        file_extension = '.'
        stripped_file_name = tmp[0].split(file_extension,1)[0]

        # Create arrays of known face encodings and their names
        known_face_encodings = [
            target_face_encoding
        ]
        known_face_names = [
            str(stripped_file_name)
        ]

        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True
        
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            facecoords = []
            facecoordsArea = []
            face_names = []
            
            for (x,y,w,h) in face_locations:
                cx = x + w//2 
                cy = y + h //2
                area = w*h

                facecoordsArea.append(area)
                facecoords.append([cx,cy])

            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]

                face_names.append(name)

        process_this_frame = not process_this_frame
        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to w and h size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        #get the right index for the face area from the known names array
        if len(facecoords) != 0:
            #Checking for strings in a string array https://www.askpython.com/python/list/find-string-in-list-python
            if stripped_file_name in face_names:
                # get index of name to match to area of face https://stackoverflow.com/questions/176918/finding-the-index-of-an-item-in-a-list
                i = face_names.index(stripped_file_name)
                return frame, [facecoords[i],facecoordsArea[i]]

            elif "Unknown" in face_names:
                return frame, [[0,0],0]

        else:
            return frame,[[0,0],0]
            
    except exception as e:
        if not tmp:
            print("No target set...", e)
            print("looking for target(s)...")
            tmp = os.listdir(default_folder)
            
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
        

        if speedUD > 0:
            print("UP")
        elif speedUD <0:
            print("DOWN")
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
            tello.yaw_velocity = speedLR
            tello.up_down_velocity = speedUD
            #tello.left_right_velocity = speedLR
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
