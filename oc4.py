#Import tkinter library
import tkinter as tk
from tkinter import *
from tkinter import simpledialog
import time
import traceback
from adafruit_servokit import ServoKit

import cv2
print(cv2.__version__)

#window dimensions
winx=750
winy=250


#servo angles
yaw=90.0
pitch=90.0

cam=cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cam.set(cv2.CAP_PROP_FPS, 30)
#cv2.CAP_PROP
face_cascade=cv2.CascadeClassifier('/home/jetson/dev/oc1/cascade/facecuda.xml')

#eye_cascade=cv2.CascadeClassifier('/home/jetson/dev/oc1/cascade/eye.xml')


try:
  
    #create instance of servokit
    myKit=ServoKit(channels=16)

    #set servos to default deg
    myKit.servo[0].angle=45
    myKit.servo[1].angle=90

    #FOV CALIBRATION
    pan0=35
    pan1=145
    tilt0 = 1
    tilt1=75


    #Target Geometry input rectanble dimensions
    xbound=640
    ybound=480

    #Initiate Target
    targetX=320
    targetY=240


    #Default pan and tile settings
    pout=90
    tout=40
    
    #p0inp=tk.Entry(win,textvariable)
    
    txincr=2
    tyincr=1

    while True:
        ret, frame = cam.read()

        #targetX+=txincr
        #targetY+=tyincr
        

        #frame =cv2.resize(frame ,(xbound,ybound))

        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        faces=face_cascade.detectMultiScale(gray,1.3,5)
        
        for (x,y,w,h) in faces:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)

            targetX=(int)((x+x+w)/2)
            targetY=(int)((y+y+h)/2)
            print("target at: "+str(targetX) + ", "+str(targetY))

            cv2.rectangle(frame,(targetX,targetY),(targetX+3,targetY+3),(0,255,0),2)

            roi_gray=gray[y:y+h,x:x+w]
            roi_color=frame[y:y+h,x:x+w]
            """
            eyes=eye_cascade.detectMultiScale(roi_gray)
            for (xEye,yEye,wEye,hEye) in eyes:
                cv2.rectangle(roi_color,(xEye,yEye),(xEye+wEye,yEye+hEye),(255,0,0),2)
            """
    

        print("Target set to "+str(targetX) + ", "+str(targetY))
        
        #p0inp=simpledialog.askinteger("Pitch 0", "Enter the Value (0-640)")
        
        pout=pan1 - ( (targetX/(xbound-0))*(pan1-pan0)  )

        #t0inp=simpledialog.askinteger("Tilt 0", "Enter the Value (0-480)")

        tout= ( (targetY/(ybound-0))*(tilt1-tilt0)  )

        #print ("sending to "+ str(p0inp)+", "+str(t0inp))

        print ("sending to "+ str(pout)+", "+str(tout))

        #CONTROL EDGE
       
        if(pout>=pan1):
            pout=pan1
            print("At pan bound "+str(pout))

        if(pout<=pan0):
            pout=pan0
            print("At pan bound "+str(pout))

        if(tout>=tilt1):
            tout=tilt1
            print("At tilt bound "+str(tout))

        if(tout<=tilt0):
            tout=tilt0
            print("At tilt bound "+str(tout))

        myKit.servo[1].angle=pout
        myKit.servo[0].angle=tout



        #time.sleep(1)

        cv2.imshow('nanoCam',frame)
        cv2.moveWindow('nanoCam',0,0)

        if cv2.waitKey(10)==ord('q'):
            break


    cam.release()
    cv2.destroyAllWindows()




except Exception as e:
    # Print the exception details
    print("An exception occurred: {type(e).__name__}: {e}")
    traceback.print_exc()



#    p.stop
#    GPIO.cleanup() # this ensures a clean exit
    print("stopped and cleaned in exception")


finally:  
    myKit.servo[0].angle=90
    myKit.servo[1].angle=90
    print("cleaned final")
