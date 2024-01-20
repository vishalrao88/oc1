"""This module is for showing the camera on opencv"""

import cv2
print (cv2.__version__)

# flip=2
dispW=640
dispH=480


#cam=cv2.VideoCapture(camSet)
cam=cv2.VideoCapture(0)
#cam=cv2.VideoCapture('myCam.avi')

#setting paramters
cam.set(cv2.CAP_PROP_FRAME_WIDTH,dispW)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,dispH)
cam.set(cv2.CAP_PROP_FPS, 30)

framenum=0

#video file param
#outVid=cv2.VideoWriter('myCam.avi',cv2.VideoWriter_fourcc(*'XVID'),30,(dispW,dispH))

while True:
    ret, frame=cam.read() #reads latest frame from camera
    
    frame=cv2.rectangle(frame,(140,100),(180,140),(0,255,00),4)
    farme=cv2.circle(frame,(320,240),50,(255,0,30),-1)

    #show frame in a window goproCam moved to 0,0 
    #cv2.imshow('goproCam',frame)
    #cv2.moveWindow('goproCam',0,0)
    
    print("Frame: " + str(framenum))
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #reading a color and gray frame
    frameSmall=cv2.resize(frame,(dispW,dispH))
    graySmall=cv2.resize(gray,(dispW,dispH))
 
    #showing the gray frame at 10,10
    cv2.imshow('Native',frameSmall)
    cv2.moveWindow('Native', 0,0)


    #writing file to disk AAAAAAAAAAAAAAAAAAAAA
 #   outVid.write(frame)
    
    framenum+=1



    #exit if q is presed.
    if cv2.waitKey(50)==ord('q'):
        break

cam.release()
#outVid.release()
cv2.destroyAllWindows()
