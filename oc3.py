import cv2
print(cv2.__version__)
import numpy as np

dispW=640
dispH=480
flip=2
#Uncomment These next Two Line for Pi Camera
#camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
#cam= cv2.VideoCapture(camSet)

#Or, if you have a WEB cam, uncomment the next line
#(If it does not work, try setting to '1' instead of '0')
cam=cv2.VideoCapture(0)
#width=cam.get(cv2.CAP_PROP_FRAME_WIDTH)
#height=cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
width,height=dispW,dispH



face_cascade=cv2.CascadeClassifier('/home/jetson/dev/oc1/cascade/facecuda.xml')

eye_cascade=cv2.CascadeClassifier('/home/jetson/dev/oc1/cascade/eye.xml')




print('width:',width,'height:',height)
while True:   
    ret, frame = cam.read()
    frame=cv2.resize(frame,(dispW,dispH))

    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    faces=face_cascade.detectMultiScale(gray,1.3,5)
    """
    for (x,y,w,h) in faces:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)

        targetX=(int)((x+x+w)/2)
        targetY=(int)((y+y+h)/2)
        print("target at: "+str(targetX) + ", "+str(targetY))

        cv2.rectangle(frame,(targetX,targetY),(targetX+3,targetY+3),(0,255,0),2)

        roi_gray=gray[y:y+h,x:x+w]
        roi_color=frame[y:y+h,x:x+w]
        eyes=eye_cascade.detectMultiScale(roi_gray)
        for (xEye,yEye,wEye,hEye) in eyes:
            cv2.rectangle(roi_color,(xEye,yEye),(xEye+wEye,yEye+hEye),(255,0,0),2)

   """
    cv2.imshow('nanoCam',frame)
    cv2.moveWindow('nanoCam',0,0)


    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()