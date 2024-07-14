"""
This program uses Nvidia jetson to read a frame from the USB camera, process it 
with a ovject detection model (YOLOv5) and point the pan tilt servo 
and RC car steering to the center of the first person found in the frame. It also 
reads a priximity sensor (ultrasonic sensor) and redirects if distance <30 cm

RC control section:
Reads PWM signals from an RC receiver and sends them to the appropriate
Channel on a servo driver PCA 9685

RC receiver is connected to an esp32. ESP32 code decodes PWM signal and sends 4 channels 
of data via USB serial console. This code is executed on Jetson Nano: it reads the 
serial data, decodes the values per channel and sends those values to the PCA9685
servo driver

Ch1 is Steering
Ch2 is Throttle
Ch3 is Mode. Mode 1 = Do nothing. Mode 2 = Fly by wire. Mode 3 = Autonomous

"""

import serial
#import time
from adafruit_servokit import ServoKit
#from jetson_inference import detectNet
#from jetson_utils import videoSource, videoOutput
#import datetime
import subprocess


import sys
import cv2 
import imutils
from yoloDet import YoloTRT


## FUNCTIONS


def detectPerson(detections, class_label='person', confidence_threshold=0.5):
    """
    Get the first instance of a person in the list of detections.

    Parameters:
    - detections: List of detection results, where each detection is a dictionary or an object.
    - class_label: The label for the target class (e.g., 'person').
    - confidence_threshold: Minimum confidence score required for a detection to be considered.

    Returns:
    - The first detected instance of the specified class, or None if not found.
    """
    for detection in detections:
        if detection['label'] == class_label and detection['confidence'] >= confidence_threshold:
            return detection
    return None



def movePanTilt(targetX, targetY, panMin=20, panMax=160, tiltMin=15, tiltMax=90):
    """
    Takes the target x and y in pixels, servo's pan and tilt min and max 
     angles and moves the servos within proportion of image size xbound and ybound
    
    #FOV CALIBRATION (MIN-MAX ANGLES FOR SERVO)
    panMin=35
    panMax=145
    tiltMin=15
    tiltMax=90

    """
        
    panOut= panMax - ( (targetX/(xbound-0))*(panMax-panMin)  )

    #tiltOut= tiltMax - ( (targetY/(ybound-0))*(tiltMax-tiltMin)  )
    tiltOut=  ( (targetY/(ybound-0))*(tiltMax-tiltMin)  )

    #print ("sending to "+ str(panOut)+", "+str(tiltOut))

    #CONTROL EDGE
    
    if(panOut>=panMax):
        panOut=panMax
        #print("At pan bound "+str(panOut))

    if(panOut<=panMin):
        panOut=panMin
        #print("At pan bound "+str(panOut))

    if(tiltOut>=tiltMax):
        tiltOut=tiltMax
        #print("At tilt bound "+str(tiltOut))

    if(tiltOut<=tiltMin):
        tiltOut=tiltMin
        #print("At tilt bound "+str(tiltOut))

    myKit.servo[1].angle=panOut
    myKit.servo[0].angle=tiltOut
    ##mykit.servo[4].angle=panOut
    
    return panOut, tiltOut
    ## END OF movePanTilt


def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



## PARAMETERS AND VARIABLES

## Set the network model used to process video
#detectnet peoplenet-pruned that came with the jetson sdk
#net = detectNet(model="peoplenet-pruned", threshold=0.6)
#22 net = detectNet(model="~/dev/exp/tensorrtx/yolov5/build/yolov5n.engine", threshold=0.6)

# use path for library and engine file
#model = YoloTRT(library="/home/jetson/dev/exp/tensorrtx/yolov5/build/libmyplugins.so", engine="/home/jetson/dev/exp/tensorrtx/yolov5/build/yolov5n.engine", conf=0.5, yolo_ver="v5")

model = YoloTRT(library="/home/jetson/dev/oc1/libmyplugins.so", engine="/home/jetson/dev/oc1/yolov5n.engine", conf=0.5, yolo_ver="v5")

#cap = cv2.VideoCapture("videos/testvideo.mp4")

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS, 30)



#net = detectNet("ssd-mobilenet-v2", threshold=0.5)


#Getting video source and out using nvidia utils 
#22 camera = videoSource("/dev/video0",["--input-width=640", "--input-height=480"])      # '/dev/video0' for V4L2
#22 display = videoOutput("display://0") # 'my_video.mp4' for file


#audio file locations
audio_target_aquired = "/home/jetson/dev/contalarm.wav"
audio_proximity_warning="/home/jetson/dev/3beeps.wav"

#audio_process = subprocess.Popen(['aplay', audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# Initialize serial connection Serial 
arduino = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=5,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    writeTimeout=2
)

#start the servo driver
myKit=ServoKit(channels=16)

#set servos to defaults deg
myKit.servo[0].angle=45
myKit.servo[1].angle=90


#Target Geometry input rectanble dimensions
xbound=640
ybound=480

#Initiate Target
targetX=320
targetY=240

#Default pan and tile settings
panOut=90
tiltOut=40

# Default increments to targetX targetY
txincr=2
tyincr=1


# SET STATE. GET READY

frameskip=60
counter=0
rcmode='Stand By'

## EXECUTION LOOP
"""

Loop on the videoOutput window

Process image detect first face
point Turret at the face
RC Control



"""

try:


    #22 while display.IsStreaming():
    while True:

        #22 img = camera.Capture()

        ret, frame = cap.read()
        #frame = imutils.resize(frame, width=640)
        detections, t = model.Inference(frame)
        # for obj in detections:
        #    print(obj['class'], obj['conf'], obj['box'])
        #print("FPS: {} sec".format(1/t))
        #print("Inf time:",t)
        cv2.imshow("Output", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        counter+=1

        """
        if(counter==frameskip):
            counter=0
            print("skipping loop")
            # Get current time
            current_time = datetime.datetime.now()
            print("Current time:", current_time)
            continue
        """
        
        
        #detections = net.Detect(img)
        
        detectcount=0
        pback=90
        tback=0

        #22 display.Render(img)
        #22 display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        
        for detect in detections:
            detectcount += 1
            # print("detected item number: ", detectcount)
            item = detect['class']

            # Accessing the bounding box coordinates
            left = detect['box'][0]   # x_min
            top = detect['box'][1]    # y_min
            right = detect['box'][2]  # x_max
            bottom = detect['box'][3] # y_max

            """
            top = detect.Top
            left = detect.Left
            bottom = detect.Bottom
            right = detect.Right
            item = net.GetClassDesc(ID)
            """


            if(item!='person'):
                #skip the rest of loop if no person found
                continue

            ## Detected a person! setting target variables and moving servo

            ## Play audio file with mpv.
            #subprocess.run(['aplay', audio_target_aquired], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            #print(item,top,left,bottom,right)

            targetX=(right+left)/2.0
            targetY=top
            
            #print("Target set to "+str(targetX) + ", "+str(targetY))
            
            pback,tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)
            #print("we got back ",pback,tback )

            # exit the for loop if we found a person
            break
        
    

        ## Read and decode serial input from esp32/arduino
        if arduino.in_waiting > 0:

            #flush the serial input
            arduino.reset_input_buffer()
            #read till new line
            rcdata = arduino.readline().decode().strip()
            channel_readings = rcdata.split('|')
            rcvalues = {}
            if len(channel_readings)!=5:
                print("Bad RC Data")
                continue

            for reading in channel_readings:
                rcchannel, rcvalue = reading.split(':')
                rcchannel = rcchannel.strip()
                rcvalue = int(rcvalue.strip())
                rcvalues[rcchannel] = rcvalue
            #print("Signal:", rcvalues)
            if len(rcvalues)!=5:
                continue

            ## MODE 1
            if (rcvalues['Ch3']<1000): 
                if(rcmode!='Monitor'):
                    print("Mode: ", rcmode, "--> Monitor")
                rcmode='Monitor'
                #print("Default mode Distance: ", rcvalues['Dist'])
                

            ## MODE 2
            if 1200 <= rcvalues['Ch3'] <= 1600:
                
                if(rcmode!='Fly By Wire'):
                    print("Mode: ", rcmode, "--> Fly By Wire")
                rcmode = 'Fly By Wire'
                #print("Mode: ", rcmode)
                #print("Distance: ", rcvalues['Dist'])
                distance=rcvalues['Dist1']
                distance=100
                if (distance<50): 
                    print("PROXIMITY")
                    #audio_process.poll()  # Check if the subprocess has finished
                    #if audio_process.returncode is not None:  # If the subprocess has finished, restart it
                        #audio_process = subprocess.Popen(['aplay', audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                    #subprocess.run(['aplay',audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
               
                # if (distance<50):
                #     subprocess.run(['aplay',audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                
                steer = 180 - mapPWMtoAngle(rcvalues['Ch1'])
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle




            ## MODE 3
            if (rcvalues['Ch3']>2000):
                
                if(rcmode!='Auto'):
                    print("Mode: ", rcmode, "--> Auto")

                rcmode='Auto'
                
                

                distance=rcvalues['Dist1']
                #distance=100
                if (distance<50):
                    print("PROXIMITY")
                    #audio_process.poll()  # Check if the subprocess has finished
                    #if audio_process.returncode is not None:  # If the subprocess has finished, restart it
                        #audio_process = subprocess.Popen(['aplay', audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                    #subprocess.run(['aplay',audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # steer angle set to pan angle
                steer = 180 - pback
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle
        
        
        #display.Render(img)
        #display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))






except Exception as e:
    print(e)
    print("An exception occurred: " + repr(e))
    print("Exception message: " + str(e))
finally:
    arduino.close()
    cap.release()
    cv2.destroyAllWindows()
    print("full exit")
