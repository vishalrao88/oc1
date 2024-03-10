"""
This program reads PWM signals from an RC receiver and sends them to tha appropriate
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
import time
from adafruit_servokit import ServoKit
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
import datetime
import subprocess



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



def movePanTilt(targetX, targetY, panMin=35, panMax=145, tiltMin=15, tiltMax=90):
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
net = detectNet(model="peoplenet-pruned", threshold=0.5)
#net = detectNet("ssd-mobilenet-v2", threshold=0.5)


#Getting video source and out using nvidia utils 
camera = videoSource("/dev/video0",["--input-width=640", "--input-height=480"])      # '/dev/video0' for V4L2
display = videoOutput("display://0") # 'my_video.mp4' for file


#audio file locations
audio_target_aquired = "/home/jetson/dev/target-acquired-101soundboards.mp3"


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

frameskip=60
counter=0





## EXECUTION LOOP
"""

Loop on the videoOutput window

Process image detect first face
point Turret at the face
RC Control



"""

try:


    while display.IsStreaming():

        img = camera.Capture()
        counter+=1
        if img is None: # capture timeout
            continue

        """
        if(counter==frameskip):
            counter=0
            print("skipping loop")
            # Get current time
            current_time = datetime.datetime.now()
            print("Current time:", current_time)
            continue
        """
        detections = net.Detect(img)

        detectcount=0
        for detect in detections:
            detectcount += 1
            # print("detected item number: ", detectcount)
            ID = detect.ClassID
            top = detect.Top
            left = detect.Left
            bottom = detect.Bottom
            right = detect.Right
            item = net.GetClassDesc(ID)
            


            if(item!='person'):
                #skip the rest of loop if no person found
                continue

            ## Detected a person! setting target variables and moving servo

            ## Play audio file with mpv.
            #subprocess.run(['mpv', audio_target_aquired])

            #print(item,top,left,bottom,right)

            targetX=(right+left)/2.0
            targetY=top
            
            #print("Target set to "+str(targetX) + ", "+str(targetY))
            
            pback,tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)
            #print("we got back ",pback,tback )

            # exit the for loop if we found a person
            break
    
    

        ## Read and decode serial input from esp32/arduino

        #flush the serial input
        arduino.flushInput()

        #read till new line
        rcdata = arduino.readline().decode().strip()
        if rcdata:
            channel_readings = rcdata.split('|')
            rcvalues = {}
            for reading in channel_readings:
                rcchannel, rcvalue = reading.split(':')
                rcchannel = rcchannel.strip()
                rcvalue = int(rcvalue.strip())
                rcvalues[rcchannel] = rcvalue
            #print("Signal:", rcvalues)

            ## MODE 1
            if (rcvalues['Ch3']<1000): ## NEED TO DEFAULT TO CH 3 <1000 = default
                rcmode='default'
                #print("Mode: ", rcmode)
                

            ## MODE 2
            if 1200 <= rcvalues['Ch3'] <= 1600:
                rcmode = 'Fly By Wire'
                #print("Mode: ", rcmode)

                steer = 180 - mapPWMtoAngle(rcvalues['Ch1'])
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle




            ## MODE 3
            if (rcvalues['Ch3']>2000):
                rcmode='Auto'
                print("Mode: ", rcmode)

                # steer angle set to pan angle
                steer = 180 - pback
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle


        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))






except Exception as e:
    print(e)
finally:
    arduino.close()
