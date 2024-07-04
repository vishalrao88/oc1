"""
This program uses Nvidia jetson to read a frame from the USB camera, process it 
with a ovject detection model (peoplenet-pruned) and point the pan tilt servo 
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
import time
from adafruit_servokit import ServoKit

import datetime
import subprocess

from ultralytics import YOLO
import torch
import traceback


## PARAMETERS AND VARIABLES

## Set the network model used to process video
#detectnet peoplenet-pruned that came with the jetson sdk
#net = detectNet(model="peoplenet-pruned", threshold=0.5)
#net = detectNet("ssd-mobilenet-v2", threshold=0.5)
torch.cuda.set_device(0)
model_path = "yolov8n.onnx"

try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Error loading model: {e}")
    raise
source = 0
print("Running inference on source:", source)



#Getting video source and out using nvidia utils 
#camera = videoSource("/dev/video0",["--input-width=640", "--input-height=480"])      # '/dev/video0' for V4L2
#15#camera = videoSource("/dev/video0", ["--input-width=640","--input-height=480", "--codec=h264", "--input-decoder=omx"])  # '/dev/video0' for V4L2

#pipeline = "v4l2src device=/dev/video0 ! video/x-h264,width=640,height=480,framerate=30/1 ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw(memory:NVMM),format=RGBA ! nvvidconv ! video/x-raw,width=640,height=480,format=BGRx ! videoconvert"
#camera = videoSource(pipeline)



##15 display = videoOutput("display://0") 




#audio file locations
#15a audio_target_aquired = "/home/jetson/dev/contalarm.wav"
#15a audio_proximity_warning="/home/jetson/dev/3beeps.wav"

#15a audio_process = subprocess.Popen(['aplay', audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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

frameskip=20
counter=0





## EXECUTION LOOP
"""

Loop on the videoOutput window

Process image detect first face
point Turret at the face
RC Control



"""

try:

    results = model(source, save=False, stream=True, show=True, project=False)

    for result in results:

        ##15 img = camera.Capture()
        counter+=1
        #if img is None: # capture timeout
        #    continue

        
        if(counter==frameskip):
            counter=0
            print("skipping loop")
            # Get current time
            current_time = datetime.datetime.now()
            print("Current time:", current_time)
            continue
        
        
        
        ##15 detections = net.Detect(img)
        
        detectcount=0
        pback=90
        tback=0

        boxes = result.boxes

        for box in boxes:
            left, top, right, bottom = box.xyxy[0]
            confidence = box.conf[0]
            class_id = box.cls[0]
            item = result.names[int(class_id)]
            


            if(item!='person'):
                #skip the rest of loop if no person found
                continue

            ## Detected a person! setting target variables and moving servo


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
        if arduino.in_waiting > 0:
            #data12 = arduino.read(arduino.in_waiting)  # Read the available data
            print(f"Received: data")
        else:
            print("No data available")
            continue

        #read till new line
        rcdata = arduino.readline().decode().strip()
        print(rcdata)

        if rcdata:
            channel_readings = rcdata.split('|')
            rcvalues = {}
            if len(channel_readings)!=5:
                print("bad Data")
                continue
            for reading in channel_readings:
                rcchannel, rcvalue = reading.split(':')
                rcchannel = rcchannel.strip()
                rcvalue = int(rcvalue.strip())
                rcvalues[rcchannel] = rcvalue
            



            ## MODE 1
            if (rcvalues['Ch3']<1000): 
                rcmode='default'
                #print("Default mode Distance: ", rcvalues['Dist'])
                

            ## MODE 2
            if 1200 <= rcvalues['Ch3'] <= 1600:
                rcmode = 'Fly By Wire'
                #print("Mode: ", rcmode)
                #print("Distance: ", rcvalues['Dist'])
                distance=rcvalues['Dist1']
                if (distance<50): 
                    #15a audio_process.poll()  # Check if the subprocess has finished
                    print("PROXIMITY")

                # if (distance<50):
                #     subprocess.run(['aplay',audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                
                steer = 180 - mapPWMtoAngle(rcvalues['Ch1'])
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle




            ## MODE 3
            if (rcvalues['Ch3']>2000):
                rcmode='Auto'
                print("Mode: ", rcmode)

                distance=rcvalues['Dist1']
                
                if (distance<50):
                    print("PROX ")
                    #15aaudio_process.poll()  # Check if the subprocess has finished
                    #15aif audio_process.returncode is not None:  # If the subprocess has finished, restart it
                        #15aaudio_process = subprocess.Popen(['aplay', audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                    #subprocess.run(['aplay',audio_proximity_warning], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # steer angle set to pan angle
                #15 steer = 180 - pback
                
                myKit.servo[4].angle = steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                #print("Throttle", throttle)
                myKit.continuous_servo[5].throttle = throttle
        
        ##15 display.Render(img)
        ##15  display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))



except Exception as e:
    print(e)
    traceback.print_exc()
finally:
    arduino.close()
