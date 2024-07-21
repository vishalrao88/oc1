"""
This program uses Nvidia jetson to read a frame from the USB camera, process it 
with a ovject detection model (YOLOV8) and point the pan tilt servo 
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
import traceback

#import time
#from adafruit_servokit import ServoKit

#import datetime
import subprocess

from ultralytics import YOLO
#import torch
import traceback


## PARAMETERS AND VARIABLES

## Set the network model used to process video
#detectnet peoplenet-pruned that came with the jetson sdk
#net = detectNet(model="peoplenet-pruned", threshold=0.5)
#net = detectNet("ssd-mobilenet-v2", threshold=0.5)
#torch.cuda.set_device(0)
model_path = "yolov8s.pt"

try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Error loading model: {e}")
    raise
#source = 'https://youtu.be/VuJuU636Yrg'
source=0
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
    port='COM5',
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

def read_latest_serial_data(serial_port):
    latest_data = ""
    while serial_port.in_waiting > 0:
        line = serial_port.readline().decode().strip()  # Read a line from the buffer
        latest_data = line  # Update latest_data with the most recent line
    return latest_data


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

    #myKit.servo[1].angle=panOut
    #myKit.servo[0].angle=tiltOut
    
    ##mykit.servo[4].angle=panOut
    
    return panOut, tiltOut
    ## END OF movePanTilt


def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


"""
#start the servo driver
myKit=ServoKit(channels=16)

#set servos to defaults deg
myKit.servo[0].angle=45
myKit.servo[1].angle=90
"""

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

frameskip=10
counter=0





## EXECUTION LOOP
"""

Loop on the videoOutput window

Process image detect first face
point Turret at the face
RC Control



"""

try:
    results = model(source, save=False, stream=True, imgsz=640, show=True, project=False, conf=0.5)

    for result in results:
        counter += 1

        # Skip frames if needed
        if counter == frameskip:
            counter = 0
            continue
        
        detectcount = 0
        pback = 90
        tback = 0

        boxes = result.boxes

        for box in boxes:
            class_id = box.cls[0]
            item = result.names[int(class_id)]
            if item != 'person':
                continue

            left, top, right, bottom = box.xyxy[0]
            confidence = box.conf[0]

            # Detected a person, update servo
            targetX = (right + left) / 2.0
            targetY = top
            pback, tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)
            break  # Exit loop if person is found
        
        # Read and decode serial input from ESP32/Arduino
        
        if arduino.in_waiting > 0:
            #52 rcdata = arduino.readline().decode().strip()
            rcdata = read_latest_serial_data(arduino)
            channel_readings = rcdata.split('|')
            if len(channel_readings) != 5:
                print("Bad Data")
                continue
            
            rcvalues = {}
            for reading in channel_readings:
                try:
                    rcchannel, rcvalue = reading.split(':')
                    rcvalues[rcchannel.strip()] = int(rcvalue.strip())
                except ValueError:
                    print("Error parsing RC data")
                    continue
            
            # Handle modes based on RC values
            ch3_value = rcvalues.get('Ch3', 0)
            dist = rcvalues.get('Dist1', 0)
            steer = 180 - mapPWMtoAngle(rcvalues.get('Ch1', 0))
            throttle = mapPWMtoThrottle(rcvalues.get('Ch2', 0))

            if ch3_value < 1000: 
                rcmode = 'default'
                
            elif 1200 <= ch3_value <= 1600:
                rcmode = 'Fly By Wire'
                if dist < 50:
                    print("PROXIMITY")
                #myKit.servo[4].angle = steer
                #myKit.continuous_servo[5].throttle = throttle

            elif ch3_value > 2000:
                rcmode = 'Auto'
                if dist < 50:
                    print("PROX")
                #myKit.servo[4].angle = steer
                #myKit.continuous_servo[5].throttle = throttle
        
except Exception as e:
    print(e)
    traceback.print_exc()
    print("Cutting throttle")
    #myKit.continuous_servo[5].throttle = 0
    print("done cutting throttle")

finally:
    arduino.close()
    print("full exit")