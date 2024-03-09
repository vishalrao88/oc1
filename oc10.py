## Detect faces using a jetson_utils detection network
## Output detection boxes to console
## Output detection boxes to servos pan-tilt

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from adafruit_servokit import ServoKit
import datetime
import subprocess






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
    Takes the target x and y in pixels, pan and tilt min and max edges and moves the servos
    Usually set pan and tilts 
    #FOV CALIBRATION (MAX EDGE ANGLES)
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
        print("At pan bound "+str(panOut))

    if(panOut<=panMin):
        panOut=panMin
        print("At pan bound "+str(panOut))

    if(tiltOut>=tiltMax):
        tiltOut=tiltMax
        print("At tilt bound "+str(tiltOut))

    if(tiltOut<=tiltMin):
        tiltOut=tiltMin
        print("At tilt bound "+str(tiltOut))

    myKit.servo[1].angle=panOut
    myKit.servo[0].angle=tiltOut
    ##mykit.servo[4].angle=panOut
    
    return panOut, tiltOut
    ## END OF movePanTilt





#### START THE CODE

#detectnet peoplenet-pruned came with the jetson sdk
net = detectNet(model="peoplenet-pruned", threshold=0.5)


#net = detectNet("ssd-mobilenet-v2", threshold=0.5)


#Getting video source and out using nvidia utils 
camera = videoSource("/dev/video0",["--input-width=640", "--input-height=480"])      # '/dev/video0' for V4L2
display = videoOutput("display://0") # 'my_video.mp4' for file


#audio file locations
audio_target_aquired = "/home/jetson/dev/target-acquired-101soundboards.mp3"


#create instance of servokit
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



# Loop on the python windo staying open
while display.IsStreaming():
    
    img = camera.Capture()
    counter+=1
    if img is None: # capture timeout
        continue

    
    if(counter==frameskip):
        counter=0
        print("skipping loop")
        # Get current time
        current_time = datetime.datetime.now()
        # Extract seconds value
        #seconds = current_time.second
        #print("Current seconds value:", seconds)
        print("Current time:", current_time)
        continue

    detections = net.Detect(img)

    detectcount=0
    for detect in detections:
        detectcount+=1
        #print("detected item number: ", detectcount)
        ID=detect.ClassID
        top=detect.Top
        left=detect.Left
        bottom=detect.Bottom
        right=detect.Right
        item=net.GetClassDesc(ID)
        


        if(item!='person'):
            #print("skipped non person")
            continue

        ##detected a person. setting target variables and moving servo

        subprocess.run(['mpv', audio_target_aquired])

        print(item,top,left,bottom,right)

        targetX=(right+left)/2.0
        targetY=top

        #targetX+=txincr
        #targetY+=tyincr
        
        #print("Target set to "+str(targetX) + ", "+str(targetY))
        
        pback,tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)

        #print("we got back ",pback,tback )

        break

    
    
    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))