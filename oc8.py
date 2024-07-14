## Detect faces using a jetson_utils detection network
## Output detection boxes to console
## Output detection boxes to servos pan-tilt

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from adafruit_servokit import ServoKit




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

    #code for pan tilt
        
    panOut=panMax - ( (targetX/(xbound-0))*(panMax-panMin)  )

    tiltOut=tiltMax - ( (targetY/(ybound-0))*(tiltMax-tiltMin)  )

    print ("sending to "+ str(pout)+", "+str(tout))

    #CONTROL EDGE
    
    if(pout>=panMax):
        pout=panMax
        print("At pan bound "+str(pout))

    if(pout<=panMin):
        pout=panMin
        print("At pan bound "+str(pout))

    if(tout>=tiltMax):
        tout=tiltMax
        print("At tilt bound "+str(tout))

    if(tout<=tiltMin):
        tout=tiltMin
        print("At tilt bound "+str(tout))

    #myKit.servo[1].angle=pout
    #myKit.servo[0].angle=tout
    ##mykit.servo[4].angle=pout

    ## END OF movePanTilt





#### START THE CODE

net = detectNet(model="peoplenet-pruned", threshold=0.5)

#net = detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = videoSource("/dev/video0")      # '/dev/video0' for V4L2
display = videoOutput("display://0") # 'my_video.mp4' for file


#create instance of servokit
myKit=ServoKit(channels=16)

#set servos to defaults deg
myKit.servo[0].angle=45
myKit.servo[1].angle=90


#Target Geometry input rectanble dimensions
xbound=1920
ybound=1080

#Initiate Target
targetX=320
targetY=240

#Default pan and tile settings
pout=90
tout=40

# Default increments to targetX targetY
txincr=2
tyincr=1

frameskip=60
counter=0

while display.IsStreaming():
    
    img = camera.Capture()
    counter+=1
    if img is None: # capture timeout
        continue

    
    if(counter==frameskip):
        counter=0
        print("skipping loop")
        continue

    detections = net.Detect(img)

    detectcount=0
    for detect in detections:
        detectcount+=1
        print("detected item number: ", detectcount)
        ID=detect.ClassID
        top=detect.Top
        left=detect.Left
        bottom=detect.Bottom
        right=detect.Right
        item=net.GetClassDesc(ID)
        


        if(item!='person'):
            print("skipped non person")
            continue

        ##detect

        print(item,top,left,bottom,right)

        targetX=(right+left)/2.0
        print(targetX)
        targetY=top

        targetX+=txincr
        targetY+=tyincr
        
        print("Target set to "+str(targetX) + ", "+str(targetY))
        
        #p0inp=simpledialog.askinteger("Pitch 0", "Enter the Value (0-640)")
        
        pout=pan1 - ( (targetX/(xbound-0))*(pan1-pan0)  )

        #t0inp=simpledialog.askinteger("Tilt 0", "Enter the Value (0-480)")

        tout=tilt1 - ( (targetY/(ybound-0))*(tilt1-tilt0)  )

        #print ("sending to "+ str(p0inp)+", "+str(t0inp))

        print ("sending to "+ str(pout)+", "+str(tout))

        #CONTROL EDGE
       
        if(pout>=pan1):
            pout=pan1
            txincr*=-1
            print("At pan bound "+str(pout))

        if(pout<=pan0):
            pout=pan0
            txincr*=-1
            print("At pan bound "+str(pout))

        if(tout>=tilt1):
            tout=tilt1
            tyincr*=-1            
            print("At tilt bound "+str(tout))

        if(tout<=tilt0):
            tout=tilt0
            tyincr*=-1
            print("At tilt bound "+str(tout))

        #myKit.servo[1].angle=pout
        #myKit.servo[0].angle=tout

        ##mykit.servo[4].angle=pout
        break


    



    
    display.Render(img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))