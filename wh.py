def movePanTilt(targetX, targetY, panMax, panMin, tiltMax, tiltMin):
    """
    Takes the taargetx, target y in pixels, pan and tilt min and max edges and moves the servos
    Usually set pan and tilts 
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