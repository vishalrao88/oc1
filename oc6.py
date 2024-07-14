# Calibrate the ESC with 1500

import time
import traceback
from adafruit_servokit import ServoKit



try:
  
    #create instance of servokit
    myKit = ServoKit(channels=16)

    #set servos to default deg
    myKit.servo[0].angle = 45
    myKit.servo[1].angle = 90
    myKit.servo[4].angle = 90


    print("Set initial angles on 0 1 4, waiting 5 seconds")
    time.sleep(5)

    print("Waking Up")


    print("doing something")

    myKit.servo[1].angle = 45
    myKit.servo[4].angle = 45


    #myKit.servo[5].set_pulse_width_range(1000, 2000)
    myKit.continuous_servo[5].throttle = 0


    time.sleep(25)
    print("waking up")


    print("bye bye clean")




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
    myKit.servo[4].angle=90
    myKit.continuous_servo[5].throttle = 0
    print("cleaned final")
