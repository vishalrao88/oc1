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


def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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

try:

    #start the servo driver
    myKit=ServoKit(channels=16)

    #flush the serial input
    arduino.flushInput()

    while True:
        rcdata = arduino.readline().decode().strip()
        if rcdata:
            channel_readings = rcdata.split('|')
            rcvalues = {}
            for reading in channel_readings:
                rcchannel, rcvalue = reading.split(':')
                rcchannel = rcchannel.strip()
                rcvalue = int(rcvalue.strip())
                rcvalues[rcchannel] = rcvalue
            print("Signal:", rcvalues)

            ## MODE 1
            if (rcvalues['Ch3']<1000):
                rcmode='default'
                

            ## MODE 2
            if 1200 <= rcvalues['Ch3'] <= 1600:
                rcmode = 'Fly By Wire'

                steer=180-mapPWMtoAngle(rcvalues['Ch1'])
                
                myKit.servo[4].angle=steer

                throttle=mapPWMtoThrottle(rcvalues['Ch2'])
                print("Throttle", throttle)
                myKit.continuous_servo[5].throttle=throttle




            ## MODE 3
            if (rcvalues['Ch3']>2000):
                rcmode='auto'


            print("Mode: ", rcmode)


        #time.sleep(1)
        #end of while loop
        arduino.flushInput()



except Exception as e:
    print(e)
finally:
    arduino.close()
