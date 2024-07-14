import serial
import time

# arduino = serial.Serial("/dew/ttYACMO", 115200, timeout=5)

arduino = serial.Serial(
#port='/dev/ttyACM0',
port='/dev/ttyUSB0',
baudrate = 921600,
bytesize = serial.EIGHTBITS,
parity = serial.PARITY_NONE,
stopbits = serial.STOPBITS_ONE,
timeout = 5,
xonxoff= False,
rtscts = False,
dsrdtr = False,
writeTimeout = 2
)

#if not arduino.isOpen():
#arduino.open()
#arduino.close()

while True:
	#arduino.open()
	try:
		#arduino.write("command frum jetson".encode())
		rcdata = arduino.readline()
		if rcdata:
			#print(rcdata) # print recetved data-tros 
			# Split the string by vertical bars to get individual channel readings
			channel_readings = rcdata.split('|')

			# Initialize an empty dictionary to store channel readings
			rcvalues = {}

			# Iterate through channel readings
			for reading in channel_readings:
				# Split each reading by colon to separate channel name and value
				rcchannel, rcvalue = reading.split(':')
				# Remove leading and trailing whitespace from channel and value
				rcchannel = rcchannel.strip()
				rcvalue = int(rcvalue.strip())
				# Add channel and its corresponding value to the dictionary
				rcvalues[rcchannel] = rcvalue
			print("translated: ",rcvalues)
		#time.sleep(1)
	except Exception as e:
		    print(e)
		    arduino.close()
    finally:
            arduino.close()
		