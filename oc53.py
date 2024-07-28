import serial
import traceback
from ultralytics import YOLO

# PARAMETERS AND VARIABLES
model_path = "yolov8n.onnx"
try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Error loading model: {e}")
    raise

source = 0
print("Running inference on source:", source)

# Initialize serial connection Serial 
arduino = serial.Serial(
    port='COM3',
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=5,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    write_timeout=2
)

# FUNCTIONS
def read_latest_serial_data(serial_port):
    latest_data = ""
    while serial_port.in_waiting > 0:
        try:
            line = serial_port.readline().decode().strip()  # Read a line from the buffer
            if line:  # Only update if line is not empty
                latest_data = line
        except UnicodeDecodeError as e:
            print(f"Error decoding serial data: {e}")
            continue
    return latest_data

def movePanTilt(targetX, targetY, panMin=20, panMax=160, tiltMin=15, tiltMax=90):
    panOut = panMax - ((targetX / xbound) * (panMax - panMin))
    tiltOut = (targetY / ybound) * (tiltMax - tiltMin)

    panOut = max(min(panOut, panMax), panMin)
    tiltOut = max(min(tiltOut, tiltMax), tiltMin)

    return panOut, tiltOut

def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Target Geometry input rectangle dimensions
xbound = 640
ybound = 480

try:
    arduino.reset_input_buffer()
    results = model(source, save=False, stream=True, imgsz=640, show=True, project=False, conf=0.5)

    for result in results:
        #counter += 1


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

            targetX = (right + left) / 2.0
            targetY = top
            pback, tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)

            
            break  # Exit loop if person is found

        if arduino.in_waiting > 0:
            rcdata = read_latest_serial_data(arduino)
            #print(rcdata)
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

            ch3_value = rcvalues.get('Ch3', 0)
            dist = rcvalues.get('Dist1', 0)
            #steer = 180 - mapPWMtoAngle(rcvalues.get('Ch1', 0))
            #throttle = mapPWMtoThrottle(rcvalues.get('Ch2', 0))

            if ch3_value < 1000:
                rcmode = 'default'
                ws = f"{int(tback)}|{int(pback)}\n"
                arduino.write(ws.encode())

            elif 1200 <= ch3_value <= 1600:
                rcmode = 'Fly By Wire'
                ws = f"{int(tback)}|{int(pback)}|{int(rcvalues.get('Ch1', 0))}|{int(rcvalues.get('Ch2', 0))}\n"
                arduino.write(ws.encode())
                if dist < 50:
                    print("PROXIMITY")
            elif ch3_value > 2000:
                rcmode = 'Auto'
                ws = f"{int(tback)}|{int(pback)}|{int(pback)}|1500\n"
                arduino.write(ws.encode())
                if dist < 50:
                    print("PROX")

except Exception as e:
    print(e)
    traceback.print_exc()
    print("Cutting throttle")

finally:
    arduino.close()
    print("full exit")
