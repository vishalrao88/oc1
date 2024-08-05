import serial
import traceback
from ultralytics import YOLO
import threading
import time

# PARAMETERS AND VARIABLES
model_path = "yolov8n_openvino_model"
try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Error loading model: {e}")
    raise

source = 0
#source = 'https://www.youtube.com/watch?v=tTUwmi0olXI'
print("Running inference on source:", source)

# Target Geometry input rectangle dimensions
xbound = 640
ybound = 480

running = True



# Initialize serial connection
arduino = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1,  # Short timeout for non-blocking read
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    write_timeout=100
)

# FUNCTIONS
def read_latest_serial_data(serial_port):
    buffer = ""
    while serial_port.in_waiting > 0:
        try:
            buffer += serial_port.read(serial_port.in_waiting).decode()
            if '\n' in buffer:
                lines = buffer.split('\n')
                latest_data = lines[-2]  # The second-to-last line (latest complete line)
                buffer = lines[-1]  # Remaining data (possibly incomplete line)
                return latest_data.strip()
        except UnicodeDecodeError as e:
            print(f"Error decoding serial data: {e}")
            buffer = ""  # Clear buffer on decode error
    return ""

def movePanTilt(targetX, targetY, panMin=1800, panMax=1200, tiltMin=15, tiltMax=90, xbound=640, ybound=480):
    # Calculate panOut with reverse scaling
    panOut = panMax + ((targetX / xbound) * (panMin - panMax))
    
    # Calculate tiltOut normally
    tiltOut = (targetY / ybound) * (tiltMax - tiltMin) + tiltMin

    # Clamp the values to their respective min and max
    panOut = max(min(panOut, max(panMin, panMax)), min(panMin, panMax))
    tiltOut = max(min(tiltOut, max(tiltMin, tiltMax)), min(tiltMin, tiltMax))

    return panOut, tiltOut


def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Thread function for inference processing
def process_inference():
    global running
    pback = 90
    tback = 0

    try:
        results = model(source, save=False, stream=True, imgsz=640, show=True, project=False, conf=0.5)
        for result in results:
            if not running:
                break

            detectcount = 0


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
                
                detectcount=1
                break  # Exit loop if person is found

            if arduino.in_waiting > 0:
                rcdata = read_latest_serial_data(arduino)
                ws=""
                if rcdata:
                    #print("Read data: ", rcdata)
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

                    if ch3_value < 1000:
                        rcmode = 'default'
                        if detectcount>0:
                            ws = f"{int(tback)}|{int(pback)}\n"
                        arduino.write(ws.encode())

                    elif 1200 <= ch3_value <= 1600:
                        rcmode = 'Fly By Wire'
                        #print(rcmode)
                        #if detectcount>0:
                        ws = f"{int(tback)}|{int(pback)}|{rcvalues.get('Ch1', 0)}|{rcvalues.get('Ch2', 0)}\n"
                        arduino.write(ws.encode())
                        if dist < 50:
                            print("PROXIMITY")
                    elif ch3_value > 2000:
                        rcmode = 'Auto'

                        if (detectcount>0):
                            st,dxx = movePanTilt(targetX, targetY, 1800,1200,15,90)
                            print(rcmode,":",st)
                            ws = f"{int(tback)}|{int(pback)}|{int(st)}\n"
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

# Start inference processing in a separate thread
inference_thread = threading.Thread(target=process_inference)
inference_thread.start()

try:
    while True:
        time.sleep(1)
        
except KeyboardInterrupt:
    running = False
    inference_thread.join()
    print("Stopped")
