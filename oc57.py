"""
oc57.py - 3-Thread Architecture for Person-Tracking RC Car

Thread 1 (Inference): Runs YOLO detection, updates shared state with detection results
Thread 2 (Serial I/O): Reads RC data from ESP32, applies mode logic, sends servo commands
Thread 3 (Main): Supervises threads, handles Ctrl+C, ensures safe shutdown

Based on oc56.py with decoupled inference and serial I/O for better performance.
"""

import serial
import traceback
from ultralytics import YOLO
import threading
import time


# ============================================================
# SHARED STATE
# ============================================================
class SharedState:
    """Thread-safe shared state between inference and serial threads."""
    def __init__(self):
        self.lock = threading.Lock()
        # Detection results (written by inference thread, read by serial thread)
        self.target_x = 320.0
        self.target_y = 240.0
        self.person_detected = False
        self.pan_out = 90
        self.tilt_out = 0
        self.pan_out_auto = 1500  # PWM-range steering for auto mode

    def update_detection(self, target_x, target_y, pan_out, tilt_out, pan_out_auto):
        with self.lock:
            self.target_x = target_x
            self.target_y = target_y
            self.person_detected = True
            self.pan_out = pan_out
            self.tilt_out = tilt_out
            self.pan_out_auto = pan_out_auto

    def clear_detection(self):
        with self.lock:
            self.person_detected = False

    def get_state(self):
        with self.lock:
            return {
                'target_x': self.target_x,
                'target_y': self.target_y,
                'person_detected': self.person_detected,
                'pan_out': self.pan_out,
                'tilt_out': self.tilt_out,
                'pan_out_auto': self.pan_out_auto,
            }


# ============================================================
# PARAMETERS AND VARIABLES
# ============================================================
model_path = "yolov8n.pt"
try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Error loading model: {e}")
    raise

source = 0
print("Running inference on source:", source)

xbound = 640
ybound = 480

running = True
shared_state = SharedState()

# Initialize serial connection
arduino = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    write_timeout=100
)


# ============================================================
# FUNCTIONS (unchanged from oc56.py)
# ============================================================
def read_latest_serial_data(serial_port):
    buffer = ""
    while serial_port.in_waiting > 0:
        try:
            buffer += serial_port.read(serial_port.in_waiting).decode()
            if '\n' in buffer:
                lines = buffer.split('\n')
                latest_data = lines[-2]
                buffer = lines[-1]
                return latest_data.strip()
        except UnicodeDecodeError as e:
            print(f"Error decoding serial data: {e}")
            buffer = ""
    return ""


def movePanTilt(targetX, targetY, panMin=1800, panMax=1200, tiltMin=15, tiltMax=90, xbound=640, ybound=480):
    panOut = panMax + ((targetX / xbound) * (panMin - panMax))
    tiltOut = (targetY / ybound) * (tiltMax - tiltMin) + tiltMin
    panOut = max(min(panOut, max(panMin, panMax)), min(panMin, panMax))
    tiltOut = max(min(tiltOut, max(tiltMin, tiltMax)), min(tiltMin, tiltMax))
    return panOut, tiltOut


def mapPWMtoThrottle(value, in_min=987, in_max=2012, out_min=-1, out_max=1):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def mapPWMtoAngle(value, in_min=987, in_max=2012, out_min=20, out_max=160):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# ============================================================
# THREAD 1: INFERENCE
# ============================================================
def inference_thread_func():
    """Runs YOLO detection and updates shared state. No serial I/O."""
    global running

    try:
        results = model(source, save=False, stream=True, imgsz=640, show=True, project=False, conf=0.5, device=0)
        for result in results:
            if not running:
                break

            person_found = False
            boxes = result.boxes
            for box in boxes:
                class_id = box.cls[0]
                item = result.names[int(class_id)]
                if item != 'person':
                    continue

                left, top, right, bottom = box.xyxy[0]

                targetX = (right + left) / 2.0
                targetY = top

                # Calculate servo angles (degree-based for turret)
                pback, tback = movePanTilt(targetX, targetY, 35, 145, 15, 90)

                # Calculate PWM-based steering for auto mode
                st, _ = movePanTilt(targetX, targetY, 1800, 1200, 15, 90)

                shared_state.update_detection(targetX, targetY, pback, tback, st)
                person_found = True
                break  # Track first person found

            if not person_found:
                shared_state.clear_detection()

    except Exception as e:
        print(f"Inference error: {e}")
        traceback.print_exc()

    print("Inference thread exiting")


# ============================================================
# THREAD 2: SERIAL I/O
# ============================================================
def serial_thread_func():
    """Reads RC data from ESP32, applies mode logic, sends servo commands."""
    global running

    try:
        while running:
            if arduino.in_waiting > 0:
                rcdata = read_latest_serial_data(arduino)
                if not rcdata:
                    continue

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

                # Get latest detection state
                state = shared_state.get_state()
                ws = ""

                if ch3_value < 1000:
                    rcmode = 'default'
                    if state['person_detected']:
                        ws = f"{int(state['tilt_out'])}|{int(state['pan_out'])}\n"
                    if ws:
                        arduino.write(ws.encode())

                elif 1200 <= ch3_value <= 1600:
                    rcmode = 'Fly By Wire'
                    ws = f"{int(state['tilt_out'])}|{int(state['pan_out'])}|{rcvalues.get('Ch1', 0)}|{rcvalues.get('Ch2', 0)}\n"
                    arduino.write(ws.encode())
                    if dist < 50:
                        print("PROXIMITY")

                elif ch3_value > 2000:
                    rcmode = 'Auto'
                    if state['person_detected']:
                        print(rcmode, ":", state['pan_out_auto'])
                        ws = f"{int(state['tilt_out'])}|{int(state['pan_out'])}|{int(state['pan_out_auto'])}\n"
                        arduino.write(ws.encode())
                    if dist < 50:
                        print("PROX")

            else:
                # No serial data waiting — sleep briefly to avoid busy-waiting
                time.sleep(0.01)

    except Exception as e:
        print(f"Serial error: {e}")
        traceback.print_exc()

    print("Serial thread exiting")


# ============================================================
# MAIN THREAD: SUPERVISOR
# ============================================================
print("Starting threads...")

inference_thread = threading.Thread(target=inference_thread_func, name="InferenceThread", daemon=True)
serial_thread = threading.Thread(target=serial_thread_func, name="SerialThread", daemon=True)

inference_thread.start()
serial_thread.start()

try:
    while running:
        time.sleep(0.5)
        # Check if inference thread died unexpectedly
        if not inference_thread.is_alive():
            print("Inference thread stopped")
            running = False

except KeyboardInterrupt:
    print("\nShutting down...")
    running = False

finally:
    # Wait for threads to finish
    inference_thread.join(timeout=5)
    serial_thread.join(timeout=5)

    # Safety: send neutral command and close serial
    try:
        print("Cutting throttle")
        arduino.write(b"0|90|1500|1500\n")
        time.sleep(0.1)
        arduino.close()
    except Exception:
        pass

    print("Full exit")
