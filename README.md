This project uses a mini-PC mounted on a RC car to point a turret and drive towards objects detected on its camera. 

The PC/Jetson captures video from the webcam and uses pre-trained models (Nvidia peoplenet-pruned, YOLO v8 v11) object detection network.
Jetson uses a PCA 9685 to control the turret and driving servos. Jetson points pan-tilt servos (turret) to first person detected.
ESP32 reads PWM signal from RC receiver's 4 channels, each with PWM values. ESP32 converts RC values to a string and sends to USB serial console.
Jetson reads serial data from USB and controls steering servos according to the 'mode' set by channel 3: 
  Mode 1: do nothing. car will not move
  Mode 2: Fly by Wire. pass thru RC signals to steer and throttle servos
  Mode 3: Autonomous. Steering angle auto-set to pan servo. Throttle is fly by wire (RC/human control)

Software:
oc11.py has python code to run on Jetson Nano
esp32ReadPWM.ino has C code to run on ESP32

Hardware:
NVIDIA Jetson Nano
Adafruit Feather Huzzah ESP32
PCA9685 Servo Driver
MG Servos with pan tilt brackets
Logitech C920 webcam
Redcat Everest 10 Crawler (4 wheel steering and ESC Throttle)
Buck converters
Batteries: 2x 2S Lipo 7.4V 5200 maH , 2s Lipo 7.4V 1500maH

Connections:
RC receiver -- GPIO pins --> ESP32
ESP32 -- USB --> Jetson
Jetson -- i2c Serial --> PCA9685
PCA9685 --> Pan Tilt Servos
PCA9685 --> Steer and Throttle servos on car

