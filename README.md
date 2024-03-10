This project uses a Jetson Nano and ESP32 on a 1:10 RC Crawler with steering and ESC throttle control

The Jetson uses Nvidia captures video from a pre-trained models like TAO peoplenet-pruned object detection network
Jetson points pan-tilt servos (turret) to first person detected
ESP32 reads PWM signal from RC receiver - 4 channels, each with PWM values. Converts to string and sends to USB serial console
Jetson reads serial data from USB. 
  Mode 1: do nothing. 
  Mode 2: Fly by Wire. pass thru signals to steer and throttle servos
  Mode 3: Autonomous. Steering angle set to pan servo. Throttle is fly by wire (rc control)


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

