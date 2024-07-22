#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "crsf.h"
#include "SR04.h"
#define RXD2 16
#define TXD2 17
#define TRIG_PIN 32
#define ECHO_PIN 14

#define PCLENCOM 4
#define PCLENDELIM '|'

#define CH1DEF 1500
#define CH2DEF 1500
#define CH3DEF 989
#define CH4DEF 1500

#define SBUS_BUFFER_SIZE 25



uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};
size_t pcCommCount;
String serialString;
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long a;

void handleCommand(String command) {
  command.trim();
  if (command.startsWith("servo1:")) {
    int position = command.substring(7).toInt();
    //setServoPosition(0, position); // Control servo 1
  } else if (command.startsWith("servo2:")) {
    int position = command.substring(7).toInt();
    //setServoPosition(1, position); // Control servo 2
  } else if (command == "getDistance") {
    Serial.print("Distance:");
    Serial.println(a);
  }
}

void parseComm(const std::string &str, std::string pcComm[], size_t &count) {
    count = 0;
    size_t start = 0;
    size_t end = str.find(PCLENDELIM);

    while (end != std::string::npos && count < PCLENCOM) {
        pcComm[count++] = str.substr(start, end - start);
        start = end + 1;
        end = str.find(PCLENDELIM, start);
    }

    if (count < PCLENCOM) {
        pcComm[count++] = str.substr(start);
    }
}


void setup() {
  Serial.begin(921600);       // PC Serial
  Serial1.begin(921600, SERIAL_8N1, RXD2, TXD2);  // Serial1 for receiver
  Serial1.setTimeout(30);     // Set timeout for Serial1
}


void loop() {
  a = sr04.Distance();  // Measure distance
  
  //default values
  _raw_rc_values[0] = CH1DEF;  // Default value for channel 1
  _raw_rc_values[1] = CH2DEF;  // Default value for channel 2
  _raw_rc_values[2] = CH3DEF;  // Default value for channel 3
  _raw_rc_values[3] = CH4DEF;  // Default value for channel 4

  // Handle incoming serial data from Serial1
  if (Serial1.available()) {
    size_t numBytesRead = Serial1.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if (numBytesRead > 0) {
      crsf_parse(_rcs_buf, SBUS_BUFFER_SIZE, _raw_rc_values, &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
    }
  }

  // Print RC values and distance
  Serial.print("Ch1:"); Serial.print(_raw_rc_values[0]);
  Serial.print(" | Ch2:"); Serial.print(_raw_rc_values[1]);
  Serial.print(" | Ch3:"); Serial.print(_raw_rc_values[2]);
  Serial.print(" | Ch4:"); Serial.print(_raw_rc_values[3]);
  Serial.print(" | Dist1:"); Serial.println(a);

  // Handle incoming serial commands from the PC
  if (Serial.available()) {
    serialString = Serial.readStringUntil('\n'); // Read input as Arduino String
    std::string command = serialString.c_str(); // Convert Arduino String to std::string

    std::string pcComm[PCLENCOM];
    pcCommCount=0;
    parseComm(command, pcComm, pcCommCount);

    // Optional: Print parsed command parts for debugging
    for (size_t i = 0; i < pcCommCount; ++i) {
      Serial.print("Parsed Part ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(pcComm[i].c_str());
    }

  }
}
