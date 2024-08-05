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

#define STEER 4
#define THROTTLE 5
#define PAN 1
#define TILT 0

#define SBUS_BUFFER_SIZE 25

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};
size_t pcCommCount;
String serialString;
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

//bool directModeActive = false;

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

uint16_t angle2us(uint16_t angle, uint16_t minAngle = 0, uint16_t maxAngle = 180) {
    uint16_t us = 1000 + static_cast<uint16_t>((angle - minAngle) * (1000.0 / (maxAngle - minAngle)));
    return us;
}

void setup() {
    Serial.begin(921600); // PC Serial
    Serial1.begin(921600, SERIAL_8N1, RXD2, TXD2); // Serial1 for receiver
    Serial1.setTimeout(1); // Set timeout for Serial1 rc receiver
    Serial.setTimeout(1); // Set timeout for Serial interface to pc

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);

    //DirectModeActive = false;

    // Default values
    _raw_rc_values[0] = CH1DEF; // Default value for channel 1
    _raw_rc_values[1] = CH2DEF; // Default value for channel 2
    _raw_rc_values[2] = CH3DEF; // Default value for channel 3
    _raw_rc_values[3] = CH4DEF; // Default value for channel 4

    Serial1.flush();
    Serial.flush();
}

void handleRCData() {
    if (Serial1.available()) {
        size_t numBytesRead = Serial1.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
        if (numBytesRead > 0) {
            crsf_parse(_rcs_buf, SBUS_BUFFER_SIZE, _raw_rc_values, &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
        }
    }
}

void handlePCData() {
    if (Serial.available()) {
        serialString = Serial.readStringUntil('\n'); // Read input as Arduino String
        std::string command = serialString.c_str(); // Convert Arduino String to std::string

        std::string pcComm[PCLENCOM];
        pcCommCount = 0;
        parseComm(command, pcComm, pcCommCount);

        uint16_t us;
        int angle;

        if (pcCommCount >= 1) {
            angle = std::stoi(pcComm[0].c_str());
            us = angle2us(angle);
            pwm.writeMicroseconds(TILT, us);
        }
        if (pcCommCount >= 2) {
            angle = std::stoi(pcComm[1].c_str());
            us = angle2us(angle);
            pwm.writeMicroseconds(PAN, us);
        }
        if (pcCommCount == 3) {
            us = std::stoi(pcComm[2].c_str());
            //us = angle2us(angle);
            pwm.writeMicroseconds(STEER, us);
        }
        if (pcCommCount == 4) {
            us = std::stoi(pcComm[2].c_str());
            pwm.writeMicroseconds(STEER, us);
            us = std::stoi(pcComm[3].c_str());
            pwm.writeMicroseconds(THROTTLE, us);

        }
    }
}


void checkDirectMode() {
    bool DirectModeActive = _raw_rc_values[2] <= 1000;
    
    if (DirectModeActive) {
        pwm.writeMicroseconds(STEER, _raw_rc_values[0]); // steer
        pwm.writeMicroseconds(THROTTLE, _raw_rc_values[1]); // throttle
    }
}

void printStatus() {
    Serial.print("Ch1:"); Serial.print(_raw_rc_values[0]);
    Serial.print(" | Ch2:"); Serial.print(_raw_rc_values[1]);
    Serial.print(" | Ch3:"); Serial.print(_raw_rc_values[2]);
    Serial.print(" | Ch4:"); Serial.print(_raw_rc_values[3]);
    Serial.println(" | Dist1:1999");
}

void loop() {
    handleRCData();
    checkDirectMode();
    printStatus();
    handlePCData();

}
