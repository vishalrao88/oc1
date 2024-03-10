// This read 4 channels of PWM data connected to GPIO pins and write the results to serial console

#define CH1 12
#define CH2 27
#define CH3 33
#define CH4 15
//#define CH5 10
//#define CH6 11



//Servo object
//Servo ESC1;
//Servo steer;

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value; 

/*
int ch5Value;
// Boolean to represent switch value
bool ch6Value;
*/ 

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
//Serial.print(channelInput);
//  Serial.print(" Pulse Input: ");
//  Serial.println(ch);
  
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

/* 
// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
 
*/


void setup(){
  // Set up serial monitor
  Serial.begin(921600);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);



//TIMSK0=0;
/*
  ESC1.attach(9); //Adds ESC to certain pin. arm();
  steer.attach(10); //Adds ESC to certain pin. arm();
  ESC1.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  steer.writeMicroseconds(1500);
  delay(5000);  // delay to allow the ESC to recognize the stopped signal.
*/

}
 
 
void loop() {
  // Get values for each channel
  ch1Value = readChannel(CH1, 1000, 2000, 1500);
//  ch2Value = readChannel(CH2, -100, 100, 0);
  ch2Value = readChannel(CH2, 1000, 2000, 1500);
  ch3Value = readChannel(CH3, 1000, 2000, 1500);
  ch4Value = readChannel(CH4, 1000, 2000, 1500);



  // Print to Serial Monitor
  Serial.print("Ch1:");
  Serial.print(ch1Value);
  Serial.print(" | Ch2:");
  Serial.print(ch2Value);
  Serial.print(" | Ch3:");
  Serial.print(ch3Value);
  Serial.print(" | Ch4:");
  Serial.println(ch4Value);


//  int pwmVal = map(ch2Value,0, 1023, 1100, 1900);

  //ESC1.writeMicroseconds(ch2Value);
  
 // if (ch1Value>=1460 && ch1Value<=1510) {
 //   ch1Value=1500;
  //  Serial.println("saved");
  //}
 // steer.writeMicroseconds(ch1Value);
 // steer.write(ch1Value);
 
 //steer.writeMicroseconds(ch1Value);
 // delay(5);
}