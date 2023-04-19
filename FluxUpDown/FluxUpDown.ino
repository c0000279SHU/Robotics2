#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  175 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
#define LegMin 200
#define LegMid 250
#define LegMax 300
#define HipFrontReach 100
#define HipMin 175 //forwards
#define HipMax 350 //backwards
#define HipMid 295
  //leg pins
#define FrontLeftLeg 0
#define FrontLeftHip 8
#define RearLeftLeg 1
#define RearLeftHip 11
#define FrontRightLeg 3
#define FrontRightHip 9
#define RearRightLeg 2
#define RearRightHip 10
int lastPWML=0;
int lastPWMH=0;
//Servo Controller

//Controller Variables
#define RJoyXpin A2
#define RJoyYpin A1
#define LJoyXpin A5
#define LJoyYpin A6
int RJoyX,RJoyY,LJoyX,LJoyY;
#define BtnPin 0

//sets the positive and negative deadzones of the controller
#define JoystickHigh 400
#define JoystickLow 400
#define Deadzone 200
bool inDeadRX, inDeadRY, inDeadLX, inDeadLY;

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  initLegs();
}

//----SERVO FUNCTIONS--------
void centerHips(){
  //eases hips to the mid point from the previous point
  if(lastPWMH<HipMid){
    for (uint16_t pulselen = lastPWMH; pulselen < HipMid; pulselen++) {
        pwm.setPWM(FrontLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightHip, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(RearRightHip, 0, pulselen); delay(5);
      }
  }
  else if(lastPWMH>HipMid){
    for (uint16_t pulselen = lastPWMH; pulselen > HipMid; pulselen--) {
        pwm.setPWM(FrontLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightHip, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(RearRightHip, 0, pulselen); delay(5);
      }
  }
  lastPWMH=HipMid;
}
void midLegs(){
  //eases legs to the mid point from the previous point
  if(lastPWML<LegMid){
    for (uint16_t pulselen = lastPWML; pulselen < LegMid; pulselen++) {
        pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
      }
  }
  else if(lastPWML>LegMid){
    for (uint16_t pulselen = lastPWML; pulselen > LegMid; pulselen--) {
        pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
      }
  }
  lastPWML=LegMid;
}
void stand(){
  //center hips to stand
  if(lastPWMH!=HipMid){
    centerHips();
    delay(100); 
  }
  //eases legs from last location to standing locatoin
  for (uint16_t pulselen = lastPWML; pulselen < LegMax; pulselen++) {
    pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
  }
  lastPWML=LegMax;

}
void sit(){
  //center hips to sit
  if(lastPWMH!=HipMid){
    centerHips();
    delay(100); 
  }
  //eases legs from last location to sitting locatoin
  for (uint16_t pulselen = lastPWML; pulselen > LegMin; pulselen--) {
    pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
  }
  lastPWML=LegMin;
}
void initLegs(){
  //*
  lastPWML=LegMid;
  lastPWMH=HipMid;

  pwm.setPWM(FrontLeftLeg, 0, LegMax); delay(5);
  pwm.setPWM(FrontLeftHip, 0, HipMid); delay(5);

  pwm.setPWM(FrontLeftLeg, 0, LegMid); delay(5);
  pwm.setPWM(FrontLeftHip, 0, HipMid); delay(5);

  pwm.setPWM(RearRightLeg, 0, LegMid); delay(5);
  pwm.setPWM(RearRightHip, 0, HipMid); delay(5);

  pwm.setPWM(RearLeftLeg, 0, LegMid); delay(5);
  pwm.setPWM(RearLeftHip, 0, HipMid); delay(5);
  //*/
}

//-----SERVO FUNCTIONS END--------

void loop() {
  //First take input from the controller
  RJoyX=map(analogRead(RJoyXpin), 0, 1025, JoystickLow, -JoystickHigh);
  RJoyY=map(analogRead(RJoyYpin), 0, 1025, JoystickLow, -JoystickHigh);
  LJoyX=map(analogRead(LJoyXpin), 0, 1025, JoystickLow, -JoystickHigh);
  LJoyY=map(analogRead(LJoyYpin), 0, 1025, JoystickLow, -JoystickHigh);
  //This maps the Joystick values to easy to work with values
  if(RJoyX > -Deadzone && RJoyX < Deadzone){inDeadRX=true;} else {inDeadRX=false;}
  if(RJoyY > -Deadzone && RJoyY < Deadzone){inDeadRY=true;} else {inDeadRY=false;}
  if(LJoyX > -Deadzone && LJoyX < Deadzone){inDeadLX=true;} else {inDeadLX=false;}
  if(LJoyY > -Deadzone && LJoyY < Deadzone){inDeadLY=true;} else {inDeadLY=false;}
  //This determines if a joystick is in the "deadzone" AKA center

  //if left controller is "forwards"
  //sit down
  else if(!inDeadLX && LJoyX > Deadzone){sit();}

  //if left controller if "backwards"
  //stand up
  else if(!inDeadLX && LJoyX < -Deadzone){stand();}

  //*
  Serial.print("RJoyX: ");
  Serial.print(RJoyX);
  Serial.print("\t");
  
  Serial.print("RJoyY: ");
  Serial.print(RJoyY);
  Serial.print("\t");

  Serial.print("LJoyX: ");
  Serial.print(LJoyX);
  Serial.print("\t");

  Serial.print("KJoyY: ");
  Serial.println(LJoyY);
  //*/

}
