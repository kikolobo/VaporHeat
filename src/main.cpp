#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include "Heater.h"
#include "TempSensor.h"
#include "Oven.h"
#include "Buzzer.h"
// #include "Comm.h"


const uint8_t HEATER1_PIN = A10;
const uint8_t HEATER2_PIN = A9;
const uint8_t HEATER3_PIN = A8;
const uint8_t TEMPA_PIN = A4;
const uint8_t TEMPB_PIN = A5;
const uint8_t TEMPC_PIN = A6;
const uint8_t BUZZER_PIN = A12;
const uint8_t LED_PIN = 13;

const float OFFSET_OVEN_A = 14.10;
const float OFFSET_OVEN_B = 21.5;

const uint32_t TIME_MS_TO_TEMP_OVEN_A = 60000;
const uint32_t TIME_MS_TO_TEMP_OVEN_B = 60000;

uint64_t prevPrintTimeStamp = 0;


int stableCounter = 0;
bool steadyState=false;

CRGB leds[1];

xlab::Heater* heaterA = new xlab::Heater(HEATER1_PIN, 4);
xlab::Heater* heaterB = new xlab::Heater(HEATER2_PIN, 15);

xlab::TempSensor* tempSensorA = new xlab::TempSensor(TEMPA_PIN);
xlab::TempSensor* tempSensorB = new xlab::TempSensor(TEMPB_PIN);
xlab::TempSensor* tempSensorC = new xlab::TempSensor(TEMPC_PIN);

xlab::Oven::PIDConstants const60 = xlab::Oven::PIDConstants(10.75,0.70,15.63);
xlab::Oven::PIDConstants const90 = xlab::Oven::PIDConstants(10.75,0.70,15.63);

xlab::Oven* ovenA = new xlab::Oven(heaterA, tempSensorA, const60, 120, 150, TIME_MS_TO_TEMP_OVEN_A);
xlab::Oven* ovenB = new xlab::Oven(heaterB, tempSensorB, const90, 120, 220, TIME_MS_TO_TEMP_OVEN_B);


xlab::Buzzer buzzer = xlab::Buzzer(BUZZER_PIN, 3);

// xlab::Comm comm = xlab::Comm();

void checkForSerialCommands();

void setup() {   
  analogReadResolution(12);
  Serial.begin(115200); 
  ovenB->setTemp(0);
  ovenA->setTemp(62.0 + OFFSET_OVEN_A);
  

  FastLED.addLeds<SK6812, LED_PIN, RGB>(leds, 1).setCorrection(Typical8mmPixel);  
  Serial.println("[Boot] v1.0");
  leds[0] = CRGB::Red;
  FastLED.show();
  // comm.setupBLE();
}

void loop() { 
  ovenA->heartBeat();
  ovenB->heartBeat();
  
  
  if (ovenA->isAwaiting() == true && ovenB->isSteady() == false) 
  {           
    if (ovenB->isAwaiting() == false && ovenB->getState() == xlab::Oven::State::OFF) {
      leds[0] = CRGB::Yellow;
      FastLED.show();
      Serial.println("# OV_B = Heating");
      ovenB->setTemp(92.5 + OFFSET_OVEN_B);            
    } 

    if (ovenB->isSteady() == true) {      
      leds[0] = CRGB::Green;
      FastLED.show();
    }   
  }
  

  

  checkForSerialCommands();


  if (millis() - prevPrintTimeStamp > 100) {

    
    prevPrintTimeStamp = millis();

    String message;

    message = message + String(ovenA->getTargetTemp()) + "\t";
    message = message + String(ovenA->getTemp()) + "\t";
    message = message + String(ovenA->getPowerOutput()) + "\t";
    message = message + ovenA->getStateString() + "\t";
    // Serial.print(ovenA.getSteadyness()); Serial.print("\t");
    message = message + String(ovenB->getTargetTemp()) + "\t";
    message = message + String(ovenB->getTemp()) + "\t";
    message = message + String(ovenB->getPowerOutput()) + "\t";
    message = message + ovenB->getStateString() + "\t";
    message = message + String(tempSensorC->readTempDegC()) + "\t";
    // Serial.print(ovenB.getSteadyness()); Serial.print("\t");            
    Serial.println(message);     
    // comm.send(message);



  }

  
  
}




void checkForSerialCommands() {
  char incomingByte = 0;
  bool inMsg = false;

  if (Serial.available()) {
   incomingByte = Serial.read();
   inMsg = true;
  }

  // String m = comm.read();
  // if (m.length() > 0) {
  //   incomingByte = m.toInt();    
  //   inMsg = true;
  // }

    if (inMsg == true) {
    xlab::Oven::PIDConstants pidConstants = ovenA->getPIDConstants();

    float kP = pidConstants.Kp;
    float kI = pidConstants.Ki;
    float kD = pidConstants.Kd;
    
    if (incomingByte == 'q') {
      kP = kP - 0.5;
        ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == 'w') {
      kP = kP + 0.5;
      ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == 'a') {
      kI = kI - 0.1;
        ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == 's') {
      kI = kI + 0.1;
      ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == 'z') {
      kD = kD - 0.05;
        ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == 'x') {
      kD = kD + 0.05;
      ovenA->tunePID(kP, kI, kD);
    }

    if (incomingByte == '1') {
      ovenA->setTemp(62.0 + OFFSET_OVEN_A);      
    }

    if (incomingByte == '2') {
      ovenB->setTemp(92.0 + OFFSET_OVEN_B);      
    }

    if (incomingByte == '0') {
      ovenB->setTemp(0);      
      ovenA->setTemp(0);      
    }


    if (incomingByte == 't') {
      Serial.println("----------AutoTune Start-----------");
      Serial.println("Aiming for: " + String(ovenA->getTargetTemp()));
      ovenA->setTemp(ovenA->getTargetTemp());
      ovenA->autoTune();      
    }

    Serial.println("# ");
    Serial.print("(");
    Serial.print(kP);
    Serial.print("-");
    Serial.print(kI);
    Serial.print("-");
    Serial.print(kD);
    Serial.print("-");
    Serial.print(ovenA->getTargetTemp());
    Serial.println(")");

    }


}