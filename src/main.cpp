#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include "Heater.h"
#include "TempSensor.h"
#include "Oven.h"
#include "Buzzer.h"
#include "Indicator.h"
#include "StateMachine.h"
// #include "Comm.h"

const uint8_t HEATER1_PIN = A10;
const uint8_t HEATER2_PIN = A9;
const uint8_t HEATER3_PIN = A8;
const uint8_t TEMPA_PIN = A4;
const uint8_t TEMPB_PIN = A5;
const uint8_t TEMPC_PIN = A6;
const uint8_t BUZZER_PIN = A12;
const uint8_t LED_PIN = A7;
const uint8_t SW_A = 21;

const float OFFSET_OVEN_A = 14.10;
const float OFFSET_OVEN_B = 21.5;

const uint32_t TIME_MS_TO_TEMP_OVEN_A = 60000;
const uint32_t TIME_MS_TO_TEMP_OVEN_B = 60000;

uint64_t prevPrintTimeStamp = 0;

bool swA_State = LOW;
bool swA_LastState = LOW;
bool swACommandPending = false;

int stableCounter = 0;
bool steadyState=false;

uint32_t targetTimeForTest = 0;
uint32_t stopwatchTimeStamp = 0;
uint32_t stopwatchDownSWTimeStamp = 0;

// CRGB leds[1];

xlab::Heater* heaterA = new xlab::Heater(HEATER1_PIN, 4);
xlab::Heater* heaterB = new xlab::Heater(HEATER2_PIN, 15);

xlab::TempSensor* tempSensorA = new xlab::TempSensor(TEMPA_PIN);
xlab::TempSensor* tempSensorB = new xlab::TempSensor(TEMPB_PIN);
xlab::TempSensor* tempSensorC = new xlab::TempSensor(TEMPC_PIN);

xlab::Oven::PIDConstants const60 = xlab::Oven::PIDConstants(10.75,0.70,15.63);
xlab::Oven::PIDConstants const90 = xlab::Oven::PIDConstants(10.75,0.70,15.63);

xlab::Oven* ovenA = new xlab::Oven(heaterA, tempSensorA, const60, 120, 200, TIME_MS_TO_TEMP_OVEN_A);
xlab::Oven* ovenB = new xlab::Oven(heaterB, tempSensorB, const90, 120, 230, TIME_MS_TO_TEMP_OVEN_B);

xlab::Indicator* indicator = new xlab::Indicator(LED_PIN);

xlab::StateMachine* ovenState = new xlab::StateMachine();
xlab::StateMachine* testState = new xlab::StateMachine();

void checkForSerialCommands();
void logEvents();
void updateState();
void checkForButtonCommands();
void manageMachine();

void setup() {   
  analogReadResolution(12);
  pinMode(SW_A, INPUT_PULLDOWN);
  indicator->setFrequency(1000);
  indicator->setBrightness(100);
  Serial.begin(115200); 
  indicator->setInstantColor(CRGB::White);
  delay(500);
  

  
  ovenA->setTemp(62.0 + OFFSET_OVEN_A); //Heat the first OVEN.
  ovenB->setTemp(0);
  ovenState->setState(xlab::StateMachine::State::OVEN_60_HEATING);
  testState->setState(xlab::StateMachine::State::WAITING);
  
  
  Serial.println("[Boot] v1.1");
  indicator->setColor(CRGB::OrangeRed);
}

void loop() { 
  ovenA->heartBeat();
  ovenB->heartBeat();
  indicator->heartBeat();
  // buzzer->heartBeat();

  
  updateState();
  checkForButtonCommands();  
  manageMachine();
  checkForSerialCommands();
  logEvents();  
}

// ###################################################

void updateState() 
{
  if (ovenA->isAwaiting() == true && ovenB->isSteady() == false) 
    {           
      if (ovenB->isAwaiting() == false && ovenB->getState() == xlab::Oven::State::OFF) {        
        indicator->setColor(CRGB::Yellow);        
        Serial.println("# OV_B = Heating");
        ovenB->setTemp(92.5 + OFFSET_OVEN_B);            
        ovenState->setState(xlab::StateMachine::State::OVEN_90_HEATING);
      }      
    } else if ((ovenB->isSteady() == true && ovenA->isSteady() == true) && ovenState->getState() == xlab::StateMachine::State::OVEN_90_HEATING) { 
        indicator->setFrequency(0);
        indicator->setColor(CRGB::DarkGreen);
        ovenState->setState(xlab::StateMachine::State::OVENS_READY);
        testState->setState(xlab::StateMachine::State::READY_FOR_TESTING);
      }   

}


void logEvents() 
{
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
    message = message + String(swA_State) + "\t";
    
    // Serial.print(ovenB.getSteadyness()); Serial.print("\t");            
    Serial.println(message);     
    // comm.send(message);
  }
}



void checkForButtonCommands() {
  swA_State = digitalRead(SW_A);
  if (swA_State == HIGH && swA_LastState == LOW) {
      swA_LastState = HIGH;
      stopwatchDownSWTimeStamp = millis();
  }

  if (swA_State == LOW && swA_LastState == HIGH) {
      swA_LastState = LOW;
      swACommandPending = true;
  }

  if ((swA_State == HIGH && swA_LastState == HIGH && stopwatchDownSWTimeStamp > 0) && (ovenState->getState() == xlab::StateMachine::State::OVENS_READY)) {
      if (millis() - stopwatchDownSWTimeStamp >= 5000) {
        Serial.println("# RESET COMMAND");
          testState->setState(xlab::StateMachine::State::READY_FOR_TESTING);          
          indicator->setColor(CRGB::DarkGreen);
          indicator->setFrequency(0);
        stopwatchDownSWTimeStamp = 0;
      }
  }
  

}

void manageMachine() {
  if (swACommandPending == true) {
    Serial.println("#Switch Command Detected");
    swACommandPending = false;
    if (ovenState->getState() == xlab::StateMachine::State::OVENS_READY) {
      
        if (testState->getState() == xlab::StateMachine::State::READY_FOR_TESTING) {          
          Serial.println("#OVEN_90_TESTING");          
          testState->setState(xlab::StateMachine::State::OVEN_90_TESTING);
          targetTimeForTest = 300000; 
          stopwatchTimeStamp = millis();
          indicator->setColor(CRGB::Blue);
          indicator->setFrequency(100);
        }

        if (testState->getState() == xlab::StateMachine::State::OVEN_90_TEST_READY) {
          Serial.println("#OVEN_60_TESTING");
          testState->setState(xlab::StateMachine::State::OVEN_60_TESTING);
          targetTimeForTest = 900000;
          stopwatchTimeStamp = millis();
          indicator->setColor(CRGB::Purple);
          indicator->setFrequency(100);
        } 

        if (testState->getState() == xlab::StateMachine::State::OVEN_60_TEST_READY) {
          Serial.println("#OVEN_60_TEST_READY");
          testState->setState(xlab::StateMachine::State::READY_FOR_TESTING);          
          indicator->setColor(CRGB::DarkGreen);
          indicator->setFrequency(0);
        }      

    }
  }

  uint32_t timeLeft = millis() - stopwatchTimeStamp;
  if (testState->getState() == xlab::StateMachine::State::OVEN_90_TESTING && timeLeft >= targetTimeForTest) {
     Serial.println("#OVEN_90_TEST_READY");
     testState->setState(xlab::StateMachine::State::OVEN_90_TEST_READY);     
     indicator->setFrequency(0);
  }

  if (testState->getState() == xlab::StateMachine::State::OVEN_60_TESTING && timeLeft >= targetTimeForTest) {
     Serial.println("#OVEN_60_TEST_READY");
     testState->setState(xlab::StateMachine::State::OVEN_60_TEST_READY);
     indicator->setFrequency(0);
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