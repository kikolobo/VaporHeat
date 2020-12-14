//------------------------------------------------------------------------------
// @file: Heater.cpp
// @created on: November 21, 2020
// 
// LICENCE
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "Heater.h"



namespace xlab { 
    Heater::Heater(const uint8_t elementPin, const uint8_t pwmChannel)
    {        
        pinMode(elementPin, OUTPUT);
        ledcSetup(pwmChannel, 5000, 8); //PWM
        ledcAttachPin(elementPin, pwmChannel);
        elementPin_ = elementPin;
        pwmChannel_ = pwmChannel;
    }

    Heater::~Heater() {};

    void Heater::setPower(uint32_t power)
    {
        power_ = power;
        ledcWrite(pwmChannel_, power);
    }
    
    uint32_t Heater::getPower() {
        return power_;
    }

}