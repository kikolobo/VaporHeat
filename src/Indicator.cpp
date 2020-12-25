//------------------------------------------------------------------------------
// @file: Heater.cpp
// @created on: Dec 22, 2020
// 
// LICENCE
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "Indicator.h"



namespace xlab { 
    Indicator::Indicator(const int ledPin)
    {        
        ledPin_ = ledPin;
        lastState_ = LOW;
        FastLED.addLeds<WS2811, A7, GRB>(leds_, 1).setCorrection(Typical8mmPixel);          
    }
    
    Indicator::~Indicator() {};

    void Indicator::setBrightness(uint8_t brightness) 
    {
        brightness_ = brightness;
        FastLED.setBrightness(brightness_);
    }
    
    void Indicator::setInstantColor(CRGB color) {
        color_ = color;
        FastLED.show();
    }

    void Indicator::setColor(CRGB color) 
    {
        color_ = color;
    }

    void Indicator::setFrequency(uint32_t frequency) 
    {
        frequency_ = frequency;
    }

    void Indicator::heartBeat()
    {   
        if (frequency_ == 0) {
            leds_[0] = color_;
            FastLED.show();
            return;
        }
       

        if ((millis() - lastChange_) >= frequency_) {
            lastState_ = !lastState_;            
            lastChange_ = millis();
            if (lastState_ == LOW) {
                leds_[0] = CRGB::Black;
            } else {
                leds_[0] = color_;
            }            
            FastLED.setBrightness(brightness_);
            FastLED.show();
        }
    }
    
    

}