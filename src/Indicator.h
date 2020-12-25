//------------------------------------------------------------------------------
// @file: Indicator.h
// @created on: Dec 22, 2020 
// 
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef INDICATOR_H_
#define INDICATOR_H_

// INCLUDES --------------------------------------------------------------------
#include "Arduino.h"
#define FASTLED_INTERNAL
#include <FastLED.h>


// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab { 
class Indicator
{
public:
    // Initialize the heating element
	// 
	// @param elementPin[in]: Pin Number
    Indicator(const int ledPin);
    ~Indicator();

    // Set power to the element via MOSFET
	// 	
    // @return void
    void setBrightness(uint8_t brightness);
    void setColor(CRGB color);
    void setInstantColor(CRGB color);
    void setFrequency(uint32_t frequency = 0);
    void heartBeat();
       
private:        
    CRGB color_ = CRGB::White;
    uint32_t frequency_ = 0;
    uint8_t brightness_ = 100;
    uint32_t lastChange_;
    bool lastState_;
    CRGB leds_[];
    uint8_t ledPin_;
};


} //NameSpace

#endif