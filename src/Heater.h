//------------------------------------------------------------------------------
// @file: Heater.h
// @created on: Nov 21st, 2020 
// 
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef HEATER_H_
#define HEATER_H_

// INCLUDES --------------------------------------------------------------------
#include "Arduino.h"

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab { 

class Heater
{
public:
    // Initialize the heating element
	// 
	// @param elementPin[in]: Pin Number
    Heater(const uint8_t elementPin, const uint8_t pwmChannel = 0);
    ~Heater();

    // Set power to the element via MOSFET
	// 	
    // @return void
    void setPower(uint32_t power = 0);
    uint32_t getPower();
       
private:
    // Pin number of element
    uint8_t elementPin_;
    uint8_t pwmChannel_;
    uint32_t power_;
};


} //NameSpace

#endif